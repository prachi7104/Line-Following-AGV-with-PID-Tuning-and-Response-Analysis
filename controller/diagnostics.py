"""Read-only diagnostics for the sensor bar and the line-following controller (Member 2).

Nothing here changes how the simulation runs. The functions only observe:

* ``ideal_line_readings`` / ``readings_for_offset``: the sensor readings an ideal straight line
  would produce, computed from the same sub-ray geometry ``LineSensor`` uses, without PyBullet.
* ``sensor_characteristic``: the static sensed-error vs true-error curve of the bar, with its
  quantisation step, dead band, detection limit and linear range.
* ``step_with_diagnostics`` / ``trace_controller``: step a ``LineFollowController`` and report
  its P, I and D terms and four separate limit flags (candidate saturation, integral frozen,
  final output over the limit, command at the limit); see ``StepDiagnostics``.
* ``run_trial_with_diagnostics``: the same loop as ``experiments.run_experiments.run_trial``,
  with a diagnostic record per step. Its DataFrame is identical to ``run_trial``'s.
* ``line_loss_events`` / ``summarize_run``: line-loss and reacquisition events and saturation in
  a logged run (any DataFrame in the ``common.schema`` format, including the published CSVs).

    python -m controller.diagnostics      # prints the sensor characteristic and run summaries
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from common.config import DEFAULT, SimConfig
from common.schema import COLUMNS
from controller.pid import LineFollowController, make_controller
from controller.sensors import error_from_readings, ray_endpoints, readings_from_hits

SAT_TOL = 1e-9  # |u| within this of omega_max counts as saturated


# --------------------------------------------------------------------------------------------
# Sensor model
# --------------------------------------------------------------------------------------------

def ideal_line_readings(pose, cfg: SimConfig = DEFAULT) -> np.ndarray:
    """Readings for an ideal line of width ``cfg.line_width`` centred on the world x axis.

    Uses ``controller.sensors.ray_endpoints`` for the ray positions, so the geometry is the one
    ``LineSensor`` casts in PyBullet; a ray "hits" the line when its foot is within half a line
    width of y = 0.
    """
    starts, _ = ray_endpoints(pose, cfg)
    on_line = np.abs(starts[:, 1]) < cfg.line_width / 2
    return readings_from_hits(on_line.astype(int), 1, cfg)


def readings_for_offset(e_true: float, cfg: SimConfig = DEFAULT) -> np.ndarray:
    """Readings when the sensor bar is ``e_true`` metres LEFT of the line, heading along it."""
    return ideal_line_readings((0.0, float(e_true), 0.0), cfg)


def sensed_error(e_true: float, cfg: SimConfig = DEFAULT) -> float | None:
    """The error the centroid estimator reports for a true bar offset ``e_true`` (None if lost)."""
    return error_from_readings(readings_for_offset(e_true, cfg), cfg)


@dataclass(frozen=True)
class SensorCharacteristic:
    quantisation_step: float  # spacing between neighbouring sensed-error levels [m]
    dead_band: float          # largest |e_true| that still reads as exactly 0 [m]
    detection_limit: float    # largest |e_true| at which any ray still sees the line [m]
    linear_range: float       # |e_true| up to which |e_sensed - e_true| <= step / 2 [m]
    max_error_in_linear_range: float  # max |e_sensed - e_true| inside linear_range [m]
    e_true: np.ndarray        # swept true offsets [m]
    e_sensed: np.ndarray      # sensed error per offset, NaN where the line is lost [m]


def sensor_characteristic(cfg: SimConfig = DEFAULT, span: float = 0.12,
                          resolution: float = 1e-5) -> SensorCharacteristic:
    """Sweep the true bar offset over [-span, span] and measure the static sensor response.

    The grid is shifted by half a resolution step so no sample lands exactly on a ray/line-edge
    coincidence, where "inside the line" is a tie.
    """
    n = int(round(2 * span / resolution))
    e_true = -span + (np.arange(n) + 0.5) * resolution
    e_sensed = np.array([np.nan if (s := sensed_error(e, cfg)) is None else s for e in e_true])

    seen = ~np.isnan(e_sensed)
    detection_limit = float(np.max(np.abs(e_true[seen])))
    dead_band = float(np.max(np.abs(e_true[seen & (e_sensed == 0.0)])))

    levels = np.unique(np.round(e_sensed[seen], 12))
    step = float(np.min(np.diff(levels)))

    err = np.abs(e_sensed - e_true)
    outside = np.abs(e_true)[~seen | (err > step / 2 + 1e-9)]
    linear_range = float(np.min(outside)) - resolution if outside.size else span
    inside = np.abs(e_true) <= linear_range
    return SensorCharacteristic(
        quantisation_step=step,
        dead_band=dead_band,
        detection_limit=detection_limit,
        linear_range=linear_range,
        max_error_in_linear_range=float(np.max(err[inside])),
        e_true=e_true,
        e_sensed=e_sensed,
    )


# --------------------------------------------------------------------------------------------
# Controller instrumentation
# --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class StepDiagnostics:
    """One controller step. The four limit-related flags answer different questions:

    * ``candidate_saturated``: |kp*e + kd*d_f + ki*(I + e*dt)| > limit. This is the test
      ``PID.update`` makes *before* deciding whether to integrate.
    * ``integral_frozen``: candidate saturated AND e has the same sign as that candidate output,
      so conditional integration skipped this step (anti-windup acted).
    * ``output_saturated``: the final |p + i + d| > limit, i.e. the clamp changed the output.
    * ``at_limit``: the command actually sent has |u| == limit (within ``SAT_TOL``). This is
      also true when the final output lands exactly on the limit without exceeding it.

    They can differ on the same step. Example: PI with e = 0.15 m from reset. The candidate is
    3.003 rad/s, so the integral is frozen; the final output is then exactly 20 * 0.15 = 3.0,
    so it is at the limit but not over it (output_saturated is False).
    """
    e_used: float            # error fed to the PID [m] (held value when the line is lost)
    line_detected: bool
    p_term: float            # kp * e
    i_term: float            # ki * integral (after this step's update)
    d_term: float            # kd * filtered derivative
    candidate: float         # kp*e + kd*d_f + ki*(I_before + e*dt), checked before integrating
    unclamped: float         # final p + i + d, before the +-limit clamp
    u: float                 # yaw-rate command actually returned, u = -clamp(unclamped)
    candidate_saturated: bool
    integral_frozen: bool
    output_saturated: bool
    at_limit: bool


def step_with_diagnostics(controller: LineFollowController, e_sensed: float | None) -> StepDiagnostics:
    """Call ``controller.step(e_sensed)`` once and describe what the PID did.

    The controller is stepped exactly as in the simulation; this function only reads its state
    before and after, so the returned ``u`` is the same value ``controller.step`` returns. The
    candidate output is recomputed with the same expression, in the same order, as
    ``PID.update``.
    """
    pid = controller.pid
    integral_before = pid.integral
    out = controller.step(e_sensed)
    e = out.e_used
    p_term = pid.kp * e
    i_term = pid.ki * pid.integral
    d_term = pid.kd * pid.d_filtered
    candidate = (pid.kp * e + pid.kd * pid.d_filtered) + pid.ki * (integral_before + e * pid.dt)
    candidate_saturated = abs(candidate) > pid.limit
    unclamped = p_term + i_term + d_term
    return StepDiagnostics(
        e_used=e,
        line_detected=out.line_detected,
        p_term=p_term,
        i_term=i_term,
        d_term=d_term,
        candidate=candidate,
        unclamped=unclamped,
        u=out.u,
        candidate_saturated=candidate_saturated,
        integral_frozen=candidate_saturated and e * candidate > 0.0,
        output_saturated=abs(unclamped) > pid.limit,
        at_limit=abs(out.u) >= pid.limit - SAT_TOL,
    )


def trace_controller(mode: str, errors, e0: float = 0.0,
                     cfg: SimConfig = DEFAULT) -> list[StepDiagnostics]:
    """Run a fresh ``mode`` controller, reset to ``e0``, over a sequence of sensed errors."""
    controller = make_controller(mode, cfg)
    controller.reset(e0)
    return [step_with_diagnostics(controller, e) for e in errors]


def run_trial_with_diagnostics(track, mode: str, cfg: SimConfig = DEFAULT,
                               start_offset: float = 0.0, max_time: float | None = None
                               ) -> tuple[pd.DataFrame, list[StepDiagnostics]]:
    """``run_trial`` (headless) plus one ``StepDiagnostics`` per logged step.

    Mirrors ``experiments.run_experiments.run_trial`` step for step; the test suite checks that
    the returned DataFrame equals ``run_trial``'s exactly.
    """
    from controller.sensors import LineSensor
    from simulation.world import Simulation

    max_time = cfg.max_time if max_time is None else max_time
    sim = Simulation(track, cfg, gui=False, start_offset=start_offset)
    try:
        sensor = LineSensor(sim.client, sim.line_id, cfg)
        controller = make_controller(mode, cfg)
        first = sensor.read(sim.robot.pose())
        controller.reset(first.error if first.error is not None else start_offset)

        rows, diags = [], []
        t = 0.0
        while t < max_time:
            x, y, yaw = sim.robot.pose()
            if track.end_reached(x, y):
                break
            diag = step_with_diagnostics(controller, sensor.read((x, y, yaw)).error)
            sim.robot.set_command(cfg.v, diag.u)
            rows.append({
                "t": t, "x": x, "y": y, "yaw": yaw,
                "e_true": track.lateral_error(x, y),
                "e_sensed": diag.e_used,
                "u": diag.u,
                "controller": mode,
                "line_detected": int(diag.line_detected),
                "segment": track.segment(x, y),
            })
            diags.append(diag)
            sim.step()
            t += cfg.dt
    finally:
        sim.close()
    return pd.DataFrame(rows, columns=COLUMNS), diags


# --------------------------------------------------------------------------------------------
# Run-level summaries
# --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class LossEvent:
    start_step: int
    n_steps: int
    reacquired: bool         # False if the run ended with the line still lost


def line_loss_events(line_detected) -> list[LossEvent]:
    """Contiguous stretches of steps in which no sensor saw the line."""
    lost = ~np.asarray(line_detected, dtype=bool)
    events, k, n = [], 0, len(lost)
    while k < n:
        if lost[k]:
            j = k
            while j < n and lost[j]:
                j += 1
            events.append(LossEvent(start_step=k, n_steps=j - k, reacquired=j < n))
            k = j
        else:
            k += 1
    return events


@dataclass(frozen=True)
class RunSummary:
    n_steps: int
    detected_fraction: float
    loss_events: list[LossEvent]
    longest_loss_s: float
    first_detection_s: float | None   # None if the line was never seen
    max_abs_u: float
    at_limit_fraction: float          # share of steps whose command |u| is at omega_max; the
                                      # same definition as StepDiagnostics.at_limit. A logged
                                      # run has only u, so this is all it can measure.


def summarize_run(df: pd.DataFrame, cfg: SimConfig = DEFAULT) -> RunSummary:
    """Line-loss and saturation summary of a logged run (``common.schema`` columns)."""
    detected = df["line_detected"].to_numpy().astype(bool)
    u = np.abs(df["u"].to_numpy(dtype=float))
    events = line_loss_events(detected)
    first = int(np.argmax(detected)) if detected.any() else None
    return RunSummary(
        n_steps=len(df),
        detected_fraction=float(detected.mean()) if len(df) else 0.0,
        loss_events=events,
        longest_loss_s=max((e.n_steps for e in events), default=0) * cfg.dt,
        first_detection_s=None if first is None else float(df["t"].iloc[first]),
        max_abs_u=float(u.max()) if len(df) else 0.0,
        at_limit_fraction=float(np.mean(u >= cfg.omega_max - SAT_TOL)) if len(df) else 0.0,
    )


def main() -> None:
    from common.schema import read_run

    c = sensor_characteristic()
    print("sensor characteristic (default config)")
    print(f"  quantisation step      {c.quantisation_step * 1000:.3f} mm")
    print(f"  dead band              +-{c.dead_band * 1000:.3f} mm")
    print(f"  linear range           +-{c.linear_range * 1000:.2f} mm "
          f"(max error {c.max_error_in_linear_range * 1000:.3f} mm)")
    print(f"  detection limit        +-{c.detection_limit * 1000:.2f} mm")
    run_dir = Path("results/runs")
    for path in sorted(run_dir.glob("*.csv")):
        s = summarize_run(read_run(path))
        print(f"{path.name:18s} steps={s.n_steps:5d} detected={s.detected_fraction:.3f} "
              f"loss_events={len(s.loss_events)} max|u|={s.max_abs_u:.3f} "
              f"at_limit={s.at_limit_fraction:.3f}")


if __name__ == "__main__":
    main()
