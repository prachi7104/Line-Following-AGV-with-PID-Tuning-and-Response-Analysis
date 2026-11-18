"""Sensor-controller module tests (Member 2): controller.diagnostics and the behaviour it measures.

Existing coverage this file does NOT repeat: sub-ray layout, single centroid values, readings
from hit ids, ray endpoints (tests/test_sensors.py); PID P/I/D maths at one step, saturation,
anti-windup recovery with custom gains, sign and line-loss hold (tests/test_pid.py); the six
published runs end to end (tests/test_integration.py).

What is new here:
  * the analytic sensor model agrees with PyBullet ray casting at many offsets and headings;
  * the bar's static characteristic (3 mm step, 1.5 mm dead band, detection limit);
  * the diagnostics observe the controller without changing a single output value;
  * the derivative filter over many steps and anti-windup with the frozen project gains;
  * the four limit flags (candidate saturation, integral frozen, output over the limit, command
    at the limit) at their boundaries, including PI at e = 0.15 m;
  * line loss at the start of a run, the held error, saturation and reacquisition, in PyBullet;
  * the published CSVs never lose the line and never reach the limit, as docs/RESULTS.md states;
  * all six published scenarios: the instrumented loop equals the experiment runner, and its
    CSV output reproduces the published file.
"""
import math
from pathlib import Path

import numpy as np
import pybullet  # noqa: F401  (hard requirement: missing pybullet must fail, not skip)
import pytest

from common.config import DEFAULT, GAINS
from common.schema import read_run, write_run
from controller.diagnostics import (
    SAT_TOL,
    ideal_line_readings,
    line_loss_events,
    readings_for_offset,
    run_trial_with_diagnostics,
    sensed_error,
    sensor_characteristic,
    step_with_diagnostics,
    summarize_run,
    trace_controller,
)
from controller.pid import make_controller
from controller.sensors import LineSensor, ray_endpoints
from experiments.run_experiments import MODES, run_trial
from simulation.track import CurveTrack, StraightTrack
from simulation.world import Simulation

RUN_DIR = Path(__file__).resolve().parents[1] / "results" / "runs"
PUBLISHED = sorted(RUN_DIR.glob("*.csv"))
FAR_OFFSET = 0.15      # well beyond the ~88.5 mm detection limit
X_PROBE = 1.0          # bar position along the straight line, away from both line ends


@pytest.fixture(scope="module")
def characteristic():
    return sensor_characteristic(DEFAULT, span=0.12, resolution=2e-5)


@pytest.fixture(scope="module")
def straight_sim():
    sim = Simulation(StraightTrack(), start_offset=0.0)
    yield sim
    sim.close()


def _clear_of_edges(pose, margin=2e-4):
    """True if no sub-ray foot lies within ``margin`` of a line edge (a tie for either model)."""
    starts, _ = ray_endpoints(pose, DEFAULT)
    return np.min(np.abs(np.abs(starts[:, 1]) - DEFAULT.line_width / 2)) > margin


# ------------------------------------------------------------------ sensor model vs PyBullet

@pytest.mark.parametrize("yaw", [0.0, 0.15, -0.30])
def test_ideal_model_matches_pybullet_raycast(straight_sim, yaw):
    sensor = LineSensor(straight_sim.client, straight_sim.line_id, DEFAULT)
    checked = 0
    for y in np.linspace(-0.095, 0.095, 41) + 0.00037:
        bar_x = X_PROBE - DEFAULT.d * math.cos(yaw)
        pose = (bar_x, float(y), yaw)
        if not _clear_of_edges(pose):
            continue
        sim_reading = sensor.read(pose)
        np.testing.assert_allclose(sim_reading.readings, ideal_line_readings(pose, DEFAULT),
                                   err_msg=f"pose={pose}")
        checked += 1
    assert checked >= 30, "too few offsets checked; the sweep did not exercise the bar"


def test_detection_limit_matches_pybullet(straight_sim, characteristic):
    sensor = LineSensor(straight_sim.client, straight_sim.line_id, DEFAULT)
    limit = characteristic.detection_limit
    for side in (+1, -1):
        inside = sensor.read((X_PROBE - DEFAULT.d, side * (limit - 0.001), 0.0))
        outside = sensor.read((X_PROBE - DEFAULT.d, side * (limit + 0.001), 0.0))
        assert inside.error is not None
        assert outside.error is None
        np.testing.assert_allclose(outside.readings, 0.0)


# ------------------------------------------------------------------ static characteristic

def test_quantisation_and_dead_band(characteristic):
    step = DEFAULT.sensor_spacing / DEFAULT.sub_rays_per_sensor     # 3 mm
    assert characteristic.quantisation_step == pytest.approx(step, abs=1e-9)
    # docs/RESULTS.md section 4: offsets under about 1.5 mm read as zero
    assert characteristic.dead_band == pytest.approx(step / 2, abs=5e-5)
    assert sensed_error(0.0012) == 0.0
    assert sensed_error(0.0018) == pytest.approx(step)


def test_linear_range_error_bounded_by_half_step(characteristic):
    c = characteristic
    step = c.quantisation_step
    inside = np.abs(c.e_true) <= c.linear_range
    assert np.all(np.abs(c.e_sensed[inside] - c.e_true[inside]) <= step / 2 + 1e-9)
    # the line stays fully under the outer sub-rays out to about 60 mm
    assert 0.055 < c.linear_range < c.detection_limit


def test_transfer_curve_monotonic_and_odd(characteristic):
    c = characteristic
    seen = ~np.isnan(c.e_sensed)
    assert np.all(np.diff(c.e_sensed[seen]) >= -1e-12)
    for e in (0.004, 0.021, 0.047, 0.073):
        assert sensed_error(-e) == pytest.approx(-sensed_error(e), abs=1e-12)


def test_detection_limit_from_geometry(characteristic):
    outer_ray = DEFAULT.sensor_centres()[-1] + DEFAULT.sensor_spacing / 2 \
        - DEFAULT.sensor_spacing / (2 * DEFAULT.sub_rays_per_sensor)
    expected = outer_ray + DEFAULT.line_width / 2                   # 73.5 + 15 = 88.5 mm
    assert characteristic.detection_limit == pytest.approx(expected, abs=5e-5)
    assert sensed_error(expected + 0.001) is None
    np.testing.assert_allclose(readings_for_offset(0.2), 0.0)


# ------------------------------------------------------------------ controller diagnostics

ERRORS = [0.05, 0.049, None, None, 0.03, -0.01, 0.0, None, 0.2, 0.2, -0.004, 0.0015]


@pytest.mark.parametrize("mode", MODES)
def test_diagnostics_do_not_change_controller_output(mode):
    plain = make_controller(mode)
    plain.reset(0.05)
    expected = [plain.step(e) for e in ERRORS]
    traced = trace_controller(mode, ERRORS, e0=0.05)
    assert [d.u for d in traced] == [o.u for o in expected]
    assert [d.e_used for d in traced] == [o.e_used for o in expected]
    assert [d.line_detected for d in traced] == [o.line_detected for o in expected]


@pytest.mark.parametrize("mode", MODES)
def test_term_decomposition(mode):
    kp, ki, kd = GAINS[mode]
    for d in trace_controller(mode, ERRORS, e0=0.05):
        assert d.p_term == pytest.approx(kp * d.e_used)
        assert d.unclamped == pytest.approx(d.p_term + d.i_term + d.d_term)
        clamp = max(-DEFAULT.omega_max, min(DEFAULT.omega_max, d.unclamped))
        assert d.u == pytest.approx(-clamp)
        assert d.output_saturated == (abs(d.unclamped) > DEFAULT.omega_max)
        assert d.candidate_saturated == (abs(d.candidate) > DEFAULT.omega_max)
        assert d.at_limit == (abs(d.u) >= DEFAULT.omega_max - SAT_TOL)
        assert d.integral_frozen == (d.candidate_saturated and d.e_used * d.candidate > 0)
        if ki == 0.0:
            assert d.i_term == 0.0
        if kd == 0.0:
            assert d.d_term == 0.0


def test_derivative_filter_first_order_decay():
    """A step in e gives a D term that decays as alpha/dt * (1-alpha)^k with tau = 50 ms."""
    dt, tau = DEFAULT.dt, DEFAULT.d_filter_tau
    alpha = dt / (tau + dt)
    step = 0.01
    trace = trace_controller("PID", [step] * 60, e0=0.0)
    kd = GAINS["PID"][2]
    for k, d in enumerate(trace):
        assert d.d_term == pytest.approx(kd * step / dt * alpha * (1 - alpha) ** k, rel=1e-9)
    # after one time constant (12 steps) the D term has fallen to (1-alpha)^12 = 0.383, close to
    # the continuous-time 1/e = 0.368
    n_tau = int(round(tau / dt))
    ratio = trace[n_tau].d_term / trace[0].d_term
    assert ratio == pytest.approx((1 - alpha) ** n_tau, rel=1e-9)
    assert ratio == pytest.approx(math.exp(-1), rel=0.05)


@pytest.mark.parametrize("mode", MODES)
def test_integral_frozen_flag_matches_pid_state(mode):
    """integral_frozen is derived from the candidate; check it against what PID.update did."""
    ctrl = make_controller(mode)
    ctrl.reset(0.05)
    seq = ERRORS + [0.15] * 5 + [0.2] * 5 + [-0.2] * 5 + [0.01] * 5
    n_frozen = 0
    for e in seq:
        before = ctrl.pid.integral
        d = step_with_diagnostics(ctrl, e)
        if d.e_used != 0.0:
            assert d.integral_frozen == (ctrl.pid.integral == before)
        if d.integral_frozen:
            assert ctrl.pid.integral == before
            n_frozen += 1
        else:
            assert ctrl.pid.integral == before + d.e_used * DEFAULT.dt
    assert n_frozen >= 10, "the sequence never exercised anti-windup"


def test_boundary_p_output_exactly_at_limit():
    """P, e = 0.15 m: kp*e = 3.0 exactly. At the limit, but nothing exceeds it."""
    d = trace_controller("P", [0.15], e0=0.15)[0]
    assert d.candidate == 3.0 and d.unclamped == 3.0
    assert d.u == -DEFAULT.omega_max
    assert d.at_limit
    assert not d.output_saturated
    assert not d.candidate_saturated and not d.integral_frozen


def test_boundary_p_output_exceeding_limit():
    """P, e = 0.2 m: kp*e = 4 rad/s. Over the limit, clamped to it."""
    d = trace_controller("P", [0.2], e0=0.2)[0]
    assert d.unclamped == pytest.approx(4.0)
    assert d.output_saturated and d.candidate_saturated and d.at_limit
    assert d.u == -DEFAULT.omega_max


def test_boundary_pi_015_candidate_saturation_freezes_integral_then_recovers():
    """PI (20, 5, 0), e = 0.15 m from reset.

    Candidate = 3.0 + 5 * 0.15 * dt = 3.003 > 3, pushing further, so conditional integration
    skips the step and the integral stays 0. The final output is then kp*e = 3.0 exactly:
    at the limit but not over it. The flags differ on purpose.
    """
    kp, ki, _ = GAINS["PI"]
    dt = DEFAULT.dt
    ctrl = make_controller("PI")
    ctrl.reset(0.15)
    for _ in range(50):
        d = step_with_diagnostics(ctrl, 0.15)
        assert d.candidate == pytest.approx(kp * 0.15 + ki * 0.15 * dt, rel=1e-12)
        assert d.candidate_saturated and d.integral_frozen
        assert ctrl.pid.integral == 0.0 and d.i_term == 0.0
        assert d.unclamped == 3.0 and not d.output_saturated
        assert d.at_limit and d.u == -DEFAULT.omega_max

    # recovery: e = 0.10 m gives candidate 2.002 < 3; integration resumes from 0 at once
    r = step_with_diagnostics(ctrl, 0.10)
    assert not (r.candidate_saturated or r.integral_frozen or r.output_saturated or r.at_limit)
    assert ctrl.pid.integral == pytest.approx(0.10 * dt)
    assert r.u == pytest.approx(-(kp * 0.10 + ki * 0.10 * dt))


@pytest.mark.parametrize("mode", ["PI", "PID"])
def test_antiwindup_freezes_integral_with_project_gains(mode):
    # kp * 0.2 = 4 rad/s > omega_max: saturated and pushing further on every step
    trace = trace_controller(mode, [0.2] * 200, e0=0.2)
    assert all(d.candidate_saturated and d.integral_frozen for d in trace)
    assert all(d.output_saturated and d.at_limit for d in trace)
    assert all(d.i_term == 0.0 for d in trace)
    assert all(d.u == -DEFAULT.omega_max for d in trace)
    # once the error drops back, integration resumes at once: nothing was wound up
    ctrl = make_controller(mode)
    ctrl.reset(0.2)
    for _ in range(200):
        ctrl.step(0.2)
    back = step_with_diagnostics(ctrl, 0.01)
    assert not back.integral_frozen
    assert back.i_term == pytest.approx(GAINS[mode][1] * 0.01 * DEFAULT.dt)
    if mode == "PI":
        assert not back.output_saturated and not back.at_limit
        assert back.u == pytest.approx(-back.unclamped)
    else:
        # PID: the 0.2 -> 0.01 drop gives a large filtered-D kick that saturates the other way;
        # the output then opposes e, so conditional integration correctly keeps integrating
        assert back.candidate_saturated and not back.integral_frozen
        assert back.output_saturated and back.at_limit and back.unclamped < 0 < back.u


# ------------------------------------------------------------------ line loss and reacquisition

def test_line_loss_events_segmentation():
    events = line_loss_events([0, 0, 1, 1, 0, 1, 1, 0, 0])
    assert [(e.start_step, e.n_steps, e.reacquired) for e in events] == [
        (0, 2, True), (4, 1, True), (7, 2, False)]
    assert line_loss_events([1, 1, 1]) == []


def test_run_trial_with_diagnostics_is_identical_to_run_trial():
    track = StraightTrack(DEFAULT.straight_length)
    reference = run_trial(track, "PID", DEFAULT, FAR_OFFSET, max_time=2.0)
    df, diags = run_trial_with_diagnostics(track, "PID", DEFAULT, FAR_OFFSET, max_time=2.0)
    assert df.equals(reference)
    assert len(diags) == len(df)
    assert [d.u for d in diags] == df["u"].tolist()


@pytest.mark.parametrize("mode", MODES)
def test_line_lost_at_start_is_reacquired(mode, characteristic):
    assert FAR_OFFSET > characteristic.detection_limit
    df, diags = run_trial_with_diagnostics(StraightTrack(DEFAULT.straight_length), mode,
                                           DEFAULT, FAR_OFFSET)
    s = summarize_run(df)

    # exactly one loss stretch, at the very start, and the line is then never lost again
    assert len(s.loss_events) == 1
    first = s.loss_events[0]
    assert first.start_step == 0 and first.reacquired
    assert df["line_detected"].iloc[first.n_steps:].all()
    assert 0.1 < s.first_detection_s < 0.5

    # while lost, the controller held the initial error (never 0) and steered hard right
    lost = df.iloc[:first.n_steps]
    assert (lost["e_sensed"] == FAR_OFFSET).all()
    assert lost["u"].iloc[0] == -DEFAULT.omega_max
    assert (lost["u"] < 0).all()
    if mode != "P":
        assert any(d.integral_frozen for d in diags[:first.n_steps])
    # the logged-run summary and the per-step flag use the same at-limit definition
    assert s.at_limit_fraction == pytest.approx(np.mean([d.at_limit for d in diags]))
    assert s.at_limit_fraction > 0.0

    # it finishes the track and settles inside the band, never leaving the +-limit
    assert np.all(np.abs(df["u"]) <= DEFAULT.omega_max + 1e-12)
    assert abs(df["e_true"].iloc[-1]) <= DEFAULT.settle_band
    assert df["t"].iloc[-1] < DEFAULT.max_time - 0.2


# ------------------------------------------------------------------ published results

def test_published_runs_present():
    assert [p.name for p in PUBLISHED] == [
        "curve_P.csv", "curve_PI.csv", "curve_PID.csv",
        "straight_P.csv", "straight_PI.csv", "straight_PID.csv"]


@pytest.mark.parametrize("path", PUBLISHED, ids=[p.stem for p in PUBLISHED])
def test_published_runs_never_lose_line_or_saturate(path):
    """docs/RESULTS.md: line lost 0 % in every run; max |u| about 1 rad/s, so anti-windup never acts."""
    s = summarize_run(read_run(path))
    assert s.detected_fraction == 1.0
    assert s.loss_events == []
    assert s.first_detection_s == 0.0
    assert s.at_limit_fraction == 0.0
    assert s.max_abs_u < 1.1


# ------------------------------------------------------------------ six-scenario equivalence

SCENARIOS = [(kind, mode) for kind in ("straight", "curve") for mode in MODES]
_EQUIV: dict = {}


def _scenario(kind, mode):
    """(run_trial DataFrame, instrumented DataFrame, diagnostics) for one published scenario."""
    if (kind, mode) not in _EQUIV:
        if kind == "straight":
            track, offset = StraightTrack(DEFAULT.straight_length), DEFAULT.initial_offset
        else:
            track = CurveTrack(DEFAULT.curve_lead, DEFAULT.curve_radius, DEFAULT.curve_angle)
            offset = 0.0
        reference = run_trial(track, mode, DEFAULT, offset)
        df, diags = run_trial_with_diagnostics(track, mode, DEFAULT, offset)
        _EQUIV[(kind, mode)] = (reference, df, diags)
    return _EQUIV[(kind, mode)]


def _normalised_bytes(path):
    return Path(path).read_bytes().replace(b"\r\n", b"\n")


@pytest.mark.parametrize(("kind", "mode"), SCENARIOS, ids=[f"{k}-{m}" for k, m in SCENARIOS])
def test_instrumented_run_equals_experiment_runner(kind, mode):
    reference, df, diags = _scenario(kind, mode)
    assert df.equals(reference)
    assert [d.u for d in diags] == reference["u"].tolist()
    # in the published scenarios nothing reaches the limit and anti-windup never acts
    assert not any(d.at_limit or d.output_saturated or d.integral_frozen for d in diags)


@pytest.mark.parametrize(("kind", "mode"), SCENARIOS, ids=[f"{k}-{m}" for k, m in SCENARIOS])
def test_instrumented_run_reproduces_published_csv(kind, mode, tmp_path):
    """Write the instrumented run with the project's CSV writer into a temporary directory and
    compare it with the published file (line endings normalised). Exact reproduction is what
    the locked reference environment gives; other PyBullet builds may differ in the last digits
    (README, Limitations)."""
    _, df, _ = _scenario(kind, mode)
    out = tmp_path / f"{kind}_{mode}.csv"
    write_run(out, df.to_dict("records"))
    assert _normalised_bytes(out) == _normalised_bytes(RUN_DIR / f"{kind}_{mode}.csv")
