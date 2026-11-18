"""Run the controlled experiments and log one CSV row per simulation step.

    python -m experiments.run_experiments --experiment straight|curve|lookahead|all

Each trial: build the world, read the sensor once to initialise the controller with the
starting error (so the D term sees no artificial jump), then loop
    read sensors -> controller -> wheel command -> log row -> physics step
until the track end is reached or the timeout expires.
"""
import argparse
import time
from dataclasses import replace
from pathlib import Path

import pandas as pd

from common.config import DEFAULT, SimConfig
from common.schema import COLUMNS, write_run
from controller.pid import make_controller
from controller.sensors import LineSensor
from simulation.track import CurveTrack, StraightTrack
from simulation.world import Simulation

RUN_DIR = Path("results/runs")
MODES = ("P", "PI", "PID")
LOOKAHEADS = (0.05, 0.10, 0.15)


def run_trial(track, mode: str, cfg: SimConfig = DEFAULT, start_offset: float = 0.0,
              max_time: float | None = None, gui: bool = False, realtime: bool = False) -> pd.DataFrame:
    max_time = cfg.max_time if max_time is None else max_time
    sim = Simulation(track, cfg, gui=gui, start_offset=start_offset)
    try:
        sensor = LineSensor(sim.client, sim.line_id, cfg)
        controller = make_controller(mode, cfg)
        first = sensor.read(sim.robot.pose())
        controller.reset(first.error if first.error is not None else start_offset)

        rows = []
        t = 0.0
        while t < max_time:
            x, y, yaw = sim.robot.pose()
            if track.end_reached(x, y):
                break
            out = controller.step(sensor.read((x, y, yaw)).error)
            sim.robot.set_command(cfg.v, out.u)
            rows.append({
                "t": t, "x": x, "y": y, "yaw": yaw,
                "e_true": track.lateral_error(x, y),
                "e_sensed": out.e_used,
                "u": out.u,
                "controller": mode,
                "line_detected": int(out.line_detected),
                "segment": track.segment(x, y),
            })
            sim.step()
            if realtime:
                time.sleep(cfg.dt)
            t += cfg.dt
    finally:
        sim.close()
    return pd.DataFrame(rows, columns=COLUMNS)


def _save(df: pd.DataFrame, name: str) -> Path:
    path = RUN_DIR / f"{name}.csv"
    write_run(path, df.to_dict("records"))
    return path


def straight_experiment(cfg: SimConfig = DEFAULT) -> list[Path]:
    """5 cm lateral-reference step on the straight track, all three controllers."""
    return [_save(run_trial(StraightTrack(cfg.straight_length), m, cfg, cfg.initial_offset), f"straight_{m}")
            for m in MODES]


def curve_experiment(cfg: SimConfig = DEFAULT) -> list[Path]:
    """Start on the line, follow the lead-in and the constant-curvature arc, all three controllers."""
    track = CurveTrack(cfg.curve_lead, cfg.curve_radius, cfg.curve_angle)
    return [_save(run_trial(track, m, cfg, 0.0), f"curve_{m}") for m in MODES]


def lookahead_experiment(cfg: SimConfig = DEFAULT) -> list[Path]:
    """Optional: the straight-track step with P control for several sensor look-ahead distances."""
    paths = []
    for d in LOOKAHEADS:
        c = replace(cfg, d=d)
        df = run_trial(StraightTrack(c.straight_length), "P", c, c.initial_offset)
        paths.append(_save(df, f"lookahead_d{round(d * 100):02d}_P"))
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--experiment", choices=["straight", "curve", "lookahead", "all"], default="all")
    args = parser.parse_args()
    jobs = {"straight": straight_experiment, "curve": curve_experiment, "lookahead": lookahead_experiment}
    selected = ["straight", "curve"] if args.experiment == "all" else [args.experiment]
    for name in selected:
        for path in jobs[name]():
            print(f"wrote {path}")


if __name__ == "__main__":
    main()
