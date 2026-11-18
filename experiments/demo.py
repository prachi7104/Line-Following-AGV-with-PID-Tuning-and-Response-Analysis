"""Live GUI demo for the presentation video.

    python -m experiments.demo --track curve --controller PID

Opens the PyBullet window, runs one trial in real time with the camera following the robot,
then prints the run's metrics so they can be read out during the demo.
"""
import argparse

from analysis.metrics import curve_metrics, step_metrics
from common.config import DEFAULT
from experiments.run_experiments import run_trial
from simulation.track import CurveTrack, StraightTrack


def main() -> None:
    parser = argparse.ArgumentParser(description="Real-time PyBullet demo of the line-following AGV")
    parser.add_argument("--track", choices=["straight", "curve"], default="curve")
    parser.add_argument("--controller", choices=["P", "PI", "PID"], default="PID")
    args = parser.parse_args()

    cfg = DEFAULT
    if args.track == "straight":
        track, offset = StraightTrack(cfg.straight_length), cfg.initial_offset
    else:
        track, offset = CurveTrack(cfg.curve_lead, cfg.curve_radius, cfg.curve_angle), 0.0

    df = run_trial(track, args.controller, cfg, offset, gui=True, realtime=True)
    t, e = df["t"].to_numpy(), df["e_true"].to_numpy()
    if args.track == "straight":
        metrics = step_metrics(t, e, e[0], cfg.settle_band)
    else:
        metrics = curve_metrics(t, e, df["segment"].to_numpy())
    print(f"{args.controller} on {args.track} track:")
    for key, value in metrics.items():
        print(f"  {key:15s} {'not meaningful' if value is None else f'{value:.4f}'}")


if __name__ == "__main__":
    main()
