"""Figures and summary tables for the P / PI / PID comparison.

Usage (from the repo root):
    python -m analysis.plot_results           # real runs in results/runs/ -> results/figures/, results/summary_*.md
    python -m analysis.plot_results --dummy   # FAKE data -> results/dummy/
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from common.config import DEFAULT  # noqa: E402

MODES = ("P", "PI", "PID")
COLORS = {"P": "#1f77b4", "PI": "#ff7f0e", "PID": "#2ca02c"}
REPO = Path(__file__).resolve().parents[1]
RESULTS = REPO / "results"


def _color(name: str, i: int) -> str:
    return COLORS.get(name, f"C{i}")


def plot_error_vs_time(runs: dict[str, pd.DataFrame], title: str, path, band: float | None = None) -> Path:
    """e_true [mm] against t [s], one line per controller; dashed +-band if given."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for i, (name, df) in enumerate(runs.items()):
        ax.plot(df["t"], df["e_true"] * 1000.0, color=_color(name, i), lw=1.4, label=name)
    if band is not None:
        for s in (1, -1):
            ax.axhline(s * band * 1000.0, color="0.4", ls="--", lw=0.9,
                       label=f"±{band * 1000:g} mm band" if s == 1 else None)
    ax.axhline(0.0, color="0.75", lw=0.8, zorder=0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("lateral error e_true [mm]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_paths(runs: dict[str, pd.DataFrame], track_xy: np.ndarray, path, title: str = "Robot paths") -> Path:
    """Robot x-y paths (axle midpoint) over the line polyline, equal axes."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    track_xy = np.asarray(track_xy, dtype=float)
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(track_xy[:, 0], track_xy[:, 1], color="0.15", lw=4, alpha=0.25, label="line")
    for i, (name, df) in enumerate(runs.items()):
        ax.plot(df["x"], df["y"], color=_color(name, i), lw=1.2, label=name)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


CURVE_NOTE = (
    "Units: metres (errors) and metre-seconds (iae); arc rows only. `steady_state` is the signed mean "
    "of e_true over the final 40 % of the arc's duration. It is a tail-window measurement, not a "
    "converged equilibrium: the PI/PID integrator is still converging when the arc ends "
    "(see docs/RESULTS.md section 2)."
)


def _write_summary(paths: dict[str, Path], kind: str, out: Path, header: str) -> Path:
    from analysis.metrics import summarize, to_markdown

    out.parent.mkdir(parents=True, exist_ok=True)
    note = "\n" + CURVE_NOTE + "\n" if kind == "curve" else ""
    out.write_text(header + "\n\n" + to_markdown(summarize(paths, kind)) + note, encoding="utf-8")
    return out


def _run_dummy() -> list[Path]:
    from analysis.dummy_data import dummy_track_xy, make_dummy_run
    from common.schema import write_run

    out_dir = RESULTS / "dummy"
    written: list[Path] = []
    for kind in ("straight", "curve"):
        runs = {m: make_dummy_run(m, kind=kind) for m in MODES}
        csvs = {}
        for m, df in runs.items():
            p = out_dir / "runs" / f"{kind}_{m}.csv"
            write_run(p, df.to_dict("records"))
            csvs[m] = p
        written.append(plot_error_vs_time(runs, f"DUMMY DATA - {kind} - lateral error", out_dir / f"error_{kind}.png",
                                          band=DEFAULT.settle_band if kind == "straight" else None))
        written.append(plot_paths(runs, dummy_track_xy(kind), out_dir / f"paths_{kind}.png",
                                  title=f"DUMMY DATA - {kind} - robot paths"))
        written.append(_write_summary(csvs, kind, out_dir / f"summary_{kind}.md",
                                      f"# DUMMY DATA - {kind} summary (fake, not results)"))
    return written


def _run_real() -> list[Path]:
    from common.schema import read_run
    from simulation.track import CurveTrack, StraightTrack  # imported here on purpose: optional dependency

    tracks = {
        "straight": StraightTrack(length=DEFAULT.straight_length),
        "curve": CurveTrack(lead=DEFAULT.curve_lead, radius=DEFAULT.curve_radius, angle=DEFAULT.curve_angle),
    }
    run_dir = RESULTS / "runs"
    fig_dir = RESULTS / "figures"
    written: list[Path] = []
    for kind, track in tracks.items():
        csvs = {m: run_dir / f"{kind}_{m}.csv" for m in MODES}
        csvs = {m: p for m, p in csvs.items() if p.exists()}
        if not csvs:
            print(f"no {kind} runs found in {run_dir}, skipping", file=sys.stderr)
            continue
        runs = {m: read_run(p) for m, p in csvs.items()}
        written.append(plot_error_vs_time(runs, f"{kind} track - lateral error", fig_dir / f"error_{kind}.png",
                                          band=DEFAULT.settle_band if kind == "straight" else None))
        written.append(plot_paths(runs, track.polyline(), fig_dir / f"paths_{kind}.png",
                                  title=f"{kind} track - robot paths"))
        written.append(_write_summary(csvs, kind, RESULTS / f"summary_{kind}.md", f"# {kind} track summary"))
    if not written:
        raise SystemExit("no runs found; run `python -m experiments.run_experiments` first or use --dummy")
    return written


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dummy", action="store_true", help="use fake data and write into results/dummy/")
    args = parser.parse_args(argv)
    for p in (_run_dummy() if args.dummy else _run_real()):
        print(p.relative_to(REPO))


if __name__ == "__main__":
    main()
