"""Response metrics on logged runs, following the definitions in docs/PROJECT_SPEC.md.

All metrics use ground-truth lateral error `e_true` [m] against time `t` [s].
A metric that is not meaningful for a run is returned as None (reported as
"not meaningful"), never forced to a number.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pandas as pd

from common.config import DEFAULT
from common.schema import read_run

NOT_MEANINGFUL = "not meaningful"


def _first_fall_time(t: np.ndarray, a: np.ndarray, level: float) -> float | None:
    """Time at which a (= |e|) first falls to `level`, linearly interpolated between samples."""
    below = np.flatnonzero(a <= level)
    if below.size == 0:
        return None
    i = int(below[0])
    if i == 0:
        return float(t[0])
    t0, t1, a0, a1 = t[i - 1], t[i], a[i - 1], a[i]
    frac = (a0 - level) / (a0 - a1)  # a0 > level >= a1, so a0 > a1
    return float(t0 + frac * (t1 - t0))


def step_metrics(t, e, e0: float, band: float, min_hold: float = 0.5) -> dict:
    """Transient metrics for a lateral-offset step response.

    rise_time      time between |e| first falling to 90 % and to 10 % of |e0|; None if it never reaches 10 %.
    overshoot_pct  largest excursion past the line on the side opposite e0, as % of |e0|; 0.0 if it never crosses.
    settling_time  time from which |e| <= band holds until the last sample; None if the last sample is
                   outside, or if the run stays inside for less than `min_hold` seconds before it ends
                   (a "remains within" of one sample would let an oscillation look settled).
    iae            trapezoidal integral of |e| over t.
    """
    t = np.asarray(t, dtype=float)
    e = np.asarray(e, dtype=float)
    a = np.abs(e)
    amp = abs(float(e0))

    rise_time = overshoot_pct = None
    if amp > 0.0:
        t90 = _first_fall_time(t, a, 0.9 * amp)
        t10 = _first_fall_time(t, a, 0.1 * amp)
        if t90 is not None and t10 is not None:
            rise_time = t10 - t90
        overshoot_pct = 100.0 * max(float(np.max(-math.copysign(1.0, e0) * e)), 0.0) / amp

    settling_time = None
    inside = a <= band
    if inside[-1]:
        outside = np.flatnonzero(~inside)
        settling_time = float(t[0]) if outside.size == 0 else float(t[outside[-1] + 1])
        if t[-1] - settling_time < min_hold:
            settling_time = None

    return {
        "rise_time": rise_time,
        "overshoot_pct": overshoot_pct,
        "settling_time": settling_time,
        "iae": float(np.trapezoid(a, t)),
    }


def curve_metrics(t, e, segment, tail_fraction: float = 0.4) -> dict:
    """Curve-tracking metrics computed on rows where segment == "arc" only.

    steady_state is the signed mean of e over the last `tail_fraction` of the arc's time span. It is
    a tail-window measurement, not a converged equilibrium: if the response is still settling when
    the arc ends (e.g. a slow integrator), it reports where the error was, not where it would go.
    All values are None when there are no arc rows.
    """
    t = np.asarray(t, dtype=float)
    e = np.asarray(e, dtype=float)
    arc = np.asarray(segment) == "arc"
    if not arc.any():
        return {"mean_abs": None, "max_abs": None, "steady_state": None, "iae": None}

    ta, ea = t[arc], e[arc]
    tail_start = ta[-1] - tail_fraction * (ta[-1] - ta[0])
    tail = ta >= tail_start
    return {
        "mean_abs": float(np.mean(np.abs(ea))),
        "max_abs": float(np.max(np.abs(ea))),
        "steady_state": float(np.mean(ea[tail])),
        "iae": float(np.trapezoid(np.abs(ea), ta)),
    }


def summarize(run_paths: dict[str, Path], kind: str) -> pd.DataFrame:
    """One row per controller. kind "straight" -> step_metrics, kind "curve" -> curve_metrics.

    Adds `line_lost_frac`: fraction of samples with line_detected == 0.
    """
    if kind not in ("straight", "curve"):
        raise ValueError(f"unknown kind {kind!r}; expected 'straight' or 'curve'")
    rows = []
    for controller, path in run_paths.items():
        df = read_run(path)
        t = df["t"].to_numpy()
        e = df["e_true"].to_numpy()
        if kind == "straight":
            m = step_metrics(t, e, e0=float(e[0]), band=DEFAULT.settle_band)
        else:
            m = curve_metrics(t, e, df["segment"].to_numpy())
        lost = float(np.mean(df["line_detected"].to_numpy() == 0))
        rows.append({"controller": controller, **m, "line_lost_frac": lost})
    return pd.DataFrame(rows)


def _cell(v) -> str:
    if v is None:
        return NOT_MEANINGFUL
    if isinstance(v, (float, np.floating)):
        if math.isnan(v):
            return NOT_MEANINGFUL
        return f"{v:.4g}"
    return str(v)


def to_markdown(df: pd.DataFrame) -> str:
    """Plain pipe table (no `tabulate` dependency); None/NaN are written as "not meaningful"."""
    cols = [str(c) for c in df.columns]
    lines = [
        "| " + " | ".join(cols) + " |",
        "| " + " | ".join("---" for _ in cols) + " |",
    ]
    for row in df.itertuples(index=False):
        lines.append("| " + " | ".join(_cell(v) for v in row) + " |")
    return "\n".join(lines) + "\n"
