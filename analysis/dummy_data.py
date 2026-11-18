"""FAKE runs for developing plots and metrics before the simulator exists.

Nothing here comes from physics. Straight runs are damped cosines with a different
damping per controller; curve runs have a 1 s lead-in followed by an arc where the
P error settles to a constant offset and the PI / PID errors decay to zero.
Never report numbers derived from this module as results.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

from common.config import DEFAULT, GAINS
from common.schema import COLUMNS

# (damping ratio, natural frequency [rad/s]) per controller -- arbitrary, for shape only.
_SHAPE = {"P": (0.15, 2.0), "PI": (0.10, 2.0), "PID": (0.60, 2.5)}
_P_CURVE_OFFSET = 0.02  # [m] constant arc offset of the fake P run


def _damped(tau: np.ndarray, zeta: float, wn: float) -> np.ndarray:
    wd = wn * np.sqrt(1.0 - zeta**2)
    return np.exp(-zeta * wn * tau) * np.cos(wd * tau)


def _frame(mode, t, x, y, yaw, e, segment, cfg) -> pd.DataFrame:
    e_sensed = e + cfg.d * np.sin(yaw - _line_heading(segment, t, cfg))
    kp = GAINS[mode][0]
    u = np.clip(-kp * e_sensed, -cfg.omega_max, cfg.omega_max)
    half_span = cfg.sensor_centres()[-1] + cfg.sensor_spacing / 2
    detected = (np.abs(e_sensed) < half_span).astype(int)
    df = pd.DataFrame(
        {
            "t": t, "x": x, "y": y, "yaw": yaw, "e_true": e, "e_sensed": e_sensed, "u": u,
            "controller": mode, "line_detected": detected, "segment": segment,
        }
    )
    return df[COLUMNS]


def _line_heading(segment, t, cfg) -> np.ndarray:
    heading = np.zeros_like(t)
    arc = np.asarray(segment) == "arc"
    heading[arc] = cfg.v * (t[arc] - cfg.curve_lead / cfg.v) / cfg.curve_radius
    return heading


def make_dummy_run(mode: str, t_end: float = 10.0, dt: float = 1 / 240, kind: str = "straight",
                   cfg=DEFAULT) -> pd.DataFrame:
    """Fake run in schema column order. kind is "straight" or "curve"."""
    if mode not in _SHAPE:
        raise ValueError(f"unknown mode {mode!r}")
    zeta, wn = _SHAPE[mode]
    v = cfg.v

    if kind == "straight":
        t = np.arange(0.0, t_end + dt / 2, dt)
        e = cfg.initial_offset * _damped(t, zeta, wn)
        x, y = v * t, e  # straight line along +x, e > 0 is left (+y)
        yaw = np.arctan2(np.gradient(e, t), v)
        return _frame(mode, t, x, y, yaw, e, np.array(["straight"] * len(t)), cfg)

    if kind != "curve":
        raise ValueError(f"unknown kind {kind!r}")

    t_lead = cfg.curve_lead / v                                   # 1 m lead-in at v
    t_arc_end = t_lead + cfg.curve_angle * cfg.curve_radius / v   # arc length / v
    t = np.arange(0.0, min(t_end, t_arc_end) + dt / 2, dt)
    arc = t >= t_lead
    tau = np.where(arc, t - t_lead, 0.0)

    transient = 0.012 * np.exp(-zeta * wn * tau) * np.sin(wn * np.sqrt(1 - zeta**2) * tau)
    if mode == "P":
        base = _P_CURVE_OFFSET * (1.0 - np.exp(-1.5 * tau))
    else:  # integral action removes the offset
        base = 0.025 * (np.exp(-0.6 * tau) - np.exp(-1.5 * tau))
    e = np.where(arc, base + transient, 0.0)

    phi = v * tau / cfg.curve_radius
    r = cfg.curve_radius - e  # e > 0 is inside a left turn
    x = np.where(arc, cfg.curve_lead + r * np.sin(phi), v * t)
    y = np.where(arc, cfg.curve_radius - r * np.cos(phi), e)
    yaw = np.where(arc, phi, 0.0)
    segment = np.where(arc, "arc", "straight")
    return _frame(mode, t, x, y, yaw, e, segment, cfg)


def dummy_track_xy(kind: str, step: float = 0.02, cfg=DEFAULT) -> np.ndarray:
    """Line polyline (N, 2) matching the fake runs, so --dummy needs no simulation import."""
    if kind == "straight":
        x = np.arange(0.0, cfg.straight_length + step / 2, step)
        return np.column_stack([x, np.zeros_like(x)])
    lead = np.arange(0.0, cfg.curve_lead, step)
    phi = np.arange(0.0, cfg.curve_angle + 1e-9, step / cfg.curve_radius)
    arc = np.column_stack([cfg.curve_lead + cfg.curve_radius * np.sin(phi),
                           cfg.curve_radius * (1.0 - np.cos(phi))])
    return np.vstack([np.column_stack([lead, np.zeros_like(lead)]), arc])
