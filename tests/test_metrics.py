"""Tests for analysis.metrics, analysis.dummy_data and analysis.plot_results (Task 4)."""
import math

import numpy as np
import pandas as pd
import pytest
from scipy.integrate import quad
from scipy.optimize import brentq

from analysis.dummy_data import make_dummy_run
from analysis.metrics import curve_metrics, step_metrics, summarize, to_markdown
from analysis.plot_results import plot_error_vs_time
from common.config import DEFAULT
from common.schema import COLUMNS, write_run

E0 = 0.05
BAND = DEFAULT.settle_band


def _analytic(t):
    return E0 * math.exp(-t) * math.cos(2.0 * t)


def test_metrics_known_response():
    t = np.arange(0.0, 10.0 + 1e-9, 1e-3)  # 1 kHz
    e = E0 * np.exp(-t) * np.cos(2.0 * t)
    m = step_metrics(t, e, E0, BAND)

    # e is positive and decreasing on [0, pi/4], so both crossings live there.
    t90 = brentq(lambda s: _analytic(s) - 0.9 * E0, 0.0, math.pi / 4)
    t10 = brentq(lambda s: _analytic(s) - 0.1 * E0, 0.0, math.pi / 4)
    assert m["rise_time"] == pytest.approx(t10 - t90, abs=2e-3)

    dense = np.linspace(0.0, 10.0, 2_000_001)
    expected_os = 100.0 * np.max(-E0 * np.exp(-dense) * np.cos(2.0 * dense)) / E0
    assert m["overshoot_pct"] == pytest.approx(expected_os, abs=0.5)

    zeros = [math.pi / 4 + k * math.pi / 2 for k in range(7)]
    iae, _ = quad(lambda s: abs(_analytic(s)), 0.0, 10.0, points=zeros, limit=200)
    assert m["iae"] == pytest.approx(iae, rel=0.01)

    # The envelope 0.05 e^-t drops below 2.5 mm at ln(20) ~ 3.0 s, so it settles before that.
    assert m["settling_time"] is not None
    assert 1.0 < m["settling_time"] < math.log(E0 / BAND)
    assert np.all(np.abs(e[t >= m["settling_time"]]) <= BAND)


def test_no_overshoot_is_zero():
    t = np.arange(0.0, 5.0, 1e-3)
    e = E0 * np.exp(-2.0 * t)
    m = step_metrics(t, e, E0, BAND)
    assert m["overshoot_pct"] == 0.0
    assert m["rise_time"] == pytest.approx(math.log(9.0) / 2.0, abs=2e-3)


def test_overshoot_respects_sign_of_e0():
    t = np.arange(0.0, 10.0, 1e-3)
    e = -E0 * np.exp(-t) * np.cos(2.0 * t)
    m_neg = step_metrics(t, e, -E0, BAND)
    m_pos = step_metrics(t, -e, E0, BAND)
    assert m_neg["overshoot_pct"] == pytest.approx(m_pos["overshoot_pct"])
    assert m_neg["overshoot_pct"] > 20.0


def test_never_settles_returns_none():
    t = np.arange(0.0, 10.0 + 1e-9, 1e-3)
    e = 0.01 * np.cos(2.0 * np.pi * t)  # sustained 1 cm oscillation, ends outside the band
    m = step_metrics(t, e, E0, BAND)
    assert m["settling_time"] is None


def test_oscillation_ending_inside_band_is_not_settled():
    # sustained 1 cm oscillation whose LAST samples happen to pass through zero
    t = np.arange(0.0, 10.25 + 1e-9, 1e-3)
    e = 0.01 * np.cos(2.0 * np.pi * t)  # cos(2*pi*10.25) = 0 -> final sample inside the band
    assert abs(e[-1]) <= BAND
    assert step_metrics(t, e, E0, BAND)["settling_time"] is None


def test_settled_from_start_returns_t0():
    t = np.arange(0.5, 2.0, 1e-3)
    e = np.full_like(t, 0.001)
    assert step_metrics(t, e, E0, BAND)["settling_time"] == pytest.approx(0.5)


def test_never_reaches_10pct_rise_none():
    t = np.arange(0.0, 10.0, 1e-3)
    e = E0 * (0.5 + 0.5 * np.exp(-t))  # floors at 50 % of e0
    m = step_metrics(t, e, E0, BAND)
    assert m["rise_time"] is None
    assert m["settling_time"] is None
    assert m["overshoot_pct"] == 0.0


def test_curve_steady_state():
    dt = 1e-3
    t_straight = np.arange(0.0, 1.0, dt)
    t_arc = np.arange(1.0, 9.0, dt)
    tau = t_arc - 1.0
    e_arc = 0.02 * (1.0 - np.exp(-5.0 * tau))  # constant 0.02 well before the last 40 %
    t = np.concatenate([t_straight, t_arc])
    e = np.concatenate([np.full_like(t_straight, 0.5), e_arc])  # straight rows are huge, must be ignored
    seg = np.array(["straight"] * len(t_straight) + ["arc"] * len(t_arc))

    m = curve_metrics(t, e, seg)
    assert m["steady_state"] == pytest.approx(0.02, abs=1e-4)
    assert m["max_abs"] == pytest.approx(0.02, abs=1e-4)
    assert m["mean_abs"] < 0.02
    assert m["iae"] == pytest.approx(np.trapezoid(np.abs(e_arc), t_arc))


def test_curve_without_arc_rows_is_none():
    t = np.arange(0.0, 1.0, 1e-3)
    m = curve_metrics(t, np.zeros_like(t), np.array(["straight"] * len(t)))
    assert m == {"mean_abs": None, "max_abs": None, "steady_state": None, "iae": None}


def test_markdown_not_meaningful():
    df = pd.DataFrame(
        {"controller": ["P", "PI"], "rise_time": [0.42, None], "settling_time": [float("nan"), 3.1]}
    )
    md = to_markdown(df)
    lines = md.strip().splitlines()
    assert lines[0].startswith("|") and "controller" in lines[0] and "rise_time" in lines[0]
    assert set(lines[1].replace("|", "").strip()) <= {"-", " ", ":"}
    assert len(lines) == 4
    assert md.count("not meaningful") == 2
    assert "nan" not in md.lower().replace("not meaningful", "")
    assert "None" not in md


def test_dummy_run_schema():
    for kind in ("straight", "curve"):
        df = make_dummy_run("PID", kind=kind)
        assert list(df.columns) == COLUMNS
        assert (df["controller"] == "PID").all()
    curve_p = make_dummy_run("P", kind="curve")
    assert set(curve_p["segment"]) == {"straight", "arc"}


def test_summarize_from_csv(tmp_path):
    paths = {}
    for mode in ("P", "PI", "PID"):
        p = tmp_path / f"straight_{mode}.csv"
        write_run(p, make_dummy_run(mode, kind="straight").to_dict("records"))
        paths[mode] = p
    df = summarize(paths, "straight")
    assert list(df["controller"]) == ["P", "PI", "PID"]
    for col in ("rise_time", "overshoot_pct", "settling_time", "iae", "line_lost_frac"):
        assert col in df.columns
    assert ((df["line_lost_frac"] >= 0) & (df["line_lost_frac"] <= 1)).all()

    curve_paths = {}
    for mode in ("P", "PID"):
        p = tmp_path / f"curve_{mode}.csv"
        write_run(p, make_dummy_run(mode, kind="curve").to_dict("records"))
        curve_paths[mode] = p
    dfc = summarize(curve_paths, "curve")
    p_row = dfc.set_index("controller").loc["P"]
    pid_row = dfc.set_index("controller").loc["PID"]
    assert abs(p_row["steady_state"]) > abs(pid_row["steady_state"])


def test_plot_error_vs_time_writes_png(tmp_path):
    runs = {m: make_dummy_run(m, kind="straight") for m in ("P", "PI", "PID")}
    out = plot_error_vs_time(runs, "dummy straight", tmp_path / "err.png", band=BAND)
    assert out.exists()
    assert out.stat().st_size > 0
    assert out.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
