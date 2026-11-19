"""Independent checks of analysis/metrics.py against closed-form responses.

Each test feeds the metric functions a signal whose rise time, overshoot, settling time
or IAE is known exactly from mathematics, so the expected value does not come from the
code under test.
"""
import math

import numpy as np
import pandas as pd
import pytest

from analysis.metrics import curve_metrics, step_metrics, summarize, to_markdown
from common.schema import write_run

E0 = 0.05
BAND = 0.0025  # 2.5 mm, the project's settle band


def exponential(t, rate=2.0):
    """Non-oscillating decay: e = E0 * exp(-rate * t)."""
    return E0 * np.exp(-rate * t)


def damped_cosine(t):
    """Under-damped decay: e = E0 * exp(-t) * cos(2t)."""
    return E0 * np.exp(-t) * np.cos(2.0 * t)


def test_rise_time_of_exponential():
    t = np.arange(0.0, 5.0, 1e-3)
    m = step_metrics(t, exponential(t), E0, BAND)
    # |e| = 0.9*E0 at t = ln(1/0.9)/2 and 0.1*E0 at ln(10)/2, so the rise time is ln(9)/2
    assert m["rise_time"] == pytest.approx(math.log(9.0) / 2.0, abs=2e-3)


def test_exponential_has_no_overshoot():
    t = np.arange(0.0, 5.0, 1e-3)
    assert step_metrics(t, exponential(t), E0, BAND)["overshoot_pct"] == 0.0


def test_settling_time_of_exponential():
    t = np.arange(0.0, 5.0, 1e-3)
    m = step_metrics(t, exponential(t), E0, BAND)
    # |e| = band when exp(-2t) = band/E0, i.e. t = ln(E0/band)/2
    assert m["settling_time"] == pytest.approx(math.log(E0 / BAND) / 2.0, abs=2e-3)


def test_iae_of_exponential():
    t = np.arange(0.0, 10.0, 1e-3)
    m = step_metrics(t, exponential(t), E0, BAND)
    assert m["iae"] == pytest.approx(E0 / 2.0, rel=0.01)  # integral of E0*exp(-2t) is E0/2


def test_overshoot_of_damped_cosine():
    t = np.arange(0.0, 10.0, 1e-3)
    m = step_metrics(t, damped_cosine(t), E0, BAND)
    # opposite-side peak is where tan(2t) = -1/2, i.e. t* = (pi - atan(0.5)) / 2
    t_star = (math.pi - math.atan(0.5)) / 2.0
    expected = -100.0 * math.exp(-t_star) * math.cos(2.0 * t_star)
    assert m["overshoot_pct"] == pytest.approx(expected, abs=0.1)


def test_overshoot_respects_sign_of_e0():
    t = np.arange(0.0, 10.0, 1e-3)
    e = damped_cosine(t)
    positive = step_metrics(t, e, E0, BAND)
    negative = step_metrics(t, -e, -E0, BAND)
    assert negative["overshoot_pct"] == pytest.approx(positive["overshoot_pct"])
    assert negative["rise_time"] == pytest.approx(positive["rise_time"])


def test_never_reaching_ten_percent_gives_no_rise_time():
    t = np.arange(0.0, 2.0, 1e-3)
    e = np.full_like(t, E0)  # never moves
    m = step_metrics(t, e, E0, BAND)
    assert m["rise_time"] is None
    assert m["settling_time"] is None


def test_zero_initial_offset_has_no_rise_or_overshoot():
    t = np.arange(0.0, 2.0, 1e-3)
    m = step_metrics(t, np.zeros_like(t), 0.0, BAND)
    assert m["rise_time"] is None
    assert m["overshoot_pct"] is None


def test_settling_requires_last_sample_inside_band():
    t = np.arange(0.0, 5.0, 1e-3)
    e = exponential(t)
    e[-1] = 0.01  # last sample jumps outside the band
    assert step_metrics(t, e, E0, BAND)["settling_time"] is None


def test_short_final_hold_is_not_settled():
    t = np.arange(0.0, 5.0, 1e-3)
    e = np.full_like(t, 0.01)
    e[t >= 4.8] = 0.0  # inside the band for only 0.2 s before the run ends
    assert step_metrics(t, e, E0, BAND, min_hold=0.5)["settling_time"] is None
    assert step_metrics(t, e, E0, BAND, min_hold=0.1)["settling_time"] == pytest.approx(4.8, abs=2e-3)


def test_curve_metrics_use_arc_rows_only():
    t = np.arange(0.0, 10.0, 0.01)
    segment = np.where(t < 4.0, "straight", "arc")
    e = np.where(t < 4.0, 0.5, 0.01)  # huge error on the straight, constant 1 cm on the arc
    m = curve_metrics(t, e, segment)
    assert m["mean_abs"] == pytest.approx(0.01)
    assert m["max_abs"] == pytest.approx(0.01)
    assert m["steady_state"] == pytest.approx(0.01)


def test_curve_steady_state_is_tail_window_mean():
    t = np.linspace(0.0, 10.0, 1001)
    segment = np.full(t.shape, "arc")
    e = np.where(t < 6.0, 0.02, 0.004)  # last 40 % of 10 s starts at t = 6
    assert curve_metrics(t, e, segment)["steady_state"] == pytest.approx(0.004)


def test_curve_metrics_without_arc_are_none():
    t = np.arange(0.0, 1.0, 0.01)
    m = curve_metrics(t, np.zeros_like(t), np.full(t.shape, "straight"))
    assert m == {"mean_abs": None, "max_abs": None, "steady_state": None, "iae": None}


def _write_constant_run(path, mode, e_true, segment, line_detected):
    rows = []
    for i in range(200):
        rows.append({"t": i * 0.01, "x": 0.0, "y": 0.0, "yaw": 0.0, "e_true": e_true,
                     "e_sensed": e_true, "u": 0.0, "controller": mode,
                     "line_detected": line_detected[i], "segment": segment})
    write_run(path, rows)


def test_summarize_curve_and_line_lost_fraction(tmp_path):
    detected = [1] * 150 + [0] * 50  # the line is lost in 50 of the 200 samples
    _write_constant_run(tmp_path / "a.csv", "P", 0.01, "arc", detected)
    df = summarize({"P": tmp_path / "a.csv"}, "curve")
    assert list(df["controller"]) == ["P"]
    assert df["mean_abs"].iloc[0] == pytest.approx(0.01)
    assert df["line_lost_frac"].iloc[0] == pytest.approx(0.25)


def test_summarize_straight_constant_run_has_no_overshoot_and_no_line_loss(tmp_path):
    _write_constant_run(tmp_path / "s.csv", "PI", 0.001, "straight", [1] * 200)
    df = summarize({"PI": tmp_path / "s.csv"}, "straight")
    assert df["overshoot_pct"].iloc[0] == 0.0
    assert df["line_lost_frac"].iloc[0] == 0.0


def test_summarize_rejects_unknown_kind(tmp_path):
    with pytest.raises(ValueError):
        summarize({}, "zigzag")


def test_markdown_marks_missing_values_not_meaningful():
    df = pd.DataFrame([{"controller": "P", "rise_time": None, "iae": 0.05402}])
    text = to_markdown(df)
    assert "| controller | rise_time | iae |" in text
    assert "| P | not meaningful | 0.05402 |" in text
