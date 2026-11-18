"""End-to-end checks: simulator + sensors + controller + logging together.

Every controller (P, PI, PID) runs on both tracks exactly as `experiments.run_experiments` runs
it (full length, default timeout). Each of the six runs takes about 1 s and is shared by the tests
through a module-scoped cache.

pybullet is imported directly, not with importorskip: if it is missing these tests must fail,
not be skipped silently.
"""
import math

import numpy as np
import pybullet  # noqa: F401  (hard requirement, see module docstring)
import pytest

from common.config import DEFAULT
from common.schema import COLUMNS
from experiments.run_experiments import MODES, run_trial
from simulation.track import CurveTrack, StraightTrack

CASES = [(track, mode) for track in ("straight", "curve") for mode in MODES]
_RUNS: dict = {}


def _track(kind):
    if kind == "straight":
        return StraightTrack(DEFAULT.straight_length), DEFAULT.initial_offset
    return CurveTrack(DEFAULT.curve_lead, DEFAULT.curve_radius, DEFAULT.curve_angle), 0.0


def _run(kind, mode):
    if (kind, mode) not in _RUNS:
        track, offset = _track(kind)
        _RUNS[(kind, mode)] = run_trial(track, mode, DEFAULT, offset)
    return _RUNS[(kind, mode)]


@pytest.fixture(params=CASES, ids=[f"{k}-{m}" for k, m in CASES])
def case(request):
    kind, mode = request.param
    return kind, mode, _run(kind, mode)


def test_schema_and_labels(case):
    kind, mode, df = case
    assert list(df.columns) == COLUMNS
    assert set(df["controller"]) == {mode}
    assert not df[["t", "x", "y", "yaw", "e_true", "e_sensed", "u"]].isna().any().any()
    assert np.all(np.diff(df["t"]) > 0)
    expected_segments = {"straight"} if kind == "straight" else {"straight", "arc"}
    assert set(df["segment"]) == expected_segments


def test_initial_conditions(case):
    kind, _, df = case
    expected = DEFAULT.initial_offset if kind == "straight" else 0.0
    first = df.iloc[0]
    assert first["t"] == 0.0
    assert first["x"] == pytest.approx(0.0, abs=1e-3)
    assert first["e_true"] == pytest.approx(expected, abs=0.002)
    assert first["yaw"] == pytest.approx(0.0, abs=0.01)
    # the controller is initialised with the first reading, so it sees the true offset at once
    assert first["e_sensed"] == pytest.approx(expected, abs=0.002)


def test_track_completed(case):
    """The run ends because the axle reached the end of the track, not because of the timeout."""
    kind, _, df = case
    track, _ = _track(kind)
    last = df.iloc[-1]
    assert last["t"] < DEFAULT.max_time - 0.5
    step = DEFAULT.v * DEFAULT.dt
    if kind == "straight":
        assert last["x"] >= track.length - 3 * step
    else:
        end = (track.lead + track.radius * math.sin(track.angle),
               track.radius - track.radius * math.cos(track.angle))
        assert math.hypot(last["x"] - end[0], last["y"] - end[1]) < 0.03
    # nominal travel time = path length / v
    length = track.length if kind == "straight" else track.lead + track.radius * track.angle
    assert last["t"] == pytest.approx(length / DEFAULT.v, rel=0.03)


def test_line_always_detected(case):
    _, _, df = case
    assert df["line_detected"].all()


def test_tracking_error_bounded(case):
    kind, _, df = case
    e = df["e_true"].to_numpy()
    u = df["u"].to_numpy()
    assert np.all(np.abs(u) <= DEFAULT.omega_max + 1e-12)
    if kind == "straight":
        # never further from the line than the starting offset, and settled at the end
        assert np.max(np.abs(e)) <= DEFAULT.initial_offset + 0.002
        assert abs(e[-1]) <= DEFAULT.settle_band
    else:
        # never more than 2.5 cm off on the curve (sensor bar half-width is 7.5 cm)
        assert np.max(np.abs(e)) < 0.025


@pytest.mark.parametrize("mode", MODES)
def test_offset_left_turns_right(mode):
    # robot starts 5 cm LEFT of the line, so it must yaw RIGHT (negative) early on
    df = _run("straight", mode)
    early = df[df["t"] <= 0.5]
    assert early["u"].iloc[0] < 0
    assert early["yaw"].iloc[-1] < 0


def test_curve_p_offset_outside_pi_pid_reduce_it():
    """P keeps an offset on the outside of the left turn (e < 0); PI and PID shrink it."""
    tail = {}
    for mode in MODES:
        df = _run("curve", mode)
        arc = df[df["segment"] == "arc"]
        t = arc["t"].to_numpy()
        tail[mode] = arc["e_true"][t >= t[-1] - 0.4 * (t[-1] - t[0])].mean()
    assert tail["P"] < -0.01
    assert abs(tail["PI"]) < abs(tail["P"]) / 3
    assert abs(tail["PID"]) < abs(tail["P"]) / 3
