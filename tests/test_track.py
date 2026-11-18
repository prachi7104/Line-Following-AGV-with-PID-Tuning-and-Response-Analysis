"""Pure-geometry tests for simulation.track (no pybullet needed)."""
import math

import pytest

from simulation.track import CurveTrack, StraightTrack


def test_straight_sign():
    t = StraightTrack()
    assert t.lateral_error(1.0, 0.05) == pytest.approx(0.05)
    assert t.lateral_error(1.0, -0.05) == pytest.approx(-0.05)


def test_curve_sign_inside_positive():
    t = CurveTrack()
    # 45 degrees into the arc, 5 cm towards the centre (inside of the left turn).
    phi = math.pi / 4
    r_in = t.radius - 0.05
    x = t.lead + r_in * math.sin(phi)
    y = t.radius - r_in * math.cos(phi)
    assert t.lateral_error(x, y) == pytest.approx(0.05)
    # and 5 cm outside is negative
    r_out = t.radius + 0.05
    x = t.lead + r_out * math.sin(phi)
    y = t.radius - r_out * math.cos(phi)
    assert t.lateral_error(x, y) == pytest.approx(-0.05)


def test_curve_lead_in_sign():
    t = CurveTrack()
    assert t.lateral_error(0.5, 0.05) == pytest.approx(0.05)
    assert t.lateral_error(0.5, -0.05) == pytest.approx(-0.05)


def test_curve_segments():
    t = CurveTrack()
    assert t.segment(0.5, 0) == "straight"
    assert t.segment(1.75, 0.75) == "arc"


def test_straight_segment_and_end():
    t = StraightTrack()
    assert t.segment(2.0, 0.0) == "straight"
    assert not t.end_reached(3.9, 0.0)
    assert t.end_reached(4.0, 0.0)


def test_curve_end():
    t = CurveTrack()
    assert t.end_reached(1.0, 1.5)
    assert not t.end_reached(1.75, 0.75)   # half way round
    assert not t.end_reached(0.5, 0.0)     # still on the lead-in


def test_start_pose_offset():
    assert StraightTrack().start_pose(0.05) == pytest.approx((0, 0.05, 0))
    assert CurveTrack().start_pose(0.05) == pytest.approx((0, 0.05, 0))


@pytest.mark.parametrize("track", [StraightTrack(), CurveTrack()], ids=lambda t: t.name)
def test_polyline_on_line(track):
    pts = track.polyline()
    assert pts.ndim == 2 and pts.shape[1] == 2 and len(pts) > 10
    for x, y in pts:
        assert abs(track.lateral_error(x, y)) < 1e-6


def test_polyline_endpoints():
    s = StraightTrack().polyline()
    assert s[0] == pytest.approx([0, 0])
    assert s[-1] == pytest.approx([4.0, 0])
    c = CurveTrack().polyline()
    assert c[0] == pytest.approx([0, 0])
    assert c[-1] == pytest.approx([1.0, 1.5], abs=1e-9)


def test_names_differ():
    assert StraightTrack().name != CurveTrack().name


def test_curve_angle_beyond_three_quarter_turn_rejected():
    # past 1.5*pi the arc overlaps the lead-in region test, so the geometry would be ambiguous
    with pytest.raises(ValueError):
        CurveTrack(1.0, 0.75, 1.75 * math.pi)
