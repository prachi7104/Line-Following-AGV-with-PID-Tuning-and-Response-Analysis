"""PyBullet integration tests for simulation.world / simulation.robot (DIRECT mode)."""
import math

import pybullet as p  # required: a missing pybullet must fail these tests, not skip them
import pytest

from common.config import DEFAULT  # noqa: E402
from simulation.track import CurveTrack, StraightTrack  # noqa: E402
from simulation.world import Simulation  # noqa: E402


@pytest.fixture
def sim_offset():
    sim = Simulation(StraightTrack(), start_offset=0.05)
    yield sim
    sim.close()


@pytest.fixture
def sim_on_line():
    sim = Simulation(StraightTrack(), start_offset=0.0)
    yield sim
    sim.close()


def _run(sim, seconds):
    for _ in range(int(round(seconds / DEFAULT.dt))):
        sim.step()


def test_spawn_pose(sim_offset):
    for _ in range(10):
        sim_offset.step()
    x, y, yaw = sim_offset.robot.pose()
    assert x == pytest.approx(0.0, abs=0.005)
    assert y == pytest.approx(0.05, abs=0.005)
    assert yaw == pytest.approx(0.0, abs=0.01)


def test_drives_straight(sim_on_line):
    sim_on_line.robot.set_command(0.3, 0.0)
    _run(sim_on_line, 2.0)
    x, y, yaw = sim_on_line.robot.pose()
    assert x == pytest.approx(0.6, abs=0.06)
    assert abs(yaw) < 0.05


def test_turns_left(sim_on_line):
    _, _, yaw0 = sim_on_line.robot.pose()
    sim_on_line.robot.set_command(0.3, 1.0)
    _run(sim_on_line, 0.5)
    _, _, yaw1 = sim_on_line.robot.pose()
    assert yaw1 > yaw0 + 0.2


def _down_ray_hit(sim, x, y):
    hits = p.rayTestBatch([(x, y, DEFAULT.ray_z_top)], [(x, y, DEFAULT.ray_z_bottom)],
                          physicsClientId=sim.client)
    return hits[0][0]


def test_rays_see_line_under_robot(sim_on_line):
    for _ in range(10):
        sim_on_line.step()
    x, y, yaw = sim_on_line.robot.pose()
    sx = x + DEFAULT.d * math.cos(yaw)
    sy = y + DEFAULT.d * math.sin(yaw)
    assert _down_ray_hit(sim_on_line, sx, sy) == sim_on_line.line_id
    assert _down_ray_hit(sim_on_line, sx, sy + 0.2) != sim_on_line.line_id
    assert _down_ray_hit(sim_on_line, sx, sy - 0.2) != sim_on_line.line_id


def test_rays_see_line_on_arc():
    sim = Simulation(CurveTrack())
    try:
        t = sim.track
        phi = math.pi / 3
        x = t.lead + t.radius * math.sin(phi)
        y = t.radius - t.radius * math.cos(phi)
        assert _down_ray_hit(sim, x, y) == sim.line_id
        assert _down_ray_hit(sim, t.lead, t.radius) != sim.line_id  # arc centre
    finally:
        sim.close()


def test_no_bump(sim_offset):
    # Start 5 cm left of the line and turn right so the whole robot drives across it.
    rob = sim_offset.robot
    rob.set_command(0.3, -1.0)
    crossed = False
    for _ in range(int(round(1.5 / DEFAULT.dt))):
        sim_offset.step()
        x, y, _ = rob.pose()
        crossed |= y < -0.05
        _, orn = p.getBasePositionAndOrientation(rob.body_id, physicsClientId=sim_offset.client)
        roll, pitch, _ = p.getEulerFromQuaternion(orn)
        assert abs(roll) < 0.02
        assert abs(pitch) < 0.02
        assert p.getContactPoints(rob.body_id, sim_offset.line_id,
                                  physicsClientId=sim_offset.client) == ()
    assert crossed, "robot never crossed the line, test did not exercise anything"


def test_reset(sim_on_line):
    rob = sim_on_line.robot
    rob.set_command(0.3, 0.5)
    _run(sim_on_line, 0.5)
    rob.reset((0.2, -0.03, 0.1))
    rob.set_command(0.0, 0.0)
    for _ in range(5):
        sim_on_line.step()
    x, y, yaw = rob.pose()
    assert (x, y, yaw) == pytest.approx((0.2, -0.03, 0.1), abs=0.005)
