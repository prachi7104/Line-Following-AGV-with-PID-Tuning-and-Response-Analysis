"""Verification tests for the robot simulation module (Member 1 contribution).

Covers:
  - Pure yaw rotation (zero forward speed) for both positive and negative yaw rates,
    verifying differential wheel velocities and yaw direction without linear translation.
  - Multi-world isolation: verifying that multiple PyBullet DIRECT simulation instances
    run independently and that closing one client does not disrupt another.
  - Geometry consistency validation: verifying that AGV initialization rejects SimConfig
    parameters that mismatch the frozen agv.urdf physical dimensions.
"""
from __future__ import annotations

import math
import pybullet as p
import pytest

from common.config import DEFAULT, SimConfig
from simulation.robot import AGV
from simulation.track import StraightTrack
from simulation.world import Simulation


def _step_seconds(sim: Simulation, seconds: float) -> None:
    for _ in range(int(round(seconds / DEFAULT.dt))):
        sim.step()


@pytest.mark.parametrize(
    ("omega_cmd", "expected_yaw_sign", "expected_left_w_sign", "expected_right_w_sign"),
    [
        (1.5, +1.0, -1.0, +1.0),   # CCW / Left turn: left wheel reverses, right wheel forwards
        (-1.5, -1.0, +1.0, -1.0),  # CW / Right turn: left wheel forwards, right wheel reverses
    ],
    ids=["pure_yaw_ccw", "pure_yaw_cw"],
)
def test_pure_yaw_commands(omega_cmd: float, expected_yaw_sign: float,
                           expected_left_w_sign: float, expected_right_w_sign: float):
    """Test pure yaw in-place rotation with zero forward speed for both signs.

    Differential kinematics: omega_R = (v + u*W/2)/r, omega_L = (v - u*W/2)/r.
    With v = 0: omega_R = +u*W/(2*r), omega_L = -u*W/(2*r).
    At u = 1.5 rad/s: |omega_wheel| = 1.5 * 0.08 / 0.035 ≈ 3.43 rad/s.
    """
    with Simulation(StraightTrack(), start_offset=0.0) as sim:
        rob = sim.robot
        rob.set_command(0.0, omega_cmd)
        _step_seconds(sim, 0.5)

        x, y, yaw = rob.pose()
        # Axle midpoint should remain nearly stationary during pure yaw rotation
        disp = math.hypot(x, y)
        assert disp < 0.02, f"Axle midpoint moved {disp:.4f} m during pure yaw; expected < 0.02 m"

        # Yaw should rotate in the commanded direction
        assert yaw * expected_yaw_sign > 0.4, (
            f"Yaw {yaw:.4f} rad did not rotate sufficiently in sign direction {expected_yaw_sign}"
        )

        # Inspect wheel joint velocities
        left_state = p.getJointState(rob.body_id, rob.left_joint, physicsClientId=sim.client)
        right_state = p.getJointState(rob.body_id, rob.right_joint, physicsClientId=sim.client)
        w_left = left_state[1]
        w_right = right_state[1]

        # Wheels must spin in opposite directions
        assert w_left * expected_left_w_sign > 1.5, (
            f"Left wheel angular velocity {w_left:.3f} has wrong sign or magnitude"
        )
        assert w_right * expected_right_w_sign > 1.5, (
            f"Right wheel angular velocity {w_right:.3f} has wrong sign or magnitude"
        )
        # Symmetry check between left and right wheel speeds
        assert abs(abs(w_left) - abs(w_right)) < 0.2, (
            f"Asymmetric wheel speeds: left={w_left:.3f}, right={w_right:.3f}"
        )


def test_simultaneous_direct_worlds_isolation():
    """Verify that multiple PyBullet DIRECT worlds do not leak state or disrupt each other."""
    sim1 = Simulation(StraightTrack(), start_offset=0.0)
    sim2 = Simulation(StraightTrack(), start_offset=0.0)
    try:
        assert sim1.client != sim2.client, "Simulations must obtain distinct physics client IDs"

        # Drive sim1 forward while sim2 stays commanded at rest; step both concurrently
        sim1.robot.set_command(0.3, 0.0)
        sim2.robot.set_command(0.0, 0.0)
        for _ in range(int(round(0.8 / DEFAULT.dt))):
            sim1.step()
            sim2.step()

        x1, y1, _ = sim1.robot.pose()
        x2, y2, _ = sim2.robot.pose()

        # sim1 has moved forward, sim2 must remain undisturbed at origin
        assert x1 > 0.20, f"sim1 did not move forward: x1={x1:.4f}"
        assert math.hypot(x2, y2) < 0.005, f"sim2 was disturbed by sim1: x2={x2:.4f}, y2={y2:.4f}"

        # Close sim1 and verify that sim2 remains completely functional
        sim1.close()
        assert sim1.client is None

        sim2.robot.set_command(0.3, 0.0)
        for _ in range(int(round(0.8 / DEFAULT.dt))):
            sim2.step()

        x2_stepped, _, _ = sim2.robot.pose()
        assert x2_stepped > 0.20, (
            f"sim2 failed to advance after sim1 was closed: x2={x2_stepped:.4f}"
        )
    finally:
        sim1.close()
        sim2.close()


@pytest.mark.parametrize(
    "bad_cfg",
    [
        SimConfig(wheel_radius=0.050),   # Mismatched radius (URDF has 0.035 m)
        SimConfig(track_width=0.200),    # Mismatched track width (URDF has 0.160 m)
    ],
    ids=["bad_wheel_radius", "bad_track_width"],
)
def test_geometry_mismatch_rejected(bad_cfg: SimConfig):
    """Verify that AGV initialization validates URDF geometry and fails loudly on mismatch."""
    client = p.connect(p.DIRECT)
    try:
        with pytest.raises(ValueError, match=r"agv\.urdf wheel geometry .* does not match SimConfig"):
            AGV(client, bad_cfg)
    finally:
        if p.isConnected(client):
            p.disconnect(client)
