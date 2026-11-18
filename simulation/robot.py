"""Differential-drive AGV wrapper around the PyBullet body loaded from assets/agv.urdf."""
from __future__ import annotations

from pathlib import Path

import pybullet as p

from common.config import DEFAULT, SimConfig

URDF_PATH = Path(__file__).resolve().parent / "assets" / "agv.urdf"
LEFT_WHEEL_JOINT = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"
CASTER_LINKS = ("front_caster", "rear_caster")
WHEEL_LATERAL_FRICTION = 1.0
GEOMETRY_TOL = 1e-6   # URDF vs SimConfig consistency check [m]


class AGV:
    """Unicycle-commanded differential-drive robot.

    pose() reports the axle midpoint (the URDF base frame) as (x, y, yaw).
    set_command(v, omega) converts a forward speed and yaw rate into wheel speeds.
    """

    def __init__(self, client: int, cfg: SimConfig = DEFAULT,
                 pose: tuple[float, float, float] = (0.0, 0.0, 0.0)):
        self.client = client
        self.cfg = cfg
        self.body_id = p.loadURDF(str(URDF_PATH), basePosition=self._base_xyz(pose),
                                  baseOrientation=p.getQuaternionFromEuler([0, 0, pose[2]]),
                                  flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_IMPLICIT_CYLINDER,
                                  physicsClientId=client)
        joints, links = {}, {}
        for j in range(p.getNumJoints(self.body_id, physicsClientId=client)):
            info = p.getJointInfo(self.body_id, j, physicsClientId=client)
            joints[info[1].decode()] = j
            links[info[12].decode()] = j
        self.left_joint = joints[LEFT_WHEEL_JOINT]
        self.right_joint = joints[RIGHT_WHEEL_JOINT]
        self.link_indices = [-1] + sorted(joints.values())
        self._check_geometry()

        for j in (self.left_joint, self.right_joint):
            p.changeDynamics(self.body_id, j, lateralFriction=WHEEL_LATERAL_FRICTION,
                             physicsClientId=client)
        for name in CASTER_LINKS:
            p.changeDynamics(self.body_id, links[name], lateralFriction=0.0,
                             spinningFriction=0.0, rollingFriction=0.0, physicsClientId=client)
        self.set_command(0.0, 0.0)

    # --- geometry ---
    def _base_xyz(self, pose):
        return [float(pose[0]), float(pose[1]), self.cfg.wheel_radius]

    def _check_geometry(self) -> None:
        """Fail loudly if the URDF drifted away from common/config.py."""
        # Read the radius from the visual cylinder (same numbers as the collision cylinder in
        # the URDF; collision data is a mesh unless URDF_USE_IMPLICIT_CYLINDER): (length, r, 0).
        visual = {v[1]: v for v in p.getVisualShapeData(self.body_id, physicsClientId=self.client)}
        for j, sign in ((self.left_joint, 1.0), (self.right_joint, -1.0)):
            wheel_y = p.getJointInfo(self.body_id, j, physicsClientId=self.client)[14][1]
            radius = visual[j][3][1]
            if abs(wheel_y - sign * self.cfg.track_width / 2) > GEOMETRY_TOL or \
                    abs(radius - self.cfg.wheel_radius) > GEOMETRY_TOL:
                raise ValueError(
                    f"agv.urdf wheel geometry (y={wheel_y}, r={radius}) does not match "
                    f"SimConfig (track_width/2={self.cfg.track_width / 2}, "
                    f"wheel_radius={self.cfg.wheel_radius})")

    # --- interface ---
    def pose(self) -> tuple[float, float, float]:
        """(x, y, yaw) of the axle midpoint in the world frame."""
        pos, orn = p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.client)
        # PyBullet reports the base inertial frame; map it back to the URDF link frame.
        local_pos, local_orn = p.getDynamicsInfo(self.body_id, -1,
                                                 physicsClientId=self.client)[3:5]
        inv_pos, inv_orn = p.invertTransform(local_pos, local_orn)
        link_pos, link_orn = p.multiplyTransforms(pos, orn, inv_pos, inv_orn)
        yaw = p.getEulerFromQuaternion(link_orn)[2]
        return float(link_pos[0]), float(link_pos[1]), float(yaw)

    def set_command(self, v: float, omega: float) -> None:
        """Unicycle command: forward speed v [m/s], yaw rate omega [rad/s], left positive."""
        half_w = self.cfg.track_width / 2.0
        r = self.cfg.wheel_radius
        omega_right = (v + omega * half_w) / r
        omega_left = (v - omega * half_w) / r
        p.setJointMotorControlArray(self.body_id, [self.left_joint, self.right_joint],
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=[omega_left, omega_right],
                                    forces=[self.cfg.wheel_force, self.cfg.wheel_force],
                                    physicsClientId=self.client)

    def reset(self, pose: tuple[float, float, float]) -> None:
        """Teleport the axle midpoint to pose = (x, y, yaw), at rest, wheels stopped."""
        local_pos, local_orn = p.getDynamicsInfo(self.body_id, -1,
                                                 physicsClientId=self.client)[3:5]
        link_orn = p.getQuaternionFromEuler([0.0, 0.0, float(pose[2])])
        com_pos, com_orn = p.multiplyTransforms(self._base_xyz(pose), link_orn,
                                                local_pos, local_orn)
        p.resetBasePositionAndOrientation(self.body_id, com_pos, com_orn,
                                          physicsClientId=self.client)
        p.resetBaseVelocity(self.body_id, [0, 0, 0], [0, 0, 0], physicsClientId=self.client)
        for j in (self.left_joint, self.right_joint):
            p.resetJointState(self.body_id, j, 0.0, 0.0, physicsClientId=self.client)
        self.set_command(0.0, 0.0)
