"""PyBullet world: floor, line body and AGV, stepped at cfg.dt."""
from __future__ import annotations

import pybullet as p
import pybullet_data

from common.config import DEFAULT, SimConfig
from simulation.robot import AGV
from simulation.track import spawn_line

GRAVITY = -9.81
SETTLE_STEPS = 20          # zero-command steps after spawn so contacts settle before step 0
# GUI follow camera (visual only, no effect on the physics)
CAMERA_DISTANCE = 0.9
CAMERA_YAW = -90.0         # looking along +x from behind at the start pose
CAMERA_PITCH = -45.0


class Simulation:
    def __init__(self, track, cfg: SimConfig = DEFAULT, gui: bool = False,
                 start_offset: float = 0.0):
        self.track = track
        self.cfg = cfg
        self.gui = gui
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(0, 0, GRAVITY, physicsClientId=self.client)
        p.setTimeStep(cfg.dt, physicsClientId=self.client)
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)

        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client)
        self.line_id = spawn_line(self.client, track, cfg)
        self.start_pose = track.start_pose(start_offset)
        self.robot = AGV(self.client, cfg, self.start_pose)

        # The robot drives over the line, never bumps into it: only rays may see the line.
        # The line body is a base plus fixed links (see spawn_line), so disable every pair.
        line_links = range(-1, p.getNumJoints(self.line_id, physicsClientId=self.client))
        for robot_link in self.robot.link_indices:
            for line_link in line_links:
                p.setCollisionFilterPair(self.robot.body_id, self.line_id, robot_link, line_link,
                                         enableCollision=0, physicsClientId=self.client)

        for _ in range(SETTLE_STEPS):
            p.stepSimulation(physicsClientId=self.client)
        # Settling may nudge the robot by a fraction of a millimetre; put it back exactly.
        self.robot.reset(self.start_pose)
        self._follow_camera()

    def step(self) -> None:
        """Advance the physics by one cfg.dt step."""
        p.stepSimulation(physicsClientId=self.client)
        self._follow_camera()

    def _follow_camera(self) -> None:
        if not self.gui:
            return
        x, y, _ = self.robot.pose()
        p.resetDebugVisualizerCamera(CAMERA_DISTANCE, CAMERA_YAW, CAMERA_PITCH, [x, y, 0.0],
                                     physicsClientId=self.client)

    def close(self) -> None:
        if self.client is not None and p.isConnected(self.client):
            p.disconnect(self.client)
        self.client = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
