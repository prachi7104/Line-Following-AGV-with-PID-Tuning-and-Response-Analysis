"""Track geometry (pure NumPy) and the helper that turns a track into one PyBullet line body.

Both tracks start at the origin heading +x. Sign convention: lateral_error > 0 <=> the point
is LEFT of the line looking along the direction of travel.

pybullet is imported lazily inside spawn_line() so this module (and its tests) work without it.
"""
from __future__ import annotations

import math

import numpy as np

from common.config import DEFAULT, SimConfig

TWO_PI = 2.0 * math.pi
LINE_RGBA = [0.05, 0.05, 0.05, 1.0]   # dark line on the light default floor
MAX_COMPOUND_SHAPES = 16              # PyBullet limit on children per createCollisionShapeArray


class StraightTrack:
    """Straight line along +x from (0, 0) to (length, 0)."""

    name = "straight"

    def __init__(self, length: float = DEFAULT.straight_length):
        self.length = float(length)

    def lateral_error(self, x: float, y: float) -> float:
        return float(y)

    def segment(self, x: float, y: float) -> str:
        return "straight"

    def start_pose(self, offset: float) -> tuple[float, float, float]:
        return (0.0, float(offset), 0.0)

    def end_reached(self, x: float, y: float) -> bool:
        return bool(x >= self.length)

    def polyline(self, step: float = 0.02) -> np.ndarray:
        n = max(1, math.ceil(self.length / step))
        xs = np.linspace(0.0, self.length, n + 1)
        return np.column_stack([xs, np.zeros_like(xs)])


class CurveTrack:
    """Straight lead-in along +x of length `lead`, then a LEFT arc of `radius` through `angle`.

    The arc centre is (lead, radius). Arc progress phi = atan2(x - lead, radius - y), so phi = 0
    where the arc meets the lead-in and phi = angle at the end of the track.
    """

    name = "curve"

    def __init__(self, lead: float = DEFAULT.curve_lead, radius: float = DEFAULT.curve_radius,
                 angle: float = DEFAULT.curve_angle):
        if not 0.0 < angle <= 1.5 * math.pi:
            # Past 1.5*pi the arc re-enters the region x < lead, y < radius that _on_lead_in uses
            # to recognise the lead-in, so lateral_error/segment/end_reached would be wrong there.
            raise ValueError(f"curve angle must be in (0, 1.5*pi], got {angle}")
        self.lead = float(lead)
        self.radius = float(radius)
        self.angle = float(angle)

    # --- geometry helpers ---
    def _on_lead_in(self, x: float, y: float) -> bool:
        return x < self.lead and y < self.radius

    def _phi(self, x: float, y: float) -> float:
        """Arc progress in [0, 2*pi); only meaningful off the lead-in."""
        return math.atan2(x - self.lead, self.radius - y) % TWO_PI

    # --- public interface ---
    def lateral_error(self, x: float, y: float) -> float:
        if self._on_lead_in(x, y):
            return float(y)
        return float(self.radius - math.hypot(x - self.lead, y - self.radius))

    def segment(self, x: float, y: float) -> str:
        return "straight" if self._on_lead_in(x, y) else "arc"

    def start_pose(self, offset: float) -> tuple[float, float, float]:
        return (0.0, float(offset), 0.0)

    def end_reached(self, x: float, y: float) -> bool:
        if self._on_lead_in(x, y):
            return False
        return bool(self._phi(x, y) >= self.angle)

    def polyline(self, step: float = 0.02) -> np.ndarray:
        n_lead = max(1, math.ceil(self.lead / step))
        xs = np.linspace(0.0, self.lead, n_lead + 1)
        lead_pts = np.column_stack([xs, np.zeros_like(xs)])
        n_arc = max(1, math.ceil(self.radius * self.angle / step))
        phis = np.linspace(0.0, self.angle, n_arc + 1)[1:]   # phi = 0 is the last lead-in point
        arc_pts = np.column_stack([self.lead + self.radius * np.sin(phis),
                                   self.radius - self.radius * np.cos(phis)])
        return np.vstack([lead_pts, arc_pts])


def _merge_collinear(pts: np.ndarray, tol: float = 1e-9) -> np.ndarray:
    """Drop interior polyline points that lie on a straight run, so a straight becomes one box."""
    keep = [0]
    for i in range(1, len(pts) - 1):
        a = pts[i] - pts[keep[-1]]
        b = pts[i + 1] - pts[i]
        cross = a[0] * b[1] - a[1] * b[0]
        if abs(cross) > tol * np.linalg.norm(a) * np.linalg.norm(b):
            keep.append(i)
    keep.append(len(pts) - 1)
    return pts[keep]


def spawn_line(client: int, track, cfg: SimConfig = DEFAULT) -> int:
    """Create the line as ONE static multibody made of thin boxes along track.polyline().

    Each box is cfg.line_width wide and cfg.line_height tall, resting on the floor (z = 0).
    Boxes that meet another box are lengthened by a quarter line width at that joint so the
    outside corner of every bend is covered (no gaps between segments).
    Returns the body unique id.
    """
    import pybullet as p

    raw = np.asarray(track.polyline(), dtype=float)
    # Run-out: the tape continues past the finish along the final heading. The sensor bar sits
    # cfg.d ahead of the axle and the run ends when the AXLE reaches the finish, so without this
    # the last ~d of every run would read "line lost" only because the tape ran out.
    # Geometry (lateral_error, segment, end_reached) is unchanged; only the physical line is longer.
    tail = raw[-1] - raw[-2]
    runout = cfg.d + cfg.line_width
    raw = np.vstack([raw, raw[-1] + tail / np.linalg.norm(tail) * runout])
    pts = _merge_collinear(raw)
    overlap = cfg.line_width / 4.0
    half_w = cfg.line_width / 2.0
    half_h = cfg.line_height / 2.0

    half_extents, positions, orientations = [], [], []
    n_seg = len(pts) - 1
    for i in range(n_seg):
        a, b = pts[i], pts[i + 1]
        seg = b - a
        length = float(np.linalg.norm(seg))
        u = seg / length
        ext_back = overlap if i > 0 else 0.0
        ext_front = overlap if i < n_seg - 1 else 0.0
        start = a - u * ext_back
        end = b + u * ext_front
        mid = (start + end) / 2.0
        heading = math.atan2(u[1], u[0])
        half_extents.append([(length + ext_back + ext_front) / 2.0, half_w, half_h])
        positions.append([float(mid[0]), float(mid[1]), half_h])
        orientations.append(p.getQuaternionFromEuler([0.0, 0.0, heading]))

    # PyBullet caps a compound shape at MAX_COMPOUND_SHAPES children, so the boxes are packed
    # into chunks: chunk 0 is the base, every further chunk is a fixed, massless link. It is
    # still ONE body, so every ray hit on the line reports objectUniqueId == the returned id.
    col_ids, vis_ids = [], []
    for k in range(0, n_seg, MAX_COMPOUND_SHAPES):
        sl = slice(k, k + MAX_COMPOUND_SHAPES)
        types = [p.GEOM_BOX] * len(half_extents[sl])
        col_ids.append(p.createCollisionShapeArray(
            shapeTypes=types, halfExtents=half_extents[sl],
            collisionFramePositions=positions[sl], collisionFrameOrientations=orientations[sl],
            physicsClientId=client))
        vis_ids.append(p.createVisualShapeArray(
            shapeTypes=types, halfExtents=half_extents[sl],
            visualFramePositions=positions[sl], visualFrameOrientations=orientations[sl],
            physicsClientId=client))
    n_links = len(col_ids) - 1
    origin, identity = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    body = p.createMultiBody(
        baseMass=0.0, baseCollisionShapeIndex=col_ids[0], baseVisualShapeIndex=vis_ids[0],
        basePosition=origin,
        linkMasses=[0.0] * n_links,
        linkCollisionShapeIndices=col_ids[1:], linkVisualShapeIndices=vis_ids[1:],
        linkPositions=[origin] * n_links, linkOrientations=[identity] * n_links,
        linkInertialFramePositions=[origin] * n_links,
        linkInertialFrameOrientations=[identity] * n_links,
        linkParentIndices=[0] * n_links, linkJointTypes=[p.JOINT_FIXED] * n_links,
        linkJointAxis=[[0.0, 0.0, 1.0]] * n_links,
        physicsClientId=client)
    for link in range(-1, n_links):
        p.changeVisualShape(body, link, rgbaColor=LINE_RGBA, physicsClientId=client)
    return body
