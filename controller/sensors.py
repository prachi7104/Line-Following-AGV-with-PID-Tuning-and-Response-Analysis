"""Simulated 5-sensor analog reflectance bar (see docs/DESIGN.md, section 2).

Each sensor looks at a floor patch as wide as the sensor spacing and is modelled by
``cfg.sub_rays_per_sensor`` downward rays spread evenly across that patch. A sensor's reading
is the fraction of its rays whose hit body is the line, so it is a number in [0, 1].

The signed error is the weighted centroid of the readings, negated so that it follows the
project convention (e > 0 <=> robot LEFT of the line):

    y_line   = sum(c_i * r_i) / sum(r_i)     (line position in the robot frame, left positive)
    e_sensed = -y_line                       [m]

When no ray sees the line the error is ``None``, never 0.0.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from common.config import DEFAULT, SimConfig


def subray_offsets(cfg: SimConfig = DEFAULT) -> tuple[np.ndarray, np.ndarray]:
    """Lateral offsets of every sub-ray (robot frame, left positive) and the sensor index of each.

    Sensor i with centre c_i and spacing s gets N rays at c_i - s/2 + (j + 0.5) * s / N.
    """
    n = cfg.sub_rays_per_sensor
    s = cfg.sensor_spacing
    centres = cfg.sensor_centres()
    within = -s / 2 + (np.arange(n) + 0.5) * s / n          # (N,) offsets inside one patch
    offsets = (centres[:, None] + within[None, :]).ravel()  # sensor-major order
    sensor_index = np.repeat(np.arange(cfg.n_sensors), n)
    return offsets, sensor_index


def ray_endpoints(pose, cfg: SimConfig = DEFAULT) -> tuple[np.ndarray, np.ndarray]:
    """World-frame start and end points, each (n_rays, 3), of the sensor rays.

    ``pose`` is (x, y, yaw) of the wheel-axle midpoint. Rays sit a distance ``cfg.d`` ahead of
    the axle and run straight down from ``cfg.ray_z_top`` to ``cfg.ray_z_bottom``.
    """
    x, y, yaw = pose
    lateral, _ = subray_offsets(cfg)
    c, s = np.cos(yaw), np.sin(yaw)
    px = x + cfg.d * c - lateral * s
    py = y + cfg.d * s + lateral * c
    starts = np.column_stack([px, py, np.full_like(px, cfg.ray_z_top)])
    ends = np.column_stack([px, py, np.full_like(px, cfg.ray_z_bottom)])
    return starts, ends


def readings_from_hits(hit_ids, line_id: int, cfg: SimConfig = DEFAULT) -> np.ndarray:
    """Fraction of each sensor's sub-rays whose hit object is the line body: shape (n_sensors,)."""
    hit_ids = np.asarray(hit_ids)
    _, sensor_index = subray_offsets(cfg)
    on_line = (hit_ids == line_id).astype(float)
    counts = np.bincount(sensor_index, weights=on_line, minlength=cfg.n_sensors)
    return counts / cfg.sub_rays_per_sensor


def error_from_readings(readings, cfg: SimConfig = DEFAULT) -> float | None:
    """Signed lateral error in metres, e = -sum(c_i r_i) / sum(r_i); None if nothing is seen."""
    r = np.asarray(readings, dtype=float)
    total = r.sum()
    if total <= 0.0:
        return None
    y_line = float(np.dot(cfg.sensor_centres(), r) / total)
    return -y_line


@dataclass
class SensorReading:
    error: float | None
    readings: np.ndarray


class LineSensor:
    """The sensor bar attached to a PyBullet client. One ``rayTestBatch`` call per read."""

    def __init__(self, client: int, line_id: int, cfg: SimConfig = DEFAULT):
        self.client = client
        self.line_id = line_id
        self.cfg = cfg

    def read(self, pose) -> SensorReading:
        import pybullet as p

        starts, ends = ray_endpoints(pose, self.cfg)
        results = p.rayTestBatch(
            starts.tolist(), ends.tolist(), physicsClientId=self.client
        )
        hit_ids = np.array([res[0] for res in results])
        readings = readings_from_hits(hit_ids, self.line_id, self.cfg)
        return SensorReading(error=error_from_readings(readings, self.cfg), readings=readings)
