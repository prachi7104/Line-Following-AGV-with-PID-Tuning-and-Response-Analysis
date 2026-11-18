"""Tests for controller.sensors: sub-ray layout, ray geometry, readings and the centroid error."""
import math

import numpy as np
import pytest

from common.config import DEFAULT
from controller.sensors import (
    error_from_readings,
    ray_endpoints,
    readings_from_hits,
    subray_offsets,
)

CFG = DEFAULT


def test_subray_layout():
    offsets, idx = subray_offsets(CFG)
    n = CFG.n_sensors * CFG.sub_rays_per_sensor
    assert offsets.shape == (n,) and idx.shape == (n,)
    # Each sensor's rays are centred on that sensor and span its footprint.
    for i, c in enumerate(CFG.sensor_centres()):
        mine = offsets[idx == i]
        assert len(mine) == CFG.sub_rays_per_sensor
        assert mine.mean() == pytest.approx(c, abs=1e-12)
        assert mine.min() > c - CFG.sensor_spacing / 2
        assert mine.max() < c + CFG.sensor_spacing / 2


def test_line_centred_zero_error():
    r = np.array([0.0, 0.0, 1.0, 0.0, 0.0])
    assert error_from_readings(r, CFG) == pytest.approx(0.0, abs=1e-12)


def test_line_to_right_means_left_error():
    # Line under the -0.03 m (right) sensor -> robot is left of the line -> e > 0.
    r = np.array([0.0, 1.0, 0.0, 0.0, 0.0])
    assert error_from_readings(r, CFG) == pytest.approx(0.03, abs=1e-12)


def test_partial_interpolates():
    r = np.array([0.0, 0.5, 0.5, 0.0, 0.0])
    assert error_from_readings(r, CFG) == pytest.approx(0.015, abs=1e-12)


def test_no_hit_returns_none():
    assert error_from_readings(np.zeros(5), CFG) is None


def test_readings_from_hits_counts_only_line():
    line_id, floor_id = 7, 0
    _, idx = subray_offsets(CFG)
    hits = np.full(idx.shape, floor_id)
    hits[idx == 1] = line_id                      # sensor 1 fully on the line
    hits[(idx == 2)] = -1                         # sensor 2 misses everything
    sensor3 = np.flatnonzero(idx == 3)
    hits[sensor3[:3]] = line_id                   # sensor 3 partly on the line
    r = readings_from_hits(hits, line_id, CFG)
    assert r.shape == (5,)
    np.testing.assert_allclose(r, [0.0, 1.0, 0.0, 0.3, 0.0])


def test_endpoints_ahead_by_d():
    starts, ends = ray_endpoints((0.0, 0.0, 0.0), CFG)
    assert starts.shape == (50, 3) and ends.shape == (50, 3)
    np.testing.assert_allclose(starts[:, 0], CFG.d)
    np.testing.assert_allclose(ends[:, 0], CFG.d)
    np.testing.assert_allclose(starts[:, 2], 0.012)
    np.testing.assert_allclose(ends[:, 2], -0.005)
    offsets, _ = subray_offsets(CFG)
    np.testing.assert_allclose(starts[:, 1], offsets)
    np.testing.assert_allclose(ends[:, :2], starts[:, :2])


def test_endpoints_rotate():
    starts, _ = ray_endpoints((0.0, 0.0, math.pi / 2), CFG)
    np.testing.assert_allclose(starts[:, 1], 0.10, atol=1e-12)
    offsets, _ = subray_offsets(CFG)
    # Left-positive offsets point along -x when the robot faces +y.
    np.testing.assert_allclose(starts[:, 0], -offsets, atol=1e-12)


def test_linesensor_with_box():
    import pybullet as p  # required: fail, do not skip, when pybullet is missing
    import pybullet_data

    from controller.sensors import LineSensor

    client = p.connect(p.DIRECT)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
        p.loadURDF("plane.urdf", physicsClientId=client)
        half = [1.0, CFG.line_width / 2, CFG.line_height / 2]
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=client)
        line_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=col,
            basePosition=[0.0, 0.0, CFG.line_height / 2],
            physicsClientId=client,
        )
        sensor = LineSensor(client, line_id, CFG)

        # Bar at x = 0, robot 3 cm left of the line -> line under the -0.03 sensor.
        reading = sensor.read((-0.10, 0.03, 0.0))
        assert reading.error is not None
        assert reading.error == pytest.approx(0.03, abs=0.003)
        assert reading.readings.shape == (5,)

        # Far from the line: only floor hits -> no error.
        far = sensor.read((-0.10, 0.5, 0.0))
        assert far.error is None
        np.testing.assert_allclose(far.readings, 0.0)
    finally:
        p.disconnect(client)
