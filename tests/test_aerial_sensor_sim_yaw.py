"""The Gazebo aerodrone sensor sim must render what a body-fixed nadir
camera sees (image top toward the nose), the same physical model as the
airplane satellite camera sim and DemNav's de-rotation
(tests/test_camera_sim_yaw.py)."""
import math
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

import synthetic_world as sw
from conftest import REPO, install_ros_stubs, load_script

RES_M = 5.0
AREA_KM = 2.0


@pytest.fixture
def modules():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('rclpy.executors', 'geographic_msgs', 'geographic_msgs.msg'):
            sys.modules[name] = MagicMock()
        sensor_sim = load_script(
            REPO / 'applications/gazebo/scripts/aerodrone_sensor_sim.py', 'aerodrone_sensor_sim_review')
        camera_sim = load_script(
            REPO / 'systems/airplane/scripts/satellite_camera_sim.py', 'satellite_camera_sim_review')
        yield sensor_sim, camera_sim


def dem_grid():
    half = AREA_KM * 500.0
    cells = int(2 * half / RES_M)
    index = np.arange(cells, dtype=np.float64)
    east = index * RES_M - half
    north = half - index * RES_M
    east_grid, north_grid = np.meshgrid(east, north)
    return sw.terrain_height(east_grid, north_grid).astype(np.float32)


def quaternion(yaw):
    return NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def make_sim(module, yaw, east, north, altitude):
    sim = module.AerodroneSensorSim.__new__(module.AerodroneSensorSim)
    sim.synthetic_width, sim.synthetic_height = 64, 48
    sim.fov_deg = 69.0
    sim.area_km, sim.dem_resolution_m = AREA_KM, RES_M
    sim.origin_alt_m = 0.0
    sim.dem_grid = dem_grid()
    sim.rng = NS(normal=lambda mean, std, size: np.zeros(size))
    sim.latest_odom = NS(pose=NS(pose=NS(position=NS(x=east, y=north, z=altitude),
                                         orientation=quaternion(yaw))))
    return sim


@pytest.mark.parametrize('yaw_deg', [0.0, 37.0, 90.0, 180.0, -120.0])
def test_synthetic_depth_is_a_body_fixed_nadir_view(modules, yaw_deg):
    sensor_sim, camera_sim = modules
    east, north, altitude = 120.0, -80.0, 400.0
    sim = make_sim(sensor_sim, math.radians(yaw_deg), east, north, altitude)
    depth = sim._synthetic_dem_depth()
    assert depth.shape == (48, 64)
    assert np.isfinite(depth).all()

    view_height = altitude - float(sw.terrain_height(east, north))
    camera = NS(east_m=east, north_m=north, yaw_rad=math.radians(yaw_deg),
                camera_mount_yaw=0.0, fov=math.radians(69.0))
    ground_east, ground_north = camera_sim.SatelliteCameraSim._ground_grid(
        camera, 64, 48, view_height)
    expected = altitude - sw.terrain_height(ground_east, ground_north)
    correlation = np.corrcoef(depth.ravel(), expected.ravel())[0, 1]
    assert correlation > 0.98
    assert np.median(np.abs(depth - expected)) < 2.0


def test_heading_reverses_the_image(modules):
    sensor_sim, _ = modules
    east_view = make_sim(sensor_sim, 0.0, 0.0, 0.0, 400.0)._synthetic_dem_depth()
    west_view = make_sim(sensor_sim, math.pi, 0.0, 0.0, 400.0)._synthetic_dem_depth()
    # A half turn about the principal point (width/2, height/2) maps pixel
    # (u, v) to (width - u, height - v).
    np.testing.assert_allclose(west_view[1:, 1:], east_view[47:0:-1, 63:0:-1], atol=0.5)
