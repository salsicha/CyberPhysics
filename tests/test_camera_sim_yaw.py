"""The airplane camera sim must render what a body-fixed nadir camera sees,
and demnav's de-rotation must turn that back into north-up imagery at
every heading. The contract is pinned against the physical model, not
against either implementation."""
import math
import sys
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest

import synthetic_world as sw
from conftest import REPO, install_ros_stubs, load_script

install_ros_stubs()
for _name in ('geographic_msgs', 'geographic_msgs.msg'):
    sys.modules.setdefault(_name, MagicMock())
camera_sim = load_script(
    REPO / 'systems/airplane/scripts/satellite_camera_sim.py', 'camera_sim')

from demnav_pkg.terrain_match import derotation_angle_deg  # noqa: E402


class Stub:
    pass


def make_stub(yaw, east=812.0, north=-433.0, mount=0.0):
    stub = Stub()
    stub.east_m = east
    stub.north_m = north
    stub.yaw_rad = yaw
    stub.camera_mount_yaw = mount
    stub.fov = math.radians(69.0)
    return stub


def ground(stub, width=64, height=48, view_height=358.0):
    return camera_sim.SatelliteCameraSim._ground_grid(
        stub, width, height, view_height)


def render_depth(stub, width, height, view_height, altitude):
    east, north = ground(stub, width, height, view_height)
    return np.maximum(
        1.0, altitude - sw.terrain_height(east, north)).astype(np.float32)


def offset(stub, east, north, row, col):
    return np.array([east[row, col] - stub.east_m,
                     north[row, col] - stub.north_m])


@pytest.mark.parametrize('yaw_deg,mount_deg,forward,starboard', [
    # ENU yaw: 0 = nose east, 90 = nose north.
    (0.0, 0.0, (1, 0), (0, -1)),
    (90.0, 0.0, (0, 1), (1, 0)),
    (180.0, 0.0, (-1, 0), (0, 1)),
    (-90.0, 0.0, (0, -1), (-1, 0)),
    # Image top turned 90 deg to port of the nose, nose north: top west.
    (90.0, 90.0, (-1, 0), (0, 1)),
])
def test_image_top_is_heading_and_right_is_clockwise(
        yaw_deg, mount_deg, forward, starboard):
    stub = make_stub(math.radians(yaw_deg), mount=math.radians(mount_deg))
    east, north = ground(stub)
    # The principal point (width/2, height/2) images the vehicle.
    assert offset(stub, east, north, 24, 32) == pytest.approx([0, 0],
                                                               abs=1e-9)
    top = offset(stub, east, north, 0, 32)
    right = offset(stub, east, north, 24, 63)
    np.testing.assert_allclose(top / np.linalg.norm(top), forward,
                               atol=1e-9)
    np.testing.assert_allclose(right / np.linalg.norm(right), starboard,
                               atol=1e-9)


@pytest.mark.parametrize('yaw_deg', [0.0, 45.0, 90.0, 180.0, -90.0, 32.0,
                                     -119.0, 138.0])
@pytest.mark.parametrize('mount_deg', [0.0, 90.0])
def test_demnav_derotation_recovers_north_up(yaw_deg, mount_deg):
    """Rotating the rendered body-fixed frame by demnav's de-rotation angle
    must reproduce an independently rendered north-up view (image top
    north, image right east)."""
    width, height, view_height, altitude = 160, 120, 358.0, 460.0
    gsd = 2.0 * view_height * math.tan(math.radians(69.0) / 2) / width
    cols, rows = np.meshgrid(np.arange(width) - width / 2,
                             np.arange(height) - height / 2)
    depth_north = np.maximum(1.0, altitude - sw.terrain_height(
        812.0 + cols * gsd, -433.0 - rows * gsd)).astype(np.float32)
    depth_body = render_depth(
        make_stub(math.radians(yaw_deg), mount=math.radians(mount_deg)),
        width, height, view_height, altitude)
    angle = derotation_angle_deg(math.radians(yaw_deg),
                                 math.radians(mount_deg))
    matrix = cv2.getRotationMatrix2D((width / 2.0, height / 2.0), angle, 1.0)
    recovered = cv2.warpAffine(depth_body, matrix, (width, height),
                               borderMode=cv2.BORDER_CONSTANT,
                               borderValue=float(np.median(depth_body)))
    # Compare inside the circle every rotation keeps in frame.
    inside = cols ** 2 + rows ** 2 < 55 ** 2
    error = np.abs(recovered - depth_north)[inside]
    assert np.median(error) < 0.5
    assert np.percentile(error, 95) < 3.0


def test_footprint_scales_with_view_height():
    stub = make_stub(math.radians(90.0))
    east_low, _ = ground(stub, 64, 48, 100.0)
    east_high, _ = ground(stub, 64, 48, 200.0)
    span_low = east_low[0, -1] - east_low[0, 0]
    span_high = east_high[0, -1] - east_high[0, 0]
    assert span_high == pytest.approx(2.0 * span_low, rel=1e-9)


def test_attitude_helpers_match_euler_convention():
    """Quaternion helpers of the matcher nodes recover ZYX Euler angles."""
    from demnav_pkg import camera_geometry as demnav_geometry
    from wildnav_pkg import camera_geometry as wildnav_geometry

    roll, pitch, yaw = math.radians(20.0), math.radians(-10.0), \
        math.radians(30.0)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

    q = Stub()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    for module in (demnav_geometry, wildnav_geometry):
        quaternion = (q.x, q.y, q.z, q.w)
        got_roll, got_pitch = module.roll_pitch_rad(quaternion)
        assert got_roll == pytest.approx(roll, abs=1e-9)
        assert got_pitch == pytest.approx(pitch, abs=1e-9)
        assert module.heading_rad(module.quaternion_matrix(
            *quaternion)) == pytest.approx(yaw, abs=1e-9)
