"""The airplane camera sim must render body-fixed frames that demnav's
-yaw de-rotation turns back into north-up imagery — the exact contract
that broke in review. These tests pin it."""
import math

import cv2
import numpy as np
import pytest

import synthetic_world as sw
from conftest import REPO, install_ros_stubs, load_script

install_ros_stubs()
camera_sim = load_script(
    REPO / 'systems/airplane/scripts/satellite_camera_sim.py', 'camera_sim')


class Stub:
    pass


def make_stub(yaw, east=812.0, north=-433.0):
    stub = Stub()
    stub.east_m = east
    stub.north_m = north
    stub.yaw_rad = yaw
    stub.fov = math.radians(69.0)
    return stub


def render_depth(stub, width, height, view_height, altitude):
    east, north = camera_sim.SatelliteCameraSim._ground_grid(
        stub, width, height, view_height)
    return np.maximum(
        1.0, altitude - sw.terrain_height(east, north)).astype(np.float32)


def test_zero_yaw_is_north_up():
    width, height, view_height = 64, 48, 358.0
    stub = make_stub(0.0)
    east, north = camera_sim.SatelliteCameraSim._ground_grid(
        stub, width, height, view_height)
    gsd = 2.0 * view_height * math.tan(stub.fov * 0.5) / width
    # Row 0 is the northern edge, column 0 the western edge.
    assert east[0, 0] == pytest.approx(stub.east_m - width * 0.5 * gsd)
    assert north[0, 0] == pytest.approx(
        stub.north_m + height * 0.5 * gsd, abs=gsd)
    assert np.all(np.diff(east[0]) > 0)
    assert np.all(np.diff(north[:, 0]) < 0)


@pytest.mark.parametrize('yaw_deg', [32.0, -45.0, -119.0, 138.0])
def test_demnav_derotation_recovers_north_up(yaw_deg):
    """demnav_node._run_matching rotates incoming depth by -yaw; applied to
    the sim's rendered frame that must reproduce the north-up view."""
    width, height, view_height, altitude = 160, 120, 358.0, 460.0
    depth_north = render_depth(make_stub(0.0), width, height,
                               view_height, altitude)
    depth_yawed = render_depth(make_stub(math.radians(yaw_deg)), width,
                               height, view_height, altitude)
    matrix = cv2.getRotationMatrix2D(
        (width / 2.0, height / 2.0), -yaw_deg, 1.0)
    fill = float(np.nanmedian(depth_yawed))
    recovered = cv2.warpAffine(depth_yawed, matrix, (width, height),
                               borderMode=cv2.BORDER_CONSTANT,
                               borderValue=fill)
    # Compare away from the warp-fill borders.
    error = np.abs(recovered[30:90, 40:120] - depth_north[30:90, 40:120])
    assert np.median(error) < 1.0
    assert np.percentile(error, 95) < 5.0


def test_footprint_scales_with_view_height():
    stub = make_stub(0.0)
    east_low, _ = camera_sim.SatelliteCameraSim._ground_grid(
        stub, 64, 48, 100.0)
    east_high, _ = camera_sim.SatelliteCameraSim._ground_grid(
        stub, 64, 48, 200.0)
    span_low = east_low[0, -1] - east_low[0, 0]
    span_high = east_high[0, -1] - east_high[0, 0]
    assert span_high == pytest.approx(2.0 * span_low, rel=1e-9)


def test_attitude_helpers_match_euler_convention():
    """Quaternion helpers in the matcher nodes recover ZYX Euler angles."""
    from demnav_pkg import demnav_node
    from wildnav_pkg import wildnav_node

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

    for module in (demnav_node, wildnav_node):
        got_roll, got_pitch = module._roll_pitch_from_quaternion(q)
        assert got_roll == pytest.approx(roll, abs=1e-9)
        assert got_pitch == pytest.approx(pitch, abs=1e-9)
    assert demnav_node._yaw_from_quaternion(q) == pytest.approx(
        yaw, abs=1e-9)
