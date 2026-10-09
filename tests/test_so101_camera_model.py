"""One SO-101 camera model: renderer, CameraInfo and TF must agree.

Isaac renders from the scenario "camera" block (pose of groot_camera_link in
the world, resolution, horizontal FOV). CameraInfo must carry the intrinsics
of that FOV, and the URDF mount (TF) must place the optical frame where the
renderer is. The validator must check against the renderer, not a constant.
"""
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from conftest import REPO

sys.path.insert(0, str(REPO / 'applications/so101/scripts'))
import so101_common as common
from validate_calibration_and_dataset import (
    validate_scenario_camera,
    validate_urdf_alignment,
)

SCENARIO = REPO / 'systems/so101/scenarios/picking_table.json'
URDF = REPO / 'systems/so101/urdf/so101.urdf'
XACRO = REPO / 'systems/so101/urdf/so101.urdf.xacro'

# isaacsim.sensors.camera (Isaac Sim 6.0.1): W_U maps USD camera axes to the
# "world" camera axes Camera(position, orientation) is given in; U_R maps ROS
# optical axes to USD camera axes.
ISAAC_W_U = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
ISAAC_U_R = np.diag([1.0, -1.0, -1.0])


def scenario():
    return json.loads(SCENARIO.read_text())


def urdf_camera_origin(path):
    root = ET.parse(path).getroot()
    origin = next(j for j in root.iter('joint')
                  if j.get('name') == 'base_to_groot_camera').find('origin')
    return ([float(v) for v in origin.get('xyz').split()],
            [float(v) for v in origin.get('rpy').split()])


def world_from_base(data):
    spawn = data['robot_spawn']
    rotation = common.rpy_to_matrix(spawn['rpy'])
    return rotation, np.asarray(spawn['xyz']) + rotation @ np.asarray(common.SPAWN_TO_BASE_LINK)


def test_camera_info_matches_rendered_field_of_view():
    camera = common.scenario_camera(scenario())
    width, height = camera['resolution']
    values = common.camera_info_values(width, height, camera['horizontal_fov_deg'])
    fx = values['k'][0]
    assert fx == pytest.approx(465.6, abs=0.1)  # not the 60-degree 554
    assert 2 * math.degrees(math.atan(width / 2 / fx)) == pytest.approx(69.0)
    assert values['k'][4] == fx  # Isaac keeps pixels square
    assert common.CAMERA_FX == pytest.approx(fx)


@pytest.mark.parametrize('path', [URDF, XACRO], ids=['urdf', 'xacro'])
def test_urdf_mount_equals_scenario_camera(path):
    expected_xyz, expected_rpy = common.base_to_camera_link(scenario())
    xyz, rpy = urdf_camera_origin(path)
    np.testing.assert_allclose(xyz, expected_xyz, atol=1e-6)
    np.testing.assert_allclose(common.rpy_to_matrix(rpy), common.rpy_to_matrix(expected_rpy),
                               atol=1e-6)
    np.testing.assert_allclose(common.BASE_TO_CAMERA_LINK_TRANSLATION, expected_xyz, atol=1e-6)
    np.testing.assert_allclose(common.BASE_TO_CAMERA_LINK_RPY, expected_rpy, atol=1e-6)


def test_tf_optical_frame_is_isaac_ros_camera_frame():
    data = scenario()
    camera = common.scenario_camera(data)
    link_to_optical = common.rpy_to_matrix(common.CAMERA_LINK_TO_OPTICAL_RPY)
    # TF: world -> base_link -> groot_camera_link -> optical frame.
    base_rotation, base_xyz = world_from_base(data)
    mount_xyz, mount_rpy = urdf_camera_origin(URDF)
    tf_rotation = base_rotation @ common.rpy_to_matrix(mount_rpy) @ link_to_optical
    tf_position = base_xyz + base_rotation @ np.asarray(mount_xyz)
    # Isaac: Camera(position=xyz, orientation=rpy) uses the "world" camera axes.
    isaac_rotation = common.rpy_to_matrix(camera['rpy']) @ ISAAC_W_U @ ISAAC_U_R
    np.testing.assert_allclose(tf_rotation, isaac_rotation, atol=1e-6)
    np.testing.assert_allclose(tf_position, camera['xyz'], atol=1e-6)


def project(points, data):
    camera = common.scenario_camera(data)
    width, height = camera['resolution']
    k = np.asarray(common.camera_info_values(width, height, camera['horizontal_fov_deg'])['k'])
    k = k.reshape(3, 3)
    rotation = common.rpy_to_matrix(camera['rpy']) @ common.rpy_to_matrix(
        common.CAMERA_LINK_TO_OPTICAL_RPY)
    pixels = {}
    for name, point in points.items():
        in_camera = rotation.T @ (np.asarray(point) - np.asarray(camera['xyz']))
        if in_camera[2] <= 0:
            pixels[name] = None
            continue
        u, v, w = k @ in_camera
        pixels[name] = (u / w, v / w)
    return pixels, width, height


def test_camera_sees_the_robot_objects_and_both_bins():
    data = scenario()
    _, base_xyz = world_from_base(data)
    points = {'robot_base': base_xyz}
    points.update({entry['id']: entry['xyz'] for entry in data['objects']})
    points.update({entry['name']: entry['xyz'] for entry in data['static_assets']
                   if entry['type'] == 'target_bin'})
    pixels, width, height = project(points, data)
    margin = 40
    for name, pixel in pixels.items():
        assert pixel is not None, f'{name} is behind the camera'
        u, v = pixel
        assert margin <= u <= width - margin and margin <= v <= height - margin, (name, pixel)


def test_validator_checks_the_renderer_not_a_constant(tmp_path):
    metrics = validate_urdf_alignment(URDF, SCENARIO)
    assert metrics['camera_mount_xyz_error_m'] < 1e-6

    moved = tmp_path / 'moved.urdf'
    moved.write_text(URDF.read_text().replace(
        'xyz="0.17 -0.52 0.4665" rpy="0.0 0.785 1.449"', 'xyz="0.24 -0.28 0.34" rpy="0.0 0.72 2.36"'))
    with pytest.raises(AssertionError, match='scenario camera'):
        validate_urdf_alignment(moved, SCENARIO)

    camera = common.scenario_camera(scenario())
    info = dict(common.camera_info_values(640, 480, camera['horizontal_fov_deg']),
                frame_id=camera['frame_id'])
    good = tmp_path / 'camera_info.json'
    good.write_text(json.dumps(info))
    assert validate_scenario_camera(SCENARIO, good)['camera_info_k_error'] < 1e-9
    stale = tmp_path / 'stale_camera_info.json'
    stale.write_text(json.dumps(dict(info, k=[554.0, 0, 320, 0, 554.0, 240, 0, 0, 1],
                                     p=[554.0, 0, 320, 0, 0, 554.0, 240, 0, 0, 0, 1, 0])))
    with pytest.raises(AssertionError, match='camera_info.k'):
        validate_scenario_camera(SCENARIO, stale)
