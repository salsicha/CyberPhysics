"""The SO-101 picking scene: one source of truth for its heights.

systems/so101/scenarios/picking_table.json is the scene. Everything in it
rests on what it stands on (the arm on its 31 mm riser, the riser, bins and
objects on the table), Gazebo's world is generated from it, the launch file
spawns the arm where it says, and the shared constants are read from it, so
Isaac (kinematic objects) and Gazebo (free objects) start the task from the
same state as the printed station.
"""
import copy
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from unittest.mock import MagicMock

import numpy as np
import pytest

from conftest import REPO, install_ros_stubs

install_ros_stubs()
sys.path.insert(0, str(REPO / 'applications/so101/scripts'))
import so101_common as common  # noqa: E402
import scenario_world  # noqa: E402
import score_picking_task  # noqa: E402

SCENARIO = REPO / 'systems/so101/scenarios/picking_table.json'
WORLD = REPO / 'systems/so101/worlds/picking_table.sdf'
URDF = REPO / 'systems/so101/urdf/so101.urdf'
LAUNCH = REPO / 'applications/so101/launch/gazebo.launch.py'


def scenario():
    return json.loads(SCENARIO.read_text())


def test_scenario_is_physically_consistent():
    data = scenario()
    assert common.scene_support_errors(data) == []
    resting = [entry for entry in data['static_assets'] + data['objects'] if entry.get('type') != 'table']
    assert all('rests_on' in entry for entry in resting)
    assert data['robot_spawn']['rests_on'] == 'arm_base_mount'


def test_heights_are_the_printed_stations():
    data = scenario()
    entities = common.scene_entities(data)
    table_top = common.table_top_xyz(data)[2]
    assert table_top == pytest.approx(0.400)
    assert score_picking_task.table_top_z(data) == pytest.approx(table_top)   # the scorer's workspace floor
    mount_lower, mount_upper = common.entity_bounds(entities['arm_base_mount'])
    # the 31 mm riser (hardware/parts/so101/station_base_plate) on the table, the arm on it
    assert mount_lower[2] == pytest.approx(table_top)
    assert mount_upper[2] - mount_lower[2] == pytest.approx(0.031)
    assert data['robot_spawn']['xyz'][2] == pytest.approx(mount_upper[2])
    for name in ('left_bin', 'right_bin', 'occluder_box', 'red_block_target', 'green_block_distractor',
                 'blue_cylinder_distractor'):
        assert common.entity_bounds(entities[name])[0][2] == pytest.approx(table_top), name
        assert entities[name]['xyz'] == pytest.approx(common.resting_xyz(data, entities[name]))
    # the AprilGrid board stands in its printed feet (slot floor 6 mm up)
    sys.path.insert(0, str(REPO))
    from hardware.parts.so101 import calib_board_foot
    board = entities['calibration_board']
    assert board['standoff_m'] * 1000.0 == pytest.approx(calib_board_foot.BLOCK[2] - calib_board_foot.SLOT_DEPTH)
    assert common.entity_bounds(board)[0][2] == pytest.approx(table_top + board['standoff_m'])


def old_scene():
    """The scene before the fix: arm 31 mm above the table but the mount
    60 mm tall, objects and bins floating."""
    data = copy.deepcopy(scenario())
    entities = common.scene_entities(data)
    entities['arm_base_mount']['xyz'][2], entities['arm_base_mount']['size'][2] = 0.43, 0.06
    for name in ('left_bin', 'right_bin'):
        entities[name]['xyz'][2] = 0.45
    for entry in data['objects']:
        entry['xyz'][2] = 0.46 + common.entity_half_extents(entry)[2]
    return data


def test_checker_finds_the_old_scenes_inconsistencies():
    errors = '\n'.join(common.scene_support_errors(old_scene()))
    assert "robot_spawn's underside is at 0.4310 m but arm_base_mount's top is at 0.4600 m" in errors
    assert 'arm_base_mount and robot_base overlap by 29.0 mm' in errors
    assert "left_bin's underside is at 0.4325 m but workbench's top is at 0.4000 m" in errors
    assert "red_block_target's underside is at 0.4600 m" in errors
    with pytest.raises(ValueError, match='inconsistent scenario'):
        scenario_world.world_sdf(old_scene())


def test_checker_finds_interpenetrating_objects_and_missing_supports():
    data = scenario()
    entities = common.scene_entities(data)
    entities['red_block_target']['xyz'][:2] = entities['right_bin']['xyz'][:2]
    del entities['green_block_distractor']['rests_on']
    errors = '\n'.join(common.scene_support_errors(data))
    assert 'red_block_target and right_bin overlap' in errors
    assert 'green_block_distractor does not say what it rests on' in errors


def test_gazebo_world_is_generated_from_the_scenario():
    """Edit the scenario and run scenario_world.py; never the SDF."""
    assert WORLD.read_text() == scenario_world.world_sdf(scenario())
    assert scenario_world.main(['--check']) == 0


def sdf_models():
    world = ET.parse(WORLD).getroot().find('world')
    return {model.get('name'): model for model in world.findall('model')}


def test_gazebo_world_places_everything_where_the_scenario_does():
    data = scenario()
    models = sdf_models()
    for name, entry in common.scene_entities(data).items():
        pose = [float(value) for value in models[name].find('pose').text.split()]
        np.testing.assert_allclose(pose[:3], entry['xyz'], atol=1e-9, err_msg=name)
        np.testing.assert_allclose(pose[3:], entry.get('rpy', [0, 0, 0]), atol=1e-9, err_msg=name)
        static = models[name].find('static')
        is_object = name in {entry['id'] for entry in data['objects']}
        assert (static is None) == is_object, name   # objects are free bodies, assets static
        if is_object:
            assert float(models[name].find('link/inertial/mass').text) == pytest.approx(entry['mass_kg'])
    assert 'rear_panel' not in WORLD.read_text()      # nothing that is not in the scenario


def test_gazebo_camera_is_the_scenario_camera():
    camera = common.scenario_camera(scenario())
    model = sdf_models()[camera['name']]
    pose = [float(value) for value in model.find('pose').text.split()]
    np.testing.assert_allclose(pose, list(camera['xyz']) + list(camera['rpy']), atol=1e-9)
    sensor = model.find('link/sensor')
    assert sensor.get('type') == 'rgbd_camera'
    assert sensor.find('gz_frame_id').text == common.CAMERA_FRAME_ID
    assert float(sensor.find('update_rate').text) == pytest.approx(camera['frame_rate_hz'])
    assert math.degrees(float(sensor.find('camera/horizontal_fov').text)) == pytest.approx(
        camera['horizontal_fov_deg'], abs=1e-4)
    image = sensor.find('camera/image')
    assert [int(image.find('width').text), int(image.find('height').text)] == camera['resolution']
    launch = LAUNCH.read_text()
    for topic in (common.RGB_TOPIC, common.RGB_CAMERA_INFO_TOPIC, common.DEPTH_TOPIC,
                  common.DEPTH_CAMERA_INFO_TOPIC):
        assert f'"{topic}"' in launch, topic
    assert sensor.find('topic').text == '/so101/camera' and 'GZ_CAMERA = "/so101/camera"' in launch


def test_no_consumer_hard_codes_the_scene_heights():
    """The spawn height and the scene's z values live in the scenario only
    (the URDF's camera joint is checked against it instead)."""
    data = scenario()
    literals = {f"{data['robot_spawn']['xyz'][2]:g}"}
    literals |= {f"{entry['xyz'][2]:g}" for entry in data['static_assets'] + data['objects']}
    literals |= {f'{value:g}' for value in data['camera']['xyz'] + data['camera']['rpy'][1:2]}
    # values specific enough to be the scene's (three or more decimals)
    patterns = [re.compile(rf'(?<![\d.]){re.escape(value)}(?!\d)') for value in literals
                if len(value.split('.')[-1]) >= 3]
    assert len(patterns) >= 8
    sources = [LAUNCH, REPO / 'applications/so101/launch/description.launch.py']
    for directory in ('applications/so101/scripts', 'applications/groot/scripts', 'applications/smolvla/scripts'):
        sources += sorted((REPO / directory).glob('*.py'))
    sources += sorted((REPO / 'compositions').glob('so101_*.yaml'))
    for path in sources:
        text = path.read_text()
        assert not [pattern.pattern for pattern in patterns if pattern.search(text)], path
    assert 'robot_spawn' in LAUNCH.read_text()


def test_common_constants_come_from_the_scenario_and_the_urdf():
    data = scenario()
    camera = data['camera']
    assert (common.CAMERA_WIDTH, common.CAMERA_HEIGHT) == tuple(camera['resolution'])
    assert common.CAMERA_HFOV_DEG == camera['horizontal_fov_deg']
    assert common.DEFAULT_CAMERA_XYZ == tuple(camera['xyz'])
    assert common.WORLD_TO_TABLE_TRANSLATION == pytest.approx((0.42, 0.0, 0.40))
    root = ET.parse(URDF).getroot()
    base = next(link for link in root.iter('link') if link.get('name') == 'base_link')
    size = [float(value) for value in base.find('collision/geometry/box').get('size').split()]
    assert tuple(size) == common.BASE_LINK_SIZE
    joint = next(j for j in root.iter('joint') if j.get('name') == 'world_to_base')
    assert [float(value) for value in joint.find('origin').get('xyz').split()] == list(common.SPAWN_TO_BASE_LINK)
    assert common.SPAWN_TO_BASE_LINK[2] == pytest.approx(common.BASE_LINK_SIZE[2] / 2)  # spawn = underside


def test_scenario_file_can_be_overridden(tmp_path, monkeypatch):
    other = tmp_path / 'scene.json'
    other.write_text(json.dumps(scenario()))
    monkeypatch.setenv(common.SCENARIO_FILE_ENV, str(other))
    assert common.scenario_file() == other
    assert common.load_scenario_file() == scenario()


def test_isaac_scene_reports_an_inconsistent_scenario(capsys):
    sys.modules.setdefault('pxr', MagicMock())
    from so101_task_scene import SO101TaskScene
    SO101TaskScene(MagicMock(), old_scene())
    assert 'SO-101 scenario inconsistency' in capsys.readouterr().out
    SO101TaskScene(MagicMock(), scenario())
    assert 'inconsistency' not in capsys.readouterr().out
