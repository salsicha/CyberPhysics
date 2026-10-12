"""What nvblox maps in the simulator compositions
(applications/depthanything/README.md, "Accuracy";
applications/isaac/RACECAR_TURRET.md, "Depth for nvblox"): the simulated
camera's own depth rather than Depth Anything's, the RacecarSim camera
mount measured from that depth, and the Isaac arena's dynamic mapping of
the walking people. The hardware compositions are unchanged."""
import math
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

from conftest import REPO

LAUNCH = REPO / 'applications/nvblox/racecarneo_nvblox.launch.py'


def services(composition):
    return yaml.safe_load((REPO / 'compositions' / composition).read_text())['services']


def command(composition, service):
    text = services(composition)[service].get('command', '')
    return ' '.join(text) if isinstance(text, list) else text


def argument(composition, service, name):
    """A launch argument of a service's command, its ${VAR:-default}
    resolved to the default, or the launch file's default."""
    match = re.search(rf'{name}:=(\S+)', command(composition, service))
    if match is None:
        return re.search(rf"\('{name}', '([^']+)'", LAUNCH.read_text()).group(1)
    return re.sub(r'\$\{\w+:-([^}]+)\}', r'\1', match.group(1))


def test_simulators_map_the_cameras_depth_and_the_car_keeps_depth_anything():
    assert argument('racecarneo.yaml', 'racecarneo_nvblox', 'depth_image_topic') == \
        '/camera/depth/image_rect_raw'
    assert argument('racecar_turret_isaac.yaml', 'racecar_turret_nvblox', 'depth_image_topic') == \
        '/camera/depth/image_rect_raw'
    assert argument('racecarneo_hardware.yaml', 'racecarneo_nvblox', 'depth_image_topic') == \
        '/depth_anything/depth/image'
    # Depth Anything still runs in the simulators, publishing as on the car.
    for composition, service in (('racecarneo.yaml', 'racecarneo_depthanything'),
                                 ('racecar_turret_isaac.yaml', 'racecar_turret_depthanything')):
        assert '/depth_anything/depth/image' in command(composition, service)
    assert 'racecarneo_depthanything' not in services('racecarneo.yaml')['racecarneo_nvblox']['depends_on']


def test_the_nvblox_launch_takes_an_overrides_file_that_defaults_to_nothing_new():
    launch = LAUNCH.read_text()
    assert "'overrides_file'" in launch and 'default_value=config_file' in launch
    assert "LaunchConfiguration('overrides_file')" in launch.split('parameters=[')[1].split(']')[0]
    assert 'overrides_file' not in command('racecarneo_hardware.yaml', 'racecarneo_nvblox')


def test_the_isaac_arena_maps_people_as_dynamic_and_nav2_reads_the_combined_slice():
    overrides = yaml.safe_load((REPO / 'systems/racecar_turret/config/nvblox_isaac.yaml').read_text())
    params = overrides['/**']['ros__parameters']
    assert params['mapping_type'] == 'dynamic'
    assert 0.0 < params['static_mapper']['tsdf_decay_factor'] < 0.95
    base = yaml.safe_load((REPO / 'systems/racecarneo/config/nvblox.yaml').read_text())
    static = base['/**']['ros__parameters']['static_mapper']
    for key in ('esdf_slice_height', 'esdf_slice_min_height', 'esdf_slice_max_height'):
        assert params['dynamic_mapper'][key] == static[key], key
    assert base['/**']['ros__parameters']['mapping_type'] == 'static_tsdf'  # the car's
    nvblox = command('racecar_turret_isaac.yaml', 'racecar_turret_nvblox')
    assert argument('racecar_turret_isaac.yaml', 'racecar_turret_nvblox',
                    'overrides_file') == '/workspace/nvblox_isaac.yaml'
    assert any(v.endswith(':/workspace/nvblox_isaac.yaml:ro')
               for v in services('racecar_turret_isaac.yaml')['racecar_turret_nvblox']['volumes'])
    assert 'use_sim_time: true' in nvblox
    nav2 = command('racecar_turret_isaac.yaml', 'racecar_turret_nav2')
    assert 's#/nvblox_node/static_map_slice#${RACECAR_TURRET_NVBLOX_SLICE:-/nvblox_node/combined_map_slice}#' in nav2
    recorder = command('racecar_turret_isaac.yaml', 'racecar_turret_map_recorder')
    assert '--slice-topic ${RACECAR_TURRET_NVBLOX_SLICE:-/nvblox_node/combined_map_slice}' in recorder
    # nav2.yaml itself (the car's) reads the static slice.
    assert '/nvblox_node/static_map_slice' in (REPO / 'systems/racecarneo/config/nav2.yaml').read_text()


def test_racecarsim_camera_mount_is_the_measured_one():
    """RacecarSim's depth puts the floor 0.120 m below its camera, pitched
    0.81 degrees up; the prefab's level 0.145 m lifted the near floor into
    nvblox's 0.02 m obstacle band."""
    root = ET.parse(REPO / 'systems/racecarneo/urdf/racecarneo.urdf.xacro').getroot()
    xacro = '{http://www.ros.org/wiki/xacro}'
    sim = next(b for b in root.findall(xacro + 'if') if b.get('value') == '$(arg sim_geometry)')
    origin = next(j for j in sim.findall('joint') if j.get('name') == 'camera_joint').find('origin')
    x, y, z = (float(v) for v in origin.get('xyz').split())
    roll, pitch, yaw = (float(v) for v in origin.get('rpy').split())
    assert (x, y, z) == (0.10, 0.0, 0.120)
    # Negative pitch about y: the optical axis tilts up (ROS rpy).
    assert math.degrees(pitch) == pytest.approx(-0.81, abs=0.01) and roll == yaw == 0.0
