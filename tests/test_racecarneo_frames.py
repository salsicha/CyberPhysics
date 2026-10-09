"""RACECAR Neo frames: one URDF for sim and hardware, matching topics.yaml.

The URDF is parsed directly (its only xacro features are xacro:if/unless on
boolean args), the
compositions are read as YAML, and the bridge is loaded with ROS stubbed out.
"""
import ast
import math
import shlex
import sys
import xml.etree.ElementTree as ET
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script

SYSTEM = REPO / 'systems/racecarneo'
XACRO = '{http://www.ros.org/wiki/xacro}'
TOPICS = yaml.safe_load((SYSTEM / 'config/topics.yaml').read_text())
FRAMES = TOPICS['frames']
GLOBAL_FRAMES = {FRAMES['map'], FRAMES['odom']}


def rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis roll/pitch/yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def load_urdf(camera_optical_frames=None, sim_geometry=None):
    """Return (links, {child: (parent, 4x4 parent_T_child)}) after xacro:if/unless."""
    root = ET.parse(SYSTEM / 'urdf/racecarneo.urdf.xacro').getroot()
    args = {arg.get('name'): arg.get('default') for arg in root.iter(XACRO + 'arg')}
    if camera_optical_frames is not None:
        args['camera_optical_frames'] = str(camera_optical_frames).lower()
    if sim_geometry is not None:
        args['sim_geometry'] = str(sim_geometry).lower()
    links, joints = set(), {}

    def condition(element):
        value = element.get('value')
        for name, arg_value in args.items():
            value = value.replace(f'$(arg {name})', arg_value)
        return value.strip().lower() in ('true', '1')

    def visit(element):
        for child in element:
            if child.tag == XACRO + 'if':
                if condition(child):
                    visit(child)
            elif child.tag == XACRO + 'unless':
                if not condition(child):
                    visit(child)
            elif child.tag == 'link':
                links.add(child.get('name'))
            elif child.tag == 'joint':
                assert child.get('type') == 'fixed'
                origin = child.find('origin')
                xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
                rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
                transform = np.eye(4)
                transform[:3, :3] = rpy_matrix(*rpy)
                transform[:3, 3] = xyz
                joints[child.find('child').get('link')] = (child.find('parent').get('link'), transform)

    visit(root)
    return links, joints


def base_to(frame, joints):
    transform = np.eye(4)
    while frame != FRAMES['base']:
        parent, parent_t_frame = joints[frame]
        transform = parent_t_frame @ transform
        frame = parent
    return transform


def compose_services(name):
    return yaml.safe_load((REPO / 'compositions' / name).read_text())['services']


def launch_args(command):
    return dict(token.split(':=', 1) for token in shlex.split(command) if ':=' in token)


SENSOR_FRAMES = [f for key, f in FRAMES.items() if f not in GLOBAL_FRAMES and key != 'base']


def test_urdf_defines_every_sensor_frame_in_topics_yaml():
    links, joints = load_urdf()  # defaults: the simulator configuration
    assert FRAMES['base'] in links
    for frame in SENSOR_FRAMES:
        assert frame in links, frame
        base_to(frame, joints)  # connected to base_link


@pytest.mark.parametrize('key', ['camera_color', 'camera_depth'])
def test_optical_frames_look_forward(key):
    _links, joints = load_urdf()
    rotation = base_to(FRAMES[key], joints)[:3, :3]
    # REP-103 optical frame: z forward, x right, y down (in base_link: +x, -y, -z).
    assert rotation[:, 2] == pytest.approx([1, 0, 0], abs=1e-9)
    assert rotation[:, 0] == pytest.approx([0, -1, 0], abs=1e-9)
    assert rotation[:, 1] == pytest.approx([0, 0, -1], abs=1e-9)


def test_hardware_realsense_names_match_topics_and_urdf():
    links, _joints = load_urdf(camera_optical_frames=False)
    # The RealSense driver publishes the optical frames on hardware.
    assert FRAMES['camera_color'] not in links and FRAMES['camera_depth'] not in links

    args = launch_args(compose_services('racecarneo_hardware.yaml')['racecarneo_camera']['command'])
    prefix = args['tf_prefix'] + args['camera_name']
    # realsense2_camera 4.58 frame ids: <tf_prefix><camera_name>_link / _<stream>_optical_frame
    assert prefix + '_link' in links
    assert prefix + '_color_optical_frame' == FRAMES['camera_color']
    assert prefix + '_depth_optical_frame' == FRAMES['camera_depth']
    # Topics: /<camera_namespace>/<camera_name>/<stream>/...
    node = '/'.join(part for part in (args['camera_namespace'], args['camera_name']) if part.strip('/'))
    node = '/' + node.strip('/')
    assert node + '/color/image_raw' == TOPICS['topics']['color_image']
    assert node + '/color/camera_info' == TOPICS['topics']['camera_info']
    assert node + '/depth/image_rect_raw' == TOPICS['topics']['depth_image']


@pytest.mark.parametrize('composition,optical,sim_geometry', [
    ('racecarneo.yaml', 'true', 'true'), ('racecarneo_hardware.yaml', 'false', 'false')])
def test_compositions_publish_the_urdf(composition, optical, sim_geometry):
    commands = [service.get('command', '') for service in compose_services(composition).values()]
    launches = [c for c in commands if 'racecarneo_description.launch.py' in c]
    assert len(launches) == 1
    args = launch_args(launches[0])
    assert args['camera_optical_frames'] == optical
    assert args.get('sim_geometry', 'false') == sim_geometry


@pytest.mark.parametrize('sim_geometry', [False, True])
def test_sim_and_hardware_geometry_define_the_same_frames(sim_geometry):
    links, joints = load_urdf(sim_geometry=sim_geometry)
    for frame in SENSOR_FRAMES:
        assert frame in links
        assert base_to(frame, joints) is not None


def test_bridge_frames_match_topics_yaml():
    params = yaml.safe_load((SYSTEM / 'config/bridge.yaml').read_text())
    params = params['racecarneo_bridge']['ros__parameters']
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('ackermann_msgs', 'ackermann_msgs.msg', 'rcl_interfaces',
                     'rcl_interfaces.msg', 'tf2_ros'):
            sys.modules[name] = MagicMock()
        module = load_script(REPO / 'applications/racecarneo/scripts/racecarneo_ros_bridge.py',
                             'racecarneo_bridge_frames')
    bridge = module.RacecarNeoBridge.__new__(module.RacecarNeoBridge)
    bridge.frame_prefix = params['frame_prefix']
    bridge.map_frame, bridge.odom_frame, bridge.base_frame = (
        params['map_frame'], params['odom_frame'], params['base_frame'])
    assert (bridge.map_frame, bridge.odom_frame, bridge.base_frame) == (
        FRAMES['map'], FRAMES['odom'], FRAMES['base'])
    # Header frame ids the bridge stamps on its sensor messages.
    for local, key in [('camera_color_optical_frame', 'camera_color'),
                       ('camera_depth_optical_frame', 'camera_depth'),
                       ('lidar_link', 'lidar'), ('imu_link', 'imu')]:
        assert bridge._frame(local) == FRAMES[key]


def test_validator_checks_frames_the_urdf_provides():
    source = (REPO / 'applications/racecarneo/scripts/validate_stack_integration.py').read_text()
    required = next(ast.literal_eval(node.value) for node in ast.walk(ast.parse(source))
                    if isinstance(node, ast.Assign)
                    and getattr(node.targets[0], 'id', None) == 'REQUIRED_TF')
    links, _joints = load_urdf()
    for parent, child in required:
        for frame in (parent, child):
            assert frame in links or frame in GLOBAL_FRAMES, frame
