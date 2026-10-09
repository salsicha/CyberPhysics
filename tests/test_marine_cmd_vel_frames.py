"""MAVROS mirror: body-frame cmd_vel for the marine waypoint followers.

pipeline_waypoint_nav.py and harbor_waypoint_nav.py publish linear.x as
forward speed and angular.z as yaw rate. Forwarding that as an ENU velocity
made ArduSub always drive east; the bridges now forward it as a body-frame
PositionTarget.
"""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script


class PositionTarget:
    FRAME_LOCAL_NED = 1
    FRAME_BODY_NED = 8
    IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
    IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
    FORCE = 512
    IGNORE_YAW, IGNORE_YAW_RATE = 1024, 2048

    def __init__(self):
        self.header = NS(stamp=None, frame_id='')
        self.coordinate_frame = 0
        self.type_mask = 0
        self.velocity = NS(x=0.0, y=0.0, z=0.0)
        self.yaw_rate = 0.0


class FakeNode:
    def __init__(self, name):
        if hasattr(self, 'params'):
            return
        self.params = {}
        self.publishers = {}
        self.callbacks = {}

    def declare_parameter(self, name, value):
        self.params.setdefault(name, value)

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def create_publisher(self, msg_type, topic, qos):
        return self.publishers.setdefault(topic, MagicMock(msg_type=msg_type))

    def create_subscription(self, msg_type, topic, callback, qos):
        self.callbacks[topic] = callback

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: 'now'))

    def get_logger(self):
        return MagicMock()


@pytest.fixture
def module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('mavros_msgs', 'mavros_msgs.msg'):
            sys.modules[name] = MagicMock()
        sys.modules['mavros_msgs.msg'].PositionTarget = PositionTarget
        sys.modules['rclpy.node'].Node = FakeNode
        yield load_script(
            REPO / 'applications/aerostack2/mavros_mirror/mavros_mirror/__init__.py',
            'mavros_mirror_review')


def make_mirror(module, path):
    params = yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']
    node = module.MavrosMirror.__new__(module.MavrosMirror)
    FakeNode.__init__(node, 'mavros_mirror')
    node.params.update(params)
    module.MavrosMirror.__init__(node)
    return node, params


def twist(forward, yaw_rate, up=0.0):
    return NS(linear=NS(x=forward, y=0.0, z=up), angular=NS(x=0.0, y=0.0, z=yaw_rate))


@pytest.mark.parametrize('system', ['submarine', 'boat'])
def test_simulated_marine_bridges_forward_body_frame_velocity(module, system):
    node, params = make_mirror(module, REPO / f'systems/{system}/config/bridge_sim.yaml')
    assert params['cmd_vel_frame'] == 'body_flu'
    node.callbacks[params['cmd_vel_topic']](twist(0.45, -0.2, up=-0.1))
    out = node.publishers['/mavros/setpoint_raw/local'].publish.call_args.args[0]
    assert out.coordinate_frame == PositionTarget.FRAME_BODY_NED
    assert (out.velocity.x, out.velocity.y, out.velocity.z) == (0.45, 0.0, -0.1)
    assert out.yaw_rate == -0.2
    # Velocity and yaw rate are used; position, acceleration and yaw are not.
    for used in ('IGNORE_VX', 'IGNORE_VY', 'IGNORE_VZ', 'IGNORE_YAW_RATE'):
        assert not out.type_mask & getattr(PositionTarget, used)
    for ignored in ('IGNORE_PX', 'IGNORE_PY', 'IGNORE_PZ', 'IGNORE_AFX',
                    'IGNORE_AFY', 'IGNORE_AFZ', 'IGNORE_YAW'):
        assert out.type_mask & getattr(PositionTarget, ignored)
    assert '/mavros/setpoint_velocity/cmd_vel' not in node.publishers


@pytest.mark.parametrize('system', ['submarine', 'boat'])
def test_hardware_marine_bridges_use_the_same_frame_convention(system):
    for name in ('bridge.yaml', 'bridge_sim.yaml'):
        params = yaml.safe_load(
            (REPO / f'systems/{system}/config/{name}').read_text())['/**']['ros__parameters']
        assert params['cmd_vel_frame'] == 'body_flu'


def test_world_frame_velocity_is_still_forwarded_unchanged(module, tmp_path):
    path = tmp_path / 'bridge.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'namespace': 'v', 'enable_velocity_setpoints': True,
        'cmd_vel_topic': '/v/cmd_vel'}}}))
    node, _ = make_mirror(module, path)
    msg = twist(1.0, 0.0)
    node.callbacks['/v/cmd_vel'](msg)
    out = node.publishers['/mavros/setpoint_velocity/cmd_vel'].publish.call_args.args[0]
    assert out.twist is msg


def test_unknown_cmd_vel_frame_is_rejected(module, tmp_path):
    path = tmp_path / 'bridge.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'cmd_vel_frame': 'body_frd'}}}))
    with pytest.raises(ValueError, match='cmd_vel_frame'):
        make_mirror(module, path)
