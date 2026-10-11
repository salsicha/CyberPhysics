"""cmd_vel_to_ackermann: manual driving priority, the latched stop, the
teleop deadman and the control status (systems/racecar_turret/interfaces.md).

The node is built without ROS (stubbed modules, no __init__); time is the
module's time.monotonic, replaced by a settable clock.
"""
import json
import math
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script

WHEELBASE, MAX_SPEED, MAX_STEER, TIMEOUT = 0.33, 0.35, 0.42, 0.5


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now


@pytest.fixture
def mux():
    with patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['ackermann_msgs'] = MagicMock()
        sys.modules['ackermann_msgs.msg'] = MagicMock()
        module = load_script(REPO / 'applications/racecarneo/scripts/cmd_vel_to_ackermann.py',
                             'ackermann_mux')
    clock = Clock()
    module.time = NS(monotonic=clock.monotonic)
    node = module.CmdVelToAckermann.__new__(module.CmdVelToAckermann)
    node.wheelbase = WHEELBASE
    node.max_speed = MAX_SPEED
    node.max_steering_angle = MAX_STEER
    node.teleop_timeout_s = TIMEOUT
    node.pub = MagicMock()
    node.status_pub = MagicMock()
    node.get_clock = MagicMock()
    node.get_logger = MagicMock()
    return module, node, clock


def twist(speed, yaw_rate=0.0):
    return NS(linear=NS(x=speed), angular=NS(z=yaw_rate))


def published(node):
    """(speed, steering) of every command published so far, then forget them."""
    commands = [(call.args[0].drive.speed, call.args[0].drive.steering_angle)
                for call in node.pub.publish.call_args_list]
    node.pub.publish.reset_mock()
    return commands


def test_cmd_vel_alone_is_converted_as_before(mux):
    _module, node, _clock = mux
    node._cmd_vel(twist(0.2, 0.2))
    [(speed, steering)] = published(node)
    assert speed == pytest.approx(0.2)
    assert 0.2 * math.tan(steering) / WHEELBASE == pytest.approx(0.2)
    assert node.status()['source'] == 'nav2'


def test_teleop_takes_priority_while_it_keeps_arriving(mux):
    _module, node, clock = mux
    node._cmd_vel_teleop(twist(-0.1))
    assert published(node) == [(pytest.approx(-0.1), 0.0)]
    for _ in range(4):
        clock.now += 0.1
        node._cmd_vel(twist(0.3))           # Nav2 still running: ignored
        node._cmd_vel_teleop(twist(-0.1))
    assert published(node) == [(pytest.approx(-0.1), 0.0)] * 4
    assert node.status()['source'] == 'teleop'

    # Nav2 drives again once teleop has been silent for the timeout.
    clock.now += TIMEOUT + 0.01
    node._cmd_vel(twist(0.3))
    assert published(node) == [(pytest.approx(0.3), 0.0)]
    assert node.status()['source'] == 'nav2'


def test_teleop_deadman_commands_zero_speed_once(mux):
    _module, node, clock = mux
    node._cmd_vel_teleop(twist(0.25, 0.3))
    steering = published(node)[0][1]
    clock.now += TIMEOUT - 0.05
    node._check_teleop_timeout()
    assert published(node) == []            # still within the timeout
    clock.now += 0.1
    node._check_teleop_timeout()
    # Zero speed, steering left where it was; only once.
    assert published(node) == [(0.0, steering)]
    node._check_teleop_timeout()
    assert published(node) == []
    assert node.status()['source'] == 'none'


def test_stop_request_latches_zero_speed_until_released(mux):
    _module, node, clock = mux
    node._cmd_vel(twist(0.3, 0.3))
    steering = published(node)[0][1]
    assert steering != 0.0

    node._stop_request(NS(data=True))
    assert published(node) == [(0.0, steering)]
    node._stop_request(NS(data=True))       # repeated (latched + volatile reader)
    assert published(node) == []
    for source in (node._cmd_vel, node._cmd_vel_teleop):
        source(twist(0.35, -0.5))
        assert published(node) == [(0.0, steering)]
    status = node.status()
    assert status['stopped'] is True and status['speed_mps'] == 0.0

    node._stop_request(NS(data=False))
    assert published(node) == []            # release itself commands nothing
    clock.now += TIMEOUT + 0.01
    node._cmd_vel(twist(0.2))
    assert published(node) == [(pytest.approx(0.2), 0.0)]
    assert node.status()['stopped'] is False


def test_non_finite_teleop_is_not_forwarded_but_holds_priority(mux):
    _module, node, _clock = mux
    node._cmd_vel_teleop(twist(math.nan))
    assert published(node) == []
    node._cmd_vel(twist(0.3))
    assert published(node) == []


def test_status_json_matches_the_contract(mux):
    module, node, _clock = mux
    module.String = lambda data: NS(data=data)
    node._cmd_vel_teleop(twist(0.1, 0.1))
    node._publish_status()
    payload = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert set(payload) == {'source', 'stopped', 'speed_mps', 'steering_rad'}
    assert payload['source'] == 'teleop' and payload['stopped'] is False
    assert payload['speed_mps'] == pytest.approx(0.1)
    assert 0.1 * math.tan(payload['steering_rad']) / WHEELBASE == pytest.approx(0.1)


def test_config_declares_the_contract_topics():
    params = yaml.safe_load((REPO / 'systems/racecarneo/config/cmd_vel_to_ackermann.yaml').read_text())
    params = params['racecarneo_cmd_vel_to_ackermann']['ros__parameters']
    # Relative names resolve in the root namespace the compositions run in.
    assert params['cmd_vel_topic'] == 'cmd_vel'
    assert params['teleop_topic'] == 'cmd_vel_teleop'
    assert params['teleop_timeout_s'] == TIMEOUT
    assert params['stop_request_topic'] == 'racecarneo/stop_request'
    assert params['control_status_topic'] == 'racecarneo/control_status'
    assert params['status_rate_hz'] >= 2.0
