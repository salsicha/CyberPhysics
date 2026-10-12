"""Turret Arduino bridge: no fabricated joint states, e-stop and link gating.

Drives the bridge's real methods with ROS stubbed out and a fake serial port.
"""
import math
from array import array
import sys
import time
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, install_ros_stubs, load_script


class RclpyJointState:
    """sensor_msgs/JointState as rclpy keeps it: position an array.array,
    so concatenating a list to it fails as it does in ROS."""

    def __init__(self):
        self.header = NS(stamp=None)
        self.name = []
        self._position = array('d')

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, values):
        self._position = array('d', values)


@pytest.fixture(scope='module')
def module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('diagnostic_msgs', 'diagnostic_msgs.msg', 'rclpy.executors'):
            sys.modules[name] = MagicMock()
        sys.modules['serial'] = None  # import fails -> bridge treats pyserial as absent
        module = load_script(REPO / 'applications/pantiltros/scripts/turret_arduino_bridge.py',
                             'turret_arduino_bridge_review')
        module.Bool = lambda data: NS(data=data)
        module.JointState = RclpyJointState
        yield module


class FakeSerial:
    def __init__(self, incoming=b''):
        self.is_open = True
        self.written = []
        self.incoming = incoming

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, size):
        data, self.incoming = self.incoming[:size], self.incoming[size:]
        return data

    def write(self, data):
        self.written.append(data.decode('ascii'))

    def flush(self):
        pass

    def close(self):
        self.is_open = False


@pytest.fixture
def bridge(module):
    node = module.TurretArduinoBridge.__new__(module.TurretArduinoBridge)
    node.get_logger = MagicMock()
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'stamp'))
    node.serial_port = '/dev/ttyACM0'
    node.baud_rate = 115200
    node.dry_run = False
    node.pan_min_rad, node.pan_max_rad = -1.5708, 1.5708
    node.tilt_min_rad, node.tilt_max_rad = -0.65, 0.75
    node.serial_handle = FakeSerial()
    node.serial_ready_time = 0.0  # settled
    node.last_connect_attempt = time.monotonic()  # no reconnect attempts
    node.reconnect_interval_s = 2.0
    node.connect_settle_s = 2.0
    node.last_error = ''
    node.last_write_time = node.last_feedback_time = None
    node.rx_buffer = b''
    node.estop_engaged = False
    node.stop_sent = False
    node.dropped_commands = 0
    node.last_ready = None
    node.joint_state_pub = MagicMock()
    node.ready_pub = MagicMock()
    node.diagnostics_pub = MagicMock()
    return node


def command(pan, tilt):
    return NS(name=['pan_joint', 'tilt_joint'], position=[pan, tilt])


def test_parse_position_line(module):
    assert module.parse_position_line('POS pan_joint 0.25 tilt_joint -0.1') == (0.25, -0.1)
    assert module.parse_position_line('POS tilt_joint -0.1 pan_joint 0.25') == (0.25, -0.1)
    assert module.parse_position_line(
        'POS pan_joint 0.25 tilt_joint -0.1 actuator_joint 0.07 trigger_joint 0.6') == (
        0.25, -0.1, 0.07, 0.6)
    for bad in ('POS pan_joint nan tilt_joint 0', 'POS pan_joint 1 tilt_joint',
                'SET pan_joint 1 tilt_joint 0', 'POS pan_joint x tilt_joint 0',
                'POS pan_joint 1 pan_joint 2', 'POS pan_joint inf tilt_joint 0',
                'POS pan_joint 1 tilt_joint 0 actuator_joint 0.03',  # the launcher comes in pairs
                'POS pan_joint 1 tilt_joint 0 actuator_joint nan trigger_joint 0',
                'POS pan_joint 1 tilt_joint 0 actuator_joint 1 actuator_joint 2',
                'POS pan_joint 1 tilt_joint 0 trigger_joint 1 trigger_joint 2',
                'POS pan_joint 1 tilt_joint 0 pan_joint 0 trigger_joint 0'):
        assert module.parse_position_line(bad) is None


def test_command_is_written_and_never_echoed_as_joint_state(bridge):
    bridge._joint_command(command(0.5, 2.0))
    assert bridge.serial_handle.written == ['SET pan_joint 0.500000 tilt_joint 0.750000\n']
    bridge.joint_state_pub.publish.assert_not_called()


def test_non_finite_command_is_rejected(bridge):
    for value in (math.nan, math.inf):
        bridge._joint_command(command(value, 0.0))
        bridge._joint_command(command(0.0, value))
    assert bridge.serial_handle.written == []
    assert 'non-finite' in bridge.last_error


def test_commands_dropped_while_settling_or_disconnected(bridge):
    bridge.serial_ready_time = time.monotonic() + 10.0
    assert not bridge.actuator_ready()
    bridge._joint_command(command(0.1, 0.1))
    bridge.serial_ready_time = 0.0
    bridge.serial_handle.is_open = False
    assert not bridge.actuator_ready()
    bridge._joint_command(command(0.1, 0.1))
    assert bridge.serial_handle.written == []
    assert bridge.dropped_commands == 2
    bridge.joint_state_pub.publish.assert_not_called()


def test_emergency_stop_sends_stop_and_blocks_commands(bridge):
    bridge._emergency_stop(NS(data=True))
    assert bridge.serial_handle.written == ['STOP\n']
    assert bridge.ready_pub.publish.call_args[0][0].data is False
    bridge._joint_command(command(0.1, 0.1))
    bridge._poll()  # STOP is not repeated
    assert bridge.serial_handle.written == ['STOP\n']
    bridge._emergency_stop(NS(data=False))
    assert bridge.ready_pub.publish.call_args[0][0].data is True
    bridge._joint_command(command(0.1, 0.1))
    assert bridge.serial_handle.written[-1].startswith('SET pan_joint 0.100000')


def test_stop_sent_once_settled_when_release_required(bridge):
    bridge.estop_engaged = True  # require_estop_release default
    bridge.serial_ready_time = time.monotonic() + 10.0
    bridge._poll()
    assert bridge.serial_handle.written == []
    bridge.serial_ready_time = 0.0
    bridge._poll()
    assert bridge.serial_handle.written == ['STOP\n']


def test_measured_positions_are_published_from_firmware_lines(bridge):
    bridge.serial_handle.incoming = (b'POS pan_joint 0.1 tilt_joint -0.2\nnoise\nERR servo 2 hot\n'
                                     b'POS pan_joint nan tilt_joint 0\nPOS pan_joint 0.3')
    bridge._poll()
    assert bridge.joint_state_pub.publish.call_count == 1
    state = bridge.joint_state_pub.publish.call_args[0][0]
    assert list(state.position) == [0.1, -0.2]
    assert bridge.last_error == 'firmware: servo 2 hot'
    bridge.serial_handle.incoming = b' tilt_joint 0.4\n'
    bridge._poll()
    assert list(bridge.joint_state_pub.publish.call_args[0][0].position) == [0.3, 0.4]


def test_dry_run_is_never_ready(bridge):
    bridge.dry_run = True
    assert not bridge.actuator_ready()
    bridge._joint_command(command(0.1, 0.1))
    assert bridge.serial_handle.written == []


def command4(pan, tilt, actuator=None, trigger=None):
    names, values = ['pan_joint', 'tilt_joint'], [pan, tilt]
    for name, value in (('actuator_joint', actuator), ('trigger_joint', trigger)):
        if value is not None:
            names.append(name)
            values.append(value)
    return NS(name=names, position=values)


def test_launcher_travels_in_set_lines_when_fitted(bridge):
    bridge.actuator_fitted = True
    bridge._joint_command(command4(0.5, 0.1, 0.038, 0.6))
    bridge._joint_command(command4(0.5, 0.1, 1.0, 5.0))       # clamped
    bridge._joint_command(command4(0.5, 0.1, 0.02))           # no trigger_joint: 0
    bridge._joint_command(command4(0.5, 0.1))                 # no launcher: rack stopped
    bridge._joint_command(command4(0.5, 0.1, trigger=0.6))    # the trigger alone: ignored
    bridge._joint_command(command4(0.5, 0.1, -0.2, -1.0))
    assert bridge.serial_handle.written == [
        'SET pan_joint 0.500000 tilt_joint 0.100000 actuator_joint 0.038000 trigger_joint 0.600000\n',
        'SET pan_joint 0.500000 tilt_joint 0.100000 actuator_joint 0.038000 trigger_joint 0.600000\n',
        'SET pan_joint 0.500000 tilt_joint 0.100000 actuator_joint 0.020000 trigger_joint 0.000000\n',
        'SET pan_joint 0.500000 tilt_joint 0.100000\n',
        'SET pan_joint 0.500000 tilt_joint 0.100000\n',
        'SET pan_joint 0.500000 tilt_joint 0.100000 actuator_joint 0.000000 trigger_joint 0.000000\n']
    bridge._joint_command(command4(0.5, 0.1, 0.02, math.nan))
    assert len(bridge.serial_handle.written) == 6 and 'non-finite' in bridge.last_error
    # Measured: the launcher joins the joint states when POS carries it
    bridge.serial_handle.incoming = (
        b'POS pan_joint 0.1 tilt_joint -0.2 actuator_joint 0.0375 trigger_joint 0.01\n'
        b'POS pan_joint 0.1 tilt_joint -0.2\n')
    bridge._poll()
    states = [(list(c[0][0].name), list(c[0][0].position))
              for c in bridge.joint_state_pub.publish.call_args_list]
    assert states == [(['pan_joint', 'tilt_joint', 'actuator_joint', 'trigger_joint'],
                       [0.1, -0.2, 0.0375, 0.01]),
                      (['pan_joint', 'tilt_joint'], [0.1, -0.2])]
    # The emergency stop's STOP makes the launcher safe in the firmware
    bridge._emergency_stop(NS(data=True))
    assert bridge.serial_handle.written[-1] == 'STOP\n'


def test_without_a_launcher_lines_and_states_are_unchanged(bridge):
    bridge._joint_command(command4(0.5, 0.1, 0.070, 0.6))
    assert bridge.serial_handle.written == ['SET pan_joint 0.500000 tilt_joint 0.100000\n']
    assert bridge.ignored_actuator_commands == 1
    bridge.serial_handle.incoming = (
        b'POS pan_joint 0.1 tilt_joint -0.2 actuator_joint 0.03 trigger_joint 0\n')
    bridge._poll()
    state = bridge.joint_state_pub.publish.call_args[0][0]
    assert (list(state.name), list(state.position)) == (['pan_joint', 'tilt_joint'], [0.1, -0.2])
