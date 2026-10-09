"""SO-101 command-path safety: HIL gate robustness and non-finite commands.

np.clip passes NaN through, so every simulator command input, the joint-limit
controller and the Gazebo relay must reject non-finite commands rather than
clamp them. The HIL gate must survive malformed hardware feedback.
"""
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import MagicMock
import sys

import numpy as np
import pytest
import yaml

from conftest import REPO, install_ros_stubs

install_ros_stubs()
sys.modules.setdefault('tf2_ros', MagicMock())
sys.modules.setdefault('genesis', ModuleType('genesis'))
sys.path.insert(0, str(REPO / 'applications/so101/scripts'))
from gazebo_command_relay import CommandRelay
from genesis_so101 import GenesisBridge, to_numpy
from mock_so101_sim import MockSO101Sim
from so101_cbf_control import SO101CBFController
from so101_common import JOINT_NAMES, LOWER_LIMITS, UPPER_LIMITS, validated_joint_command
from so101_hil_guard import SO101HILGuard


NON_FINITE = [np.nan, np.inf, -np.inf]


def node(cls, **attributes):
    instance = cls.__new__(cls)
    instance.get_logger = MagicMock()
    for name, value in attributes.items():
        setattr(instance, name, value)
    return instance


def guard(now_ns=1_000_000_000, armed=True):
    clock = NS(now=lambda: NS(nanoseconds=clock.ns))
    clock.ns = now_ns
    return node(SO101HILGuard, armed=armed, state_timeout_s=0.5, max_joint_step=0.04,
                latest_positions=None, last_state_ns=None, warned_unarmed=False,
                warned_stale=False, publisher=MagicMock(), get_clock=lambda: clock), clock


@pytest.mark.parametrize('position', [[], [0.0] * 3])
def test_hil_guard_survives_state_with_short_position_array(position):
    gate, _ = guard()
    gate._state(NS(name=JOINT_NAMES, position=position))
    assert gate.latest_positions is None and gate.last_state_ns is None
    gate._command(NS(data=[0.0] * 6))
    gate.publisher.publish.assert_not_called()


@pytest.mark.parametrize('invalid', NON_FINITE)
def test_hil_guard_does_not_refresh_on_non_finite_state(invalid):
    gate, clock = guard()
    gate._state(NS(name=JOINT_NAMES, position=[0.0] * 6))
    clock.ns += 400_000_000
    gate._state(NS(name=JOINT_NAMES, position=[invalid] + [0.0] * 5))
    clock.ns += 200_000_000  # 0.6 s after the last valid state: stale
    gate._command(NS(data=[0.01] * 6))
    gate.publisher.publish.assert_not_called()


def test_hil_guard_forwards_only_when_armed_and_fresh():
    gate, clock = guard(armed=False)
    gate._state(NS(name=JOINT_NAMES, position=[0.0] * 6))
    gate._command(NS(data=[0.01] * 6))
    gate.publisher.publish.assert_not_called()

    gate, clock = guard(armed=True)
    gate._state(NS(name=JOINT_NAMES, position=[0.0] * 6))
    gate._command(NS(data=[1.0] * 6))
    gate.publisher.publish.assert_called_once()
    command = np.asarray(gate.publisher.publish.call_args.args[0].data)
    assert np.all(np.abs(command) <= 0.04 + 1e-12)


def test_hardware_compose_routes_policy_through_disarmed_guard():
    stack = yaml.safe_load((REPO / 'compositions/so101_hardware.yaml').read_text())
    services = stack['services']
    bridge = services['so101_groot_bridge']['command']
    gate = services['so101_hil_guard']
    assert 'command_topic:=/so101/hil/policy_commands' in bridge
    assert 'input_command_topic:=/so101/hil/policy_commands' in gate['command']
    assert 'armed:=${SO101_HIL_ARMED:-false}' in gate['command']
    assert 'hardware_command_topic:=${SO101_HARDWARE_COMMAND_TOPIC:-/so101/joint_commands}' \
        in gate['command']
    assert gate['restart'] == 'unless-stopped'
    assert set(gate['profiles']) == set(services['so101_groot_bridge']['profiles'])


@pytest.mark.parametrize('invalid', NON_FINITE)
def test_validated_joint_command_rejects_non_finite(invalid):
    with pytest.raises(ValueError, match='non-finite'):
        validated_joint_command([invalid] + [0.0] * 5)
    with pytest.raises(ValueError, match='expected 6'):
        validated_joint_command([0.0] * 5)
    np.testing.assert_array_equal(validated_joint_command([9.0] * 6), UPPER_LIMITS)


@pytest.mark.parametrize('invalid', NON_FINITE)
def test_simulators_keep_last_target_on_non_finite_command(invalid):
    bad = NS(data=[invalid, 0.1, 0.1, 0.1, 0.1, 0.01])
    good = NS(data=[0.2, 0.1, 0.1, 0.1, 0.1, 0.01])

    genesis = node(GenesisBridge, target=np.zeros(6, np.float32))
    genesis._command(good)
    genesis._command(bad)
    np.testing.assert_allclose(genesis.target, good.data, rtol=1e-6)

    mock_sim = node(MockSO101Sim, command_queue=[], command_latency_s=0.0,
                    get_clock=lambda: NS(now=lambda: NS(nanoseconds=0)))
    mock_sim._command(bad)
    assert mock_sim.command_queue == []
    mock_sim._command(NS(data=[5.0] * 6))
    np.testing.assert_allclose(mock_sim.command_queue[0][1], UPPER_LIMITS.astype(np.float32))

    relay = node(CommandRelay, publisher=MagicMock())
    relay._command(bad)
    relay.publisher.publish.assert_not_called()
    relay._command(good)
    relay.publisher.publish.assert_called_once()


@pytest.mark.parametrize('invalid', NON_FINITE)
def test_cbf_controller_never_publishes_non_finite(invalid):
    controller = node(SO101CBFController, kp=2.0, rate=50.0, margin=0.02,
                      q_des=np.zeros(6), q=np.zeros(6), state_received=False,
                      command_pub=MagicMock())
    controller._joint_callback(NS(name=JOINT_NAMES, position=[invalid] + [0.0] * 5))
    controller._control_step()
    controller.command_pub.publish.assert_not_called()

    controller._joint_callback(NS(name=JOINT_NAMES, position=[0.3] * 6))
    controller._control_step()
    command = np.asarray(controller.command_pub.publish.call_args.args[0].data)
    assert np.all(np.isfinite(command))
    assert np.all(command >= LOWER_LIMITS) and np.all(command <= UPPER_LIMITS)


def test_genesis_readings_are_copied_from_device_tensors():
    class CudaTensor:
        """Mimics a CUDA torch tensor: np.asarray() on it raises TypeError."""

        def __init__(self, values):
            self.values = np.asarray(values, dtype=np.float32)

        def __array__(self, *args, **kwargs):
            raise TypeError("can't convert cuda:0 device type tensor to numpy")

        def detach(self):
            return self

        def cpu(self):
            return NS(numpy=lambda: self.values)

    readings = CudaTensor([0.1, 0.2, 0.3, 0.4, 0.5, 0.02])
    with pytest.raises(TypeError):
        np.asarray(readings)
    np.testing.assert_allclose(to_numpy(readings), readings.values)
    np.testing.assert_allclose(to_numpy([0.0] * 6), np.zeros(6))
