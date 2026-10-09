"""RACECAR Neo vehicle model: Nav2, the converter and the simulator agree.

The simulated car (RacecarSim v2.8.3) is smaller than the hardware
description in cmd_vel_to_ackermann.yaml: its wheel colliders sit at
+-1.125 Unity units and 1 unit = 0.1 m (the simulator reports lidar and depth
in centimetres as distance * 10), so its wheelbase is 0.225 m, and
Drive.FixedUpdate steers the front wheels angle * 20 deg. A converter that
assumes the hardware wheelbase makes every simulated turn ~47% tighter than
Nav2 planned, and odometry with it under-reports the turn.
"""
import math
import shlex
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script

SIM_WHEELBASE_M = 0.225
SIM_STEERING_LOCK_RAD = math.radians(20.0)

CONFIG = REPO / 'systems/racecarneo/config'
BRIDGE = yaml.safe_load((CONFIG / 'bridge.yaml').read_text())['racecarneo_bridge']['ros__parameters']
HARDWARE = yaml.safe_load(
    (CONFIG / 'cmd_vel_to_ackermann.yaml').read_text())['racecarneo_cmd_vel_to_ackermann']['ros__parameters']
SMAC_RADIUS = yaml.safe_load((CONFIG / 'nav2.yaml').read_text())[
    'planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']


def converter_overrides(composition):
    """The `-p name:=value` overrides on a composition's converter command."""
    services = yaml.safe_load((REPO / 'compositions' / composition).read_text())['services']
    command = services['racecarneo_cmd_vel_to_ackermann']['command']
    tokens = shlex.split(command)
    return {tokens[i + 1].split(':=')[0]: float(tokens[i + 1].split(':=')[1])
            for i, token in enumerate(tokens[:-1]) if token == '-p'}


def test_sim_bridge_uses_the_simulated_car():
    assert BRIDGE['wheelbase'] == pytest.approx(SIM_WHEELBASE_M)
    assert BRIDGE['max_steering_angle'] == pytest.approx(SIM_STEERING_LOCK_RAD, abs=1e-3)


def test_sim_converter_drives_the_simulated_car_and_hardware_its_own():
    sim = converter_overrides('racecarneo.yaml')
    assert sim['wheelbase'] == pytest.approx(BRIDGE['wheelbase'])
    assert sim['max_steering_angle'] == pytest.approx(BRIDGE['max_steering_angle'])
    assert converter_overrides('racecarneo_hardware.yaml') == {}


@pytest.mark.parametrize('wheelbase,max_steer', [
    (BRIDGE['wheelbase'], BRIDGE['max_steering_angle']),
    (HARDWARE['wheelbase'], HARDWARE['max_steering_angle']),
], ids=['sim', 'hardware'])
def test_each_car_can_follow_the_nav2_minimum_turning_radius(wheelbase, max_steer):
    assert wheelbase / math.tan(max_steer) <= SMAC_RADIUS


@pytest.fixture
def chain():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('ackermann_msgs', 'ackermann_msgs.msg', 'rcl_interfaces',
                     'rcl_interfaces.msg', 'tf2_ros'):
            sys.modules[name] = MagicMock()
        converter_module = load_script(
            REPO / 'applications/racecarneo/scripts/cmd_vel_to_ackermann.py', 'vehicle_model_converter')
        bridge_module = load_script(
            REPO / 'applications/racecarneo/scripts/racecarneo_ros_bridge.py', 'vehicle_model_bridge')

        converter = converter_module.CmdVelToAckermann.__new__(converter_module.CmdVelToAckermann)
        sim = converter_overrides('racecarneo.yaml')
        converter.wheelbase = sim['wheelbase']
        converter.max_steering_angle = sim['max_steering_angle']
        converter.max_speed = HARDWARE['max_speed']
        converter.pub = MagicMock()
        converter.get_clock = MagicMock()
        converter.get_logger = MagicMock()

        bridge = bridge_module.RacecarNeoBridge.__new__(bridge_module.RacecarNeoBridge)
        bridge.max_speed = BRIDGE['max_speed']
        bridge.max_steering_angle = BRIDGE['max_steering_angle']
        bridge.get_logger = MagicMock()
        bridge._cmd_lock = MagicMock()
        yield converter, bridge


@pytest.mark.parametrize('radius', [SMAC_RADIUS, 1.0, 2.5, -SMAC_RADIUS, -1.5])
def test_nav2_curvature_reaches_the_simulated_car(chain, radius):
    converter, bridge = chain
    speed = 0.2
    converter._cmd_vel(NS(linear=NS(x=speed), angular=NS(z=speed / radius)))
    bridge._cmd_callback(converter.pub.publish.call_args.args[0])
    # RacecarSim: front wheels steer _steering * 20 deg on a 0.225 m wheelbase.
    wheel_angle = bridge._steering * SIM_STEERING_LOCK_RAD
    assert math.tan(wheel_angle) / SIM_WHEELBASE_M == pytest.approx(1.0 / radius, rel=0.01)
