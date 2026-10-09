"""Turret ROS simulator metrics must be able to fail: command saturation,
emergency-stop violations and the command_dropout variant."""
import json
import math
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, install_ros_stubs, load_script

SCRIPTS = REPO / 'applications/turret_sim/scripts'
SCENARIO = json.loads((REPO / 'applications/turret_sim/scenarios/warehouse_tracking.json').read_text())


@pytest.fixture(scope='module')
def module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['diagnostic_msgs'] = MagicMock()
        sys.modules['diagnostic_msgs.msg'] = MagicMock()
        sys.path.insert(0, str(SCRIPTS))
        try:
            yield load_script(SCRIPTS / 'turret_ros_sim.py', 'turret_ros_sim_review')
        finally:
            sys.path.remove(str(SCRIPTS))


def make_sim(module, variant='nominal'):
    node = module.TurretRosSimulator.__new__(module.TurretRosSimulator)
    node.scenario = SCENARIO
    node.variant = variant
    node.sim = module.TurretSim(SCENARIO, variant_name=variant)
    node.dt = float(SCENARIO['dt_s'])
    node.current_t = 0.0
    node.records = []
    node.reacquire_times = []
    node.false_lock_count = 0
    node.command_log = []
    node.dropped_command_count = 0
    node.emergency_stop_started = None
    node.emergency_stop_grace_s = 2 * node.dt
    return node


def command(pan, tilt):
    return NS(name=['pan_joint', 'tilt_joint'], position=[pan, tilt])


def summary(node):
    return node._report()['summary']


def test_saturated_commands_are_counted(module):
    node = make_sim(module)
    for k, pan in enumerate([0.1, 2.0, 1.5708, -0.2]):
        node.current_t = 2.0 + 0.05 * k
        node._joint_command(command(pan, 0.0))
    assert summary(node)['command_saturation_fraction'] == pytest.approx(0.5)
    assert node.sim.command_queue[1][1] == pytest.approx(1.5708)  # clamped


def test_non_finite_commands_are_ignored(module):
    node = make_sim(module)
    node._joint_command(command(math.nan, 0.0))
    node._joint_command(command(0.0, math.inf))
    assert not node.sim.command_queue and not node.command_log


def test_command_dropout_variant_drops_commands(module):
    node = make_sim(module, 'command_dropout')
    node.current_t = 10.2
    node._joint_command(command(0.1, 0.0))
    assert not node.sim.command_queue
    node.current_t = 11.0
    node._joint_command(command(0.1, 0.0))
    assert len(node.sim.command_queue) == 1
    assert summary(node)['dropped_command_count'] == 1


def test_commands_during_emergency_stop_are_violations(module):
    node = make_sim(module, 'emergency_stop')
    node.emergency_stop_started = 7.25
    node.current_t = 7.3  # within the grace period: already in flight
    node._joint_command(command(0.1, 0.0))
    node.current_t = 7.5
    node._joint_command(command(0.1, 0.0))
    assert summary(node)['emergency_stop_violation_count'] == 1
    assert len(node.sim.command_queue) == 1


def test_offline_emergency_stop_variant_freezes_joints(module):
    report = module.TurretSim(SCENARIO, variant_name='emergency_stop').run()
    stopped = [s for s in report['samples'] if s['topics']['/turret/emergency_stop']]
    assert stopped
    assert len({tuple(s['topics']['/turret/joint_states']['position']) for s in stopped}) == 1
    assert report['summary']['emergency_stop_violation_count'] == 0
