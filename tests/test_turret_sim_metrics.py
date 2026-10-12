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


# --- The optional dart launcher --------------------------------------------------

def test_simulated_launcher_moves_at_servo_speed_with_a_sear(module):
    sim = module.TurretSim(SCENARIO, launcher=True)
    assert module.TurretSim(SCENARIO).launcher is False  # not fitted by default
    config = SCENARIO['pantilt']['launcher']
    assert sim.rack_speed_m_s == pytest.approx(2000 * 2 * math.pi / 4096 * config['pinion_radius_m'])
    dt = float(SCENARIO['dt_s'])
    t, draw = 0.0, config['stroke_m'] / sim.rack_speed_m_s

    def run(until, rack, trigger, estop=False):
        nonlocal t
        while t < until - 1e-9:
            sim.launcher_queue.append((t, rack, trigger))  # commands keep coming
            sim.update_servo(t, dt, estop)
            t = round(t + dt, 6)
    run(draw + 0.2, config['stroke_m'], 0.0)  # prime
    assert sim.rack == pytest.approx(config['stroke_m'], abs=2e-5) and sim.primed
    assert sim.shots == []
    run(t + draw + 0.2, 0.0, 0.0)  # chamber
    assert sim.rack == 0.0 and sim.primed
    run(t + 0.5, 0.0, config['trigger_fire_rad'])  # fire
    assert len(sim.shots) == 1 and not sim.primed
    run(t + 0.5, 0.0, 0.0)
    assert sim.trigger == 0.0
    # The trigger without a primed sear does not fire, nor after a draw that
    # stops short of the sear (the design's 34.25 mm); a command without the
    # rack stops it where it is
    run(t + 0.5, 0.0, config['trigger_fire_rad'])
    assert len(sim.shots) == 1
    assert config['sear_catch_m'] == 0.03425 and config['stroke_m'] == 0.038
    run(t + draw + 0.2, config['sear_catch_m'] - 0.0005, 0.0)
    run(t + draw + 0.2, 0.0, 0.0)
    assert not sim.primed
    run(t + 0.5, 0.0, config['trigger_fire_rad'])
    assert len(sim.shots) == 1
    # The sear releases at trigger_release_rad, short of the fire angle
    run(t + 0.5, 0.0, 0.0)
    run(t + draw + 0.2, config['sear_catch_m'] + 0.0005, 0.0)
    run(t + draw + 0.2, 0.0, 0.0)
    assert sim.primed
    run(t + 0.5, 0.0, config['trigger_release_rad'] - 0.01)
    assert len(sim.shots) == 1 and sim.primed
    run(t + 0.5, 0.0, config['trigger_release_rad'] + 0.01)
    assert len(sim.shots) == 2 and not sim.primed
    run(t + 0.5, 0.0, 0.0)
    run(t + 0.5, 0.03, 0.0)
    rack = sim.rack
    run(t + 0.5, None, 0.0)
    assert sim.rack == pytest.approx(rack, abs=1e-3)
    # The emergency stop and the watchdog: trigger to 0, rack stopped
    sim.launcher_queue.append((t, config['stroke_m'], config['trigger_fire_rad']))
    for _ in range(40):
        sim.update_servo(t, dt, True)
        t = round(t + dt, 6)
    assert sim.trigger == 0.0 and sim.rack == pytest.approx(rack, abs=1e-3)
    run(t + 1.5, 0.0, 0.0)
    assert sim.rack == 0.0
    sim.launcher_queue.append((t, config['stroke_m'], 0.0))
    for _ in range(40):  # one command, then none: 0.5 s later it stops
        sim.update_servo(t, dt, False)
        t = round(t + dt, 6)
    assert sim.rack < config['stroke_m'] - 0.01


def test_ros_sim_carries_the_launcher_in_commands_and_reports(module):
    node = make_sim(module)
    node.actuator_fitted = True
    node.sim = module.TurretSim(SCENARIO, launcher=True)
    node.priming_commands, node.firing_commands, node.launcher_log = [], [], []
    node.last_tags, node.presses, node.locks, node.acknowledged = [], [], [], []
    node.status = None
    node.current_t = 2.0
    joints = ['pan_joint', 'tilt_joint', 'actuator_joint', 'trigger_joint']
    node._joint_command(NS(name=joints, position=[0.1, 0.0, 0.038, 0.0]))
    node._joint_command(NS(name=joints[:3], position=[0.1, 0.0, 0.0]))  # trigger 0
    node._joint_command(command(0.1, 0.0))  # no launcher: trigger 0, rack stopped
    node._joint_command(NS(name=joints, position=[0.1, 0.0, 0.0, math.nan]))
    node._joint_command(NS(name=joints, position=[0.1, 0.0, 0.0, 0.6]))
    assert [entry[1:] for entry in node.sim.launcher_queue] == [
        (0.038, 0.0), (0.0, 0.0), (None, 0.0), (0.0, 0.6)]
    assert node.priming_commands == [2.0] and node.firing_commands == [2.0]
    report = node._launcher_report()
    assert report['unexplained_command_count'] == 2  # no accepted press explains them
    node.last_tags = [{'t_s': 1.9, 'tag': {'result': 'running'}}]
    node.sim.shots = [5.5]
    report = node._launcher_report()
    assert report['unexplained_command_count'] == 0
    assert report['shots'] == [{'t_s': 5.5, 'tag_at_s': 1.9, 'press_to_shot_s': 3.6,
                                'press_to_ready_s': None, 'true_aim_error_deg': None}]


def test_operator_reloads_taps_the_box_then_fires_only_when_the_page_would(module):
    node = make_sim(module)
    node.actuator_fitted, node.tag_at, node.tag_attempts, node.tag_lock = True, 2.0, 2, True
    node.darts, node.reloaded = 3, False
    node.presses, node.locks, node.last_tags = [], [], []
    node.operator_disarms, node.acknowledged = [], []
    node.operator_pub = MagicMock()
    node.get_logger = MagicMock()

    def sent():
        return [json.loads(call[0][0].data) for call in node.operator_pub.publish.call_args_list]
    ready = {'fire_block': '', 'locked': True, 'target_id': '7', 'launcher': 'ready',
             'aim_error_deg': 0.5}
    with patch.object(module, 'String', lambda data='': NS(data=data)):
        node.status, node.current_t = ready, 1.0
        node._operate()  # before tag_at
        assert sent() == []
        node.current_t = 2.5
        node._operate()
        assert sent() == [{'action': 'reload', 'darts': 3}]
        node.status = dict(ready, locked=False)
        node._operate()  # nothing locked
        node.status = dict(ready, fire_block='aim error over 2 deg')
        node._operate()  # the box first
        assert sent()[-1] == {'action': 'lock', 'target_id': '7'}
        node.current_t = 3.0
        node._operate()  # locked, but Fire not enabled
        node.status = ready
        node._operate()
        assert sent()[-1] == {'action': 'tag', 'target_id': '7'}
        node.last_tags = [{'t_s': 3.0, 'tag': {'result': 'running'}}]
        node.status, node.current_t = dict(ready, target_id='8', launcher='priming'), 4.0
        node._operate()  # a cycle is running: no tap, even on another ID
        node.last_tags.append({'t_s': 7.0, 'tag': {'result': 'fired'}})
        node.status, node.current_t = ready, 8.0
        node._operate()  # the next dart
        assert sent()[-1] == {'action': 'tag', 'target_id': '7'}
        node.last_tags += [{'t_s': 8.0, 'tag': {'result': 'running'}},
                           {'t_s': 12.0, 'tag': {'result': 'fired'}}]
        node.current_t = 13.0
        node._operate()  # --tag-attempts (2) presses: done
    assert len(sent()) == 4


def test_status_copies_and_older_statuses_are_ignored(module):
    node = make_sim(module)
    node.status, node.status_stamp, node.last_tags = None, -math.inf, []

    def status(stamp, result, tag_stamp):
        node._status(NS(data=json.dumps({'stamp': stamp, 'launcher': 'ready', 'last_tag': {
            'target_id': '7', 'result': result, 'stamp': tag_stamp}})))
    # Two subscriptions: each status twice, a copy of an older one late
    for t, (stamp, result, tag_stamp) in enumerate([
            (10.0, 'refused', 9.0), (10.0, 'refused', 9.0), (11.0, 'running', 11.0),
            (10.0, 'refused', 9.0), (11.0, 'running', 11.0), (12.0, 'fault', 11.6),
            (11.0, 'running', 11.0), (12.0, 'fault', 11.6)]):
        node.current_t = float(t)
        status(stamp, result, tag_stamp)
    assert [entry['tag']['result'] for entry in node.last_tags] == ['refused', 'running', 'fault']
    assert node.status['stamp'] == 12.0
    node._status(NS(data='not json'))
    node._status(NS(data='[1]'))
    assert node.status['stamp'] == 12.0


def test_operator_acknowledges_a_launcher_fault_only_with_tag_lock(module):
    for tag_lock in (True, False):
        node = make_sim(module)
        node.actuator_fitted, node.tag_at, node.tag_attempts, node.tag_lock = True, 2.0, 5, tag_lock
        node.darts, node.reloaded = 3, True
        node.presses = [{'t_s': 2.5, 'target_id': '7'}]
        node.locks = [{'t_s': 2.0, 'target_id': '7'}]
        node.last_tags = [{'t_s': 2.5, 'tag': {'result': 'running'}},
                          {'t_s': 3.1, 'tag': {'result': 'fault'}}]
        node.operator_disarms, node.acknowledged = [], []
        node.operator_pub = MagicMock()
        node.get_logger = MagicMock()
        node.sim.primed = True
        node.status = {'fire_block': 'launcher fault', 'locked': True, 'target_id': '7',
                       'launcher': 'fault'}
        with patch.object(module, 'String', lambda data='': NS(data=data)):
            node.current_t = 3.3
            node._operate()  # the operator reacts after half a second
            assert node.operator_disarms == []
            for t in (3.6, 3.7, 4.5):
                node.current_t = t
                node._operate()
        node.operator_pub.publish.assert_not_called()  # no tap, no Fire
        if tag_lock:
            # one disarm for the one fault, the primed launcher cleared by hand
            assert node.operator_disarms == [(3.6, 3.6 + module.OPERATOR_DISARM_S)]
            assert node.acknowledged == [{'t_s': 3.6, 'cleared_primed': True}]
            assert node.sim.primed is False
        else:
            assert node.operator_disarms == [] and node.sim.primed is True


def test_operator_without_lock_taps_only_reloads_and_fires(module):
    node = make_sim(module)
    node.actuator_fitted, node.tag_at, node.tag_attempts, node.tag_lock = True, 2.0, 2, False
    node.darts, node.reloaded = 2, True
    node.presses, node.locks, node.last_tags = [], [], []
    node.operator_disarms, node.acknowledged = [], []
    node.operator_pub = MagicMock()
    node.get_logger = MagicMock()
    node.status = {'fire_block': '', 'locked': True, 'target_id': '7', 'launcher': 'ready'}
    node.current_t = 2.5
    with patch.object(module, 'String', lambda data='': NS(data=data)):
        node._operate()
        # The clip's two darts shot: no more presses, attempts left or not
        node.tag_attempts = 10
        node.last_tags = [{'t_s': 3.0, 'tag': {'result': 'fired'}},
                          {'t_s': 6.0, 'tag': {'result': 'fired'}}]
        node.current_t = 8.0
        node._operate()
    assert [json.loads(call[0][0].data) for call in node.operator_pub.publish.call_args_list] == [
        {'action': 'tag', 'target_id': '7'}]
