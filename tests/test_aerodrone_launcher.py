"""The aerodrone's foam dart launcher (systems/aerodrone/tracking.md, "Foam
dart launcher"): the fire control in the tracker
(applications/aerostack2/scripts/launcher_core.py, aerodrone_tracker.py),
the turret bridge's launcher-only sts mode, the simulators' stand-in
(launcher_standin.py) and the targets' halt (tracking_targets.py).

Objects only, never people; operator-confirmed only:

- FireCycle is the turret's, unchanged (the class compared node for node),
  and the turret's FireCycle tests run on this copy;
- a fire command fires one dart; every gate refuses it with its reason,
  alone (each broken in an otherwise good situation), at its boundaries,
  in the documented order; a person as the target (person, pedestrian,
  people, any case, or no class), any person in the latest detection frame
  or in the last 3 s, or no recent frame refuses it;
- every abort (disarm, stop and track commands, pre-emption, a lost or
  changed target, a person appearing, the gates failing, a fault,
  shutdown, the bridge's watchdog) leaves the trigger at 0 and the rack
  stopped, before the release, and latches a fault until re-armed;
- property tests drive the fire control and the tracker node with random
  sequences of every other command (valid and not), detection frames,
  situations, joint states (also away from rest), readiness, odometry and
  ticks, against the stand-in's plant (ST3215 speed, the sear, the bridge's
  emergency stop and watchdog): nothing ever primes or fires without a fire
  command; with fire commands mixed in, every priming and firing command
  and every shot follows an accepted fire command whose gates held, within
  its time bound, every trigger command off 0 is sent with every release
  gate holding, judged here from the test's own record of the frames and
  situations (no person seen in the last 3 s), and there is at most one
  shot per accepted command;
- the structural tests check in the source that priming and firing are
  reachable only from the fire command's handler, after the gates, and
  that the joint commands come from one place.
"""
import ast
import json
import math
import random
import sys
import threading
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest
import yaml

import test_turret_launcher as turret_tests
from conftest import REPO, install_ros_stubs, load_script

SCRIPTS = REPO / 'applications/aerostack2/scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(REPO / 'applications/demnav/scripts'))
import launcher_core as lc  # noqa: E402
import tracking_core as tc  # noqa: E402
import tracking_targets as tt  # noqa: E402

standin = load_script(SCRIPTS / 'launcher_standin.py', 'launcher_standin_review')
tracker_node = load_script(SCRIPTS / 'aerodrone_tracker.py', 'aerodrone_tracker_launcher')
TURRET = REPO / 'applications/pantiltros/scripts/pantilt_yolo_tracker.py'
LAUNCHER_CORE = SCRIPTS / 'launcher_core.py'
TRACKER = SCRIPTS / 'aerodrone_tracker.py'
CONTRACT = REPO / 'systems/aerodrone/tracking.md'
STROKE, FIRE = 0.038, 0.6
RACK_SPEED = 2000 * 2 * math.pi / 4096 * 0.0135   # the ST3215 at 2000 steps/s on the pinion
SERVO_RAD_S = 2000 * 2 * math.pi / 4096
SEAR_RELEASE = 0.5194
# One cycle's commands and shot come within three steps' timeouts and the dwell.
CYCLE_BOUND_S = 3 * 3.0 + 0.2 + 0.1


def good(**changes):
    """A situation in which every gate but the launcher's own holds:
    tracking the car '7' centered, hovering 17 m above it, its estimate
    0.3 m from the bore's ground point."""
    situation = lc.Situation(state='tracking', mode='centered', target_id='7',
                             target_class='car', target_seen_s=0.1, target_speed_mps=0.1,
                             agl_m=17.0, speed_mps=0.1, tilt_deg=1.5, offset_m=0.3,
                             track_started=-5.0,
                             drone=(0.0, 0.0, 17.0), target=(0.3, 0.0, 0.0),
                             bore_ground=(0.0, 0.0, 0.0))
    for name, value in changes.items():
        setattr(situation, name, value)
    return situation


class Rig:
    """FireControl wired as aerodrone_tracker wires it, against the
    stand-in's LauncherPlant (the bridge: commands dropped until the
    emergency stop is released, a command without the rack stops it, the
    500 ms watchdog; the rack and trigger at the ST3215's speed; the sear):
    joint states and readiness every step, the emergency stop following the
    arm state, detection frames, the joint commands the control asks for."""

    def __init__(self, **config):
        self.fc = lc.FireControl(lc.LauncherConfig(**dict({'launcher_fitted': True}, **config)))
        self.plant = standin.LauncherPlant()
        self.s = good()
        self.plant_t = 0.0
        self.estop = None
        self.commands = []  # (t, rack, trigger) sent
        self.firing = []    # times of commands with the trigger off 0
        self.priming = []   # times of commands drawing the rack further back than it is
        self.frames = []    # (sequence, t, image time, people in it)
        self.sent = []      # (sequence, t, rack, trigger, situation, armed, ready) per command
        self.sequence = 0

    def advance(self, t):
        self.plant.update(t, max(0.0, t - self.plant_t))
        self.plant_t = t
        self.fc.on_joint_state(t, self.plant.rack, self.plant.trigger)
        self.fc.on_ready(t, not self.plant.estop)

    def send(self, t, force=False):
        goals = self.fc.joint_goals(t, self.s, force)
        if goals is not None:
            rack, trigger = goals
            self.sequence += 1
            self.sent.append((self.sequence, t, rack, trigger, self.s, self.fc.armed,
                              self.fc.ready_fresh(t)))
            self.commands.append((t, rack, trigger))
            if trigger > 0.0:
                self.firing.append(t)
            if rack is not None and rack > self.plant.rack + 1e-9:
                self.priming.append(t)
            self.plant.command(t, rack, trigger)
        if self.estop != (not self.fc.armed):
            self.estop = not self.fc.armed
            self.plant.emergency_stop(self.estop)

    def frame(self, t, items, image_time='now'):
        """A detection frame at t; its image time t, or image_time (None:
        a stamp the tracker cannot place on its clock)."""
        image_time = t if image_time == 'now' else image_time
        self.sequence += 1
        self.frames.append((self.sequence, t, image_time,
                            any(lc.person_like(cls) for _, cls in items)))
        if self.fc.on_detections(t, image_time, items):
            self.send(t)

    def tick(self, t):
        self.fc.tick(t, self.s)
        self.send(t)

    def command(self, t, **command):
        result = self.fc.handle(command, t, self.s)
        self.send(t)
        return result

    def run(self, t0, t1, frames=lambda t: [('7', 'car')], dt=0.02, frame_period=0.2):
        t, next_frame = t0, t0
        while t <= t1 + 1e-9:
            self.advance(t)
            if frames is not None and t >= next_frame - 1e-9:
                next_frame += frame_period
                items = frames(t)
                if items is not None:
                    self.frame(t, items)
            self.tick(t)
            t = round(t + dt, 9)


def armed_rig(darts=3, **config):
    """Reloaded, armed, the car seen for 0.6 s: every gate holds."""
    rig = Rig(**config)
    assert rig.command(0.0, action='reload', darts=darts) == f'reloaded: {darts} darts'
    assert rig.command(0.01, action='launcher_arm') == 'launcher armed'
    rig.run(0.02, 0.6)
    assert rig.fc.refusal(0.6, '7', rig.s) == ('' if darts else 'no darts')
    return rig


def phases(rig, t0, t1, frames=lambda t: [('7', 'car')]):
    seen = []
    t = t0
    while t <= t1 + 1e-9:
        rig.run(t, t, frames)
        state = rig.fc.state(t)
        if not seen or seen[-1] != state:
            seen.append(state)
        t = round(t + 0.02, 9)
    return seen


# --- FireCycle: the turret's -----------------------------------------------------

def strip_docstrings(node):
    for item in ast.walk(node):
        body = getattr(item, 'body', None)
        if isinstance(body, list) and body and isinstance(body[0], ast.Expr) and \
                isinstance(body[0].value, ast.Constant) and isinstance(body[0].value.value, str):
            item.body = body[1:] or [ast.Pass()]
    return node


def class_node(path, name):
    tree = ast.parse(path.read_text())
    return next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == name)


def module_constant(path, name):
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and
                                                target.id == name for target in node.targets):
            return ast.literal_eval(node.value)
    raise KeyError(name)


def test_the_fire_cycle_is_the_turrets_unchanged():
    ours = ast.dump(strip_docstrings(class_node(LAUNCHER_CORE, 'FireCycle')))
    theirs = ast.dump(strip_docstrings(class_node(TURRET, 'FireCycle')))
    assert ours == theirs
    for name in ('ACTUATOR_JOINT', 'TRIGGER_JOINT', 'RACK_TOLERANCE_M', 'TRIGGER_TOLERANCE_RAD',
                 'LAUNCHER_KEEPALIVE_S'):
        assert module_constant(LAUNCHER_CORE, name) == module_constant(TURRET, name), name


@pytest.mark.parametrize('test', [
    turret_tests.test_cycle_steps_against_the_measured_launcher,
    turret_tests.test_cycle_goals_are_bounded_in_time,
    turret_tests.test_cycle_faults_hold_the_rack_and_latch_until_reset,
    turret_tests.test_cycle_aborts,
    turret_tests.test_cycle_returns_a_launcher_found_away_from_rest,
], ids=lambda test: test.__name__)
def test_the_turrets_fire_cycle_tests_on_this_copy(test):
    test(NS(FireCycle=lc.FireCycle))


# --- One fire command, one dart ---------------------------------------------------

def test_a_fire_command_fires_one_dart():
    rig = armed_rig()
    assert rig.command(0.61, action='fire', target_id='7') == 'firing at 7'
    assert rig.fc.last_fire['result'] == 'running'
    assert phases(rig, 0.62, 4.5) == ['priming', 'chambering', 'firing', 'ready']
    assert len(rig.plant.shots) == 1 and rig.fc.shots == 1 and rig.fc.darts == 2
    # The rack's draw and return (each step ends within the 2 mm tolerance)
    # and the trigger to the sear's release, at the ST3215's speed.
    travel = 2 * (STROKE - 0.002) / RACK_SPEED + SEAR_RELEASE / SERVO_RAD_S
    assert travel < rig.plant.shots[0] - 0.61 < travel + 0.2
    assert rig.fc.last_fire['result'] == 'fired' and rig.fc.last_fire['target_id'] == '7'
    # The rack was drawn back and the trigger moved only in the cycle; now at rest.
    assert rig.priming and all(0.61 <= t <= 0.61 + CYCLE_BOUND_S for t in rig.priming)
    assert rig.firing and all(rig.priming[-1] < t for t in rig.firing)
    assert (rig.plant.rack, rig.plant.trigger, rig.plant.primed) == (0.0, 0.0, False)
    # The shot is recorded: what the gates saw when the command came and at the shot.
    [record] = rig.fc.records
    assert (record['shot'], record['target_id'], record['target_class'], record['result']) == (
        1, '7', 'car', 'fired')
    assert record['at_command']['agl_m'] == 17.0 and record['at_shot']['offset_m'] == 0.3
    assert record['darts_left'] == 2
    status = rig.fc.status(4.5, rig.s)
    assert (status['launcher'], status['darts'], status['shots']) == ('ready', 2, 1)
    # The next shot waits for the minimum interval after the cycle's end.
    wait = 2.0 - (4.5 - rig.fc.last_cycle_end)
    assert rig.fc.refusal(4.5, '7', rig.s) == f'{wait:.1f} s to the next shot'
    rig.run(4.52, round(4.5 + wait + 0.02, 2))
    assert rig.fc.refusal(round(4.5 + wait + 0.02, 2), '7', rig.s) == ''


def test_the_safe_command_is_the_turrets():
    """Off rest after a fault: commands carry the trigger at 0 and no rack
    goal (the bridge stops it where it is)."""
    rig = armed_rig()
    rig.command(0.61, action='fire', target_id='7')
    rig.run(0.62, 1.0)
    assert rig.fc.state(1.0) == 'priming'
    assert rig.command(1.01, action='launcher_disarm') == 'launcher disarmed'
    assert rig.commands[-1] == (1.01, None, 0.0)
    assert rig.plant.estop and rig.fc.cycle.phase == 'fault'
    rack = rig.plant.rack
    rig.run(1.02, 2.0)
    assert rig.plant.rack == rack and rig.plant.trigger == 0.0  # stopped where it was
    assert not rig.plant.shots and 'aborted while priming (launcher disarmed)' in \
        rig.fc.last_fire['reason']


# --- The gates ---------------------------------------------------------------------

GATE_CASES = [
    # (change to the good situation, gate, start of the reason)
    (dict(state='lost'), 'tracking', 'not tracking (tracker lost)'),
    (dict(state='acquiring'), 'tracking', 'not tracking (tracker acquiring)'),
    (dict(mode='slant'), 'tracking', 'not in centered mode (slant)'),
    (dict(target_class='person'), 'not_person', 'never at people: the target is a person'),
    (dict(target_class='Pedestrian'), 'not_person', 'never at people: the target is a Pedestrian'),
    (dict(target_class=' people '), 'not_person', 'never at people'),
    (dict(target_class=''), 'not_person', "the target's class is not known"),
    (dict(target_class=None), 'not_person', "the target's class is not known"),
    (dict(target_seen_s=0.81), 'target_seen', 'the target is not in the latest frames'),
    (dict(target_seen_s=None), 'target_seen', 'the target is not in the latest frames'),
    (dict(speed_mps=0.51), 'hover', 'not hovering: 0.5 m/s (at most 0.5)'),
    (dict(speed_mps=None), 'hover', 'no recent odometry'),
    (dict(speed_mps=math.nan), 'hover', 'no recent odometry'),
    (dict(tilt_deg=5.1), 'hover', 'not hovering: tilted 5.1 deg (at most 5)'),
    (dict(tilt_deg=None), 'hover', 'no recent odometry'),
    (dict(target_speed_mps=0.8), 'hover', 'the target is moving (0.8 m/s)'),
    (dict(target_speed_mps=None), 'hover', 'the target is moving'),
    (dict(agl_m=None), 'height', 'height above ground not known (no rangefinder or DEM)'),
    (dict(agl_m=math.inf), 'height', 'height above ground not known'),
    (dict(agl_m=7.9), 'height', '7.9 m above ground, outside 8-20 m'),
    (dict(agl_m=20.1), 'height', '20.1 m above ground, outside 8-20 m'),
    (dict(agl_m=50.0), 'height', '50.0 m above ground, outside 8-20 m'),
    (dict(offset_m=1.01), 'on_target', "the target is 1.0 m from the bore's ground point"),
    (dict(offset_m=None), 'on_target', "the target's or the bore's ground position is not known"),
    (dict(offset_m=math.nan), 'on_target', "the target's or the bore's ground position"),
]


def assert_refused_alone(rig, t, gate, reason, target_id='7'):
    """Firing at target_id now is refused by gate alone, with reason (also
    the status's for the tracked target); no command moves the launcher."""
    failed = rig.fc.gates(t, target_id, rig.s)
    assert [name for name, _ in failed] == [gate], failed
    assert failed[0][1].startswith(reason), failed
    if target_id == rig.s.target_id:
        status = rig.fc.status(t, rig.s)
        assert status['fire_block'] == failed[0][1]
        assert [name for name, ok in status['fire_checks'].items() if not ok] == [gate]
    commands, phase = len(rig.commands), rig.fc.cycle.phase
    result = rig.command(t, action='fire', target_id=target_id)
    assert result == f'fire refused: {failed[0][1]}'
    assert rig.fc.last_fire['result'] == 'refused'
    assert rig.fc.cycle.phase == phase and rig.commands[commands:] == []


@pytest.mark.parametrize('change,gate,reason', GATE_CASES)
def test_each_gate_refuses_alone_with_its_reason(change, gate, reason):
    rig = armed_rig()
    rig.s = good(**change)
    assert_refused_alone(rig, 0.6, gate, reason)
    rig.run(0.6, 3.0)
    assert rig.firing == [] and rig.priming == [] and rig.plant.shots == []


@pytest.mark.parametrize('change', [dict(agl_m=8.0), dict(agl_m=20.0), dict(offset_m=1.0),
                                    dict(speed_mps=0.5), dict(tilt_deg=5.0),
                                    dict(target_speed_mps=0.5), dict(target_seen_s=0.8),
                                    dict(target_class='Car')])
def test_the_gates_take_their_limits(change):
    rig = armed_rig()
    rig.s = good(**change)
    assert rig.fc.refusal(0.6, '7', rig.s) == ''
    assert rig.command(0.61, action='fire', target_id='7') == 'firing at 7'


def test_the_launchers_own_state_and_the_command_refuse_with_the_reason():
    # Not fitted: nothing at all (and arming is refused).
    rig = Rig(launcher_fitted=False)
    assert rig.command(0.0, action='launcher_arm') == 'arm refused: no launcher'
    assert rig.fc.status(0.0, rig.s)['launcher'] == 'none'
    assert rig.command(0.1, action='fire', target_id='7').startswith('fire refused: no launcher')
    assert rig.commands == []  # no joint command ever
    # Disarmed, then the bridge not ready (or silent for 1 s).
    rig = armed_rig()
    rig.command(0.6, action='launcher_disarm')
    assert_refused_alone(rig, 0.6, 'armed', 'launcher disarmed')
    rig = armed_rig()
    rig.fc.on_ready(0.6, False)
    assert_refused_alone(rig, 0.6, 'ready', 'launcher not ready')
    rig = armed_rig()
    assert rig.fc.refusal(1.61, '7', rig.s).startswith('launcher not ready')  # silent 1 s
    # The command must name the tracked target.
    rig = armed_rig()
    assert_refused_alone(rig, 0.6, 'target', '8 is not the tracked target (7)', target_id='8')
    for missing in (None, '', '  ', True, [7], {'id': 7}, math.nan):
        assert_refused_alone(rig, 0.6, 'target', "fire needs the tracked target's target_id",
                             target_id=missing)
    assert rig.command(0.6, action='fire', target_id=7) == 'firing at 7'  # a number is an id
    # A cycle running, a fault, no position, off rest, no darts, the interval.
    assert_refused_alone(rig, 0.62, 'launcher_rest', 'a fire cycle is running')
    rig.command(0.63, action='launcher_disarm')
    rig.command(0.64, action='launcher_arm')  # the fault reset: back to rest
    rig.run(0.65, 1.0)
    assert rig.fc.state(1.0) == 'ready'
    rig = armed_rig()
    rig.fc.cycle.start(0.6, '7')
    rig.fc.abort(0.6, 'test')
    rig.send(0.6)  # the safe command the abort asks for
    rig.fc.last_cycle_end = -math.inf  # (the interval apart)
    assert_refused_alone(rig, 0.6, 'launcher_rest', 'launcher fault: aborted while priming (test)')
    rig = armed_rig()
    assert_refused_alone(rig, 1.2, 'launcher_rest', 'no launcher position')  # 0.6 s silent
    rig = armed_rig()
    rig.fc.on_joint_state(0.6, 0.01, 0.0)
    rig.fc.cycle.update(0.6, 0.01, 0.0)  # found away from rest: returning
    rig.send(0.6)  # its command back to rest
    assert_refused_alone(rig, 0.6, 'launcher_rest', 'launcher not at rest')
    rig = armed_rig(darts=0)
    assert_refused_alone(rig, 0.6, 'darts', 'no darts')
    assert rig.fc.state(0.6) == 'empty'
    rig = armed_rig()
    rig.fc.last_cycle_end = 0.0
    assert_refused_alone(rig, 0.6, 'interval', '1.4 s to the next shot')


def test_the_reason_is_the_first_failing_gate_in_order():
    rig = Rig()
    rig.s = good(state='lost', target_class='person', agl_m=40.0)
    failed = rig.fc.gates(0.0, None, rig.s)
    assert [name for name, _ in failed] == ['armed', 'ready', 'tracking', 'target', 'not_person',
                                           'no_person', 'height', 'launcher_rest', 'darts']
    assert rig.fc.refusal(0.0, None, rig.s) == 'launcher disarmed'
    assert list(lc.GATES) == list(rig.fc.status(0.0, rig.s)['fire_checks'])


# --- Never at people -----------------------------------------------------------------

def test_the_person_classes_cover_every_detector_spelling():
    sys.path.insert(0, str(REPO / 'applications/oak1/scripts'))
    import tracking_messages
    model = json.loads((REPO / 'applications/oak1/models/visdrone_yolov8n_640x352.json')
                       .read_text())
    people = {name for name in list(model['classes']) + list(tracking_messages.CLASS_ALIASES)
              + ['person'] if tracking_messages.canonical_class(name) == 'person'}
    assert people == {'person', 'pedestrian', 'people'} == set(lc.PERSON_CLASSES)
    for name in people | {'PERSON', ' Pedestrian ', '', None}:
        assert lc.person_like(name), name
    for name in ('car', 'van', 'truck', 'bus', 'bicycle', 'motorcycle', 'tricycle', 'others',
                 'personal watercraft'):
        assert not lc.person_like(name), name


@pytest.mark.parametrize('cls', ['person', 'pedestrian', 'people', 'Person', ''])
def test_any_person_anywhere_in_view_refuses(cls):
    rig = armed_rig()
    rig.frame(0.6, [('7', 'car'), ('31', cls)])
    assert_refused_alone(rig, 0.6, 'no_person', 'a person is in view (#31')
    # Seen again 2.9 s after it left: still refused; 3 s clear: allowed.
    rig.run(0.62, 3.5)
    assert_refused_alone(rig, 3.5, 'no_person', 'a person was in view 2.9 s ago (#31')
    rig.run(3.52, 3.62)
    assert rig.fc.refusal(3.62, '7', rig.s) == ''


def test_a_simulation_reset_does_not_hold_a_person_forever():
    rig = armed_rig()
    rig.frame(100.0, [('7', 'car'), ('9', 'person')])
    assert rig.fc.person_block(100.5).startswith('a person is in view')
    # The clock went back 90 s (a world reset): new frames count again.
    rig.frame(10.0, [('7', 'car')])
    assert rig.fc.person_block(10.1) == ''
    rig.frame(10.2, [('7', 'car'), ('9', 'people')])
    assert rig.fc.person_block(10.3).startswith('a person is in view (#9 people)')


def test_without_recent_detections_people_cannot_be_ruled_out():
    rig = armed_rig()
    rig.run(0.62, 1.42, frames=None)  # the detector stalls
    status = rig.fc.status(1.42, rig.s)
    assert status['fire_block'] == 'no detections for 1.0 s: cannot check for people'
    # Frames whose stamp the tracker cannot place count for their people, not as frames.
    rig = armed_rig()
    rig.fc.on_detections(0.7, None, [('7', 'car')])
    assert rig.fc.person_block(1.41).startswith('no detections for')
    rig.fc.on_detections(1.42, None, [('4', 'person')])
    assert rig.fc.person_block(1.43).startswith('a person was in view')
    assert Rig().fc.person_block(0.0) == 'no detections: cannot check for people'


def test_a_person_appearing_during_the_cycle_stops_it_at_once():
    for when in (0.3, 1.2, 1.88):  # priming, chambering, the trigger's pull
        rig = armed_rig()
        rig.command(0.61, action='fire', target_id='7')
        rig.run(0.62, 0.61 + when)
        phase = rig.fc.cycle.phase
        sent = len(rig.commands)
        rig.frame(0.61 + when + 0.005, [('7', 'car'), ('9', 'person')])
        assert rig.commands[sent] == (0.61 + when + 0.005, None, 0.0), phase
        assert rig.fc.cycle.phase == 'fault' and 'a person appeared (#9 person)' in \
            rig.fc.cycle.reason, phase
        rig.run(0.62 + when, 6.0, frames=lambda t: [('7', 'car'), ('9', 'person')])
        assert rig.plant.shots == [] and rig.plant.trigger == 0.0, phase
        assert all(t < 0.61 + when + 0.005 for t in rig.firing + rig.priming), phase


def test_a_person_at_the_moment_of_release_stops_the_trigger():
    """Chambered, the next tick would pull the trigger: a person seen in the
    frame just before it, or the hold of a person seen within 3 s, keeps
    the trigger at 0."""
    rig = armed_rig()
    rig.command(0.61, action='fire', target_id='7')
    t = 0.62
    while rig.fc.cycle.phase != 'chambering' or rig.plant.rack > 0.004:
        rig.run(t, t)
        t = round(t + 0.02, 9)
    rig.frame(t, [('7', 'car'), ('2', 'pedestrian')])
    rig.run(t, t + 3.0)
    assert rig.firing == [] and rig.plant.shots == []
    assert 'aborted while chambering (a person appeared (#2 pedestrian))' in rig.fc.cycle.reason


ABORTS = {
    'disarm': lambda rig, t: rig.command(t, action='launcher_disarm'),
    'lost target': lambda rig, t: setattr(rig, 's', good(state='lost')),
    'stop (tracker idle)': lambda rig, t: setattr(rig, 's', good(state='idle', mode=None,
                                                                 target_id=None)),
    'handed over': lambda rig, t: setattr(rig, 's', good(target_id='8')),
    'slant': lambda rig, t: setattr(rig, 's', good(mode='slant')),
    'not hovering': lambda rig, t: setattr(rig, 's', good(speed_mps=1.2)),
    'tilted': lambda rig, t: setattr(rig, 's', good(tilt_deg=9.0)),
    'height': lambda rig, t: setattr(rig, 's', good(agl_m=25.0)),
    'off target': lambda rig, t: setattr(rig, 's', good(offset_m=2.0)),
    'target unseen': lambda rig, t: setattr(rig, 's', good(target_seen_s=2.0)),
    'bridge not ready': lambda rig, t: setattr(rig.plant, 'estop', True),
    # The detector gave the target's id to another car: the track restarted.
    'track restarted': lambda rig, t: setattr(rig, 's', good(track_started=t)),
}


@pytest.mark.parametrize('when', [0.3, 1.2, 1.88])
@pytest.mark.parametrize('cause', sorted(ABORTS))
def test_every_abort_stops_the_cycle_safely(cause, when):
    rig = armed_rig()
    rig.command(0.61, action='fire', target_id='7')
    rig.run(0.62, 0.61 + when)
    phase = rig.fc.cycle.phase
    assert phase in ('priming', 'chambering', 'firing')
    ABORTS[cause](rig, 0.61 + when + 0.01)
    rig.run(0.61 + when + 0.01, 0.61 + when + 0.02)
    assert rig.fc.cycle.phase == 'fault', (cause, phase)
    assert rig.commands[-1][1:] == (None, 0.0) or not rig.fc.armed
    assert rig.plant.trigger_cmd == 0.0 and rig.plant.rack_cmd == pytest.approx(rig.plant.rack)
    rig.run(0.61 + when + 0.04, 8.0)
    assert rig.plant.shots == [] and rig.plant.trigger == 0.0, (cause, phase)
    assert not rig.fc.cycle.running and rig.fc.state(8.0) == 'fault'
    # Latched until the launcher is disarmed and armed again.
    rig.s = good()
    rig.plant.estop = rig.estop
    assert rig.fc.refusal(8.0, '7', rig.s).startswith(('launcher fault', 'launcher not ready',
                                                       'launcher disarmed'))


def test_a_fault_resets_only_through_disarm_and_arm_and_the_launcher_returns_to_rest():
    rig = armed_rig()
    rig.command(0.61, action='fire', target_id='7')
    rig.run(0.62, 1.5)
    rig.s = good(state='lost')
    rig.run(1.52, 1.6)
    rig.s = good()
    rig.run(1.62, 3.0)
    assert rig.fc.state(3.0) == 'fault'
    assert rig.command(3.0, action='launcher_arm') == 'launcher armed; fault reset: returning it to rest'
    rig.run(3.02, 6.0)  # rack forward under supervision; nothing fires
    assert rig.fc.state(6.0) == 'ready' and rig.plant.rack == 0.0 and rig.plant.shots == []
    assert all(t < 1.6 for t in rig.priming)
    rig.run(6.0, 6.0)
    assert rig.command(6.02, action='fire', target_id='7') == 'firing at 7'


def test_the_bridges_watchdog_makes_it_safe_when_the_commands_stop():
    rig = armed_rig()
    rig.command(0.61, action='fire', target_id='7')
    rig.run(0.62, 1.0)
    # The tracker stops (no more ticks): the plant's watchdog returns the
    # trigger to 0 and stops the rack 0.5 s after the last command.
    t = 1.0
    while t < 3.0:
        t = round(t + 0.02, 9)
        rig.plant.update(t, 0.02)
    assert rig.plant.trigger == 0.0 and rig.plant.rack_cmd == rig.plant.rack
    assert rig.plant.shots == []


def test_reload_only_disarmed_and_within_the_clip():
    rig = Rig()
    assert rig.command(0.0, action='reload', darts=7) == \
        'reload needs darts: a whole number from 0 to 6'
    for bad in (-1, 2.5, '6', True, None):
        assert rig.command(0.0, action='reload', darts=bad).startswith('reload needs darts')
    assert rig.fc.darts == 0
    rig.command(0.0, action='launcher_arm')
    assert rig.command(0.0, action='reload', darts=6) == \
        'reload refused: disarm the launcher first'
    rig.command(0.0, action='launcher_disarm')
    assert rig.command(0.0, action='reload', darts=6) == 'reloaded: 6 darts'


def test_status_fields_are_the_contracts():
    rig = armed_rig()
    status = rig.fc.status(0.6, rig.s)
    contract = CONTRACT.read_text()
    section = contract[contract.index('## Foam dart launcher'):]
    for key in status:
        assert f'"{key}"' in section, key
    for gate in lc.GATES:
        assert f'`{gate}`' in section, gate
    assert status['launcher'] in ('none', 'ready', 'priming', 'chambering', 'firing', 'empty',
                                  'fault')
    json.dumps(status)


def test_the_defaults_and_the_deployed_values_are_the_documented_ones():
    config = lc.LauncherConfig()
    assert not config.launcher_fitted  # the aircraft without the launcher
    assert (config.fire_min_agl_m, config.fire_max_agl_m, config.fire_max_offset_m,
            config.fire_max_speed_mps, config.fire_max_tilt_deg,
            config.fire_max_target_speed_mps, config.fire_detection_timeout_s,
            config.fire_person_clear_s, config.fire_min_interval_s,
            config.launcher_capacity, config.launcher_darts) == (
        8.0, 20.0, 1.0, 0.5, 5.0, 0.5, 0.8, 3.0, 2.0, 6, 0)
    env = {}
    for line in (REPO / 'systems/aerodrone/config/tracking.env').read_text().splitlines():
        if line.startswith('TRACKING_FIRE_'):
            key, _, value = line.partition('=')
            env[key[len('TRACKING_'):].lower()] = float(value)
    assert env == {name: getattr(config, name) for name in env} and len(env) == 9
    # No option fires on its own: nothing automatic, timed or on-lock.
    for name in list(vars(config)) + list(tracker_node.PARAMETERS):
        if name == 'auto_next':  # selection only; no autonomous fire in the tracker
            assert tracker_node.PARAMETERS[name] is True
            continue
        assert not any(word in name for word in ('auto', 'timer', 'on_lock', 'periodic')), name
    with pytest.raises(ValueError):
        lc.LauncherConfig(fire_min_agl_m=20.0, fire_max_agl_m=8.0)
    with pytest.raises(ValueError):
        lc.LauncherConfig(launcher_darts=7)
    with pytest.raises(ValueError):
        lc.LauncherConfig(fire_max_offset_m=math.nan)


def test_the_height_window_from_the_dart_and_the_camera():
    """launcher_core's reasons: a 1 g Elite dart (12.7 mm, Cd about 0.75)
    fired down at 30 m/s slows toward its terminal speed, so from any height
    it lands no faster than the muzzle velocity; from the 20 m maximum a
    5 m/s wind and 3 deg of dispersion keep it within half of what the
    OAK-1 sees across its image; a person still spans about 10 px."""
    rho, g, mass, diameter, cd = 1.225, 9.81, 1.0e-3, 0.0127, 0.75
    k = 0.5 * rho * cd * math.pi * diameter ** 2 / 4 / mass

    def fall(height, wind=5.0, v0=30.0, dt=1e-4):
        x = z = vx = 0.0
        vz = v0
        while z < height:
            speed = math.hypot(vx - wind, vz)
            vx += -k * speed * (vx - wind) * dt
            vz += (g - k * speed * vz) * dt
            x, z = x + vx * dt, z + vz * dt
        return x, math.hypot(vx, vz)

    terminal = math.sqrt(g / k)
    assert 12.0 < terminal < 15.0
    half_fov = math.radians(69.0) / 2
    short_half = math.atan(math.tan(half_fov) * 352 / 640)
    config = lc.LauncherConfig()
    for height in (config.fire_min_agl_m, 15.0, config.fire_max_agl_m):
        drift, impact = fall(height)
        assert impact < 30.0
        landing = drift + height * math.tan(math.radians(3.0))
        assert landing < 0.5 * height * math.tan(short_half)
    assert 9.5 < 0.45 * 640 / (2 * config.fire_max_agl_m * math.tan(half_fov)) < 11.0
    assert config.fire_min_agl_m * math.tan(short_half) >= 3.0
    # The tracker never flies the centered mode lower than 15 m: the window
    # overlaps its limits.
    assert config.fire_min_agl_m < tc.TrackerConfig().min_agl_m < config.fire_max_agl_m


# --- Properties -----------------------------------------------------------------------

def random_situation(rng, calm):
    """Mostly good when calm; otherwise anything the tracker may report."""
    if calm and rng.random() < 0.85:
        return good(speed_mps=rng.uniform(0.0, 0.45), tilt_deg=rng.uniform(0.0, 4.5),
                    offset_m=rng.uniform(0.0, 0.9), agl_m=rng.uniform(15.0, 19.5))
    return good(
        state=rng.choice(('tracking', 'tracking', 'lost', 'idle', 'acquiring', 'error')),
        mode=rng.choice(('centered', 'centered', 'slant', None)),
        target_id=rng.choice(('7', '7', '8', None, '')),
        target_class=rng.choice(('car', 'car', 'van', 'person', 'pedestrian', 'people', '', None)),
        target_seen_s=rng.choice((0.1, 0.5, 0.9, 3.0, None, -2.0)),
        target_speed_mps=rng.choice((0.0, 0.3, 0.6, 3.5, None)),
        agl_m=rng.choice((17.0, 7.0, 8.0, 20.0, 21.0, 50.0, None, math.nan)),
        speed_mps=rng.choice((0.0, 0.2, 0.6, 4.0, None, math.inf)),
        tilt_deg=rng.choice((0.0, 3.0, 6.0, 20.0, None)),
        offset_m=rng.choice((0.1, 0.9, 1.1, 5.0, None, -1.0)),
        track_started=rng.choice((-5.0, -5.0, 3.0)))


def random_frame(rng, calm):
    items = [('7', 'car')] if rng.random() < 0.9 else []
    if rng.random() < (0.03 if calm else 0.3):
        items.append((rng.choice(('9', '31', '')), rng.choice(('person', 'pedestrian', 'people',
                                                               'Person', '', None))))
    if rng.random() < 0.2:
        items.append(('12', rng.choice(('truck', 'van', 'bicycle'))))
    return items


def random_launcher_command(rng):
    """Every launcher and other command but a valid fire, valid and not."""
    return rng.choice((
        {'action': 'launcher_arm'}, {'action': 'launcher_arm'}, {'action': 'launcher_disarm'},
        {'action': 'reload', 'darts': rng.choice((0, 3, 6, 7, -1, 'x', 2.5))},
        {'action': 'FIRE', 'target_id': '7'}, {'action': ['fire'], 'target_id': '7'},
        {'action': 'fire_at', 'target_id': '7'}, {'action': 'tag', 'target_id': '7'},
        {'target_id': '7'}, {'action': 'track', 'target_id': '7'}, {'action': 'stop'},
        {'action': None}, {'action': 'launcher_arm', 'target_id': '7', 'fire': True},
    ))


def fuzz(seed, steps, fires=False):
    """A random session of the fire control against the plant. Returns the
    rig and, with fires, the accepted fire commands with what the gates saw
    just before."""
    rng = random.Random(seed)
    calm = rng.random() < 0.5
    rig = Rig(fire_min_interval_s=rng.choice((0.0, 1.0, 2.0)),
              fire_dwell_s=rng.choice((0.0, 0.2)),
              # 0.5 s is too short for the 38 mm draw (0.92 s): those cycles fault
              fire_stroke_timeout_s=3.0 if calm else rng.choice((0.5, 3.0)),
              fire_person_clear_s=rng.choice((0.0, 3.0)) if not calm else 3.0)
    rig.command(0.0, action='reload', darts=rng.choice((3, 6)) if calm else rng.choice((0, 3, 6)))
    if calm:
        rig.command(0.0, action='launcher_arm')
    accepted = []
    t = 0.0
    for _ in range(steps):
        t = round(t + rng.choice((0.0, 0.005, 0.02, 0.02, 0.02, 0.05) + (() if calm else
                                                                          (0.3, 1.2))), 6)
        roll = rng.random()
        if roll < 0.3:
            rig.advance(t)
            if rng.random() < 0.05:  # the launcher found anywhere, even away from rest
                rig.plant.rack = rig.plant.rack_cmd = rng.choice((0.0, 0.02, STROKE))
                rig.plant.trigger = rig.plant.trigger_cmd = rng.choice((0.0, 0.0, 0.3, FIRE))
                rig.advance(t)
            if rng.random() < 0.03:
                rig.fc.on_joint_state(t, None, None)  # a malformed state: ignored
        elif roll < 0.5:
            stamp = t - rng.choice((0.0, 0.1, 0.2, 0.9, 5.0)) if rng.random() < 0.95 else None
            rig.frame(t, random_frame(rng, calm), image_time=stamp)
        elif roll < 0.68:
            rig.tick(t)
        elif roll < 0.76:
            if not calm or rng.random() < 0.2:
                rig.s = random_situation(rng, calm)
        elif roll < 0.78:
            if not calm or rng.random() < 0.1:
                rig.fc.on_ready(t, rng.random() < 0.7)
        elif fires and roll < 0.88:
            status = rig.fc.status(t, rig.s)
            target = rng.choice((rig.s.target_id, rig.s.target_id, '8', '', None))
            before = dict(status, refusal=rig.fc.refusal(t, target, rig.s),
                          release=rig.fc.gates(t, target, rig.s, launcher=False),
                          measured=rig.fc.position(t), armed=rig.fc.armed,
                          estop=rig.plant.estop, since_last=t - rig.fc.last_cycle_end,
                          target_id=rig.s.target_id, track_started=rig.s.track_started)
            rig.command(t, action='fire', target_id=target)
            if rig.fc.last_fire and rig.fc.last_fire['result'] == 'running':
                accepted.append((t, target, before))
        else:
            command = random_launcher_command(rng)
            if calm and command.get('action') == 'launcher_disarm' and rng.random() < 0.9:
                continue
            rig.command(t, **command)
    return rig, accepted


@pytest.mark.parametrize('seed', range(80))
def test_no_sequence_without_a_fire_command_primes_or_fires(seed):
    rig, _ = fuzz(seed, 500)
    assert rig.firing == [] and rig.priming == [] and rig.plant.shots == []
    assert all(trigger == 0.0 for _, _, trigger in rig.commands)
    assert not rig.plant.primed
    assert rig.fc.cycle.phase not in ('priming', 'chambering', 'firing', 'resetting')
    assert rig.fc.last_fire is None


def test_the_fuzz_commands_the_launcher_and_arms_it():
    rigs = [fuzz(seed, 500)[0] for seed in range(80)]
    # The launcher found away from rest is brought back (commands with the
    # rack at 0) without a fire command; and the sessions arm and disarm.
    assert sum(any(rack == 0.0 for _, rack, _ in rig.commands) for rig in rigs) >= 20
    assert sum(rig.fc.armed for rig in rigs) >= 20


def release_held(rig, sent, target_id, track_started):
    """The test's own judgement of the release gates when a command was
    sent, from its record: no frame that came before it showed a person
    within fire_person_clear_s, one came recently, and the situation then
    was good for the target, armed and ready."""
    sequence, t, _, _, s, armed, ready = sent
    config = rig.fc.config
    frames = [item for item in rig.frames if item[0] < sequence]
    if any(person and t - (image if image is not None else seen) <= config.fire_person_clear_s
           for _, seen, image, person in frames):
        return False
    stamped = [image for _, _, image, _ in frames if image is not None]
    if not stamped or not -0.5 <= t - max(stamped) <= config.fire_detection_timeout_s:
        return False
    return (armed and ready and s.state == 'tracking' and s.track_started == track_started
            and s.mode == 'centered' and s.target_id == target_id
            and str(s.target_class or '').strip().lower() not in ('', 'person', 'pedestrian',
                                                                   'people')
            and s.target_seen_s is not None and -0.5 <= s.target_seen_s <= 0.8
            and s.agl_m is not None and 8.0 <= s.agl_m <= 20.0
            and s.speed_mps is not None and s.speed_mps <= 0.5
            and s.tilt_deg is not None and s.tilt_deg <= 5.0
            and s.target_speed_mps is not None and s.target_speed_mps <= 0.5
            and s.offset_m is not None and s.offset_m <= 1.0)


@pytest.mark.parametrize('seed', range(80))
def test_every_prime_and_shot_follows_an_accepted_fire_whose_gates_held(seed):
    rig, accepted = fuzz(1000 + seed, 700, fires=True)
    for t, target, before in accepted:
        assert before['refusal'] == '' and before['release'] == []
        assert before['armed'] and not before['estop'] and before['launcher'] == 'ready'
        assert before['darts'] > 0 and before['fire_block'] == ''
        assert before['measured'] is not None and rig.fc.cycle.at_rest(*before['measured'])
        assert before['since_last'] >= rig.fc.config.fire_min_interval_s
        assert target == before['target_id'] and before['target_class'] not in (
            None, '', 'person', 'pedestrian', 'people')
    starts = [t for t, _, _ in accepted]
    for t in rig.priming + rig.firing + rig.plant.shots:
        assert any(start <= t <= start + CYCLE_BOUND_S for start in starts), t
    assert len(rig.plant.shots) <= len(accepted)
    # Every command that moved the trigger off 0 went out with every release
    # gate holding, by the test's own record (no person in the last 3 s).
    commands = {t: (target, before) for t, target, before in accepted}
    for sent in rig.sent:
        if sent[3] > 0.0:
            start = max(start for start in starts if start <= sent[1])
            target, before = commands[start]
            assert release_held(rig, sent, target, before['track_started']), sent[:4]


def test_the_fuzz_reaches_accepted_fires_and_shots():
    sessions = [fuzz(1000 + seed, 700, fires=True) for seed in range(80)]
    assert sum(bool(accepted) for _, accepted in sessions) >= 15
    assert sum(len(rig.plant.shots) for rig, _ in sessions) >= 8
    # Some cycles were stopped by a person or the gates.
    assert sum(any(record['result'] == 'fault' for record in rig.fc.records) or
               (rig.fc.last_fire or {}).get('result') == 'fault' for rig, _ in sessions) >= 1


# --- The tracker node -----------------------------------------------------------------

GAZEBO = tc.Intrinsics(554.3, 554.3, 320.5, 240.5, 640, 480)


class FakeDem:
    def __init__(self, up=0.0):
        self.up = up

    def height(self, east, north):
        return self.up

    def contains(self, east, north, margin=0.0):
        return True


def quaternion(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    roll, pitch, yaw = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def node_rig(**config):
    """aerodrone_tracker.Tracker without ROS: its own callbacks, a fake
    clock, publishers recording what they send, the stand-in's plant."""
    node = tracker_node.Tracker.__new__(tracker_node.Tracker)
    node.config = tc.TrackerConfig()
    node.core = tc.TrackerCore(node.config)
    node.launcher = lc.FireControl(lc.LauncherConfig(**dict({'launcher_fitted': True}, **config)))
    node.launcher_fitted = True
    node.auto_next = tracker_node.PARAMETERS['auto_next']
    node.auto_next_command = None
    node.tagged_ids = set()
    node.last_detections = None
    node.lock = threading.RLock()
    node.poses = tc.PoseHistory()
    node.terrain = tracker_node.Terrain()
    node.terrain.dem = FakeDem(0.0)
    node.bias_hold_s, node.rangefinder_timeout_s = 30.0, 0.5
    node.latest_range = None
    node.intrinsics, node.mount_yaw, node.stabilized = GAZEBO, 0.0, False
    node.camera_offset = (0.137, 0.0, -0.08)
    node.origin, node.dem_loading, node.dem_checked, node.dem_entry = None, False, 0.0, None
    node.estop_published = None
    node.status_block, node.status_block_time = None, -math.inf
    node.platform = NS(status=NS(state=3), offboard=True)
    node.behaviors, node.busy_names, node.preempt_names = {}, [], []
    node.following = False
    node.follow_handle = None
    node._execute = lambda actions, hover_reason='stopped': None
    node.clock = 100.0
    node.now = lambda: node.clock
    out = NS(commands=[], estop=[], shots=[], status=[], logs=[])
    plant = standin.LauncherPlant()

    def joint_command(msg):
        joints = dict(zip(msg.name, msg.position))
        out.commands.append((node.clock, joints))
        plant.command(node.clock, joints.get('actuator_joint'), joints.get('trigger_joint'))

    def estop(msg):
        out.estop.append(msg.data)
        plant.emergency_stop(msg.data)

    node.joint_command_pub = NS(publish=joint_command)
    node.estop_pub = NS(publish=estop)
    node.shots_pub = NS(publish=lambda msg: out.shots.append(json.loads(msg.data)))
    node.status_pub = NS(publish=lambda msg: out.status.append(json.loads(msg.data)))
    node.JointState = lambda: NS(header=NS(stamp=None), name=[], position=[])
    node.Bool = lambda data: NS(data=data)
    node.String = lambda data: NS(data=data)
    logger = NS(info=out.logs.append, warn=out.logs.append)
    node.node = NS(get_logger=lambda: logger,
                   get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None)))
    node.out, node.plant = out, plant
    node._publish_estop()
    return node


def stamp(t):
    return NS(sec=int(t), nanosec=int(round((t % 1) * 1e9)))


def detections(t, *items):
    import tracking_fakes as fakes
    frame = fakes.Detection2DArray()
    frame.header.stamp.sec, frame.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
    for track_id, cls, (u, v) in items:
        item = fakes.Detection2D(id=track_id)
        item.bbox.center.position.x, item.bbox.center.position.y = u, v
        item.bbox.size_x = item.bbox.size_y = 60.0
        item.results = [fakes.ObjectHypothesisWithPose()]
        item.results[0].hypothesis.class_id = cls
        frame.detections.append(item)
    return frame


def node_step(node, t, frame_items=None, pose=((0.0, 0.0, 17.0), (0, 0, 0))):
    """Time t: odometry, the plant, joint states and readiness, a frame,
    the launcher's tick."""
    node.clock = t
    position, angles = pose
    node.poses.add(int(round(t * 1e9)), position, quaternion(*angles))
    node.plant.update(t, max(0.0, t - getattr(node, 'plant_t', t)))
    node.plant_t = t
    node._on_launcher_joints(NS(name=['actuator_joint', 'trigger_joint'],
                                position=[node.plant.rack, node.plant.trigger],
                                header=NS(stamp=stamp(t))))
    node._on_launcher_ready(NS(data=not node.plant.estop))
    if frame_items is not None:
        node._on_detections(detections(t, *frame_items))
    node._launcher_tick()


# The car under the bore: seen from the camera 137 mm ahead of the hub (the
# image's top is the nose), below the image centre.
CAR_BELOW = ('7', 'car', (320.5, 240.5 + 554.3 * 0.137 / 16.92))


def tracking_node(**config):
    """The node tracking the car '7' centered at 17 m, hovering over it,
    reloaded and armed: every gate holds."""
    node = node_rig(**config)
    for k in range(10):
        node_step(node, 99.0 + 0.1 * k)
    node._on_command(NS(data=json.dumps({'action': 'track', 'target_id': '7',
                                         'mode': 'centered', 'altitude_agl_m': 17})))
    assert node.core.state == 'acquiring'
    node._on_command(NS(data='{"action": "reload", "darts": 6}'))
    node._on_command(NS(data='{"action": "launcher_arm"}'))
    t = 100.0
    for k in range(30):
        t = round(100.0 + 0.1 * k, 6)
        node_step(node, t, [CAR_BELOW])
    assert node.core.state == 'tracking' and node.core.target_class == 'car'
    node.clock = t
    situation = node._situation(t)
    assert node.launcher.refusal(t, '7', situation) == '', node.launcher.gates(t, '7', situation)
    return node, t



def test_drone_auto_next_selects_nearest_untagged_vehicle_without_firing():
    node, t = tracking_node()
    node.auto_next_command = node.core.command
    node.tagged_ids = {'7', 'done'}
    node.last_detections = detections(t, CAR_BELOW,
        ('person', 'person', (320.5, 240.5)),
        ('done', 'car', (320.5, 241)),
        ('far', 'car', (400, 240.5)),
        ('near', 'car', (340, 240.5)))
    node._next_after_shot(t, '7')
    assert node.core.command.target_id == 'near'
    assert node.core.command.mode == 'centered'
    assert not node.launcher.cycle.running and not node.out.shots


def test_simulated_drone_operator_fires_once_per_centered_target_and_advances():
    from simulation_trigger import SimulationTrigger
    node, start = tracking_node()
    operator = SimulationTrigger()
    second = ('8', 'car', (340.5, 244.99))
    for step in range(301):
        now = round(start + step * .05, 6)
        node_step(node, now, [CAR_BELOW, second])
        node._publish_status()
        command = operator.command(node.out.status[-1], now)
        if command is not None:
            node._on_command(NS(data=json.dumps(command)))
    fired = [shot['target_id'] for shot in node.out.shots if shot['result'] == 'fired']
    assert fired == ['7', '8']
    assert node.tagged_ids == {'7', '8'}


@pytest.mark.parametrize('change', ['stale', 'stopped', 'reselected', 'disarmed'])
def test_drone_auto_next_does_not_override_operator_or_use_old_frame(change):
    node, t = tracking_node()
    node.auto_next_command = node.core.command
    node.tagged_ids = {'7'}
    node.last_detections = detections(t, ('next', 'car', (340, 240.5)))
    if change == 'stale':
        node.last_detections.header.stamp = stamp(t - 2)
    elif change == 'stopped':
        node._on_command(NS(data='{"action":"stop"}'))
    elif change == 'reselected':
        node._on_command(NS(data='{"action":"track","target_id":"chosen","mode":"centered","altitude_agl_m":17}'))
    else:
        node._on_command(NS(data='{"action":"launcher_disarm"}'))
    expected = node.core.command
    node._next_after_shot(t, '7')
    assert node.core.command is expected
    assert not node.launcher.cycle.running and not node.out.shots

def test_the_node_routes_the_launchers_commands_to_the_fire_control_alone():
    node = node_rig()
    for text in ('{"action": "launcher_arm"}', '{"action": "fire", "target_id": "7"}',
                 '{"action": "reload", "darts": 2}', '{"action": "launcher_disarm"}'):
        node._on_command(NS(data=text))
        # The tracker neither rejects them nor changes state.
        assert (node.core.state, node.core.message) == ('idle', 'waiting for a command'), text
    assert node.out.estop == [True, False, True]  # disarmed at start, armed, disarmed
    assert node.launcher.last_fire['result'] == 'refused'
    assert node.launcher.last_fire['reason'] == 'launcher not ready (no servo bridge, or it ' \
                                                'reports not ready)'
    assert node.out.status[-1]['darts'] == 0  # reloading needs it disarmed
    assert node.out.status[-1]['fire_block'] == 'launcher disarmed'
    assert node.out.commands == [(100.0, {'trigger_joint': 0.0})]  # disarm's safe command
    # Unknown actions are still the tracker's to reject.
    node._on_command(NS(data='{"action": "FIRE", "target_id": "7"}'))
    assert node.core.message.startswith('rejected command: unknown action')


def test_the_node_fires_one_dart_at_the_car_below():
    node, t = tracking_node()
    node._on_command(NS(data='{"action": "fire", "target_id": "7"}'))
    assert node.launcher.last_fire['result'] == 'running'
    for k in range(1, 60):
        node_step(node, round(t + 0.05 * k, 6), [CAR_BELOW])
    assert len(node.plant.shots) == 1 and node.launcher.shots == 1
    [shot] = node.out.shots
    assert shot['target_id'] == '7' and shot['target_class'] == 'car' and shot['result'] == 'fired'
    assert shot['at_command']['agl_m'] == pytest.approx(17.0, abs=0.01)
    assert shot['at_shot']['offset_m'] < 0.2
    assert node.out.status[-1]['last_fire']['result'] == 'fired'


def test_the_node_situation_from_odometry_terrain_and_the_target():
    node, t = tracking_node()
    situation = node._situation(t)
    assert situation.agl_m == pytest.approx(17.0)
    assert situation.tilt_deg == pytest.approx(0.0, abs=1e-6)
    assert situation.speed_mps == pytest.approx(0.0, abs=1e-6)
    assert situation.bore_ground == pytest.approx((0.0, 0.0, 0.0), abs=0.06)
    # The camera's offset: the car seen at the image's centre-left is under the bore.
    assert situation.target[:2] == pytest.approx((0.0, 0.0), abs=0.15)
    assert situation.offset_m < 0.15
    # Rolled 5 deg right: the bore's ground point moves 1.5 m left of the hub.
    node.poses.add(int(round((t + 0.1) * 1e9)), (0.0, 0.0, 17.0), quaternion(5.0, 0.0, 0.0))
    node.clock = t + 0.1
    rolled = node._situation(t + 0.1)
    assert rolled.tilt_deg == pytest.approx(5.0)
    assert rolled.bore_ground[1] == pytest.approx((17.0 - 0.3065) * math.tan(math.radians(5.0)),
                                                  abs=0.06)
    # No DEM and no rangefinder: the height is not known.
    node.terrain.dem = None
    assert node._situation(t + 0.1).agl_m is None
    # A fresh rangefinder reading gives it.
    node.latest_range = (NS(range=16.0, min_range=0.1, max_range=40.0), t + 0.1)
    assert node._situation(t + 0.1).agl_m == pytest.approx(16.0 * math.cos(math.radians(5.0)))
    # Stale odometry: nothing about the aircraft.
    node.clock = t + 2.0
    stale = node._situation(t + 2.0)
    assert (stale.speed_mps, stale.tilt_deg, stale.agl_m, stale.offset_m) == (None,) * 4


def test_the_target_is_geolocated_from_the_camera_not_the_hub():
    pose = tc.Pose((0.0, 0.0, 17.0), quaternion(), 0, 0.0)
    flat = lambda east, north: 0.0  # noqa: E731
    centre = tc.geolocate(320.5, 240.5, (60, 60), pose, GAZEBO, flat, 0.0, False,
                          tc.TrackerConfig(), camera_offset=(0.137, 0.0, -0.08))
    assert centre.position[:2] == pytest.approx((0.137, 0.0), abs=1e-6)
    assert centre.range_m == pytest.approx(17.0 - 0.08, abs=0.05)
    hub = tc.geolocate(320.5, 240.5, (60, 60), pose, GAZEBO, flat, 0.0, False, tc.TrackerConfig())
    assert hub.position[:2] == pytest.approx((0.0, 0.0), abs=1e-6)
    turned = tc.Pose((0.0, 0.0, 17.0), quaternion(yaw_deg=90.0), 0, 0.0)
    north = tc.geolocate(320.5, 240.5, (60, 60), turned, GAZEBO, flat, 0.0, False,
                         tc.TrackerConfig(), camera_offset=(0.137, 0.0, -0.08))
    assert north.position[:2] == pytest.approx((0.0, 0.137), abs=1e-6)


def test_the_node_stops_a_cycle_on_a_person_a_command_or_a_pre_emption():
    def started():
        node, t = tracking_node()
        node._on_command(NS(data='{"action": "fire", "target_id": "7"}'))
        for k in range(1, 10):
            node_step(node, round(t + 0.05 * k, 6), [CAR_BELOW])
        assert node.launcher.cycle.phase == 'priming'
        return node, round(t + 0.5, 6)

    for cause in ('person', 'stop', 'track', 'preempt'):
        node, t = started()
        sent = len(node.out.commands)
        node.clock = t
        if cause == 'person':
            node._on_detections(detections(t, CAR_BELOW, ('9', 'person', (100.0, 100.0))))
        elif cause == 'preempt':
            node._preempt('a map goal (1;2;3)')
        else:
            text = {'stop': '{"action": "stop"}',
                    'track': '{"action": "track", "target_id": "7", "mode": "centered"}'}[cause]
            node._on_command(NS(data=text))
        assert node.launcher.cycle.phase == 'fault', cause
        assert node.out.commands[sent:][:1] == [(t, {'trigger_joint': 0.0})], cause
        for k in range(1, 80):
            node_step(node, round(t + 0.05 * k, 6), [CAR_BELOW])
        assert node.plant.shots == [] and node.plant.trigger == 0.0, cause


def test_shutdown_makes_the_launcher_safe():
    node, t = tracking_node()
    node._on_command(NS(data='{"action": "fire", "target_id": "7"}'))
    node_step(node, t + 0.05, [CAR_BELOW])
    node.shutdown()
    assert node.out.commands[-1][1] == {'trigger_joint': 0.0}
    assert node.out.estop[-1] is True and not node.launcher.armed
    assert node.launcher.cycle.phase == 'fault'
    # main() calls it before the node goes.
    main = next(item for item in ast.parse(TRACKER.read_text()).body
                if isinstance(item, ast.FunctionDef) and item.name == 'main')
    final = next(item for item in ast.walk(main) if isinstance(item, ast.Try)).finalbody
    assert [ast.unparse(statement) for statement in final[:3]] == [
        'tracker.shutdown()', 'time.sleep(0.2)', 'node.destroy_node()']
    # ... with the ROS context still up: SIGINT and SIGTERM end the spin
    # without rclpy shutting the context down first.
    source = ast.unparse(main)
    assert 'rclpy.init(signal_handler_options=SignalHandlerOptions.NO)' in source
    assert 'for number in (signal.SIGINT, signal.SIGTERM):' in source


def test_the_node_status_carries_the_launcher_and_publishes_its_changes():
    node, t = tracking_node()
    node._publish_status()
    status = node.out.status[-1]
    assert status['fire_block'] == '' and all(status['fire_checks'].values())
    assert status['state'] == 'tracking' and status['target_class'] == 'car'
    # A person appears: the status follows within the launcher's tick.
    count = len(node.out.status)
    node_step(node, t + 0.3, [CAR_BELOW, ('9', 'person', (50.0, 50.0))])
    assert len(node.out.status) > count
    assert node.out.status[-1]['fire_block'].startswith('a person is in view (#9 person)')


def node_fuzz(seed, steps):
    """Random callbacks of the node: commands (all but fire), frames with
    and without people, odometry (hovering or not), joint states, readiness,
    ticks."""
    rng = random.Random(seed)
    node = node_rig()
    t = 99.0
    texts = ['{"action": "launcher_arm"}', '{"action": "launcher_disarm"}',
             '{"action": "reload", "darts": 6}', '{"action": "reload", "darts": 9}',
             '{"action": "track", "target_id": "7", "mode": "centered", "altitude_agl_m": 17}',
             '{"action": "track", "target_id": "9", "mode": "centered"}',
             '{"action": "track", "target_id": "7", "mode": "slant", "slant_range_m": 60, '
             '"depression_deg": 70}', '{"action": "stop"}', '{"action": "FIRE"}', 'fire',
             '{"action": "fire_now", "target_id": "7"}', '{"target_id": "7"}', 'null', '[]']
    for _ in range(steps):
        t = round(t + rng.choice((0.0, 0.02, 0.05, 0.1, 0.1, 0.5)), 6)
        roll = rng.random()
        if roll < 0.45:
            items = [CAR_BELOW] if rng.random() < 0.9 else []
            if rng.random() < 0.15:
                items.append(('9', rng.choice(('person', 'pedestrian', '')), (50.0, 60.0)))
            pose = ((rng.choice((0.0, 0.0, 3.0)), 0.0, rng.choice((17.0, 17.0, 40.0))),
                    (rng.choice((0.0, 0.0, 8.0)), 0.0, 0.0))
            node_step(node, t, items if rng.random() < 0.9 else None, pose)
        elif roll < 0.55:
            node.clock = t
            node._launcher_tick()
        elif roll < 0.6:
            node.plant.rack = node.plant.rack_cmd = rng.choice((0.0, 0.02))
            node.plant.trigger = node.plant.trigger_cmd = rng.choice((0.0, 0.3))
        elif roll < 0.62:
            node.clock = t
            node._on_launcher_ready(NS(data=rng.random() < 0.5))
        elif roll < 0.63:
            node.clock = t
            node._preempt('a map goal')
        else:
            node.clock = t
            node._on_command(NS(data=rng.choice(texts)))
    return node


@pytest.mark.parametrize('seed', range(12))
def test_no_sequence_of_node_callbacks_without_a_fire_command_primes_or_fires(seed):
    node = node_fuzz(seed, 400)
    assert node.plant.shots == [] and not node.plant.primed
    assert all(joints.get('trigger_joint', 0.0) == 0.0 for _, joints in node.out.commands)
    assert all(joints.get('actuator_joint', 0.0) <= 0.0 for _, joints in node.out.commands)
    assert node.launcher.last_fire is None and node.out.shots == []


def test_the_node_fuzz_reaches_tracking_armed_and_returns_the_launcher():
    nodes = [node_fuzz(seed, 400) for seed in range(12)]
    assert sum(any(status.get('state') == 'tracking' and status.get('launcher_armed')
                   for status in node.out.status) for node in nodes) >= 3
    assert sum(bool(node.out.commands) for node in nodes) >= 3


# --- Structure: priming and firing only from the fire command ---------------------------

def functions_of(path):
    tree = ast.parse(path.read_text())
    functions = {}
    for item in tree.body:
        if isinstance(item, ast.ClassDef):
            for member in item.body:
                if isinstance(member, ast.FunctionDef):
                    functions[(item.name, member.name)] = member
        elif isinstance(item, ast.FunctionDef):
            functions[('<module>', item.name)] = item
    return tree, functions


def where(functions, tree, predicate):
    found = [(key) for key, function in functions.items()
             for node in ast.walk(function) if predicate(node)]
    top = [item for item in tree.body if not isinstance(item, (ast.ClassDef, ast.FunctionDef))]
    found += [('<module>', '') for item in top for node in ast.walk(item) if predicate(node)]
    return found


def calls(attribute):
    return lambda node: isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
        and node.func.attr == attribute


def test_priming_and_firing_are_reachable_only_from_the_fire_command():
    tree, functions = functions_of(LAUNCHER_CORE)
    # FireCycle.start, the only way into priming, only from _fire ...
    assert where(functions, tree, calls('start')) == [('FireControl', '_fire')]
    # ... after the gates refused nothing ...
    statements = [ast.unparse(item) for item in functions[('FireControl', '_fire')].body]
    refusal = statements.index('refusal = self.refusal(now, target_id, s)')
    assert statements[refusal + 1].startswith('if refusal:') and \
        statements[refusal + 1].rstrip().splitlines()[-1].strip().startswith('return')
    assert refusal < next(i for i, text in enumerate(statements) if '.start(' in text)
    # ... _fire only from handle(), for the action "fire" ...
    assert where(functions, tree, calls('_fire')) == [('FireControl', 'handle')]
    branch = next(node for node in ast.walk(functions[('FireControl', 'handle')])
                  if isinstance(node, ast.If) and ast.unparse(node.test) == "action == 'fire'")
    assert [ast.unparse(item) for item in branch.body] == ['return self._fire(command, now, s)']
    # ... the cycle advances only in tick(), told whether the release is clear ...
    updates = where(functions, tree, lambda node: isinstance(node, ast.Call) and
                    ast.unparse(node.func) == 'cycle.update')
    assert updates == [('FireControl', 'tick')]
    update = next(node for node in ast.walk(functions[('FireControl', 'tick')])
                  if isinstance(node, ast.Call) and ast.unparse(node.func) == 'cycle.update')
    assert [ast.unparse(keyword) for keyword in update.keywords] == [
        'clear=self.release_clear(now, s)']
    # ... the joint goals come from goals() in joint_goals() alone, the
    # trigger zeroed there while the release is not clear ...
    assert where(functions, tree, calls('goals')) == [('FireControl', 'joint_goals')]
    goals = ast.unparse(functions[('FireControl', 'joint_goals')])
    assert ('rack, trigger = cycle.goals(now)\n'
            '    if trigger > 0.0 and (not self.release_clear(now, s)):\n'
            '        trigger = 0.0') in goals
    # ... and nothing outside FireCycle sets its phase.
    assert [key for key in where(functions, tree, lambda node: isinstance(node, ast.Assign) and
                                 any(isinstance(element, ast.Attribute) and element.attr == 'phase'
                                     for target in node.targets
                                     for element in (target.elts if isinstance(target, ast.Tuple)
                                                     else [target])))
            if key[0] != 'FireCycle'] == []
    # The tracker: handle() only from the command callback, for the
    # launcher's actions; the callback only from its subscription.
    tree, functions = functions_of(TRACKER)
    assert where(functions, tree, calls('handle')) == [('Tracker', '_on_command')]
    branch = next(node for node in ast.walk(functions[('Tracker', '_on_command')])
                  if isinstance(node, ast.If) and
                  ast.unparse(node.test) == 'launcher_command is not None')
    assert any('self.launcher.handle(launcher_command, now, situation)' in ast.unparse(item)
               for item in branch.body)
    assert where(functions, tree, lambda node: isinstance(node, ast.Attribute) and
                 node.attr == '_on_command') == [('Tracker', '__init__')]
    # No cycle start(), no phase, no goals() in the node: joint commands
    # only from joint_goals() (in _launcher_outputs) and shutdown's safe
    # command.
    assert where(functions, tree, lambda node: calls('start')(node) and
                 'cycle' in ast.unparse(node.func)) == []
    assert where(functions, tree, lambda node: isinstance(node, ast.Attribute) and
                 node.attr in ('phase', 'cycle') and isinstance(node.ctx, ast.Store)) == []
    assert where(functions, tree, calls('goals')) == []
    assert where(functions, tree, calls('joint_goals')) == [('Tracker', '_launcher_outputs')]
    assert where(functions, tree, lambda node: isinstance(node, ast.Call) and
                 ast.unparse(node.func) == 'self.joint_command_pub.publish') == [
        ('Tracker', '_send_launcher')]
    senders = [(key, ast.unparse(node)) for key, function in functions.items()
               for node in ast.walk(function) if calls('_send_launcher')(node)]
    assert senders == [(('Tracker', '_launcher_outputs'), 'self._send_launcher(goals)'),
                       (('Tracker', 'shutdown'), 'self._send_launcher((None, 0.0))')]


# --- The bridge: the launcher alone on the bus -------------------------------------------

@pytest.fixture(scope='module')
def bridge_module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('diagnostic_msgs', 'diagnostic_msgs.msg', 'rclpy.executors'):
            sys.modules[name] = MagicMock()
        sys.modules['serial'] = None
        module = load_script(REPO / 'applications/pantiltros/scripts/turret_arduino_bridge.py',
                             'turret_arduino_bridge_launcher_only')
        module.Bool = lambda data: NS(data=data)
        module.KeyValue = lambda key, value: NS(key=key, value=value)
        module.DiagnosticStatus = MagicMock(OK=0, WARN=1, ERROR=2)
        module.DiagnosticArray = lambda: NS(header=NS(stamp=None), status=[])
        yield module


def launcher_only(bridge_module):
    import test_turret_bridge_sts as sts
    port = sts.FakeServoPort(launcher=True)
    del port.servos[sts.PAN], port.servos[sts.TILT]  # no pan and tilt servos at all
    config = sts.turret_config(bridge_module, actuator=True)._replace(pan_tilt_fitted=False)
    out = NS(positions=[], errors=[])
    controller = bridge_module.StsController(
        config, bridge_module.StsBus(port, sts.REPLY_TIMEOUT_S),
        lambda *position: out.positions.append(position), out.errors.append)
    return sts, port, controller, out


def addressed(port):
    """The servo IDs the packets sent were for (sync writes: their entries)."""
    return {packet[2] for packet in port.sent if packet[2] != 0xFE} | {
        packet[at] for packet in port.sent if packet[2] == 0xFE
        for at in range(7, len(packet) - 1, 8)}


def test_the_bridge_drives_the_launcher_alone(bridge_module):
    sts, port, controller, out = launcher_only(bridge_module)
    controller.update(0.0)
    assert out.positions == [(None, None, 0.0, 0.0)] and out.errors == []
    assert controller.have_feedback and controller.have_launcher
    assert controller.set(0.0, 0.0, 0.01, actuator=STROKE, trigger=0.0)
    assert port.servos[sts.RACK].torque and port.servos[sts.TRIGGER].torque
    assert (port.servos[sts.RACK].torque_limit, port.servos[sts.TRIGGER].torque_limit) == (550, 300)
    assert port.servos[sts.RACK].goal == sts.rack_steps(bridge_module, STROKE)
    # Without the rack the trigger returns to 0 and the rack stops where it is.
    port.servos[sts.RACK].present = sts.rack_steps(bridge_module, 0.02)
    port.servos[sts.TRIGGER].present = sts.trigger_steps(bridge_module, 0.3)
    controller.set(0.0, 0.0, 0.1)
    assert port.servos[sts.RACK].goal == sts.rack_steps(bridge_module, 0.02)
    assert port.servos[sts.TRIGGER].goal == sts.trigger_steps(bridge_module, 0.0)
    # The watchdog and STOP do the same.
    controller.set(0.0, 0.0, 0.2, actuator=STROKE, trigger=FIRE)
    controller.update(0.75)
    assert controller.state == bridge_module.StsController.HOLDING
    assert port.servos[sts.TRIGGER].goal == sts.trigger_steps(bridge_module, 0.0)
    assert port.servos[sts.RACK].goal == sts.rack_steps(bridge_module, 0.02)
    controller.stop(0.8)
    assert out.errors == []
    # Nothing was ever sent to the pan and tilt IDs.
    assert addressed(port) <= {sts.RACK, sts.TRIGGER}


def test_a_silent_launcher_servo_reports_no_position(bridge_module):
    sts, port, controller, out = launcher_only(bridge_module)
    port.servos[sts.TRIGGER].responds = False
    controller.update(0.04)
    assert out.positions == [] and not controller.have_feedback
    assert out.errors == ['servo 15 (trigger_joint) timeout']


def test_the_bridge_node_takes_launcher_commands_without_pan_and_tilt(bridge_module):
    import test_turret_bridge_sts as sts
    port = sts.FakeServoPort(launcher=True)
    del port.servos[sts.PAN], port.servos[sts.TILT]
    node = bridge_module.TurretArduinoBridge.__new__(bridge_module.TurretArduinoBridge)
    node.get_logger = MagicMock()
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'stamp'))
    node.transport, node.serial_port, node.baud_rate, node.dry_run = \
        'sts', '/host/dev/serial/by-id/usb-adapter', 1000000, False
    node.pan_min_rad, node.pan_max_rad, node.tilt_min_rad, node.tilt_max_rad = -1.57, 1.57, -0.65, 0.75
    node.actuator_fitted, node.pan_tilt_fitted = True, False
    node.actuator_min_m, node.actuator_max_m = 0.0, STROKE
    node.trigger_min_rad, node.trigger_max_rad = 0.0, FIRE
    node.sts_config = sts.turret_config(bridge_module, actuator=True)._replace(
        pan_tilt_fitted=False)
    node.servo_reply_timeout_s = sts.REPLY_TIMEOUT_S
    node.servos = node.serial_handle = None
    node.serial_ready_time, node.last_connect_attempt = 0.0, float('-inf')
    node.reconnect_interval_s = node.connect_settle_s = 2.0
    node.last_error, node.rx_buffer = '', b''
    node.last_write_time = node.last_feedback_time = None
    node.estop_engaged, node.stop_sent, node.dropped_commands, node.last_ready = True, False, 0, None
    node.launcher_position, node.ignored_actuator_commands = None, 0
    node.joint_state_pub, node.ready_pub, node.diagnostics_pub = MagicMock(), MagicMock(), \
        MagicMock()
    node.opened = []
    bridge_module.serial = NS(Serial=lambda path, baud, **options: (
        node.opened.append((path, baud, options)), port)[1])
    bridge_module.JointState = sts.RclpyJointState
    try:
        node._poll()
        assert node.opened[0][:2] == ('/host/dev/serial/by-id/usb-adapter', 1000000)
        state = node.joint_state_pub.publish.call_args[0][0]
        assert list(state.name) == ['actuator_joint', 'trigger_joint']
        # Engaged at start: STOP (the rack held, the trigger at 0) and
        # commands dropped.
        assert port.servos[sts.TRIGGER].goal == sts.trigger_steps(bridge_module, 0.0)
        writes = port.servos[sts.RACK].goal_writes
        command = NS(name=['actuator_joint', 'trigger_joint'], position=[STROKE, 0.0])
        node._joint_command(command)
        assert node.dropped_commands == 1 and port.servos[sts.RACK].goal_writes == writes
        node._emergency_stop(NS(data=False))
        node._joint_command(command)
        assert node.last_error == '' and port.servos[sts.RACK].goal == sts.rack_steps(
            bridge_module, STROKE)
        node._joint_command(NS(name=['trigger_joint'], position=[0.0]))
        assert addressed(port) <= {sts.RACK, sts.TRIGGER}
    finally:
        bridge_module.serial = None


def test_launcher_only_needs_the_sts_transport_and_the_launcher(bridge_module):
    import test_turret_bridge_sts as sts
    with pytest.raises(ValueError):
        sts.turret_config(bridge_module, actuator=False)._replace(pan_tilt_fitted=False).validate()
    source = (REPO / 'applications/pantiltros/scripts/turret_arduino_bridge.py').read_text()
    assert '"pan_tilt_fitted false (the launcher alone) needs the sts transport "' in source


def test_the_aircraft_bridge_settings_are_the_turrets_launcher():
    ours = yaml.safe_load((REPO / 'systems/aerodrone/config/launcher_bridge.yaml').read_text())[
        '/**']['ros__parameters']
    turret = yaml.safe_load((REPO / 'applications/pantiltros/config/arduino_bridge.yaml')
                            .read_text())['turret_arduino_bridge']['ros__parameters']
    for key, value in turret.items():
        if key.startswith(('actuator_', 'trigger_', 'servo_')) and not key.endswith('_topic'):
            assert ours[key] == value, key
    assert (ours['transport'], ours['baud_rate'], ours['actuator_fitted'],
            ours['pan_tilt_fitted'], ours['require_estop_release']) == (
        'sts', 1000000, True, False, True)
    for key in ('joint_command_topic', 'joint_state_topic', 'emergency_stop_topic',
                'actuator_ready_topic'):
        assert ours[key].startswith('/aerodrone/launcher/'), key
    assert tracker_node.PARAMETERS['launcher_topic_prefix'] == '/aerodrone/launcher'


# --- The stand-in -------------------------------------------------------------------------

def test_the_stand_in_moves_as_the_turret_simulators_launcher():
    turret_sim = load_script(REPO / 'applications/turret_sim/scripts/turret_sim.py',
                             'turret_sim_for_standin')
    scenario = json.loads((REPO / 'applications/turret_sim/scenarios/warehouse_tracking.json')
                          .read_text())
    sim = turret_sim.TurretSim(scenario, launcher=True)
    plant = standin.LauncherPlant()
    plant.emergency_stop(False)
    def goals(t):
        """A fire cycle's commands, every 0.1 s, then a rack stopped
        mid-draw, then silence (the watchdog)."""
        if t < 1.2:
            return STROKE, 0.0
        if t < 2.3:
            return 0.0, 0.0
        if t < 2.6:
            return 0.0, FIRE
        if t < 3.5:
            return 0.0, 0.0
        if t < 3.6:
            return None, 0.0
        if t < 4.0:
            return STROKE, 0.0
        return None

    t, dt = 0.0, 0.02
    while t < 6.0:
        t = round(t + dt, 6)
        command = goals(t)
        if command is not None and round(t * 50) % 5 == 0:
            sim.launcher_queue.append((t, *command))
            plant.command(t, *command)
        sim.update_launcher(t, dt, False)
        plant.update(t, dt)
        assert (plant.rack, plant.trigger, plant.primed) == pytest.approx(
            (sim.rack, sim.trigger, sim.primed)), t
    assert plant.shots == sim.shots and len(plant.shots) == 1


def test_the_stand_in_is_the_bridge_toward_the_tracker():
    plant = standin.LauncherPlant()
    assert not plant.command(0.0, STROKE, 0.0)  # dropped until the emergency stop is released
    plant.update(0.5, 0.5)
    assert plant.rack == 0.0
    plant.emergency_stop(False)
    assert plant.command(0.6, STROKE, 0.0)
    plant.update(0.9, 0.3)
    rack = plant.rack
    assert 0.0 < rack < STROKE
    plant.emergency_stop(True)  # STOP: the rack stopped, the trigger to 0
    plant.update(1.2, 0.3)
    assert plant.rack == pytest.approx(rack) and plant.trigger == 0.0
    plant.emergency_stop(False)
    plant.command(1.3, 1.0, 5.0)  # clamped to the stroke and the fire angle
    assert (plant.rack_cmd, plant.trigger_cmd) == (STROKE, FIRE)
    assert not plant.command(1.4, math.nan, 0.0)
    plant.update(1.85, 0.55)  # 0.55 s without commands: the watchdog
    assert plant.trigger_cmd == 0.0 and plant.rack_cmd == plant.rack
    with pytest.raises(ValueError):
        standin.LauncherPlant(stroke=0.05)


# --- The simulators: halting a target -----------------------------------------------------

def test_a_halted_target_stops_where_it_is_and_goes_on_from_there():
    car = {'name': 'car_1', 'class': 'car', 'model': 'm', 'model_dir': 'd', 'mesh': 'a',
           'mesh_npz': 'b', 'texture': 'c', 'speed_mps': 2.0, 'length_m': 100.0,
           'columns': ['s', 'east', 'north', 'up', 'yaw', 'pitch', 'roll'],
           'samples': [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [50.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0]]}
    person = dict(car, name='person_1', **{'class': 'person'})
    tracks = tt.Tracks({'schema_version': tt.SCHEMA_VERSION,
                        'navigation_origin_enu_m': [0.0, 0.0, 0.0], 'targets': [car, person]})
    east = lambda t, name='car_1': next(  # noqa: E731
        pose.east for pose in tracks.poses(t) if pose.name == name)
    assert east(5.0) == pytest.approx(10.0)
    tracks.halt('car_1')
    assert east(6.0) == pytest.approx(12.0)  # halted at the first poses() after the message
    assert east(9.0) == pytest.approx(12.0) and east(9.0, 'person_1') == pytest.approx(18.0)
    message = tracks.truth_message(9.0)
    assert [item.get('halted') for item in message['targets']] == [True, None]
    tracks.halt('')
    assert east(10.0) == pytest.approx(12.0)
    assert east(11.0) == pytest.approx(14.0)  # on from where it stopped
    assert 'halted' not in tracks.truth_message(11.0)['targets'][0]
    tracks.halt(' car_1 , person_1 ')
    tracks.poses(12.0)
    assert east(1.0) == pytest.approx(2.0)  # a world reset starts the routes again
    assert tt.HALT_TOPIC == '/simulation/targets/halt'
    for path in ('applications/gazebo/scripts/gazebo_tracking_targets.py',
                 'applications/isaac/scripts/aerodrone_navsim.py'):
        source = (REPO / path).read_text()
        assert 'HALT_TOPIC' in source and 'tracks.halt(msg.data)' in source, path
        assert 'tracks.hidden, tracks.halted)' in source, path


# --- The simulators' fire scenario (tools/tracking_sim_test.py) ---------------------------

sim_test = load_script(REPO / 'tools/tracking_sim_test.py', 'tracking_sim_test_fire')
ALL_CHECKS = {gate: True for gate in lc.GATES}


def fire_rows(name, expect, last_fire, shots=(), sim_shots=(), checks=None, held=True,
              block='', launcher_max=(0.0, 0.0)):
    """A fire step's recording: the command (with the gates seen) at 10 s,
    the outcome at 15 s."""
    step = {'name': name, 'kind': 'fire', 'target': 'car_1', 'expect': expect,
            'reason': {'fire_person': 'never at people',
                       'fire_person_in_view': 'a person is in view'}.get(name)}
    rows = [{'k': 'row', 'sim': 10.0, 'step': name,
             'event': {'command': {'action': 'fire', 'target_id': '7'}, 'gates_held': held,
                       'fire_block': block, 'fire_checks': checks or dict(ALL_CHECKS),
                       'agl_m': 17.0, 'bore_offset_m': 0.2}},
            {'k': 'row', 'sim': 15.0, 'step': name,
             'event': {'fired': 'car_1', 'last_fire': last_fire, 'shots': list(shots),
                       'sim_shots': list(sim_shots), 'launcher_max': list(launcher_max)}}]
    return rows, step


def sim_shot(at, drone, quaternion_=(0.0, 0.0, 0.0, 1.0), car=(0.0, 0.0, 100.0), yaw=0.0):
    return {'sim': at, 'shot': {'stamp': at, 'shot': 1}, 'truth': list(drone),
            'quaternion': list(quaternion_), 'att': [0.0, 0.0],
            'targets': [{'name': 'car_1', 'east': car[0], 'north': car[1], 'up': car[2],
                         'yaw': yaw}]}


def test_the_fire_scenario_scores_the_shot_at_the_car():
    fired = {'target_id': '7', 'result': 'fired', 'reason': '', 'stamp': 12.0}
    rows, step = fire_rows('fire_car', 'fired', fired, shots=[{'sim': 12.0}],
                           sim_shots=[sim_shot(12.0, (0.3, 0.2, 117.0))])
    result = sim_test.score_fire(rows, step)
    assert result['passed'], result
    assert result['shot_offset_truth_m'] == pytest.approx(math.hypot(0.3, 0.2), abs=1e-3)
    assert result['shot_after_s'] == 2.0 and result['shot_on_car']
    assert result['shot_height_truth_m'] == 17.0
    # Rolled 5 deg: the bore's ground point is 17 tan(5 deg) = 1.5 m off the car's side.
    rolled = quaternion(5.0, 0.0, 0.0)
    rows, step = fire_rows('fire_car', 'fired', fired, shots=[{'sim': 12.0}],
                           sim_shots=[sim_shot(12.0, (0.0, 0.0, 117.0), rolled)])
    result = sim_test.score_fire(rows, step)
    assert result['shot_across_m'] == pytest.approx(17.0 * math.tan(math.radians(5.0)), abs=1e-3)
    assert not result['shot_on_car'] and result['failures'] == ['shot_on_car']
    # A second shot, a late one, gates that did not hold: each fails it.
    for change, failure in ((dict(shots=[{}, {}]), 'shots'), (dict(held=False), 'gates'),
                            (dict(sim_shots=[sim_shot(14.0, (0.0, 0.0, 117.0))]),
                             'shot_after_s')):
        kwargs = dict(shots=[{'sim': 12.0}], sim_shots=[sim_shot(12.0, (0.0, 0.0, 117.0))])
        kwargs.update(change)
        rows, step = fire_rows('fire_car', 'fired', fired, **kwargs)
        assert failure in sim_test.score_fire(rows, step)['failures'], change


def test_the_fire_scenario_scores_the_refusals():
    refused = {'target_id': '9', 'result': 'refused',
               'reason': 'never at people: the target is a person', 'stamp': 10.0}
    checks = dict(ALL_CHECKS, not_person=False, no_person=False)
    rows, step = fire_rows('fire_person', 'refused', refused, checks=checks)
    assert sim_test.score_fire(rows, step)['passed']
    rows, step = fire_rows('fire_person', 'refused', refused, launcher_max=(0.01, 0.0))
    assert sim_test.score_fire(rows, step)['failures'] == ['launcher moved']
    in_view = dict(refused, reason='a person is in view (#4 person)')
    rows, step = fire_rows('fire_person_in_view', 'refused', in_view,
                           checks=dict(ALL_CHECKS, no_person=False))
    assert sim_test.score_fire(rows, step)['passed']
    # Refused while another gate failed too: not the person's refusal alone.
    rows, step = fire_rows('fire_person_in_view', 'refused', in_view,
                           checks=dict(ALL_CHECKS, no_person=False, hover=False))
    assert sim_test.score_fire(rows, step)['failures'] == ['other gates']
    rows, step = fire_rows('fire_person_in_view', 'refused',
                           dict(in_view, result='running', reason=''), sim_shots=[{}])
    assert sim_test.score_fire(rows, step)['failures'][:2] == ['refusal', 'shots']


def test_the_fire_scenario_runs_from_the_compose_layer():
    layer = yaml.safe_load((REPO / 'tools/tracking_sim_test.yaml').read_text())
    assert '--scenario ${TRACKING_TEST_SCENARIO:-tracking}' in \
        layer['services']['tracking_sim_test']['command']
    source = (REPO / 'tools/tracking_sim_test.py').read_text()
    fly = source[source.index('def fly(args):'):source.index('def main():')]
    import re
    used = set(re.findall(r'args\.([a-z_]+)', fly))
    captured = {}
    with patch.object(sim_test, 'fly', lambda args: captured.setdefault('args', args)), \
            patch.object(sys, 'argv', ['x', 'fly', '--output', '/dev/null', '--scenario', 'fire']):
        sim_test.main()
    assert used <= set(vars(captured['args'])), used - set(vars(captured['args']))
    args = captured['args']
    assert lc.LauncherConfig().fire_min_agl_m <= args.fire_altitude <= \
        lc.LauncherConfig().fire_max_agl_m
    assert tc.TrackerConfig().min_agl_m <= args.fire_altitude
    assert args.fire_view_altitude <= lc.LauncherConfig().fire_max_agl_m


# --- Compositions -------------------------------------------------------------------------

def docker_compose_available():
    import shutil
    import subprocess
    return bool(shutil.which('docker')) and subprocess.run(
        ['docker', 'compose', 'version'], capture_output=True).returncode == 0


def render(name, *profiles, **env):
    import os
    import subprocess
    args = ['docker', 'compose', '-f', str(REPO / 'compositions' / name)]
    for profile in profiles:
        args += ['--profile', profile]
    clean = {key: value for key, value in os.environ.items()
             if not key.startswith(('TRACKING_', 'COMPOSE_', 'TURRET_')) and key not in (
                 'CYBERPHYSICS_TAG', 'DRONE_NAMESPACE', 'RANGEFINDER_DEVICE')}
    clean.update(env)
    result = subprocess.run(args + ['config', '--format', 'json'], env=clean, check=True,
                            capture_output=True, text=True)
    return json.loads(result.stdout)['services']


def ros_args(service):
    import shlex
    command = service['command']
    words = command if isinstance(command, list) else shlex.split(command)
    return dict(word.split(':=', 1) for word in words if ':=' in word)


@pytest.mark.skipif(not docker_compose_available(), reason='Docker Compose is not installed')
def test_the_aircraft_runs_the_launcher_bridge_only_with_its_profile():
    plain = render('aerodrone_hardware.yaml', 'tracking')
    assert 'launcher_bridge' not in plain
    tracker = ros_args(plain['tracker'])
    assert tracker['launcher_fitted'] == 'false'  # the aircraft without the launcher
    assert tracker['camera_offset_m'] == '[0.14215,0.0,-0.0798]'
    services = render('aerodrone_hardware.yaml', 'tracking', 'launcher',
                      TRACKING_LAUNCHER_FITTED='true',
                      TURRET_ARDUINO_PORT='/dev/serial/by-id/usb-1a86_USB_Single_Serial_X-if00')
    bridge = services['launcher_bridge']
    assert ros_args(services['tracker'])['launcher_fitted'] == 'true'
    assert ros_args(bridge)['serial_port'] == \
        '/host/dev/serial/by-id/usb-1a86_USB_Single_Serial_X-if00'
    assert bridge['image'] == 'salsicha/cyberphysics:unoq-arm64'  # the small Jazzy image
    assert bridge['ulimits']['nofile'] == {'soft': 1024, 'hard': 524288}
    assert set(bridge['device_cgroup_rules']) == {'c 188:* rmw', 'c 166:* rmw'}
    mount = next(volume for volume in bridge['volumes'] if volume['target'] == '/host/dev')
    assert mount['source'] == '/dev' and mount['read_only']
    assert 'devices' not in bridge and not bridge.get('privileged')
    assert bridge['network_mode'] == 'host' and bridge['profiles'] == ['launcher']
    config = next(volume for volume in bridge['volumes']
                  if volume['target'] == '/workspace/config/launcher_bridge.yaml')
    assert config['source'] == str(REPO / 'systems/aerodrone/config/launcher_bridge.yaml')
    # Unset, the port is no device at all: never the rangefinder's /dev/ttyUSB0.
    unset = render('aerodrone_hardware.yaml', 'launcher')['launcher_bridge']
    port = ros_args(unset)['serial_port']
    assert port.startswith('/host/dev/serial/by-id/') and 'ttyUSB' not in port


@pytest.mark.skipif(not docker_compose_available(), reason='Docker Compose is not installed')
@pytest.mark.parametrize('name,offset', [('aerostack2_sim.yaml', '[0.14215,0.0,-0.0798]'),
                                         ('aerostack2_isaac.yaml', '[0.14215,0.0,-0.0798]')])
def test_the_simulators_run_the_stand_in_launcher(name, offset):
    for profile in ('tracking', 'tracking_truth'):
        services = render(name, profile)
        assert 'launcher_bridge' not in services
        tracker = ros_args(services['tracker'])
        assert tracker['launcher_fitted'] == 'true' and tracker['camera_offset_m'] == offset
        assert tracker['launcher_topic_prefix'] == '/aerodrone/launcher'
        standin_args = ros_args(services['tracking_launcher'])
        assert standin_args['use_sim_time'] == 'true'
        assert standin_args['topic_prefix'] == '/aerodrone/launcher'
        assert services['tracking_launcher']['image'] == 'salsicha/cyberphysics:aerostack2-latest'
    for key, value in (('fire_min_agl_m', '8.0'), ('fire_max_agl_m', '20.0'),
                       ('fire_max_offset_m', '1.0'), ('fire_person_clear_s', '3.0')):
        assert tracker[key] == value, key


def test_the_simulator_cameras_are_where_the_env_files_say():
    world = yaml.safe_load((REPO / 'applications/gazebo/config/world_gazebo.yaml').read_text())
    camera = next(item for item in world['drones'][0]['payload']
                  if item['model_name'] == 'downward_rgb')
    assert camera['xyz'] == [0.14215, 0.0, -0.0798]
    isaac = (REPO / 'applications/isaac/scripts/aerodrone_navsim.py').read_text()
    assert "CAMERA_POSITION_M = AIRFRAME['camera_position_flu_m']" in isaac
    hardware = (REPO / 'hardware/parts/aerodrone/_frame.py').read_text()
    assert 'CAMERA_X = 112.0 + NOSE_SHIFT' in hardware and 'NOSE_SHIFT = 25.0' in hardware
