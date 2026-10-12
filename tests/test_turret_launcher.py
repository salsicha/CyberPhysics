"""PanTiltROS foam dart launcher in the tracker (systems/racecar_turret/
interfaces.md, "Foam dart launcher"): the tag command and its gates, the
fire cycle (prime, chamber, trigger, trigger back, each verified against the
measured rack and trigger), faults and aborts, darts and reload, the torso
aim point, the too-close gate, the aim offset table, the tagged set,
next/auto_next/clear_tags and the status fields.

Safety: nothing primes or fires the launcher but an explicit tag command
naming the locked target while every gate holds. The property tests drive
the node's own callbacks with random sequences of every other command,
detections, joint states (also with the launcher measured away from rest),
emergency-stop and readiness toggles, IMU messages and timer ticks, and
check that no joint command ever moves the trigger off 0 or the rack further
back than it is, and that the plant never fires; with tag commands mixed in,
that every priming or firing command and every shot follows an accepted tag
whose gates held, within its time bound. The structural test checks in the
source that priming and firing are reachable only from the tag handler.

The node runs with ROS and Ultralytics stubbed out against a plant whose
rack and trigger follow the commands at the ST3215's speed, with a sear that
catches the plunger at the end of the draw, the bridge's 500 ms watchdog and
its emergency-stop STOP (trigger to 0, rack stopped); the cycle time is also
measured on the turret simulator's own launcher model.
"""
import ast
import importlib
import json
import math
import random
import re
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script

SCRIPT = REPO / 'applications/pantiltros/scripts/pantilt_yolo_tracker.py'
T0 = 1000.0
WIDTH, HEIGHT = 640, 480
FX = WIDTH / 2 / math.tan(math.radians(62.0) / 2)
FY = HEIGHT / 2 / math.tan(math.radians(48.0) / 2)
CX, CY = WIDTH / 2, HEIGHT / 2
MASK = np.zeros((1, 1), dtype=np.uint8)
# The design (hardware/parts/turret/_launcher.py): the rack primes at 38 mm,
# the sear catches from 34.25 mm, the trigger fires at 0.6 rad and the sear
# releases at 0.5196 rad (4.45 mm of its 5.4 mm lift)
STROKE, FIRE = 0.038, 0.6
SEAR_CATCH, SEAR_RELEASE = 0.03425, 0.5196
# ST3215 at the bridge's 2000 steps/s: the rack on the 13.5 mm pinion, the trigger
SERVO_RAD_S = 2000 * 2 * math.pi / 4096
RACK_SPEED = SERVO_RAD_S * 0.0135
PERSON_H = 120.0


@pytest.fixture(scope='module')
def tracker():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('diagnostic_msgs', 'diagnostic_msgs.msg', 'rclpy.executors', 'ultralytics'):
            sys.modules[name] = MagicMock()
        module = load_script(SCRIPT, 'pantilt_yolo_tracker_launcher')
        module.JointState = lambda: NS(header=NS(stamp=None), name=[], position=[])
        module.String = lambda data='': NS(data=data)
        module.mask_message = lambda mask, source: None
        yield module


def stamp(t):
    return NS(sec=int(t), nanosec=int(round((t % 1) * 1e9)))


def detection(u, v, w=40.0, h=PERSON_H, cls='person'):
    return {'id': '', 'class': cls, 'confidence': 0.9, 'bbox_xywh': [u, v, w, h],
            'bbox_xyxy': [u - w / 2, v - h / 2, u + w / 2, v + h / 2],
            'centroid_px': [u, v], 'mask_area_px': int(w * h), 'mask': MASK}


def person_at_torso(u, v, h=PERSON_H, **kwargs):
    """A person whose torso aim point (0.6 of the box from its top) is at (u, v)."""
    return detection(u, v - 0.1 * h, h=h, **kwargs)


def profile(name='turret_tracker.yaml'):
    path = REPO / 'systems/racecar_turret/config' / name
    return yaml.safe_load(path.read_text())['pantilt_yolo_tracker']['ros__parameters']


class Rig:
    """The tracker node without ROS, and a turret plant: pan and tilt where
    commanded; the rack and trigger moving towards the last command's goals
    at the ST3215's speed; a sear that catches the plunger when the rack
    passes SEAR_CATCH and releases it (a shot) when the trigger passes
    SEAR_RELEASE. Like the bridge, a command without
    actuator_joint returns the trigger to 0 and stops the rack where it is,
    and so do the emergency stop and 0.5 s without commands."""

    def __init__(self, tracker, **overrides):
        parameters = dict(tracker.DEFAULT_PARAMETERS)
        parameters.update(profile())
        parameters.update(actuator_fitted=True, publish_annotated_image=False,
                          show_window=False, base_compensation=False, target_lead=False,
                          # the rig's detections stay where they are drawn,
                          # whatever the aim: no drop offset unless a test sets one
                          aim_offset_range_m=[1.0, 10.0], aim_offset_pitch_deg=[0.0, 0.0])
        parameters.update(overrides)
        parameters.pop('use_sim_time', None)
        node = tracker.PanTiltYoloTracker.__new__(tracker.PanTiltYoloTracker)
        node.clock_s = T0
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(round(node.clock_s * 1e9)),
                                                   to_msg=lambda: None))
        node.get_logger = MagicMock()
        node.get_parameter = lambda name: NS(value=parameters[name])
        node._configure()
        self.tracker, self.node = tracker, node
        self.commands, self.segments, self.statuses = [], [], []
        node.command_pub = NS(publish=self._command)
        node.segments_pub = NS(publish=lambda msg: self.segments.append(json.loads(msg.data)))
        node.status_pub = NS(publish=lambda msg: self.statuses.append(json.loads(msg.data)))
        for name in ('mask_pub', 'bbox_pub', 'confidence_pub', 'pixel_error_pub',
                     'angular_error_pub', 'annotated_pub', 'locked_pub', 'diagnostics_pub',
                     'selected_id_pub'):
            setattr(node, name, MagicMock())
        node.camera_info = NS(k=[FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0])
        node.tf_buffer = None
        self.t = 0.0
        self.plant_t = 0.0  # when the plant last moved
        self.pan = self.tilt = 0.0
        self.rack = self.trigger = 0.0
        self.rack_goal = self.trigger_goal = 0.0
        self.last_command_t = -math.inf
        self.estop = parameters['require_estop_release']
        self.stuck = set()  # 'rack' and/or 'trigger': jammed or unpowered
        self.report_launcher = True
        self.ready = True
        self.primed = False  # the sear holds the plunger
        self.shots = []  # times the plunger was released
        self.shot_poses = []  # (t, pan, tilt) of the turret at each
        self.firing = []  # times of commands with the trigger off 0
        self.priming = []  # times of commands with the rack further back than it was

    def _command(self, msg):
        joints = dict(zip(msg.name, msg.position))
        self.commands.append((self.t, joints))
        if joints.get('trigger_joint', 0.0) > 0.0:
            self.firing.append(self.t)
        if joints.get('actuator_joint', 0.0) > max(self.rack, 0.0) + 1e-9:
            self.priming.append(self.t)
        if self.estop:
            return  # the bridge drops commands while the e-stop is engaged
        self.pan, self.tilt = joints['pan_joint'], joints['tilt_joint']
        if 'actuator_joint' in joints:
            self.rack_goal = joints['actuator_joint']
            self.trigger_goal = joints.get('trigger_joint', 0.0)
        else:  # the bridge: pan and tilt only, the trigger to 0, the rack stopped
            self.rack_goal, self.trigger_goal = self.rack, 0.0
        self.last_command_t = self.t

    def at(self, t):
        self.t = t
        self.node.clock_s = T0 + t
        return self

    def advance(self, t):
        """Move the plant to time t and publish its joint states and readiness."""
        dt = max(0.0, t - self.plant_t)
        self.plant_t = t
        self.at(t)
        if self.estop or t - self.last_command_t >= 0.5:
            self.trigger_goal, self.rack_goal = 0.0, self.rack  # STOP / watchdog
        rack, trigger = self.rack, self.trigger
        if 'rack' not in self.stuck:
            self.rack += max(-RACK_SPEED * dt, min(RACK_SPEED * dt, self.rack_goal - self.rack))
        if 'trigger' not in self.stuck:
            self.trigger += max(-SERVO_RAD_S * dt,
                                min(SERVO_RAD_S * dt, self.trigger_goal - self.trigger))
        # The sear: it catches the plunger when the rack is driven past
        # SEAR_CATCH and releases it (a shot) when the trigger is driven past
        # SEAR_RELEASE (a state set by hand below does neither)
        if rack < SEAR_CATCH <= self.rack:
            self.primed = True
        if self.primed and trigger < SEAR_RELEASE <= self.trigger:
            self.primed = False
            self.shots.append(t)
            self.shot_poses.append((t, self.pan, self.tilt))
        names = ['pan_joint', 'tilt_joint']
        values = [self.pan, self.tilt]
        if self.report_launcher:
            names += ['actuator_joint', 'trigger_joint']
            values += [self.rack, self.trigger]
        self.node._joint_state(NS(name=names, position=values, header=NS(stamp=stamp(T0 + t))))
        if self.ready is not None:
            self.node._actuator_ready(NS(data=self.ready and not self.estop))

    def frame(self, t, detections):
        self.at(t)
        source = NS(header=NS(stamp=stamp(T0 + t), frame_id='turret/camera_optical_frame'),
                    height=HEIGHT, width=WIDTH)
        self.node._process_frame(source, None, detections, 10.0, T0 + t)
        return self.segments[-1]

    def tick(self, t):
        self.at(t)
        self.node._control_tick()

    def command(self, t, **command):
        self.at(t)
        self.node._turret_command(NS(data=json.dumps(command)))
        return self.statuses[-1]

    def status(self, t):
        self.at(t)
        self.node._publish_status()
        return self.statuses[-1]

    def set_estop(self, t, engaged):
        self.at(t)
        self.estop = engaged
        self.node._emergency_stop(NS(data=engaged))

    def run(self, t0, t1, detections=None, dt=0.02, frame_period=0.06):
        """Plant, joint states and control ticks every dt; a frame of
        detections(t) (a list, or None for no frame) every frame_period."""
        t, next_frame = t0, t0
        while t <= t1 + 1e-9:
            self.advance(t)
            if detections is not None and t >= next_frame - 1e-9:
                next_frame += frame_period
                seen = detections(t)
                if seen is not None:
                    self.frame(t, seen)
            self.tick(t)
            t = round(t + dt, 9)
        return self.status(t1)


def two_people(dx=0.0):
    """One person whose torso is near the image centre (aim error ~0.4 deg),
    one far left."""
    return [person_at_torso(CX + 4.0 + dx, CY + 2.0), person_at_torso(120.0, CY)]


def locked_rig(tracker, darts=3, rig_class=None, **overrides):
    """Armed, ready, loaded, locked on the person at the centre."""
    rig = (rig_class or Rig)(tracker, **overrides)
    rig.set_estop(0.0, False)
    rig.run(0.0, 0.3, lambda t: two_people())
    centre = rig.segments[-1][0]['id']
    assert rig.command(0.31, action='lock', target_id=centre)['target_id'] == centre
    if darts:
        assert rig.command(0.32, action='reload', darts=darts)['darts'] == darts
    status = rig.run(0.33, 0.6, lambda t: two_people())
    assert status['locked'] and (status['fire_block'] == '' or not darts), status
    return rig, centre


def launcher_values(rig, joint, since=0.0):
    return [joints[joint] for t, joints in rig.commands if t >= since and joint in joints]


# --- FireCycle -------------------------------------------------------------------

def test_cycle_steps_against_the_measured_launcher(tracker):
    cycle = tracker.FireCycle(stroke_m=STROKE, fire_rad=FIRE, dwell_s=0.2, timeout_s=3.0)
    assert (cycle.phase, cycle.goals(0.0), cycle.running) == ('ready', (0.0, 0.0), False)
    assert cycle.update(0.0, 0.0, 0.0) is None and cycle.phase == 'ready'
    cycle.start(1.0, '7')
    assert (cycle.phase, cycle.goals(1.0), cycle.target_id) == ('priming', (STROKE, 0.0), '7')
    assert cycle.update(1.5, 0.03, 0.0) is None and cycle.phase == 'priming'
    assert cycle.update(2.0, 0.0365, 0.0) is None and cycle.phase == 'chambering'  # within 2 mm
    assert cycle.goals(2.0) == (0.0, 0.0)
    assert cycle.update(2.5, 0.0015, 0.0) is None and cycle.phase == 'firing'
    assert cycle.goals(2.5) == (0.0, FIRE)
    assert cycle.update(2.6, 0.0, 0.3) is None and cycle.phase == 'firing'
    assert cycle.update(2.7, 0.0, 0.56) is None  # released (within 0.05 rad)
    assert cycle.update(2.89, 0.0, 0.6) is None and cycle.phase == 'firing'
    assert cycle.update(2.91, 0.0, 0.6) is None
    assert (cycle.phase, cycle.goals(2.91)) == ('resetting', (0.0, 0.0))
    assert cycle.update(3.0, 0.0, 0.2) is None
    assert cycle.update(3.1, 0.0, 0.01) == ('7', 'fired', '', True)
    assert (cycle.phase, cycle.running) == ('ready', False)


def test_cycle_goals_are_bounded_in_time(tracker):
    cycle = tracker.FireCycle(stroke_m=STROKE, fire_rad=FIRE, dwell_s=0.2, timeout_s=3.0)
    cycle.start(0.0, '7')
    assert cycle.goals(3.0) == (STROKE, 0.0)
    assert cycle.goals(3.01) == (None, 0.0)  # past the timeout: held, before update() runs
    cycle.update(1.0, STROKE, 0.0)
    cycle.update(1.5, 0.0, 0.0)
    assert cycle.goals(1.5 + 3.2) == (0.0, FIRE)
    assert cycle.goals(1.5 + 3.21) == (0.0, 0.0)


def test_cycle_faults_hold_the_rack_and_latch_until_reset(tracker):
    cycle = tracker.FireCycle(stroke_m=STROKE, fire_rad=FIRE, dwell_s=0.2, timeout_s=3.0)
    cycle.start(0.0, '7')
    assert cycle.update(3.0, 0.01, 0.0) is None
    assert cycle.update(3.01, 0.01, 0.0) == ('7', 'fault', 'priming not reached in 3 s', False)
    assert (cycle.phase, cycle.goals(3.01), cycle.running) == ('fault', (None, 0.0), False)
    assert cycle.update(10.0, 0.0, 0.0) is None and cycle.phase == 'fault'  # latched
    assert cycle.reset(11.0) and cycle.phase == 'returning' and cycle.goals(11.0) == (0.0, 0.0)
    assert cycle.update(11.1, 0.0, 0.0) is None and cycle.phase == 'ready'
    # No measurement mid-cycle
    cycle.start(20.0, '8')
    assert cycle.update(20.1, None, 0.0)[1:3] == ('fault', 'no launcher position')
    # The trigger does not move
    cycle.reset(21.0)
    cycle.update(21.0, 0.0, 0.0)
    cycle.start(22.0, '9')
    cycle.update(23.0, STROKE, 0.0)
    cycle.update(24.0, 0.0, 0.0)
    assert cycle.update(27.1, 0.0, 0.0) == ('9', 'fault', 'trigger not reached in 3 s', False)


def test_cycle_aborts(tracker):
    cycle = tracker.FireCycle(stroke_m=STROKE, fire_rad=FIRE, dwell_s=0.2, timeout_s=3.0)
    assert cycle.abort(0.0, 'release') is None  # nothing to abort
    cycle.start(1.0, '3')
    assert cycle.abort(1.2, 'release') == ('3', 'fault', 'aborted while priming (release)', False)
    assert cycle.goals(1.2) == (None, 0.0)
    cycle.reset(2.0)
    cycle.update(2.0, 0.0, 0.0)
    # In chambering: a fault too (the plunger may be held by the sear)
    cycle.start(3.0, '4')
    cycle.update(3.5, STROKE, 0.0)
    assert cycle.abort(3.6, 'lost the lock')[1:3] == (
        'fault', 'aborted while chambering (lost the lock)')
    cycle.reset(4.0)
    cycle.update(4.0, 0.0, 0.0)
    # Firing before the release: a fault, and the dart may have gone
    cycle.start(5.0, '5')
    cycle.update(5.5, STROKE, 0.0)
    cycle.update(6.0, 0.0, 0.0)
    assert cycle.abort(6.1, 'emergency stop')[3] is True
    cycle.reset(7.0)
    cycle.update(7.0, 0.0, 0.0)
    # After the release the shot is gone: the cycle completes as fired
    cycle.start(8.0, '6')
    cycle.update(8.5, STROKE, 0.0)
    cycle.update(9.0, 0.0, 0.0)
    cycle.update(9.2, 0.0, FIRE)
    assert cycle.abort(9.25, 'release') is None and cycle.phase == 'resetting'
    assert cycle.update(9.5, 0.0, 0.0) == ('6', 'fired', '', True)


def test_cycle_returns_a_launcher_found_away_from_rest(tracker):
    cycle = tracker.FireCycle(stroke_m=STROKE, fire_rad=FIRE, dwell_s=0.2, timeout_s=3.0)
    assert cycle.update(1.0, 0.003, 0.0) is None and cycle.phase == 'ready'  # hysteresis
    assert cycle.update(1.1, 0.03, 0.0) is None and cycle.phase == 'returning'
    assert cycle.goals(1.1) == (0.0, 0.0)
    assert cycle.update(1.5, 0.0, 0.0) is None and cycle.phase == 'ready'
    assert cycle.update(2.0, 0.0, 0.3) is None and cycle.phase == 'returning'
    assert cycle.update(5.1, 0.0, 0.3) == ('', 'fault', 'not back at rest in 3 s', False)
    with pytest.raises(ValueError):
        tracker.FireCycle(stroke_m=0.003)


# --- The fire cycle on the node ------------------------------------------------

def phases_seen(rig, t0, t1, detections):
    seen = []
    t = t0
    while t <= t1 + 1e-9:
        rig.run(t, t, detections)
        state = rig.node._launcher_state(T0 + t)
        if not seen or seen[-1] != state:
            seen.append(state)
        t = round(t + 0.02, 9)
    return seen


def test_a_tag_fires_one_dart(tracker):
    rig, centre = locked_rig(tracker, darts=3)
    status = rig.status(0.6)
    assert (status['launcher'], status['darts'], status['tags'], status['last_tag']) == (
        'ready', 3, 0, None)
    assert set(launcher_values(rig, 'trigger_joint')) == {0.0}
    status = rig.command(0.61, action='tag', target_id=centre)
    assert status['launcher'] == 'priming' and status['last_tag']['result'] == 'running'
    assert rig.commands[-1][1]['actuator_joint'] == STROKE  # at once
    seen = phases_seen(rig, 0.62, 6.0, lambda t: two_people())
    assert seen == ['priming', 'chambering', 'firing', 'ready']
    status = rig.status(6.0)
    assert (status['darts'], status['tags'], status['tagged_ids']) == (2, 1, [centre])
    assert status['last_tag']['target_id'] == centre and status['last_tag']['result'] == 'fired'
    assert len(rig.shots) == 1 and rig.trigger == pytest.approx(0.0) and rig.rack == 0.0
    # Cycle time: press to shot and press to ready again
    shot = rig.shots[0] - 0.61
    ready = rig.node.last_cycle_end - T0 - 0.61
    print(f'fire cycle: shot {shot:.2f} s after the tag, ready again after {ready:.2f} s')
    assert shot == pytest.approx(2 * STROKE / RACK_SPEED + SEAR_RELEASE / SERVO_RAD_S, abs=0.3)
    assert ready < shot + 0.2 + FIRE / SERVO_RAD_S + 0.2
    # auto_next is off: still on the target, nothing more primes or fires
    assert status['target_id'] == centre
    assert max(rig.firing) <= rig.node.last_cycle_end - T0
    assert max(rig.priming) < rig.shots[0]


class TurretSimRig(Rig):
    """The rig with the turret simulator's own launcher model
    (applications/turret_sim: TurretSim.update_launcher, the scenario's
    command latency, servo speed, whole servo steps and sear) in place of
    the rig's."""

    def __init__(self, tracker, **overrides):
        super().__init__(tracker, **overrides)
        module = load_script(REPO / 'applications/turret_sim/scripts/turret_sim.py',
                             'turret_sim_plant')
        scenario = json.loads(
            (REPO / 'applications/turret_sim/scenarios/warehouse_tracking.json').read_text())
        self.sim = module.TurretSim(scenario, launcher=True)
        self.latency = float(scenario['pantilt']['command_latency_s'])

    def _command(self, msg):
        rack, trigger = self.rack, self.trigger
        super()._command(msg)  # the record, pan and tilt
        if self.estop:
            return
        joints = dict(zip(msg.name, msg.position))
        launcher = ((joints['actuator_joint'], joints.get('trigger_joint', 0.0))
                    if 'actuator_joint' in joints else (None, 0.0))
        self.sim.launcher_queue.append((self.t + self.latency, *launcher))
        self.rack, self.trigger = rack, trigger  # the simulator's, not the rig's

    def advance(self, t):
        dt = max(0.0, t - self.plant_t)
        self.plant_t = t
        self.at(t)
        self.sim.update_launcher(t, dt, self.estop)
        self.rack, self.trigger, self.shots = self.sim.rack, self.sim.trigger, self.sim.shots
        self.node._joint_state(NS(name=['pan_joint', 'tilt_joint', 'actuator_joint',
                                        'trigger_joint'],
                                  position=[self.pan, self.tilt, self.rack, self.trigger],
                                  header=NS(stamp=stamp(T0 + t))))
        self.node._actuator_ready(NS(data=not self.estop))


def test_fire_cycle_time_on_the_turret_sim_plant(tracker):
    """The cycle time per shot with the turret simulator's launcher at its
    20 Hz step and a steady target: what the closed-loop turret_sim runs
    measure when YOLOE keeps the target's ID for a whole cycle."""
    rig, centre = locked_rig(tracker, darts=3, rig_class=TurretSimRig)
    steady = lambda t: two_people()  # noqa: E731
    shots, readies, presses = [], [], []
    t = 0.6
    for _ in range(3):
        rig.run(t, t + 2.5, steady, dt=0.05, frame_period=0.05)  # the minimum interval
        t = round(t + 2.55, 9)
        status = rig.command(t, action='tag', target_id=centre)
        assert status['last_tag']['result'] == 'running', status
        presses.append(t)
        seen = phases_seen(rig, t, t + 6.0, steady)
        assert seen == ['priming', 'chambering', 'firing', 'ready' if len(presses) < 3 else 'empty']
        shots.append(rig.sim.shots[-1] - t)
        readies.append(rig.node.last_cycle_end - T0 - t)
        t = round(t + 6.0, 9)
    assert len(rig.sim.shots) == 3 and rig.status(t)['darts'] == 0
    assert rig.sim.rack == 0.0 and rig.sim.trigger == 0.0 and not rig.sim.primed
    print('turret_sim plant: shot ' + ', '.join(f'{s:.2f}' for s in shots)
          + ' s after Fire; ready again after ' + ', '.join(f'{r:.2f}' for r in readies) + ' s')
    travel = 2 * STROKE / RACK_SPEED + SEAR_RELEASE / SERVO_RAD_S
    for shot, ready in zip(shots, readies):
        assert travel < shot < travel + 0.5
        assert shot < ready < shot + 0.2 + FIRE / SERVO_RAD_S + 0.4
    # Nothing primed or fired outside the three cycles
    assert all(any(p <= c <= p + 6.0 for p in presses) for c in rig.priming + rig.firing)


def test_the_gates_refuse_with_the_reason(tracker):
    rig, centre = locked_rig(tracker, darts=1)
    other = rig.segments[-1][1]['id']
    for target_id, reason in ((other, f'{other} is not the locked target ({centre})'),
                              ('', "tag needs the locked target's target_id"),
                              (None, "tag needs the locked target's target_id")):
        status = rig.command(0.61, action='tag', target_id=target_id)
        assert status['launcher'] == 'ready'
        assert status['last_tag'] == {'target_id': '' if target_id is None else target_id,
                                      'result': 'refused', 'reason': reason,
                                      'stamp': pytest.approx(T0 + 0.61)}
        assert f'fire refused: {reason}' in status['message']
    # Aim error over tag_max_aim_error_deg (2 deg) though still locked (36 px)
    rig.run(0.62, 0.8, lambda t: [person_at_torso(CX + 30.0, CY), person_at_torso(120.0, CY)])
    status = rig.status(0.8)
    assert status['locked'] and status['fire_block'] == 'aim error over 2 deg'
    assert rig.command(0.8, action='tag', target_id=centre)['last_tag']['result'] == 'refused'
    # Not locked: off the lock tolerance, or no recent frame
    rig.run(0.82, 1.0, lambda t: [person_at_torso(CX + 80.0, CY), person_at_torso(120.0, CY)])
    assert rig.command(1.0, action='tag', target_id=centre)['last_tag']['reason'] == (
        f'{centre} is not locked')
    rig.run(1.02, 1.6, lambda t: None)
    assert rig.status(1.6)['fire_block'] == f'{centre} is not locked'
    rig.run(1.62, 2.0, lambda t: two_people())
    # Too close: a person box taller than 0.6 of the image
    near = lambda t: [person_at_torso(CX + 4.0, CY + 2.0, h=300.0), person_at_torso(120.0, CY)]
    rig.run(2.02, 2.2, near)
    assert rig.status(2.2)['fire_block'] == 'too close'
    rig.run(2.22, 2.4, lambda t: two_people())
    # Disarmed, the actuator not ready, stale readiness
    rig.set_estop(2.4, True)
    assert rig.command(2.4, action='tag', target_id=centre)['last_tag']['reason'] == (
        'emergency stop engaged')
    rig.set_estop(2.41, False)
    rig.ready = False
    rig.run(2.42, 2.6, lambda t: two_people())
    assert rig.status(2.6)['fire_block'] == 'actuator not ready'
    rig.ready = None
    rig.node._actuator_ready(NS(data=True))
    rig.run(2.62, 3.8, lambda t: two_people())
    assert rig.status(3.8)['fire_block'] == 'actuator not ready'
    rig.ready = True
    rig.run(3.82, 4.0, lambda t: two_people())
    # The launcher away from rest, or not measured
    rig.rack = 0.02
    rig.run(4.02, 4.04, lambda t: two_people())
    assert rig.status(4.04)['fire_block'] == 'launcher not at rest'
    rig.run(4.06, 5.0, lambda t: two_people())  # returned to rest under supervision
    assert rig.status(5.0)['fire_block'] == ''
    rig.report_launcher = False
    rig.run(5.02, 5.8, lambda t: two_people())
    status = rig.status(5.8)
    assert status['launcher'] == 'fault' and 'launcher fault: no launcher position' in (
        status['message'])
    assert status['fire_block'] == 'no launcher position'
    rig.report_launcher = True
    rig.run(5.82, 6.0, lambda t: two_people())
    assert not rig.firing and not rig.priming and not rig.shots
    assert rig.command(6.0, action='tag', target_id=centre)['last_tag']['result'] == 'running'
    rig.run(6.02, 10.0, lambda t: two_people())
    # The clip is empty
    status = rig.status(10.0)
    assert (status['darts'], status['launcher'], status['fire_block']) == (0, 'empty', 'no darts')
    assert rig.command(10.0, action='tag', target_id=centre)['last_tag']['reason'] == 'no darts'
    assert len(rig.shots) == 1


def test_one_cycle_at_a_time_and_the_minimum_interval(tracker):
    rig, centre = locked_rig(tracker, darts=3, tag_min_interval_s=2.0)
    rig.command(0.61, action='tag', target_id=centre)
    status = rig.command(0.7, action='tag', target_id=centre)
    assert status['last_tag']['reason'] == 'a fire cycle is running'
    assert status['launcher'] == 'priming'  # the refused tap did not disturb it
    t = 0.72
    while rig.node.fire_cycle.running:
        rig.run(t, t, lambda t: two_people())
        t = round(t + 0.02, 9)
    end = rig.node.last_cycle_end - T0
    assert rig.status(t)['last_tag']['result'] == 'fired' and end < 6.0
    rig.run(t, end + 1.0, lambda t: two_people())
    status = rig.command(end + 1.0, action='tag', target_id=centre)
    assert status['last_tag']['reason'] == '1.0 s to the next shot'
    rig.run(end + 1.02, end + 2.1, lambda t: two_people())
    assert rig.command(end + 2.1, action='tag', target_id=centre)['last_tag'][
        'result'] == 'running'


@pytest.mark.parametrize('phase', ['priming', 'chambering'])
@pytest.mark.parametrize('action,command', [
    ('release', {}), ('center', {}), ('manual', {'pan_rate': 0.3, 'tilt_rate': 0.0}),
    ('lock', {'point': [0.19, 0.5]}), ('next', {}),
])
def test_commands_that_move_the_aim_stop_the_cycle_safely(tracker, phase, action, command):
    rig, centre = locked_rig(tracker)
    rig.command(0.61, action='tag', target_id=centre)
    t = 0.62
    while rig.node._launcher_state(T0 + t) != phase:
        rig.run(t, t, lambda t: two_people())
        t = round(t + 0.02, 9)
    rig.run(t, t + 0.1, lambda t: two_people())
    rack_at_abort = rig.rack
    status = rig.command(t + 0.12, action=action, **command)
    assert status['launcher'] == 'fault' and status['last_tag']['result'] == 'fault'
    assert status['last_tag']['reason'] == f'aborted while {phase} ({action} command)'
    last = rig.commands[-1][1]
    assert last['trigger_joint'] == 0.0 and 'actuator_joint' not in last  # the rack stopped
    rig.run(t + 0.14, t + 3.0, lambda t: two_people())
    assert not rig.shots and rig.trigger == 0.0
    assert rig.rack <= rack_at_abort + 1e-9  # stopped, never drawn further
    assert max(rig.priming, default=0.0) <= t + 0.12
    status = rig.status(t + 3.0)
    assert status['launcher'] == 'fault' and status['darts'] == 3 and status['tags'] == 0
    # Disarm and arm acknowledge it: the launcher returns to rest
    rig.set_estop(t + 3.1, True)
    rig.set_estop(t + 3.2, False)
    rig.run(t + 3.22, t + 6.0, lambda t: two_people())
    assert rig.status(t + 6.0)['launcher'] == 'ready' and rig.rack == pytest.approx(0.0)


def test_losing_the_lock_or_readiness_or_the_emergency_stop_stops_the_cycle(tracker):
    rig, centre = locked_rig(tracker)
    rig.command(0.61, action='tag', target_id=centre)
    rig.run(0.62, 0.7, lambda t: two_people())  # (the 38 mm draw takes 0.9 s)
    rig.run(0.72, 1.4, lambda t: [person_at_torso(CX + 90.0, CY), person_at_torso(120.0, CY)])
    status = rig.status(1.4)
    assert status['last_tag']['reason'] == f'aborted while priming (lost the lock on {centre})'
    assert not rig.shots and status['launcher'] == 'fault'
    # Short detection gaps (under 0.5 s) do not stop it
    rig2, centre2 = locked_rig(tracker)
    rig2.command(0.61, action='tag', target_id=centre2)
    rig2.run(0.62, 6.0, lambda t: None if int(t * 10) % 4 == 1 else two_people())
    assert rig2.status(6.0)['last_tag']['result'] == 'fired' and len(rig2.shots) == 1
    # The emergency stop: the bridge's STOP makes it safe, nothing is commanded
    rig3, centre3 = locked_rig(tracker)
    rig3.command(0.61, action='tag', target_id=centre3)
    rig3.run(0.62, 1.0, lambda t: two_people())
    rig3.set_estop(1.0, True)
    sent = len(rig3.commands)
    rig3.run(1.02, 3.0, lambda t: two_people())
    assert len(rig3.commands) == sent and not rig3.shots and rig3.trigger == 0.0
    assert rig3.status(3.0)['last_tag']['reason'] == 'aborted while priming (emergency stop)'
    # Readiness lost mid-cycle
    rig4, centre4 = locked_rig(tracker)
    rig4.command(0.61, action='tag', target_id=centre4)
    rig4.run(0.62, 0.8, lambda t: two_people())
    rig4.ready = False
    rig4.run(0.82, 0.9, lambda t: two_people())
    assert rig4.status(0.9)['last_tag']['reason'] == 'aborted while priming (actuator not ready)'


def test_faults_stop_the_launcher_and_latch_until_rearmed(tracker):
    rig, centre = locked_rig(tracker, tag_stroke_timeout_s=1.5)
    rig.stuck = {'rack'}
    rig.command(0.61, action='tag', target_id=centre)
    rig.run(0.62, 2.3, lambda t: two_people())
    status = rig.status(2.3)
    assert status['launcher'] == 'fault'
    assert status['last_tag']['reason'] == 'priming not reached in 1.5 s'
    assert 'launcher fault: priming not reached in 1.5 s' in status['message']
    assert set(launcher_values(rig, 'trigger_joint', 2.2)) == {0.0}
    rig.stuck = set()
    rig.run(2.32, 4.0, lambda t: two_people())
    assert rig.status(4.0)['fire_block'] == 'launcher fault: priming not reached in 1.5 s'
    rig.set_estop(4.1, True)
    rig.set_estop(4.2, False)
    rig.run(4.22, 4.6, lambda t: two_people())
    assert rig.status(4.6)['fire_block'] == '' and not rig.shots
    # The trigger jammed: a fault with the trigger asked back to 0
    rig.stuck = {'trigger'}
    rig.node.fire_cycle.timeout_s = 2.5  # (the 38 mm draw takes 0.9 s)
    rig.command(4.6, action='tag', target_id=centre)
    rig.run(4.62, 12.0, lambda t: two_people())
    assert rig.status(12.0)['last_tag']['reason'] == 'trigger not reached in 2.5 s'
    assert launcher_values(rig, 'trigger_joint')[-1] == 0.0 and not rig.shots


def test_shutdown_mid_cycle_makes_the_launcher_safe(tracker):
    rig, centre = locked_rig(tracker)
    rig.command(0.61, action='tag', target_id=centre)
    rig.run(0.62, 1.0, lambda t: two_people())
    assert rig.commands[-1][1]['actuator_joint'] == STROKE
    rig.at(1.01).node._safe_for_shutdown()
    last = rig.commands[-1][1]
    assert last['trigger_joint'] == 0.0 and 'actuator_joint' not in last  # the rack stopped
    assert rig.node.last_tag['reason'] == 'aborted while priming (shutdown)'
    destroy = next(node for node in ast.walk(ast.parse(SCRIPT.read_text()))
                   if isinstance(node, ast.FunctionDef) and node.name == 'destroy_node')
    assert ast.unparse(destroy.body[0]) == 'self._safe_for_shutdown()'


def test_reload(tracker):
    rig, centre = locked_rig(tracker, darts=0)
    assert rig.status(0.6)['launcher'] == 'empty'
    for bad in (-1, 1.5, '3', True, None):
        assert 'reload needs darts' in rig.command(0.6, action='reload', darts=bad)['message']
    assert rig.command(0.61, action='reload', darts=6)['darts'] == 6
    rig.command(0.62, action='tag', target_id=centre)
    status = rig.command(0.7, action='reload', darts=10)
    assert status['darts'] == 6 and 'reload refused' in status['message']


# --- Aim point, too close and the aim offset --------------------------------------

def test_a_person_is_aimed_at_the_torso_never_the_head(tracker):
    rig, centre = locked_rig(tracker)
    node = rig.node
    # A person whose box centre is at the image centre: the torso is 0.1 h below
    rig.run(0.62, 0.7, lambda t: [detection(CX, CY), person_at_torso(120.0, CY)])
    error = node.last_frame['aim_error_deg']
    assert error == pytest.approx(math.degrees(math.atan(0.1 * PERSON_H / FY)), abs=1e-6)
    assert rig.commands[-1][1]['tilt_joint'] < 0  # aiming lower, at the torso
    # The torso point itself at the centre: no error
    rig.run(0.72, 0.8, lambda t: two_people(dx=-4.0))
    assert node.last_frame['aim_error_deg'] == pytest.approx(
        math.degrees(math.atan(2.0 / FY)), abs=1e-6)
    # Never the head: the aim point is in the lower 0.4-0.6 band of the box
    (x, y), too_close = node._aim_point(detection(100.0, 200.0, h=200.0), FY, HEIGHT)
    assert (x, y, too_close) == (100.0, pytest.approx(100.0 + 0.6 * 200.0), False)
    assert y > 100.0 + 0.25 * 200.0
    # Other classes: the mask's centroid, as without a launcher
    dog = detection(100.0, 200.0, cls='dog')
    dog['centroid_px'] = [101.0, 205.0]
    assert node._aim_point(dog, FY, HEIGHT) == ((101.0, 205.0), False)


def test_without_a_launcher_the_aim_is_unchanged(tracker):
    rig = Rig(tracker, actuator_fitted=False)
    rig.set_estop(0.0, False)
    rig.run(0.0, 0.3, lambda t: [detection(CX + 10.0, CY + 5.0)])
    centre = rig.segments[-1][0]['id']
    rig.command(0.31, action='lock', target_id=centre)
    rig.run(0.32, 0.5, lambda t: [detection(CX + 10.0, CY + 5.0)])
    assert rig.node.last_frame['aim_error_deg'] == pytest.approx(
        math.degrees(math.atan(math.hypot(10 / FX, 5 / FY))))
    status = rig.command(0.6, action='tag', target_id=centre)
    assert (status['launcher'], status['fire_block'], status['last_tag']['reason']) == (
        'none', 'no launcher', 'no launcher')
    assert rig.commands and all(set(joints) == {'pan_joint', 'tilt_joint'}
                                for _, joints in rig.commands)


def test_aim_offset_table(tracker):
    rig = Rig(tracker, aim_offset_range_m=[2.0, 6.0], aim_offset_pitch_deg=[1.0, 5.0])
    node = rig.node
    # Range from the box height: 1.7 m tall, so h px is 1.7 * fy / h metres away
    h = 1.7 * FY / 4.0  # 4 m: halfway, 3 deg up
    (_, y), _ = node._aim_point(detection(CX, CY, h=h), FY, HEIGHT)
    assert y == pytest.approx(CY - h / 2 + 0.6 * h - FY * math.tan(math.radians(3.0)))
    assert node._aim_offset_rad(1.0) == pytest.approx(math.radians(1.0))  # held at the ends
    assert node._aim_offset_rad(20.0) == pytest.approx(math.radians(5.0))
    # The tracker's default table is zero (the launcher profiles set the drop's)
    assert set(tracker.DEFAULT_PARAMETERS['aim_offset_pitch_deg']) == {0.0}
    plain = Rig(tracker).node
    assert all(plain._aim_offset_rad(r) == 0.0 for r in (0.5, 3.0, 50.0))
    with pytest.raises(ValueError):
        Rig(tracker, aim_offset_range_m=[5.0, 2.0], aim_offset_pitch_deg=[0.0, 0.0])
    with pytest.raises(ValueError):
        Rig(tracker, aim_offset_range_m=[1.0], aim_offset_pitch_deg=[0.0, 0.0])


def test_the_aim_offset_table_is_the_drop_table(tracker):
    """The launcher profiles' table: the design's drop below the bore line
    (level fire at 25 m/s nominal, distances from the muzzle) less the
    bore's height above the camera's axis, as a pitch about the tilt axis,
    against the range from the camera (where a person's box height puts it)."""
    xacro = (REPO / 'systems/racecar_turret/urdf/turret.urdf.xacro').read_text()

    def prop(name):
        return re.search(rf'name="{name}" value="([^"]+)"', xacro).group(1)
    muzzle = [float(v) for v in prop('launcher_muzzle_xyz').split()]
    camera = float(prop('camera_forward'))  # ahead of the tilt axis
    ahead, above = muzzle[0] - camera, muzzle[2]
    assert (ahead, above, muzzle[1]) == pytest.approx((0.098, 0.048, 0.0))
    drop = {2.0: 0.034, 4.0: 0.15, 6.0: 0.36, 8.0: 0.70}
    ranges = [x + ahead for x in drop]
    pitches = [math.degrees(math.atan((d - above) / (x + ahead + camera))) for x, d in drop.items()]
    for path in (REPO / 'systems/racecar_turret/config/turret_tracker.yaml',
                 REPO / 'applications/pantiltros/config/yolo_tracker.yaml',
                 REPO / 'applications/pantiltros/config/yolo_tracker_hardware.yaml'):
        params = next(iter(yaml.safe_load(path.read_text()).values()))['ros__parameters']
        assert params['aim_offset_range_m'] == pytest.approx(ranges, abs=1e-3), path
        assert params['aim_offset_pitch_deg'] == pytest.approx(pitches, abs=0.01), path
        assert params['person_height_m'] == 1.7
    # In the node: a person 5 m away gets the interpolated offset, up
    rig = Rig(tracker, aim_offset_range_m=ranges, aim_offset_pitch_deg=pitches)
    node = rig.node
    h = 1.7 * FY / 5.0
    (_, y), _ = node._aim_point(detection(CX, CY, h=h), FY, HEIGHT)
    up = float(np.interp(5.0, ranges, pitches))
    assert 1.41 < up < 2.91
    assert y == pytest.approx(CY - h / 2 + 0.6 * h - FY * math.tan(math.radians(up)))
    # Close in, the bore's height above the camera wins: aim lower
    assert node._aim_offset_rad(2.098) < 0.0
    # Not for other classes: their range is not known
    dog = detection(CX, CY, h=h, cls='dog')
    dog['centroid_px'] = [CX, CY]
    assert node._aim_point(dog, FY, HEIGHT) == ((CX, CY), False)


def test_the_launcher_numbers_are_the_designs(tracker):
    """The stroke, the sear and the trigger wherever they are copied (the
    tracker and its profiles, the bridge, the firmware, turret_sim, the UI's
    mock), against hardware/parts/turret/_launcher.py."""
    if str(REPO) not in sys.path:
        sys.path.insert(0, str(REPO))
    design = importlib.import_module('hardware.parts.turret._launcher')
    stroke, catch, fire = design.ACT_PRIME / 1000, design.ACT_CATCH / 1000, design.TRIG_FIRE_RAD
    # The trigger arm's pin rises 14 sin(angle) - 2.5 mm under the sear's lug
    release = math.asin((design.sear_release_lift() + 2.5) / design.TRIG_ARM_R)
    assert (stroke, catch, fire) == pytest.approx((0.038, 0.03425, 0.6))
    assert (STROKE, SEAR_CATCH, FIRE, SEAR_RELEASE) == pytest.approx((stroke, catch, fire, release),
                                                                     abs=1e-4)
    assert tracker.DEFAULT_PARAMETERS['launcher_stroke_m'] == stroke
    assert tracker.DEFAULT_PARAMETERS['trigger_fire_rad'] == fire
    profiles = [profile(name) for name in ('turret_tracker.yaml', 'turret_tracker_isaac.yaml')]
    for name in ('yolo_tracker.yaml', 'yolo_tracker_hardware.yaml'):
        text = (REPO / 'applications/pantiltros/config' / name).read_text()
        profiles.append(yaml.safe_load(text)['pantilt_yolo_tracker']['ros__parameters'])
    for params in profiles:
        assert (params['launcher_stroke_m'], params['trigger_fire_rad']) == (stroke, fire)
    # The tracker verifies the draw past the sear's catch and the trigger
    # past its release
    assert stroke - tracker.RACK_TOLERANCE_M > catch
    assert fire - tracker.TRIGGER_TOLERANCE_RAD > release
    bridge = yaml.safe_load((REPO / 'applications/pantiltros/config/arduino_bridge.yaml').read_text())
    bridge = bridge['turret_arduino_bridge']['ros__parameters']
    assert (bridge['actuator_max_m'], bridge['trigger_max_rad']) == (stroke, fire)
    firmware = (REPO / 'applications/arduino/scripts/turret_firmware/src/turret_config.h').read_text()
    assert '{"actuator_joint", 14, 512, 1, 0.0f, 0.038f, 0.0135f, 550}' in firmware
    assert '{"trigger_joint", 15, 2048, 1, 0.0f, 0.6f, 0.0f, 300}' in firmware
    scenario = json.loads(
        (REPO / 'applications/turret_sim/scenarios/warehouse_tracking.json').read_text())
    sim = scenario['pantilt']['launcher']
    assert (sim['stroke_m'], sim['sear_catch_m'], sim['trigger_fire_rad']) == (stroke, catch, fire)
    assert sim['trigger_release_rad'] == pytest.approx(release, abs=1e-3)
    mock = (REPO / 'applications/nicegui/scripts/mock_racecar_turret.py').read_text()
    assert 'STROKE, FIRE_RAD = 0.038, 0.6' in mock


# --- line of fire -----------------------------------------------------------------

CAR_ZONE = [-0.17, 0.26, -1.5708, -0.42]


def test_no_fire_zones_parse_and_validate(tracker):
    assert tracker.parse_no_fire_zones(tracker.DEFAULT_PARAMETERS['no_fire_zones_rad']) == []
    assert tracker.parse_no_fire_zones(CAR_ZONE + [0.5, 0.5, -1.0, 1.0]) == [tuple(CAR_ZONE)]
    for bad in ([0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.5, -0.5],
                [0.0, math.nan, 0.0, 1.0], [0.0, math.inf, 0.0, 1.0]):
        with pytest.raises(ValueError):
            tracker.parse_no_fire_zones(bad)
    with pytest.raises(ValueError):
        Rig(tracker, no_fire_zones_rad=[0.0, 1.0, 0.0])
    with pytest.raises(ValueError):
        Rig(tracker, no_fire_zones_rad=CAR_ZONE, no_fire_margin_rad=-0.1)
    zones = [tuple(CAR_ZONE)]
    assert tracker.in_no_fire_zone(zones, 0.0, -0.5)
    assert not tracker.in_no_fire_zone(zones, 0.0, -0.4)
    assert tracker.in_no_fire_zone(zones, 0.0, -0.4, margin=0.05)
    assert not tracker.in_no_fire_zone(zones, 0.3, -0.6)
    assert tracker.in_no_fire_zone(zones, 0.3, -0.6, margin=0.05)


def test_the_car_s_no_fire_zone_is_in_its_profiles_only(tracker):
    for name in ('turret_tracker.yaml', 'turret_tracker_isaac.yaml'):
        params = profile(name)
        assert params['no_fire_zones_rad'] == CAR_ZONE and params['no_fire_margin_rad'] == 0.05
    for name in ('yolo_tracker.yaml', 'yolo_tracker_hardware.yaml'):
        text = (REPO / 'applications/pantiltros/config' / name).read_text()
        params = yaml.safe_load(text)['pantilt_yolo_tracker']['ros__parameters']
        assert 'no_fire_zones_rad' not in params  # the standalone turret: none
    # The tilt range reaches into it (the URDF's -0.65 rad)
    xacro = (REPO / 'systems/racecar_turret/urdf/turret.urdf.xacro').read_text()
    assert float(re.search(r'name="tilt_min" value="([^"]+)"', xacro).group(1)) < CAR_ZONE[3]


def test_the_line_of_fire_gate_refuses_before_priming(tracker):
    # Locked at pan and tilt about 0, inside a zone around them: refused
    rig, centre = locked_rig(tracker, darts=0, no_fire_zones_rad=[-0.3, 0.3, -0.3, 0.3])
    rig.command(0.605, action='reload', darts=3)
    status = rig.command(0.61, action='tag', target_id=centre)
    assert status['last_tag']['reason'] == status['fire_block'] == 'line of fire blocked'
    assert status['launcher'] == 'ready' and rig.priming == [] and rig.firing == []
    # The car's zone (aiming low ahead) leaves it clear; so does its margin
    rig, centre = locked_rig(tracker, no_fire_zones_rad=CAR_ZONE, no_fire_margin_rad=0.05)
    assert rig.status(0.6)['fire_block'] == ''
    # Within the margin of a zone: blocked
    rig, centre = locked_rig(tracker, darts=0, no_fire_zones_rad=[0.1, 0.5, -0.3, 0.3],
                             no_fire_margin_rad=0.12)
    rig.command(0.605, action='reload', darts=3)
    assert rig.status(0.61)['fire_block'] == 'line of fire blocked'
    # Where the turret points not known (no fresh joint state): blocked
    rig, centre = locked_rig(tracker, no_fire_zones_rad=CAR_ZONE)
    rig.node.controller.measured.clear()
    assert rig.status(0.6)['fire_block'] == 'line of fire blocked'
    # Without zones, the gate is open whatever the joint states
    rig, centre = locked_rig(tracker, no_fire_zones_rad=[0.0, 0.0, 0.0, 0.0])
    rig.node.controller.measured.clear()
    assert rig.node._line_of_fire_clear(T0 + 0.6)


def test_the_line_of_fire_is_checked_at_the_moment_of_firing(tracker):
    """Clear when the tag is accepted, blocked when the dart is chambered:
    the trigger never leaves 0, the launcher stays cocked and it faults."""
    rig, centre = locked_rig(tracker, no_fire_zones_rad=CAR_ZONE)
    assert rig.command(0.61, action='tag', target_id=centre)['last_tag']['result'] == 'running'
    t = 0.62
    while rig.node.fire_cycle.phase != 'chambering':
        rig.run(t, t, lambda t: two_people())
        t = round(t + 0.02, 9)
        assert t < 4.0
    assert rig.primed  # cocked on the sear
    # The turret now where the dart would hit the car (the zone moved onto
    # it rather than the turret into the zone: the rig's people stay put)
    rig.node.no_fire_zones = [(-0.3, 0.3, -0.3, 0.3)]
    rig.run(t, 4.0, lambda t: two_people())
    status = rig.status(4.0)
    assert status['last_tag']['reason'] == 'line of fire blocked' and status['launcher'] == 'fault'
    assert rig.firing == [] and rig.shots == [] and rig.primed  # cocked: clear it by hand
    assert set(launcher_values(rig, 'trigger_joint')) == {0.0}
    assert status['darts'] == 3  # no dart left
    assert status['fire_block'] == 'launcher fault: line of fire blocked'


def test_a_blocked_line_before_the_sear_releases_stops_the_trigger(tracker):
    rig, centre = locked_rig(tracker, no_fire_zones_rad=CAR_ZONE)
    rig.command(0.61, action='tag', target_id=centre)
    t = 0.62
    while not rig.firing:  # until the trigger is asked out
        rig.run(t, t, lambda t: two_people())
        t = round(t + 0.02, 9)
        assert t < 4.0
    assert rig.node.fire_cycle.phase == 'firing' and not rig.shots  # not yet at the release
    rig.node.no_fire_zones = [(-0.3, 0.3, -0.3, 0.3)]
    fired_until = len(rig.firing)
    rig.run(t, 3.0, lambda t: two_people())
    status = rig.status(3.0)
    assert status['last_tag']['reason'] == 'line of fire blocked while firing: the dart may have gone'
    assert len(rig.firing) == fired_until  # no trigger command after it
    assert rig.shots == [] and rig.trigger == pytest.approx(0.0)
    assert status['darts'] == 2  # counted: it may have gone


SWEEPS = {}


def sweep(tracker, seed):
    """The turret sweeping (its target held just off the image centre) across
    a random no-fire stripe while the operator keeps pressing Fire and
    acknowledges faults. Returns (rig, zone, direction, blocked: the times the
    zone refused or faulted a shot)."""
    if seed in SWEEPS:
        return SWEEPS[seed]
    rng = random.Random(seed)
    direction = rng.choice((-1.0, 1.0))
    start = rng.uniform(0.3, 0.45)  # the sweep runs at 0.05-0.2 rad/s
    zone = [start, start + rng.uniform(0.02, 0.15)]
    if direction < 0:
        zone = [-zone[1], -zone[0]]
    zone += [-1.6, 1.6]
    rig, centre = locked_rig(tracker, darts=0, no_fire_zones_rad=zone,
                             no_fire_margin_rad=rng.choice((0.02, 0.05)), tag_min_interval_s=0.0)
    rig.command(0.61, action='reload', darts=1000)
    offset = direction * rng.uniform(8.0, 14.0)  # px (under 2 deg): a steady sweep, still locked
    t, blocked = 0.62, []
    while t < 16.0:
        rig.run(t, t + 0.3, lambda t: two_people(dx=offset))
        t = round(t + 0.32, 9)
        status = rig.status(t)
        if status['launcher'] == 'fault':
            if 'line of fire' in status['last_tag']['reason']:
                blocked.append(t)
            rig.set_estop(t, True)
            rig.set_estop(t + 0.02, False)
            t = round(t + 0.04, 9)
        elif status['fire_block'] == '':
            rig.command(t, action='tag', target_id=centre)
        elif status['fire_block'] == 'line of fire blocked':
            blocked.append(t)
    SWEEPS[seed] = rig, zone, direction, blocked
    return SWEEPS[seed]


@pytest.mark.parametrize('seed', range(20))
def test_no_shot_leaves_inside_a_no_fire_zone(tracker, seed):
    rig, zone, direction, _ = sweep(tracker, seed)
    pan_min, pan_max, tilt_min, tilt_max = zone
    for when, pan, tilt in rig.shot_poses:
        assert not (pan_min <= pan <= pan_max and tilt_min <= tilt <= tilt_max), (when, pan)
    assert (rig.pan > zone[1]) if direction > 0 else (rig.pan < zone[0]), (rig.pan, zone)


def test_the_sweeps_shoot_on_both_sides_of_the_zone_and_meet_it(tracker):
    sessions = [sweep(tracker, seed) for seed in range(20)]
    before = after = 0
    for rig, zone, direction, _ in sessions:
        near, far = (zone[0], zone[1]) if direction > 0 else (zone[1], zone[0])
        before += sum((pan - near) * direction < 0 for _, pan, _ in rig.shot_poses)
        after += sum((pan - far) * direction > 0 for _, pan, _ in rig.shot_poses)
    assert before >= 3 and after >= 20
    assert sum(bool(blocked) for *_, blocked in sessions) >= 10


# --- next, auto_next, clear_tags, status --------------------------------------------

def test_auto_next_aims_at_the_nearest_untagged_and_never_fires(tracker):
    rig, centre = locked_rig(tracker, darts=5, auto_next=True)
    people = lambda t: [person_at_torso(CX + 4.0, CY + 2.0), person_at_torso(120.0, CY),
                        person_at_torso(CX + 150.0, CY)]
    rig.run(0.6, 0.7, people)
    ids = [segment['id'] for segment in rig.segments[-1]]
    rig.command(0.71, action='tag', target_id=centre)
    rig.run(0.72, 6.0, people)
    status = rig.status(6.0)
    assert status['tagged_ids'] == [centre] and status['darts'] == 4
    assert status['target_id'] == ids[2] and status['mode'] == 'auto'  # 150 px, nearer than 200
    shots, firing = len(rig.shots), max(rig.firing)
    rig.run(6.02, 12.0, people)
    assert len(rig.shots) == shots and max(rig.firing) == firing  # aiming only
    assert rig.command(12.0, action='next')['target_id'] == ids[1]
    assert rig.command(12.01, action='next')['target_id'] == ids[2]
    status = rig.command(12.02, action='clear_tags')
    assert (status['tags'], status['tagged_ids']) == (0, [])
    assert rig.command(12.03, action='next')['target_id'] == centre
    assert rig.command(12.04, action='auto_next', enabled=False)['auto_next'] is False
    assert 'auto_next needs enabled' in rig.command(12.05, action='auto_next', enabled='yes')[
        'message']
    assert len(rig.shots) == shots


def test_status_fields_and_the_contract(tracker):
    rig, centre = locked_rig(tracker)
    status = rig.status(0.6)
    assert set(status) >= {'launcher', 'darts', 'tags', 'tagged_ids', 'last_tag', 'fire_block',
                           'auto_next'}
    json.dumps(status)
    contract = (REPO / 'systems/racecar_turret/interfaces.md').read_text()
    for state in ('none', 'ready', 'priming', 'chambering', 'firing', 'empty', 'fault'):
        assert f'"{state}"' in contract
    for action in ('tag', 'next', 'clear_tags', 'reload', 'auto_next'):
        assert f'"action": "{action}"' in contract
    for field in ('"launcher"', '"darts"', '"tags"', '"tagged_ids"', '"last_tag"', '"fire_block"'):
        assert field in contract
    assert {'target_id', 'result', 'stamp'} <= set(
        rig.command(0.61, action='tag', target_id=centre)['last_tag'])


def test_the_profiles_leave_the_launcher_to_the_compositions(tracker):
    for name in ('turret_tracker.yaml', 'turret_tracker_isaac.yaml'):
        params = profile(name)
        assert params['actuator_fitted'] is False and params['auto_next'] is False
        assert params['darts'] == 0
        # the car's no-fire zone on both (test_the_car_s_no_fire_zone)
        assert params['no_fire_zones_rad'] == [-0.17, 0.26, -1.5708, -0.42]
    # The drop table where a launcher is fitted; none in Isaac (no launcher)
    assert profile('turret_tracker_isaac.yaml')['aim_offset_pitch_deg'] == [0.0, 0.0]
    assert profile('turret_tracker.yaml')['aim_offset_pitch_deg'] != [0.0, 0.0]
    for name in ('actuator_fitted', 'auto_next'):
        assert tracker.DEFAULT_PARAMETERS[name] is False
    compose = yaml.safe_load((REPO / 'compositions/racecar_turret_hardware.yaml').read_text())
    tracker_command = compose['services']['turret_tracker']['command']
    assert 'actuator_fitted:=${TURRET_ACTUATOR_FITTED:-true}' in tracker_command
    servos = compose['services']['turret_servos']['environment']
    assert 'TURRET_ACTUATOR_FITTED=${TURRET_ACTUATOR_FITTED:-true}' in servos


# --- Safety: properties and structure -----------------------------------------------

def random_scene(rng):
    """People where a detector might report them: often one with its torso at
    the image centre (locked, a small aim error), others anywhere, some too
    close, other classes."""
    scene = []
    if rng.random() < 0.7:
        h = rng.choice((PERSON_H, PERSON_H, 320.0))
        scene.append([CX + rng.uniform(-6, 6), CY + rng.uniform(-6, 6) - 0.1 * h, 40.0, h,
                      'person'])
    for _ in range(rng.choice((0, 1, 2, 4))):
        scene.append([rng.uniform(0, WIDTH), rng.uniform(0, HEIGHT), rng.uniform(10, 200),
                      rng.uniform(10, 400), rng.choice(('person', 'person', 'dog'))])
    return scene


def random_command(rng, ids):
    """Every command but tag, valid and not."""
    ident = rng.choice(ids + ['', '99', 'x']) if ids else rng.choice(('1', '', 'x'))
    return rng.choice((
        {'action': 'lock', 'target_id': ident},
        {'action': 'lock', 'target_id': ident},
        {'action': 'lock', 'point': [rng.random(), rng.random()]},
        {'action': 'lock', 'point': [rng.uniform(-1, 2), 'x']},
        {'action': 'release'},
        {'action': 'center'},
        {'action': 'manual', 'pan_rate': rng.uniform(-3, 3), 'tilt_rate': rng.uniform(-3, 3)},
        {'action': 'manual', 'pan_rate': 'fast'},
        {'action': 'next'},
        {'action': 'clear_tags'},
        {'action': 'reload', 'darts': rng.choice((0, 1, 6, -1, 'x'))},
        {'action': 'reload', 'darts': 6},
        {'action': 'auto_next', 'enabled': rng.random() < 0.5},
        {'action': 'auto_next', 'enabled': 'on'},
        {'action': 'fire', 'target_id': ident},
        {'action': 'TAG', 'target_id': ident},
        {'action': ['tag'], 'target_id': ident},
        {'target_id': ident},
        'tag',
        None,
    ))


def fuzz(tracker, seed, steps, tags=False):
    """A random session: a scene that persists between frames and changes
    now and then, with the other events in between. Returns the rig and,
    with tags, the accepted tags with the state the gates saw just before."""
    rng = random.Random(seed)
    # Half the sessions are calm (armed, ready and stepping in short steps
    # most of the time), so tags get through and the second property sees
    # accepted ones.
    calm = rng.random() < 0.5
    rig = Rig(tracker, auto_next=rng.random() < 0.5,
              require_estop_release=not calm and rng.random() < 0.5,
              auto_start=calm,  # calm: follow the person at the centre from the start
              darts=rng.choice((3, 6)) if calm else rng.choice((0, 3, 6)),
              tag_min_interval_s=rng.choice((0.0, 1.0, 2.0)),
              tag_dwell_s=rng.choice((0.0, 0.2)),
              # 0.5 s is too short for the 38 mm draw (0.9 s): those cycles fault
              tag_stroke_timeout_s=3.0 if calm else rng.choice((0.5, 3.0)))
    steps_s = (0.0, 0.005, 0.02, 0.02, 0.02, 0.02, 0.05, 0.05) + (() if calm else (0.3, 1.2))
    accepted = []
    t = 0.0
    ids = []
    scene = random_scene(rng)
    for _ in range(steps):
        t = round(t + rng.choice(steps_s), 6)
        roll = rng.random()
        if roll < 0.35:
            rig.advance(t)
            if rng.random() < 0.05:  # the launcher found anywhere, even away from rest
                rig.rack = rng.choice((0.0, 0.02, STROKE, -0.002))
                rig.trigger = rng.choice((0.0, 0.0, 0.3, FIRE))
                rig.rack_goal, rig.trigger_goal = rig.rack, rig.trigger
                rig.advance(t)
            rig.report_launcher = rng.random() < 0.97
        elif roll < 0.55:
            if rng.random() < (0.01 if calm else 0.05):
                scene = random_scene(rng)
            seen = [detection(u + rng.uniform(-1, 1), v + rng.uniform(-1, 1), w, h, cls)
                    for u, v, w, h, cls in scene if rng.random() < 0.9]
            ids = [segment['id'] for segment in rig.frame(t, seen)] or ids
        elif roll < 0.7:
            rig.tick(t)
        elif roll < 0.73:
            if not calm or rng.random() < 0.2:
                rig.set_estop(t, rng.random() < 0.25)
        elif roll < 0.75:
            if not calm or rng.random() < 0.2:
                rig.ready = rng.choice((True, True, True, True, True, False, None))
                rig.at(t).node._actuator_ready(NS(data=bool(rig.ready)))
        elif roll < 0.77:
            rate = [rng.uniform(-1, 1) for _ in range(3)]
            rig.at(t).node._imu(NS(header=NS(stamp=stamp(T0 + t), frame_id='turret/base_link'),
                                   angular_velocity=NS(x=rate[0], y=rate[1], z=rate[2])))
        elif tags and roll < 0.87:
            status = rig.status(t)
            target = rng.choice((status['target_id'], status['target_id'],
                                 rng.choice(ids) if ids else '', '42'))
            before = dict(status, estop=rig.estop, ready=rig.node._actuator_ready_fresh(T0 + t),
                          measured=rig.node._launcher_position(T0 + t),
                          since_last=T0 + t - rig.node.last_cycle_end,
                          too_close=rig.node.last_frame and rig.node.last_frame['too_close'])
            result = rig.command(t, action='tag', target_id=target)
            if result['last_tag']['result'] == 'running':
                accepted.append((t, target, before))
        else:
            command = random_command(rng, ids)
            if calm and isinstance(command, dict) and command.get('action') in (
                    'lock', 'release', 'center', 'manual', 'next'):
                continue  # calm: no command that moves the aim, so cycles finish
            rig.at(t).node._turret_command(NS(data=json.dumps(command)))
    return rig, accepted


@pytest.mark.parametrize('seed', range(60))
def test_no_sequence_without_a_tag_command_primes_or_fires(tracker, seed):
    rig, _ = fuzz(tracker, seed, 400)
    assert rig.firing == [] and rig.priming == [] and rig.shots == []
    assert all(joints.get('trigger_joint', 0.0) == 0.0 for _, joints in rig.commands)
    assert rig.node.fire_cycle.phase not in ('priming', 'chambering', 'firing', 'resetting')
    assert rig.node.last_tag is None or rig.node.last_tag['result'] == 'refused'


def test_the_fuzz_commands_the_turret_and_moves_the_launcher(tracker):
    rigs = [fuzz(tracker, seed, 400)[0] for seed in range(60)]
    assert sum(bool(rig.commands) for rig in rigs) >= 45
    # The launcher was found away from rest and brought back without a tag
    assert sum(any('actuator_joint' in joints for _, joints in rig.commands)
               for rig in rigs) >= 45


@pytest.mark.parametrize('seed', range(60))
def test_every_prime_and_shot_follows_an_accepted_tag_whose_gates_held(tracker, seed):
    rig, accepted = fuzz(tracker, 1000 + seed, 600, tags=True)
    node = rig.node
    for t, target, before in accepted:
        assert before['mode'] == 'auto' and before['target_id'] == target != ''
        assert before['locked'] and before['aim_error_deg'] < node.tag_max_aim_error_deg
        assert not before['too_close']
        assert not before['estop'] and before['ready']
        assert before['launcher'] == 'ready' and before['darts'] > 0
        assert before['measured'] is not None and node.fire_cycle.at_rest(*before['measured'])
        assert before['since_last'] >= node.tag_min_interval_s
        assert before['fire_block'] == ''
    # Priming and firing commands, and shots, only within an accepted tag's
    # time bound (three steps' timeouts and the trigger's dwell)
    cycle = node.fire_cycle
    bound = 3 * cycle.timeout_s + cycle.dwell_s + 0.05
    for t in rig.priming + rig.firing + rig.shots:
        assert any(start <= t <= start + bound for start, _, _ in accepted), t
    assert len(rig.shots) <= len(accepted)


def test_the_fuzz_reaches_accepted_tags_and_shots(tracker):
    sessions = [fuzz(tracker, 1000 + seed, 600, tags=True) for seed in range(60)]
    assert sum(bool(accepted) for _, accepted in sessions) >= 10
    assert sum(len(accepted) for _, accepted in sessions) >= 12
    assert sum(len(rig.shots) for rig, _ in sessions) >= 2


def test_priming_and_firing_are_reachable_only_from_the_tag_handler():
    tree = ast.parse(SCRIPT.read_text())
    classes = {node.name: node for node in tree.body if isinstance(node, ast.ClassDef)}
    functions = {}
    for class_node in classes.values():
        for item in class_node.body:
            if isinstance(item, ast.FunctionDef):
                functions[(class_node.name, item.name)] = item

    def enclosing(target):
        """(class, method) of every node in the module that satisfies target."""
        found = []
        for (class_name, name), function in functions.items():
            found += [(class_name, name) for node in ast.walk(function) if target(node)]
        top = [node for node in tree.body if not isinstance(node, ast.ClassDef)]
        found += [('<module>', '') for item in top for node in ast.walk(item) if target(node)]
        return found

    def is_attribute(node, attribute):
        return isinstance(node, ast.Attribute) and node.attr == attribute

    def entered(phase):
        return enclosing(lambda node: isinstance(node, ast.Assign) and any(
            isinstance(element, ast.Attribute) and element.attr == 'phase'
            for target in node.targets
            for element in (target.elts if isinstance(target, ast.Tuple) else [target]))
            and any(is_attribute(value, phase) for value in ast.walk(node.value)))

    # Priming is entered only in FireCycle.start, chambering and firing only
    # in update() (from priming and chambering) ...
    assert entered('PRIMING') == [('FireCycle', 'start')]
    assert entered('CHAMBERING') == [('FireCycle', 'update')]
    assert entered('FIRING') == [('FireCycle', 'update')]
    update = ast.unparse(functions[('FireCycle', 'update')])
    assert 'if phase == self.PRIMING:' in update and 'elif phase == self.CHAMBERING:' in update
    # ... and firing only with the line of fire clear at that moment: the
    # branch that enters it faults first when it is not, and the node gives
    # update() the check of the measured and commanded joints every time
    branch = next(node for node in ast.walk(functions[('FireCycle', 'update')])
                  if isinstance(node, ast.If) and ast.unparse(node.test) == 'rack <= self.rack_tolerance')
    assert [ast.unparse(statement) for statement in branch.body] == [
        "if not clear:\n    return self._fault(now, 'line of fire blocked')",
        'self.phase, self.since = (self.FIRING, float(now))']
    assert "if self.released == -math.inf and (not clear):" in update
    updates = [node for function in functions.values() for node in ast.walk(function)
               if isinstance(node, ast.Call) and ast.unparse(node.func) == 'cycle.update']
    assert len(updates) == 1 and [ast.unparse(k) for k in updates[0].keywords] == [
        'clear=self._line_of_fire_clear(now)']
    assert enclosing(lambda node: isinstance(node, ast.Call) and is_attribute(
        node.func, 'update') and ast.unparse(node.func).endswith('fire_cycle.update')) == []
    # ... the rack is asked back (the stroke) only while priming and the
    # trigger off 0 only while firing, each bounded in time
    goals = functions[('FireCycle', 'goals')]
    assert [ast.unparse(node) for node in goals.body[1:]] == [
        'if self.phase == self.PRIMING:\n'
        '    return (self.stroke if now - self.since <= self.timeout_s else None, 0.0)',
        'if self.phase == self.FIRING and now - self.since <= self.timeout_s + self.dwell_s:\n'
        '    return (0.0, self.fire)',
        'if self.phase == self.FAULT:\n    return (None, 0.0)',
        'return (0.0, 0.0)']
    stroke = enclosing(lambda node: is_attribute(node, 'stroke') and isinstance(node.ctx, ast.Load))
    fire = enclosing(lambda node: is_attribute(node, 'fire') and isinstance(node.ctx, ast.Load))
    assert set(stroke) == {('FireCycle', 'goals'), ('FireCycle', 'update')}
    assert set(fire) == {('FireCycle', 'goals'), ('FireCycle', 'update')}
    # The node commands the launcher only through goals(), in _send, where a
    # held rack is its measured position
    assert enclosing(lambda node: isinstance(node, ast.Call) and is_attribute(
        node.func, 'goals')) == [('PanTiltYoloTracker', '_send')]
    # (a held rack is no rack in the command: the bridge stops it where it is)
    send = ast.unparse(functions[('PanTiltYoloTracker', '_send')])
    assert 'if rack is not None:\n            command.name.append(ACTUATOR_JOINT)' in send
    # (and no command carries the trigger out while the line of fire is blocked)
    assert ('rack, trigger = self.fire_cycle.goals(now)\n'
            '        if trigger > 0.0 and (not self._line_of_fire_clear(now)):\n'
            '            trigger = 0.0\n') in send
    # start() is called only from the tag handler ...
    starts = enclosing(lambda node: isinstance(node, ast.Call) and is_attribute(
        node.func, 'start'))
    assert starts == [('PanTiltYoloTracker', '_tag')]
    # ... the handler only from the command callback, for action "tag" ...
    calls = enclosing(lambda node: isinstance(node, ast.Call) and is_attribute(node.func, '_tag'))
    assert calls == [('PanTiltYoloTracker', '_turret_command')]
    dispatch = functions[('PanTiltYoloTracker', '_turret_command')]
    branch = next(node for node in ast.walk(dispatch) if isinstance(node, ast.If)
                  and ast.unparse(node.test) == "action == 'tag'")
    assert [ast.unparse(statement) for statement in branch.body] == ['self._tag(command, now)']
    # ... after the gates, and the callback only from the command subscription.
    tag = functions[('PanTiltYoloTracker', '_tag')]
    statements = [ast.unparse(statement) for statement in tag.body]
    refusal = statements.index('refusal = self._fire_refusal(now, target_id)')
    assert statements[refusal + 1].startswith('if refusal:') and statements[
        refusal + 1].rstrip().endswith('return')
    assert refusal < next(i for i, text in enumerate(statements) if '.start(' in text)
    assert enclosing(lambda node: is_attribute(node, '_turret_command')) == [
        ('PanTiltYoloTracker', '__init__')]
    # Nothing else assigns the cycle's phase from outside FireCycle
    assert enclosing(lambda node: isinstance(node, ast.Assign) and any(
        'phase' in ast.unparse(target) and 'fire_cycle' in ast.unparse(target)
        for target in node.targets)) == []
