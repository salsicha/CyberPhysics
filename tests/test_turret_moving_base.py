"""PanTiltROS tracker on a moving base (the RACECAR Turret): IMU base-rotation
compensation, target lead, /turret/command, /turret/status and the
/turret/yolo/segments format (systems/racecar_turret/interfaces.md).

The closed-loop runs drive the tracker node's own callbacks (IMU, joint
states, detections, the control timer) with the turret_sim servo model
mounted on a car that yaws, rolls and drives, seen through a pinhole camera
on the turret geometry of systems/racecar_turret/urdf. They run seconds of
node (simulation) time in milliseconds of wall time, so nothing in the loop
may read the wall clock.
"""
import copy
import json
import math
import sys
from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script

SCRIPT = REPO / 'applications/pantiltros/scripts/pantilt_yolo_tracker.py'
PROFILES = REPO / 'systems/racecar_turret/config'
SCENARIO = json.loads(
    (REPO / 'applications/turret_sim/scenarios/warehouse_tracking.json').read_text())
T0 = 1000.0  # node clock when a run starts (simulation time)
WIDTH, HEIGHT = 640, 480
FX = WIDTH / 2 / math.tan(math.radians(62.0) / 2)
FY = HEIGHT / 2 / math.tan(math.radians(48.0) / 2)
MASK = np.zeros((1, 1), dtype=np.uint8)
# Turret on the car (racecar_turret.urdf.xacro): mount, pan and tilt axis
# heights, camera ahead of the tilt axis.
MOUNT = np.array([-0.10, 0.0, 0.24])
PAN_Z, TILT_Z, CAMERA_X = 0.045, 0.055, 0.035


@pytest.fixture(scope='module')
def tracker():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('diagnostic_msgs', 'diagnostic_msgs.msg', 'rclpy.executors', 'ultralytics'):
            sys.modules[name] = MagicMock()
        module = load_script(SCRIPT, 'pantilt_yolo_tracker_moving_base')
        module.JointState = lambda: NS(header=NS(stamp=None), name=[], position=[])
        module.String = lambda data='': NS(data=data)
        module.mask_message = lambda mask, source: None
        yield module


@pytest.fixture(scope='module')
def turret_sim():
    sys.path.insert(0, str(REPO / 'applications/turret_sim/scripts'))
    try:
        yield load_script(REPO / 'applications/turret_sim/scripts/turret_sim.py',
                          'turret_sim_moving_base')
    finally:
        sys.path.remove(str(REPO / 'applications/turret_sim/scripts'))


def profile(name):
    return yaml.safe_load((PROFILES / name).read_text())['pantilt_yolo_tracker']['ros__parameters']


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quaternion(m):
    """(x, y, z, w) of a rotation matrix with a positive trace."""
    w = math.sqrt(1.0 + m[0, 0] + m[1, 1] + m[2, 2]) / 2
    return ((m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w),
            (m[1, 0] - m[0, 1]) / (4 * w), w)


def stamp(t):
    return NS(sec=int(t), nanosec=int(round((t % 1) * 1e9)))


def joint_state(t, pan, tilt):
    return NS(name=['pan_joint', 'tilt_joint'], position=[pan, tilt], header=NS(stamp=stamp(t)))


def imu(t, rate, frame='racecarneo/imu_link'):
    return NS(header=NS(stamp=stamp(t), frame_id=frame),
              angular_velocity=NS(x=rate[0], y=rate[1], z=rate[2]))


def person(u, v, w=40.0, h=120.0, cls='person', confidence=0.9):
    return {'id': '', 'class': cls, 'confidence': confidence, 'bbox_xywh': [u, v, w, h],
            'bbox_xyxy': [u - w / 2, v - h / 2, u + w / 2, v + h / 2],
            'centroid_px': [u, v], 'mask_area_px': int(w * h), 'mask': MASK}


class FakeTF:
    """tf2_ros.Buffer stand-in: static rotations of IMU frames into turret/base_link."""

    def __init__(self, rotations):
        self.rotations = rotations

    def lookup_transform(self, target, source, _time):
        if target != 'turret/base_link' or source not in self.rotations:
            raise LookupError(f'{source} -> {target}')
        x, y, z, w = quaternion(self.rotations[source])
        return NS(transform=NS(rotation=NS(x=x, y=y, z=z, w=w)))


class Harness:
    """The tracker node without ROS: its callbacks, with publishers captured."""

    def __init__(self, tracker, profile_name='turret_tracker.yaml', imu_rotation=None, **overrides):
        parameters = dict(tracker.DEFAULT_PARAMETERS)
        if profile_name:
            parameters.update(profile(profile_name))
        parameters.update(publish_annotated_image=False, show_window=False)
        parameters.update(overrides)
        parameters.pop('use_sim_time', None)
        node = tracker.PanTiltYoloTracker.__new__(tracker.PanTiltYoloTracker)
        node.clock_s = T0
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(round(node.clock_s * 1e9)),
                                                   to_msg=lambda: None))
        node.get_logger = MagicMock()
        node.get_parameter = lambda name: NS(value=parameters[name])
        node._configure()
        self.commands, self.segments, self.statuses = [], [], []
        node.command_pub = NS(publish=lambda msg: self.commands.append(
            (node.clock_s, tuple(msg.position))))
        node.segments_pub = NS(publish=lambda msg: self.segments.append(json.loads(msg.data)))
        node.status_pub = NS(publish=lambda msg: self.statuses.append(json.loads(msg.data)))
        self.selected_ids = []
        node.selected_id_pub = NS(publish=lambda msg: self.selected_ids.append(msg.data))
        for name in ('mask_pub', 'bbox_pub', 'confidence_pub', 'pixel_error_pub',
                     'angular_error_pub', 'annotated_pub', 'locked_pub', 'diagnostics_pub'):
            setattr(node, name, MagicMock())
        node.camera_info = NS(k=[FX, 0.0, WIDTH / 2, 0.0, FY, HEIGHT / 2, 0.0, 0.0, 1.0])
        node.tf_buffer = FakeTF({'racecarneo/imu_link': np.eye(3) if imu_rotation is None
                                 else imu_rotation})
        self.node = node

    def at(self, t):
        self.node.clock_s = T0 + t
        return self

    def frame(self, t, detections, captured=None):
        captured = t if captured is None else captured
        self.at(t)
        source = NS(header=NS(stamp=stamp(T0 + captured), frame_id='turret/camera_optical_frame'),
                    height=HEIGHT, width=WIDTH)
        self.node._process_frame(source, None, detections, 10.0, T0 + captured)
        return self.segments[-1]

    def command(self, t, **command):
        self.at(t)
        self.node._turret_command(NS(data=json.dumps(command)))
        return self.statuses[-1]

    def status(self, t):
        self.at(t)
        self.node._publish_status()
        return self.statuses[-1]

    def joints(self, t, pan=0.0, tilt=0.0):
        self.at(t)
        self.node._joint_state(joint_state(T0 + t, pan, tilt))

    def tick(self, t):
        self.at(t)
        self.node._control_tick()


# Closed loop on a moving car ------------------------------------------------

class Scene:
    """A car driving at `speed` along its heading yaw(t), rolled by roll(t)
    about its x axis, and a target point (the detection centroid)."""

    def __init__(self, target, speed=0.0, yaw=(lambda t: 0.0), yaw_rate=(lambda t: 0.0),
                 roll=(lambda t: 0.0), roll_rate=(lambda t: 0.0), size=(0.5, 1.7),
                 duration=8.0):
        self.target, self.speed, self.size, self.duration = target, speed, size, duration
        self.yaw, self.yaw_rate, self.roll, self.roll_rate = yaw, yaw_rate, roll, roll_rate

    def rotation(self, t):
        return rot_z(self.yaw(t)) @ rot_x(self.roll(t))

    def body_rate(self, t):
        """Angular velocity in base_link axes."""
        return rot_x(-self.roll(t)) @ np.array([0.0, 0.0, self.yaw_rate(t)]) + np.array(
            [self.roll_rate(t), 0.0, 0.0])


def slalom(peak_rate=0.94, frequency=0.3):
    amplitude = peak_rate / (2 * math.pi * frequency)
    return Scene(target=lambda t: np.array([14.0, 3.0, 0.9]), speed=1.0,
                 yaw=lambda t: amplitude * math.sin(2 * math.pi * frequency * t),
                 yaw_rate=lambda t: peak_rate * math.cos(2 * math.pi * frequency * t))


def camera_pose(position, rotation, pan, tilt):
    pan_rotation, tilt_rotation = rot_z(-pan), rot_y(-tilt)
    offset = MOUNT + np.array([0.0, 0.0, PAN_Z]) + pan_rotation @ (
        np.array([0.0, 0.0, TILT_Z]) + tilt_rotation @ np.array([CAMERA_X, 0.0, 0.0]))
    return position + rotation @ offset, rotation @ pan_rotation @ tilt_rotation


def run_car(tracker, turret_sim, scene, compensation=True, lead=True, actuation_s=0.12,
            imu_rotation=None, imu_until=math.inf, seed=3, settle_s=1.5):
    """Aim error (degrees) between the camera's optical axis and its line of
    sight to the target, from settle_s on, with the hardware profile; the servo
    starts moving actuation_s after a command (the profile assumes 0.15 s)."""
    imu_rotation = np.eye(3) if imu_rotation is None else imu_rotation
    harness = Harness(tracker, base_compensation=compensation, target_lead=lead,
                      imu_rotation=imu_rotation)
    node = harness.node
    plant = copy.deepcopy(SCENARIO)
    plant['pantilt']['command_latency_s'] = actuation_s
    sim = turret_sim.TurretSim(plant)
    rng = np.random.default_rng(seed)
    gyro_bias = rng.normal(0.0, 0.005, 3)
    dt, camera_period, inference_s = 0.005, 1.0 / 15.0, 0.07
    position = np.zeros(3)
    sight = scene.rotation(0.0).T @ (scene.target(0.0) - MOUNT)
    sim.pan, sim.tilt = tracker.direction_joints(sight)
    sim.pan_cmd, sim.tilt_cmd = sim.pan, sim.tilt
    harness.at(0.0)
    node._emergency_stop(NS(data=False))
    frames, errors, statuses = deque(), [], []
    next_frame, locked, sent = 0.0, False, 0
    for k in range(int(round(scene.duration / dt)) + 1):
        t = k * dt
        harness.at(t)
        rotation = scene.rotation(t)
        position = position + scene.speed * dt * rotation[:, 0]
        sim.update_servo(t, dt, False)
        if k % 2 == 0 and t < imu_until:
            rate = imu_rotation.T @ scene.body_rate(t) + gyro_bias + rng.normal(0.0, 0.003, 3)
            node._imu(imu(T0 + t, rate))
        if k % 4 == 0:
            node._joint_state(joint_state(T0 + t, sim.pan, sim.tilt))
        camera, camera_rotation = camera_pose(position, rotation, sim.pan, sim.tilt)
        sight = camera_rotation.T @ (scene.target(t) - camera)
        if t >= next_frame - 1e-9:
            next_frame += camera_period
            seen = []
            if sight[0] > 0.2:
                u = WIDTH / 2 - FX * sight[1] / sight[0] + rng.normal(0.0, 1.0)
                v = HEIGHT / 2 - FY * sight[2] / sight[0] + rng.normal(0.0, 1.0)
                distance = float(np.linalg.norm(sight))
                if 0 <= u < WIDTH and 0 <= v < HEIGHT:
                    seen = [person(u, v, FX * scene.size[0] / distance,
                                   FY * scene.size[1] / distance)]
            frames.append((t + inference_s, t, seen))
        while frames and frames[0][0] <= t + 1e-9:
            _, captured, seen = frames.popleft()
            harness.frame(t, seen, captured)
            if seen and not locked:
                u, v = seen[0]['bbox_xywh'][:2]
                assert harness.command(t, action='lock', point=[u / WIDTH, v / HEIGHT])[
                    'mode'] == 'auto'
                locked = True
        if k % 4 == 0:
            node._control_tick()
        for _, (pan, tilt) in harness.commands[sent:]:
            sim.command_queue.append((t + actuation_s, pan, tilt))
        sent = len(harness.commands)
        if k % 40 == 0:
            statuses.append((t, harness.status(t)))
        if t >= settle_s:
            errors.append(math.degrees(math.atan2(math.hypot(sight[1], sight[2]), sight[0])))
    assert sim.joint_limit_violations == 0
    return NS(mean=float(np.mean(errors)), p95=float(np.percentile(errors, 95)),
              max=float(np.max(errors)), statuses=statuses, delay=node.controller.delay)


def report(label, result):
    print(f'{label}: mean {result.mean:.2f} deg, p95 {result.p95:.2f} deg, '
          f'max {result.max:.2f} deg')


def test_turret_geometry_matches_the_urdf(tracker):
    for pan, tilt in [(0.3, 0.2), (-1.2, -0.5), (1.5, 0.7)]:
        axis = rot_z(-pan) @ rot_y(-tilt) @ np.array([1.0, 0.0, 0.0])
        assert tracker.joint_direction(pan, tilt) == pytest.approx(axis)
        assert tracker.direction_joints(axis) == pytest.approx((pan, tilt))
    # Positive pan turns the camera right (-y), positive tilt up (+z).
    assert tracker.joint_direction(0.1, 0.0)[1] < 0 and tracker.joint_direction(0.0, 0.1)[2] > 0
    # A point seen right of and above the image centre lies right of and
    # above the optical axis, whatever the joints.
    for pan, tilt in [(0.0, 0.0), (0.8, 0.5), (-0.6, -0.4)]:
        ray = tracker.camera_ray(pan, tilt, 0.1, 0.05)
        camera = (rot_z(-pan) @ rot_y(-tilt)).T @ ray
        assert math.atan2(-camera[1], camera[0]) == pytest.approx(0.1)
        assert math.atan2(camera[2], camera[0]) == pytest.approx(0.05)
    rotation = rot_z(0.7) @ rot_x(0.4) @ rot_y(-0.3)
    assert tracker.quaternion_matrix(*quaternion(rotation)) == pytest.approx(rotation)


def test_base_rotation_carries_the_aim_through_yaw_and_roll(tracker):
    # Base yaw maps to pan one to one whatever the tilt: the axes are parallel.
    base = tracker.BaseRotation()
    for k in range(51):
        base.add(k * 0.01, [0.0, 0.0, 0.8])
    turned = base.attitude(0.5).T @ base.attitude(0.0)
    for tilt in (0.0, 0.6):
        pan, new_tilt = tracker.direction_joints(turned @ tracker.joint_direction(0.2, tilt))
        assert pan == pytest.approx(0.2 + 0.4, abs=1e-9) and new_tilt == pytest.approx(tilt)
    # Panned 90 degrees right, a roll that lowers the right side raises the
    # target in the turret: base roll couples into tilt through the pan angle.
    base = tracker.BaseRotation()
    for k in range(11):
        base.add(k * 0.01, [0.5, 0.0, 0.0])
    rolled = base.attitude(0.1).T @ base.attitude(0.0)
    pan, tilt = tracker.direction_joints(rolled @ tracker.joint_direction(math.pi / 2, 0.0))
    assert pan == pytest.approx(math.pi / 2) and tilt == pytest.approx(0.05)
    # Rates stop: the attitude freezes after extrapolating briefly.
    assert base.fresh(0.5) and not base.fresh(0.7)
    assert np.allclose(base.attitude(5.0), base.attitude(1.0))
    assert not base.add(0.2, [math.nan, 0.0, 0.0]) and not base.add(0.2, [50.0, 0.0, 0.0])


def test_bearing_rate_estimates_a_steady_crossing(tracker):
    rate = tracker.BearingRate(window_s=0.5)
    rng = np.random.default_rng(0)
    for k in range(10):
        t = k / 15
        azimuth = 0.3 * t + rng.normal(0.0, 0.002)
        rate.add(t, tracker.joint_direction(azimuth, 0.1))
    estimate = rate.rate(9 / 15 + 0.05)
    azimuth = 0.3 * 9 / 15
    expected = 0.3 * np.array([-math.sin(azimuth), -math.cos(azimuth), 0.0]) * math.cos(0.1)
    assert np.linalg.norm(estimate - expected) < 0.015
    assert rate.rate(9 / 15 + 0.6) is None  # stale
    rate.reset()
    assert rate.rate(0.0) is None


def test_base_compensation_keeps_a_still_target_still_while_the_car_yaws(tracker, turret_sim):
    scene = slalom(peak_rate=0.94)
    off = run_car(tracker, turret_sim, scene, compensation=False, lead=False)
    on = run_car(tracker, turret_sim, scene, compensation=True, lead=False)
    both = run_car(tracker, turret_sim, scene, compensation=True, lead=True)
    report('slalom 0.94 rad/s, fixed-base', off)
    report('slalom 0.94 rad/s, base compensation', on)
    report('slalom 0.94 rad/s, compensation and lead', both)
    assert off.mean > 8.0
    assert on.mean < 0.1 * off.mean and on.p95 < 2.0
    assert both.mean < 0.1 * off.mean and both.p95 < 2.0
    assert all(status['base_compensation'] for _, status in on.statuses[1:])
    assert not any(status['base_compensation'] for _, status in off.statuses)


def test_compensation_fits_the_actuation_delay(tracker, turret_sim):
    # The profile assumes 0.15 s from command to motion; this servo takes
    # 0.04 s. Predicting 0.11 s too far would point the turret ahead of the
    # target by the yaw rate times that; the delay fitted to the measured
    # joint states removes it.
    scene = slalom(peak_rate=0.94)
    off = run_car(tracker, turret_sim, scene, compensation=False, lead=False, actuation_s=0.04)
    on = run_car(tracker, turret_sim, scene, compensation=True, lead=False, actuation_s=0.04)
    report('slalom, servo 0.04 s, fixed-base', off)
    report(f'slalom, servo 0.04 s, base compensation (fitted delay {on.delay:.3f} s)', on)
    assert on.delay == pytest.approx(0.04, abs=0.015)
    assert on.mean < 0.1 * off.mean and on.p95 < 2.0
    assert run_car(tracker, turret_sim, scene, lead=False).delay == pytest.approx(0.12, abs=0.02)


def test_compensation_handles_roll_tilt_and_a_rotated_imu(tracker, turret_sim):
    # A target high above the road (tilt 0.3-0.4 rad) while the car yaws and
    # rolls; the IMU is mounted on its side and turned, which only TF knows.
    scene = Scene(target=lambda t: np.array([10.0, -2.5, 3.5]), speed=0.6,
                  yaw=lambda t: 0.4 * math.sin(2.2 * t),
                  yaw_rate=lambda t: 0.88 * math.cos(2.2 * t),
                  roll=lambda t: 0.06 * math.sin(6.9 * t),
                  roll_rate=lambda t: 0.414 * math.cos(6.9 * t),
                  size=(0.6, 0.25), duration=6.0)
    imu_rotation = rot_z(math.pi / 2) @ rot_x(0.5 * math.pi) @ rot_y(0.3)
    off = run_car(tracker, turret_sim, scene, compensation=False, lead=False,
                  imu_rotation=imu_rotation)
    on = run_car(tracker, turret_sim, scene, compensation=True, lead=False,
                 imu_rotation=imu_rotation)
    report('roll + yaw, elevated target, fixed-base', off)
    report('roll + yaw, elevated target, base compensation', on)
    assert on.mean < 0.15 * off.mean and on.p95 < 0.15 * off.p95


def test_target_lead_cuts_the_lag_on_a_crossing_target(tracker, turret_sim):
    # A person walking across at 1.5 m/s, 5 m in front of the parked car.
    walker = Scene(target=lambda t: np.array([5.0, 3.5 - 1.5 * t, 0.9]), duration=4.5)
    no_lead = run_car(tracker, turret_sim, walker, lead=False)
    with_lead = run_car(tracker, turret_sim, walker, lead=True)
    report('walker crossing at 1.5 m/s, no lead', no_lead)
    report('walker crossing at 1.5 m/s, lead', with_lead)
    assert with_lead.mean < 0.1 * no_lead.mean and with_lead.p95 < 0.15 * no_lead.p95
    # Driving past a person standing 2.5 m from the car's path at 2 m/s.
    drive_by = Scene(target=lambda t: np.array([8.0, 2.5, 0.9]), speed=2.0, duration=3.4)
    no_lead = run_car(tracker, turret_sim, drive_by, lead=False)
    with_lead = run_car(tracker, turret_sim, drive_by, lead=True)
    report('driving past at 2 m/s, no lead', no_lead)
    report('driving past at 2 m/s, lead', with_lead)
    assert with_lead.mean < 0.35 * no_lead.mean and with_lead.p95 < 0.4 * no_lead.p95


def test_imu_dropout_falls_back_to_a_fixed_base(tracker, turret_sim):
    result = run_car(tracker, turret_sim, slalom(peak_rate=0.5), lead=False, imu_until=4.0)
    before = [status for t, status in result.statuses if 2.0 <= t < 4.0]
    after = [status for t, status in result.statuses if t >= 4.6]
    assert before and all(status['base_compensation'] for status in before)
    assert after and not any(status['base_compensation'] for status in after)
    assert all('no IMU on /imu/data_raw for 0.5 s: fixed-base aiming' in status['message']
               for status in after)
    assert result.max < 25.0  # still tracking, with the fixed-base lag


# Commands, status and segments ---------------------------------------------

def armed(tracker, **overrides):
    harness = Harness(tracker, **overrides)
    harness.at(0.0)
    harness.node._emergency_stop(NS(data=False))
    return harness


def two_people(dx=0.0):
    return [person(200.0 + dx, 240.0), person(450.0 + dx, 250.0)]


def test_lock_by_id_and_by_point(tracker):
    harness = armed(tracker)
    harness.joints(0.0)
    first = harness.frame(0.0, two_people())
    assert harness.status(0.0)['mode'] == 'idle' and not harness.commands
    left, right = first[0]['id'], first[1]['id']
    assert left != right
    status = harness.command(0.01, action='lock', target_id=right)
    assert (status['mode'], status['target_id'], status['target_class']) == (
        'auto', right, 'person')
    harness.joints(0.05)
    segments = harness.frame(0.07, two_people(dx=4.0))
    assert [s['id'] for s in segments] == [left, right]
    assert [s['selected'] for s in segments] == [False, True]
    assert harness.selected_ids[-1] == right
    assert harness.commands[-1][1][0] > 0.0  # towards the person right of centre
    # A point in the left person's box; then a point on no box keeps the lock.
    status = harness.command(0.1, action='lock', point=[205 / WIDTH, 230 / HEIGHT])
    assert status['target_id'] == left and status['message'].startswith('locked on person')
    status = harness.command(0.12, action='lock', point=[0.95, 0.05])
    assert status['target_id'] == left and 'no detection at (0.95, 0.05)' in status['message']
    status = harness.command(0.13, action='lock', target_id='99')
    assert status['target_id'] == left and 'no detection with id 99' in status['message']
    harness.joints(0.13)
    harness.frame(0.14, two_people(dx=8.0))
    assert harness.commands[-1][1][0] < harness.commands[-2][1][0]  # now turning left
    # Overlapping boxes: the nearest box centre wins.
    harness.frame(0.2, [person(300.0, 240.0, w=200.0), person(340.0, 240.0, w=200.0)])
    ids = [s['id'] for s in harness.segments[-1]]
    assert harness.command(0.21, action='lock', point=[335 / WIDTH, 0.5])['target_id'] == ids[1]


def test_release_holds_the_aim(tracker):
    harness = armed(tracker)
    harness.joints(0.0)
    first = harness.frame(0.0, two_people())
    harness.command(0.01, action='lock', target_id=first[1]['id'])
    harness.frame(0.07, two_people())
    sent = len(harness.commands)
    assert sent
    status = harness.command(0.1, action='release')
    assert (status['mode'], status['target_id'], status['locked']) == ('idle', '', False)
    for k in range(10):
        harness.joints(0.1 + 0.05 * k)
        harness.frame(0.12 + 0.05 * k, two_people())
        harness.tick(0.13 + 0.05 * k)
    assert len(harness.commands) == sent


def test_manual_rates_are_clamped_and_stop_without_messages(tracker):
    harness = armed(tracker)
    harness.joints(0.0, pan=0.2, tilt=0.1)
    harness.frame(0.0, two_people())
    harness.command(0.0, action='lock', target_id='1')
    status = harness.command(0.0, action='manual', pan_rate=5.0, tilt_rate=-0.2)
    assert (status['mode'], status['target_id']) == ('manual', '')
    for k in range(1, 16):  # 0.3 s of 50 Hz ticks
        harness.joints(0.02 * k, *(harness.commands[-1][1] if harness.commands else (0.2, 0.1)))
        harness.tick(0.02 * k)
    pan, tilt = harness.commands[-1][1]
    # Clamped to max_rate_rad_s (1.0 in the hardware profile).
    assert pan == pytest.approx(0.2 + 1.0 * 0.3, abs=1e-6)
    assert tilt == pytest.approx(0.1 - 0.2 * 0.3, abs=1e-6)
    # Deadman: no message for 0.5 s and the turret holds.
    for k in range(16, 60):
        harness.joints(0.02 * k, *harness.commands[-1][1])
        harness.tick(0.02 * k)
    last_time, held = harness.commands[-1]
    assert last_time <= T0 + 0.5 + 1e-9
    # The UI repeating the command keeps it moving, up to the joint limit.
    for k in range(60, 300):
        harness.joints(0.02 * k, *harness.commands[-1][1])
        if k % 5 == 0:
            harness.command(0.02 * k, action='manual', pan_rate=1.0, tilt_rate=0.0)
        harness.tick(0.02 * k)
    assert harness.commands[-1][1][0] == pytest.approx(1.5708)
    assert held[0] < harness.commands[-1][1][0]


def test_center_ramps_to_zero_and_stops(tracker):
    harness = armed(tracker)
    harness.joints(0.0, pan=0.6, tilt=-0.3)
    status = harness.command(0.0, action='center')
    assert status['mode'] == 'idle' and 'centring' in status['message']
    for k in range(1, 60):
        harness.joints(0.02 * k, *(harness.commands[-1][1] if harness.commands else (0.6, -0.3)))
        harness.tick(0.02 * k)
    steps = np.diff([pan for _, (pan, _) in harness.commands])
    assert np.all(steps <= 1e-9) and np.max(np.abs(steps)) <= 1.0 * 0.02 + 1e-9
    assert harness.commands[-1][1] == (0.0, 0.0)
    assert len(harness.commands) == 30  # 0.6 rad at 1 rad/s, then nothing more
    assert 'centred' in harness.status(1.2)['message']


def test_safety_gates_are_unchanged(tracker):
    harness = Harness(tracker)  # require_estop_release: true
    harness.joints(0.0)
    first = harness.frame(0.0, two_people())
    harness.command(0.0, action='lock', target_id=first[1]['id'])
    harness.frame(0.07, two_people())
    harness.command(0.1, action='manual', pan_rate=0.5, tilt_rate=0.0)
    harness.tick(0.12)
    harness.command(0.13, action='center')
    harness.tick(0.14)
    assert not harness.commands
    status = harness.status(0.15)
    assert status['mode'] == 'estop' and 'emergency stop engaged' in status['message']
    harness.at(0.2).node._emergency_stop(NS(data=False))
    harness.joints(0.2, pan=0.1)
    harness.tick(0.22)
    assert harness.commands  # centring once released
    harness.at(0.3).node._emergency_stop(NS(data=True))
    sent = len(harness.commands)
    harness.tick(0.32)
    harness.command(0.33, action='manual', pan_rate=0.5, tilt_rate=0.0)
    harness.tick(0.34)
    assert len(harness.commands) == sent
    # No joint states and no ready actuator: nothing is sent either.
    idle = armed(tracker)
    idle.command(1.0, action='manual', pan_rate=0.5, tilt_rate=0.0)
    idle.tick(1.02)
    assert not idle.commands
    assert 'actuator not ready' in idle.status(1.03)['message']


def test_bad_commands_are_reported(tracker):
    harness = armed(tracker)
    harness.at(0.0).node._turret_command(NS(data='not json'))
    assert 'ignored command' in harness.statuses[-1]['message']
    assert 'ignored command' in harness.command(0.0, action='fire')['message']
    assert 'lock needs target_id or point' in harness.command(0.0, action='lock')['message']
    assert 'lock needs' in harness.command(0.0, action='lock', point=[1.5, 0.2])['message']
    assert 'no camera image yet' in harness.command(0.0, action='lock', point=[0.5, 0.5])['message']
    status = harness.command(0.0, action='manual', pan_rate='fast')
    assert 'manual needs numeric' in status['message'] and status['mode'] == 'idle'


def test_status_reports_the_contract_fields(tracker):
    harness = armed(tracker)
    harness.joints(0.0)
    status = harness.status(0.0)
    assert set(status) >= {'mode', 'target_id', 'target_class', 'locked', 'aim_error_deg',
                           'base_compensation', 'detections', 'message'}
    assert (status['mode'], status['target_id'], status['target_class'], status['locked'],
            status['aim_error_deg'], status['detections']) == ('idle', '', '', False, None, 0)
    assert 'no camera images' in status['message']
    assert 'no IMU on /imu/data_raw for 0.5 s: fixed-base aiming' in status['message']
    first = harness.frame(0.0, [person(330.0, 245.0), person(100.0, 240.0)])
    harness.command(0.01, action='lock', target_id=first[0]['id'])
    harness.frame(0.07, [person(330.0, 245.0), person(100.0, 240.0)])
    for k in range(10):
        harness.at(0.08 + 0.01 * k).node._imu(imu(T0 + 0.08 + 0.01 * k, [0.0, 0.0, 0.1]))
    status = harness.status(0.2)
    assert (status['mode'], status['target_id'], status['target_class'], status['locked'],
            status['detections'], status['base_compensation']) == (
        'auto', first[0]['id'], 'person', True, 2, True)
    assert status['aim_error_deg'] == pytest.approx(
        math.degrees(math.atan(math.hypot(10 / FX, 5 / FY))))
    assert status['stamp'] == pytest.approx(T0 + 0.2)
    json.dumps(status)  # plain JSON types only
    harness.frame(0.3, [person(100.0, 240.0)])
    status = harness.status(0.3)
    assert status['target_id'] == first[0]['id'] and not status['locked']
    assert status['aim_error_deg'] is None and 'searching for person' in status['message']
    # Without the IMU's transform there is no compensation either.
    lost = armed(tracker)
    lost.node.tf_buffer = FakeTF({})
    lost.at(0.0).node._imu(imu(T0, [0.0, 0.0, 0.1]))
    assert 'no transform from racecarneo/imu_link to turret/base_link: fixed-base aiming' in (
        lost.status(0.0)['message'])


def test_segments_carry_stable_ids_and_the_image(tracker):
    harness = armed(tracker)
    first = harness.frame(0.0, two_people())
    for segment in first:
        assert set(segment) >= {'id', 'class', 'confidence', 'bbox_xywh', 'bbox_xyxy',
                                'centroid_px', 'mask_area_px', 'stamp', 'image_width',
                                'image_height', 'selected'}
        assert isinstance(segment['id'], str) and segment['id']
        assert (segment['image_width'], segment['image_height']) == (WIDTH, HEIGHT)
        assert segment['stamp'] == pytest.approx(T0)
        u, v, w, h = segment['bbox_xywh']  # box centre and size
        assert segment['bbox_xyxy'] == pytest.approx([u - w / 2, v - h / 2, u + w / 2, v + h / 2])
        assert 'mask' not in segment
    ids = [segment['id'] for segment in first]
    # Moving objects keep their IDs; a new one gets a new ID.
    for k in range(1, 6):
        segments = harness.frame(0.07 * k, two_people(dx=12.0 * k) + [person(600.0, 100.0)])
        assert [s['id'] for s in segments[:2]] == ids
    new_id = segments[2]['id']
    assert new_id not in ids
    # Through a 0.6 s gap the ID survives (track_timeout_s 1.0); after 1.2 s it does not.
    assert harness.frame(1.0, []) == []
    assert harness.frame(1.0, [person(600.0, 100.0)])[0]['id'] == new_id
    harness.frame(2.3, [])
    assert harness.frame(2.3, [person(600.0, 100.0)])[0]['id'] not in ids + [new_id]


def test_startup_selection_still_follows_the_configured_class(tracker):
    # The simulator profile: track target_class from initial_selection at once.
    harness = Harness(tracker, profile_name=None, actuator_latency_s=0.05)
    harness.joints(0.0)
    cart = person(0.13 * WIDTH + 5.0, 0.565 * HEIGHT, cls='blue utility cart')
    other = person(400.0, 240.0, cls='green utility cart')
    segments = harness.frame(0.0, [other, cart])
    assert [s['selected'] for s in segments] == [False, True]
    assert harness.selected_ids[-1] == 'cart_blue_01' and harness.commands
    status = harness.status(0.0)
    assert (status['mode'], status['target_class'], status['target_id']) == (
        'auto', 'blue utility cart', segments[1]['id'])


def test_no_commands_until_simulation_time_starts(tracker):
    harness = armed(tracker, require_estop_release=False)
    harness.node.clock_s = 0.0  # use_sim_time before the first /clock
    harness.node._joint_state(joint_state(5.0, 0.1, 0.0))
    assert not harness.node.controller.measured
    harness.node._turret_command(NS(data=json.dumps({'action': 'center'})))
    harness.node._control_tick()
    assert not harness.commands and 'waiting for /clock' in harness.statuses[-1]['message']
    harness.joints(5.0, pan=0.1)
    harness.tick(5.02)
    assert harness.commands and harness.commands[-1][0] == pytest.approx(T0 + 5.02)


def test_racecar_profiles_are_complete_and_conservative(tracker):
    hardware, isaac = profile('turret_tracker.yaml'), profile('turret_tracker_isaac.yaml')
    for params in (hardware, isaac):
        assert set(params) - {'use_sim_time'} == set(tracker.DEFAULT_PARAMETERS)
        assert params['camera_topic'] == '/turret/camera/image_raw'
        assert params['camera_info_topic'] == '/turret/camera/camera_info'
        assert params['command_topic'] == '/turret/command'
        assert params['status_topic'] == '/turret/status'
        assert params['target_class'] == 'person' and 'person' in params['class_prompts']
        assert params['base_compensation'] is True and params['target_lead'] is True
        assert params['base_imu_topic'] == '/imu/data_raw'
        assert params['base_frame'] == 'turret/base_link'
        assert params['confidence'] >= 0.2 and params['association_gate_fraction'] <= 0.25
        assert params['kp'] <= 0.45
    assert 'use_sim_time' not in hardware and isaac['use_sim_time'] is True
    assert hardware['require_estop_release'] is True
    assert hardware['kp'] <= 0.3 and hardware['max_rate_rad_s'] <= 1.0
    # The fixed turret keeps the fixed-base loop exactly.
    for name in ('yolo_tracker.yaml', 'yolo_tracker_hardware.yaml'):
        fixed = yaml.safe_load((REPO / 'applications/pantiltros/config' / name).read_text())
        fixed = fixed['pantilt_yolo_tracker']['ros__parameters']
        assert fixed['base_compensation'] is False and fixed['target_lead'] is False
        assert set(fixed) <= set(tracker.DEFAULT_PARAMETERS)


def test_node_wiring(tmp_path):
    publishers, subscriptions, timers = {}, {}, []

    class FakeNode:
        def __init__(self, name):
            self.values = {'ready_file': str(tmp_path / 'ready'), 'warmup': False}

        def declare_parameter(self, name, value):
            self.values.setdefault(name, value)

        def get_parameter(self, name):
            return NS(value=self.values[name])

        def create_publisher(self, msg_type, topic, qos):
            publishers[topic] = qos
            return MagicMock()

        def create_subscription(self, msg_type, topic, callback, qos):
            subscriptions.setdefault(topic, []).append(qos)

        def create_timer(self, period, callback, clock=None):
            timers.append((period, callback.__name__, clock))

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=0, to_msg=lambda: None))

        def get_logger(self):
            return MagicMock()

    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'diagnostic_msgs', 'diagnostic_msgs.msg',
                     'rclpy.executors', 'ultralytics'):
            sys.modules[name] = MagicMock()
        sys.modules['rclpy.node'].Node = FakeNode
        qos = sys.modules['rclpy.qos']
        qos.QoSProfile = lambda **kwargs: NS(**kwargs)
        rclpy = sys.modules['rclpy']
        rclpy.clock.Clock = lambda clock_type: NS(clock_type=clock_type)
        module = load_script(SCRIPT, 'pantilt_yolo_tracker_wiring')
        module.PanTiltYoloTracker()
    status = publishers['/turret/status']
    assert status.reliability is qos.ReliabilityPolicy.RELIABLE
    assert status.durability is qos.DurabilityPolicy.TRANSIENT_LOCAL
    assert '/turret/command' in subscriptions and '/imu/data_raw' in subscriptions
    assert subscriptions['/imu/data_raw'][0].depth >= 50
    control = [timer for timer in timers if timer[1] == '_control_tick']
    assert control and control[0][0] == pytest.approx(0.02) and control[0][2] is None
    status_timer = [timer for timer in timers if timer[1] == '_publish_status']
    assert status_timer[0][0] <= 0.5
    assert status_timer[0][2].clock_type is rclpy.clock.ClockType.STEADY_TIME
