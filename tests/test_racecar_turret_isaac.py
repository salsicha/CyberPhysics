"""RACECAR Turret Isaac simulation (applications/isaac/RACECAR_TURRET.md):
the vehicle, sensor, turret and scenario models of racecar_turret_sim.py,
the simulator's ROS interface with Isaac Sim and ROS stubbed, the sim
vehicle in the URDF, the scenario, and the composition.
"""
import math
import re
import sys
from types import SimpleNamespace as NS
from unittest import mock
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script

SCRIPTS = REPO / 'applications/isaac/scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(REPO / 'applications/demnav/scripts'))
import racecar_turret_sim as rts  # noqa: E402
from tracking_targets import Tracks, resolve_tracks  # noqa: E402

SYSTEM = REPO / 'systems/racecar_turret'
XACRO_NS = '{http://www.ros.org/wiki/xacro}'
COMPOSITION = REPO / 'compositions/racecar_turret_isaac.yaml'
SCENARIO = rts.load_scenario(SYSTEM / 'scenarios/isaac_arena.json')


@pytest.fixture(scope='module')
def sim_app():
    return load_script(SCRIPTS / 'racecar_turret_isaac.py', 'racecar_turret_isaac')


# --- running gear -----------------------------------------------------------

def test_ackermann_angles_steer_both_wheels_about_one_centre():
    left, right = rts.ackermann_angles(0.42, 0.33, 0.30)
    assert left > 0.42 > right > 0.0
    # Both front wheels' axles meet on the rear axle line, at the bicycle
    # model's turn centre.
    turn = 0.33 / math.tan(0.42)
    assert 0.33 / math.tan(left) + 0.15 == pytest.approx(turn)
    assert 0.33 / math.tan(right) - 0.15 == pytest.approx(turn)
    assert rts.ackermann_angles(-0.42, 0.33, 0.30) == pytest.approx((-right, -left))
    assert rts.ackermann_angles(0.0, 0.33, 0.30) == (0.0, 0.0)
    for steering in (-0.42, -0.1, 0.05, 0.3):
        assert rts.bicycle_steering(*rts.ackermann_angles(steering, 0.33, 0.30), 0.33, 0.30) \
            == pytest.approx(steering)


def test_the_knuckle_limit_holds_the_inner_wheel_at_full_lock():
    root = ET.parse(SYSTEM / 'urdf/racecar_turret.urdf.xacro').getroot()
    properties = {p.get('name'): p.get('value') for p in root.iter(XACRO_NS + 'property')}
    inner, _ = rts.ackermann_angles(0.42, float(properties['sim_wheelbase']),
                                    float(properties['sim_track']))
    assert inner < float(properties['sim_knuckle_limit'])


def test_wheel_rates_roll_every_wheel_without_slip():
    radius = 0.055
    assert rts.wheel_rates(0.3, 0.0, 0.33, 0.30, radius) == pytest.approx((0.3 / radius,) * 4)
    fl, fr, rl, rr = rts.wheel_rates(0.3, 0.3, 0.33, 0.30, radius)
    turn = 0.33 / math.tan(0.3)
    yaw_rate = 0.3 / turn
    assert rl * radius == pytest.approx(yaw_rate * (turn - 0.15))
    assert rr * radius == pytest.approx(yaw_rate * (turn + 0.15))
    assert fl * radius == pytest.approx(yaw_rate * math.hypot(0.33, turn - 0.15))
    assert rr > rl and fr > fl > rl
    assert rts.wheel_rates(-0.3, 0.3, 0.33, 0.30, radius) == pytest.approx((-fl, -fr, -rl, -rr))


def test_drive_train_lags_limits_and_stops_without_commands():
    drive = rts.DriveTrain(max_accel=2.0, speed_time_constant_s=0.12, max_steering_rate=6.5,
                           steering_time_constant_s=0.09, command_timeout_s=0.5)
    dt = 1.0 / 240.0
    assert drive.command(0.0, 5.0, 1.0)
    assert (drive.target_speed, drive.target_steering) == (1.0, 0.42)
    assert not drive.command(0.0, float('nan'), 0.0)
    drive.command(0.0, 0.35, 0.42)
    speeds, steerings = [], []
    for k in range(1, 241):
        speed, steering = drive.update(k * dt, dt)
        speeds.append(speed)
        steerings.append(steering)
        drive.command(k * dt, 0.35, 0.42)
    assert max(np.diff([0.0] + speeds)) <= 2.0 * dt + 1e-12  # ESC acceleration limit
    assert max(np.diff([0.0] + steerings)) <= 6.5 * dt + 1e-12  # servo slew
    assert speeds[-1] == pytest.approx(0.35, abs=1e-3)
    assert steerings[-1] == pytest.approx(0.42, abs=1e-3)
    # No command for longer than the watchdog: brake to zero, steering held.
    t = 1.0
    for k in range(1, 241 * 2):
        drive.update(t + k * dt, dt)
    assert drive.timed_out and drive.speed == pytest.approx(0.0, abs=1e-6)
    assert drive.steering == pytest.approx(0.42, abs=1e-3)


# --- sensors ----------------------------------------------------------------

def test_wheel_odometry_drifts_like_the_car_not_like_truth():
    dt = 0.02
    exact = rts.WheelOdometry(radius_error=0.0, steering_offset_rad=0.0, slip_std=0.0,
                              quantization_m=0.0)
    biased = rts.WheelOdometry(radius_error=0.02, steering_offset_rad=0.008, slip_std=0.0,
                               quantization_m=0.002)
    rate = 0.25 / 0.055
    for _ in range(int(20.0 / dt)):
        exact.update(dt, rate, 0.0)
        biased.update(dt, rate, 0.0)
    assert exact.x == pytest.approx(5.0) and exact.y == pytest.approx(0.0)
    assert biased.x == pytest.approx(5.0 * 1.02, abs=0.05)  # tyre radius error
    assert biased.y > 0.05 and biased.yaw > 0.0  # steering trim: veers left
    # A turn at the steering angle: yaw rate v tan(delta) / L.
    turning = rts.WheelOdometry(radius_error=0.0, steering_offset_rad=0.0, slip_std=0.0,
                                quantization_m=0.0)
    x, y, yaw, speed, yaw_rate = turning.update(dt, rate, 0.3)
    assert yaw_rate == pytest.approx(0.25 * math.tan(0.3) / 0.33)


def test_encoder_ticks_carry_the_remainder():
    odometry = rts.WheelOdometry(radius_error=0.0, steering_offset_rad=0.0, slip_std=0.0,
                                 quantization_m=0.002)
    for _ in range(1000):  # 0.5 mm per step: whole ticks every fourth
        odometry.update(0.01, 0.05 / 0.055, 0.0)
    assert odometry.x == pytest.approx(0.5, abs=0.002)


def test_imu_reads_gravity_up_and_rates_in_the_body_frame():
    imu = rts.ImuModel(accel_noise_std=0.0, gyro_noise_std=0.0, accel_bias=(0, 0, 0),
                       gyro_bias=(0, 0, 0))
    for _ in range(10):
        imu.update(0.01, [0.0, 0.0, 0.0])
    accel, gyro = imu.measure(np.eye(3), [0.0, 0.0, 0.5])
    assert accel == pytest.approx([0.0, 0.0, rts.GRAVITY])
    assert gyro == pytest.approx([0.0, 0.0, 0.5])
    # Car pitched nose-down by 0.1 rad: gravity's reaction has a +x part.
    pitched = rts.rpy_matrix(0.0, 0.1, 0.0)
    accel, _ = imu.measure(pitched, [0.0, 0.0, 0.0])
    assert accel[0] == pytest.approx(-rts.GRAVITY * math.sin(0.1))
    # Accelerating forward at 1 m/s^2 (after the filter settles).
    imu = rts.ImuModel(accel_noise_std=0.0, gyro_noise_std=0.0, accel_bias=(0, 0, 0),
                       gyro_bias=(0, 0, 0))
    for k in range(200):
        imu.update(0.005, [k * 0.005, 0.0, 0.0])
    assert imu.ideal(np.eye(3), [0, 0, 0])[0][0] == pytest.approx(1.0, abs=1e-3)


def test_scan_ranges_mark_no_return_as_inf():
    directions = rts.scan_directions(720)
    assert directions[0] == pytest.approx([-1.0, 0.0, 0.0])
    assert directions[180] == pytest.approx([0.0, -1.0, 0.0])  # counter-clockwise
    assert directions[360] == pytest.approx([1.0, 0.0, 0.0])
    ranges = rts.scan_ranges([0.1, 2.0, 12.0, float('inf'), -1.0, float('nan')], 0.15, 12.0)
    assert list(ranges[:2]) == [np.inf, 2.0] and np.isinf(ranges[2:]).all()
    rng = np.random.default_rng(0)
    noisy = rts.scan_ranges(np.full(1000, 5.0), 0.15, 12.0, 0.015, 0.0, rng)
    assert abs(noisy.mean() - 5.0) < 0.005 and 0.01 < noisy.std() < 0.02
    dropped = rts.scan_ranges(np.full(10000, 5.0), 0.15, 12.0, 0.0, 0.1, rng)
    assert 0.08 < np.isinf(dropped).mean() < 0.12


def test_depth_marks_no_surface_as_nan():
    depth = rts.depth_measurement(np.array([[1.0, np.inf], [0.0, 12.0]]), max_range=10.0)
    assert depth.dtype == np.float32 and depth[0, 0] == 1.0
    assert np.isnan(depth[0, 1]) and np.isnan(depth[1, 0]) and np.isnan(depth[1, 1])


def test_cameras_match_the_calibration_and_the_turret_camera_class():
    calibration = yaml.safe_load((REPO / 'systems/racecarneo/calibration/camera.yaml').read_text())
    camera = calibration['camera']
    import racecar_turret_isaac as app_module  # noqa: F401 (no Isaac at import)
    assert (app_module.FRONT_WIDTH, app_module.FRONT_HEIGHT, app_module.FRONT_FX) == (
        camera['width'], camera['height'], camera['fx'])
    fx, fy, cx, cy = rts.pinhole(640, 480, camera['fx'])
    assert fx == fy == camera['fx'] and (cx, cy) == (319.5, 239.5)
    focal, horizontal, vertical = rts.usd_apertures(640, 480, camera['fx'])
    assert 640 * focal / horizontal == pytest.approx(camera['fx'])
    assert vertical / horizontal == pytest.approx(480 / 640)  # square pixels
    hfov = float(app_module.parse_args([]).turret_hfov_deg)
    assert 62.0 <= hfov <= 70.0
    assert rts.hfov_of_fx(640, rts.fx_of_hfov(640, hfov)) == pytest.approx(hfov)
    # The turret camera's nominal calibration (the recommended 70 deg camera).
    turret = yaml.safe_load((SYSTEM / 'calibration/turret_camera.yaml').read_text())
    assert (turret['image_width'], turret['image_height']) == (640, 480)
    assert rts.fx_of_hfov(640, hfov) == pytest.approx(turret['camera_matrix']['data'][0], abs=0.5)


# --- turret -----------------------------------------------------------------

def test_st3215_moves_at_the_firmware_goal_speed_and_stops_on_goal():
    turret = rts.TurretActuator(latency_s=0.03)
    assert turret.axes[0].max_velocity == pytest.approx(2000 * 2 * math.pi / 4096)
    dt = 1.0 / 240.0
    positions = []
    for k in range(1, 240):
        if k % 16 == 1:  # the tracker's 15 Hz
            turret.command(k * dt, 1.0, 0.0)
        positions.append(turret.update(k * dt, dt)[0])
    assert positions[int(0.03 / dt) - 2] == 0.0  # nothing before the latency
    assert max(np.diff(positions)) <= turret.axes[0].max_velocity * dt + 1e-9
    # 1 rad at 7.67 rad/s^2 up and down (a triangular profile): 0.72 s.
    assert abs(positions[int(0.6 / dt)] - 1.0) > 0.01
    assert positions[-1] == pytest.approx(1.0, abs=rts.STS_STEP_RAD)
    # One SET only: the firmware's watchdog stops the move after 0.5 s.
    single = rts.TurretActuator(latency_s=0.0, watchdog_s=0.5)
    single.command(0.0, 1.0, 0.0)
    for k in range(1, 480):
        position = single.update(k * dt, dt)[0]
    assert 0.5 < position < 0.99


def test_turret_holds_without_commands_and_under_emergency_stop():
    dt = 1.0 / 240.0
    turret = rts.TurretActuator(latency_s=0.0, watchdog_s=0.5)
    turret.command(0.0, 0.2, 0.1)
    for k in range(1, 120):
        turret.update(k * dt, dt)
    held = turret.update(2.0, dt)
    assert held == pytest.approx((0.2, 0.1), abs=rts.STS_STEP_RAD)
    turret.command(2.0, -1.0, 0.0)
    turret.emergency_stop = True
    for k in range(1, 240):
        position = turret.update(2.0 + k * dt, dt)
    assert position == pytest.approx(held, abs=0.01)
    assert not turret.command(3.0, float('nan'), 0.0)


def test_turret_feedback_is_quantized_delayed_and_25_hz():
    turret = rts.TurretActuator(feedback_period_s=0.04, feedback_delay_s=0.01)
    dt = 1.0 / 240.0
    reports = []
    for k in range(240):
        t = k * dt
        turret.record(t, 0.3 * t, -0.1)
        report = turret.feedback(t)
        if report is not None:
            reports.append(report)
    assert len(reports) == pytest.approx(25, abs=1)
    stamp, pan, tilt = reports[10]
    # Published 10 ms after the reading, stamped with the reading's time.
    assert 10 * 0.04 - 0.01 - dt < stamp <= 10 * 0.04 - 0.01 + 1e-9
    assert pan == pytest.approx(0.3 * stamp, abs=rts.STS_STEP_RAD)
    assert round(pan / rts.STS_STEP_RAD) * rts.STS_STEP_RAD == pytest.approx(pan)


def expanded_turret_joints(**args):
    """{name: joint} of racecar_turret.urdf.xacro with its includes and the
    turret's pantilt_turret macro (turret.urdf.xacro) expanded, by the
    xacro evaluator of test_racecar_turret_platform.py."""
    import test_racecar_turret_platform as platform
    joints, _links = platform.expand(platform.XACRO_FILE, args=dict(args))
    return joints


def xacro_turret_joints():
    """The turret's joints (children turret/...) as the simulator reads them
    from the expanded URDF, as the assets expand it (launcher:=false)."""
    elements = ['<robot>']
    for name, joint in expanded_turret_joints(launcher='false').items():
        if not joint['child'].startswith('turret/'):
            continue
        xyz, rpy, axis = (' '.join(repr(v) for v in joint[key]) for key in ('xyz', 'rpy', 'axis'))
        elements.append(f'<joint name="{name}" type="{joint["type"]}">'
                        f'<parent link="{joint["parent"]}"/><child link="{joint["child"]}"/>'
                        f'<origin xyz="{xyz}" rpy="{rpy}"/><axis xyz="{axis}"/></joint>')
    return rts.urdf_joints('\n'.join(elements + ['</robot>']))


def test_turret_camera_pose_follows_the_contract_signs(sim_app):
    joints = xacro_turret_joints()
    joints['turret/base_link'].parent = 'base_link'
    position, forward, up = sim_app.turret_camera_pose(joints, np.zeros(3), np.eye(3), 0.0, 0.0)
    # turret/base_link (-0.10, 0, 0.24), pan 0.045 and tilt 0.055 up, camera 0.035 ahead.
    assert position == pytest.approx([-0.065, 0.0, 0.34])
    assert forward == pytest.approx([1, 0, 0]) and up == pytest.approx([0, 0, 1])
    _, forward, _ = sim_app.turret_camera_pose(joints, np.zeros(3), np.eye(3), 0.5, 0.0)
    assert forward[1] < 0.0  # positive pan turns the camera right
    _, forward, _ = sim_app.turret_camera_pose(joints, np.zeros(3), np.eye(3), 0.0, 0.3)
    assert forward[2] == pytest.approx(math.sin(0.3))  # positive tilt looks up
    yawed = rts.rpy_matrix(0, 0, math.pi / 2)
    position, forward, _ = sim_app.turret_camera_pose(joints, np.array([1.0, 2.0, 0.0]), yawed, 0, 0)
    assert position == pytest.approx([1.0, 2.0 - 0.065, 0.34]) and forward == pytest.approx([0, 1, 0])


def test_sim_vehicle_parts_exist_only_with_the_argument():
    root = ET.parse(SYSTEM / 'urdf/racecar_turret.urdf.xacro').getroot()
    args = {a.get('name'): a.get('default') for a in root.iter(XACRO_NS + 'arg')}
    assert args['sim_vehicle'] == 'false'
    # Outside the sim_vehicle block every joint is fixed except the turret's:
    # pan and tilt, and the launcher's unless launcher:=false (the assets')
    def movable(**args):
        return {name for name, joint in expanded_turret_joints(**args).items()
                if joint['type'] != 'fixed'}
    assert movable() == {'pan_joint', 'tilt_joint', 'actuator_joint', 'trigger_joint'}
    assert movable(launcher='false') == {'pan_joint', 'tilt_joint'}
    assert [j for j in root.findall('joint') if j.get('type') != 'fixed'] == []
    blocks = [b for b in root.findall(XACRO_NS + 'if') if b.get('value') == '$(arg sim_vehicle)']
    assert len(blocks) == 1
    called = [c.tag for c in blocks[0]]
    assert called.count(XACRO_NS + 'sim_steered_wheel') == 2
    assert called.count(XACRO_NS + 'sim_wheel') == 2
    # Macros only expand where they are called: inside the block (or inside
    # another macro).
    inside = {id(e) for b in blocks for e in b.iter()}
    inside |= {id(e) for m in root.findall(XACRO_NS + 'macro') for e in m.iter()}
    for macro in root.findall(XACRO_NS + 'macro'):
        assert all(id(e) in inside for e in root.iter(XACRO_NS + macro.get('name')))


# --- scenario ---------------------------------------------------------------

def scenario_tracks():
    import json

    models = json.loads((REPO / SCENARIO['people']['models_from']).read_text())['models']
    config = rts.people_config(SCENARIO, models)
    return Tracks(resolve_tracks(config, lambda e, n: np.zeros_like(np.asarray(e, float)),
                                 [0, 0, 0]))


def test_scenario_people_walk_clear_of_the_obstacles():
    obstacles = rts.scenario_obstacles(SCENARIO)
    tracks = scenario_tracks()
    assert {t.name for t in tracks.tracks} >= {SCENARIO['selected_target']}
    assert all(t.cls == 'person' for t in tracks.tracks)
    radius = SCENARIO['people']['collision_radius_m']
    for t in np.arange(0.0, 300.0, 0.25):
        for pose in tracks.poses(t):
            square = (pose.east, pose.north)
            assert not any(o.touches_rectangle(square, 0.0, 2 * radius, 2 * radius)
                           for o in obstacles), (pose.name, t)


def test_course_waypoints_and_spawn_are_clear_and_inside_the_arena():
    import json

    mission = json.loads((REPO / SCENARIO['mission']['file']).read_text())
    mission = {m['id']: m for m in mission['missions']}[SCENARIO['mission']['id']]
    obstacles = rts.scenario_obstacles(SCENARIO)
    length, width = SCENARIO['vehicle_footprint_m']
    (x0, y0), (x1, y1) = SCENARIO['arena']['min_xy'], SCENARIO['arena']['max_xy']
    spawn = SCENARIO['spawn']
    poses = [(spawn['x'], spawn['y'], math.radians(spawn['yaw_deg']))]
    poses += [tuple(w['pose']) for w in mission['waypoints']]
    for x, y, yaw in poses:
        assert x0 + 1.0 < x < x1 - 1.0 and y0 + 1.0 < y < y1 - 1.0
        # The car plus Nav2's inflation (0.45 m) fits at every waypoint.
        assert not any(o.touches_rectangle((x, y), yaw, length + 0.4, width + 0.4)
                       for o in obstacles), (x, y)


def test_obstacles_block_lines_of_sight_and_footprints():
    box = rts.Obstacle('box', 'box', (2.0, 0.0), size=(0.5, 1.0, 1.0), yaw=math.radians(30))
    pillar = rts.Obstacle('pillar', 'cylinder', (2.0, 3.0), radius=0.3, height=2.0)
    assert box.blocks((0, 0, 0.5), (4, 0, 0.5))
    assert not box.blocks((0, 0, 1.5), (4, 0, 1.5))  # over it
    assert not box.blocks((0, 0, 0.5), (1.5, 0, 0.5))  # stops short
    assert box.blocks((0, 0, 0.3), (4, 0, 1.4))  # rising through it
    assert pillar.blocks((0, 3, 1.0), (4, 3, 1.0))
    assert not pillar.blocks((0, 3.4, 1.0), (4, 3.4, 1.0))
    assert not pillar.blocks((0, 3, 2.5), (4, 3, 2.5))
    assert box.touches_rectangle((1.4, 0.0), 0.0, 0.5, 0.34)
    assert not box.touches_rectangle((1.0, 0.0), 0.0, 0.5, 0.34)
    assert pillar.touches_rectangle((2.0, 2.6), 0.0, 0.5, 0.34)
    assert not pillar.touches_rectangle((2.0, 2.4), 0.0, 0.5, 0.34)
    eye = (0.0, 0.0, 0.34)
    assert rts.visible_fraction(eye, (6.0, 0.0, 0.0), 1.7, [box]) <= 0.4
    assert rts.visible_fraction(eye, (6.0, 2.0, 0.0), 1.7, [box]) == 1.0


def test_collisions_count_new_contacts_once():
    pillar = rts.Obstacle('pillar', 'cylinder', (2.0, 0.0), radius=0.3, height=2.0)
    counter = rts.CollisionCounter(0.5, 0.34)
    for x in (0.0, 1.5, 1.6, 1.7, 0.0, 1.6):
        counter.update(x, (x, 0.0), 0.0, [pillar], [('person_1', (5.0, 0.0), 0.22)])
    assert counter.count == 2 and [name for _, name in counter.events] == ['pillar'] * 2
    counter.update(9.0, (4.6, 0.0), 0.0, [pillar], [('person_1', (5.0, 0.0), 0.22)])
    assert counter.count == 3 and counter.events[-1][1] == 'person_1'


def test_truth_message_has_the_contract_fields():
    spawn = rts.Spawn(1.0, 2.0, math.pi / 2)
    assert spawn.point([1.0, 3.0, 0.5]) == pytest.approx([1.0, 0.0, 0.5])
    assert spawn.vector([0.0, 1.0, 0.0]) == pytest.approx([1.0, 0.0, 0.0])
    message = rts.truth_message(
        12.5, (1.0, 2.0, 0.3, 0.25), ([0, 0, 0.34], [1, 0, 0], [0, 0, 1]),
        [{'name': 'person_1', 'class': 'person', 'position': [3, 0, 0],
          'centroid': [3, 0, 0.9], 'visible': True, 'in_view': True}], 2)
    assert set(message) == {'stamp', 'vehicle', 'turret_camera', 'targets', 'collisions'}
    assert set(message['vehicle']) == {'x', 'y', 'yaw', 'speed_mps'}
    assert {'position', 'forward'} <= set(message['turret_camera'])
    assert {'name', 'class', 'position', 'centroid', 'visible'} <= set(message['targets'][0])
    assert rts.aim_error_deg([0, 0, 0.34], [1, 0, 0], [3, 0, 0.34]) == pytest.approx(0.0)
    assert rts.aim_error_deg([0, 0, 0], [1, 0, 0], [1, 1, 0]) == pytest.approx(45.0)
    u, v = rts.project([3, -0.3, 0.34], [0, 0, 0.34], [1, 0, 0], [0, 0, 1], 500, 500, 319.5, 239.5)
    assert u == pytest.approx(319.5 + 50) and v == pytest.approx(239.5)


# --- simulator ROS interface (Isaac and ROS stubbed) ------------------------

class FakeNode:
    def __init__(self, name):
        self.name = name
        self.published = {}
        self.subscribed = {}

    def create_publisher(self, msg_type, topic, qos):
        publisher = mock.MagicMock()
        self.published[topic] = publisher
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscribed.setdefault(topic, []).append(callback)

    def get_logger(self):
        return mock.MagicMock()


def test_simulator_publishes_the_racecar_and_turret_topics(sim_app):
    topics = yaml.safe_load((REPO / 'systems/racecarneo/config/topics.yaml').read_text())['topics']
    with mock.patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('ackermann_msgs', 'ackermann_msgs.msg', 'builtin_interfaces',
                     'builtin_interfaces.msg'):
            sys.modules[name] = mock.MagicMock()
        with mock.patch.object(sys.modules['rclpy.node'], 'Node', FakeNode):
            node = sim_app.make_ros_node(70.0)
    for key in ('command', 'color_image', 'depth_image', 'camera_info', 'lidar', 'imu',
                'ackermann_feedback'):
        assert topics[key] in node.published or topics[key] in node.subscribed, key
    assert {'/odom', '/clock', '/turret/joint_states', '/turret/actuator_ready',
            '/turret/camera/image_raw', '/turret/camera/camera_info',
            '/simulation/truth'} <= set(node.published)
    assert {'/ackermann_cmd', '/turret/joint_commands', '/turret/emergency_stop'} <= set(node.subscribed)
    # The actuator is reported ready once (latched).
    node.ready_pub.publish.assert_called_once()
    node._joint_command(NS(name=['tilt_joint', 'pan_joint'], position=[0.1, -0.2]))
    assert node.turret_commands == [(-0.2, 0.1)]
    assert node.front_info['k'][0] == sim_app.FRONT_FX


def test_physics_steps_once_per_loop_and_never_inside_a_render():
    source = (SCRIPTS / 'racecar_turret_isaac.py').read_text()
    main = source[source.index('def main():'):]
    assert re.findall(r'world\.step\((.*?)\)', main) == ['render=False']
    loop = main[main.index('while app.is_running()'):]
    # Drive targets and the people's poses are set before the step; the
    # published state is read after it.
    assert loop.index('vehicle.apply(') < loop.index('world.step(') < loop.index('vehicle.state()')
    assert loop.index('place_people(') < loop.index('world.step(')
    assert 'publish_clock(t)' in loop


def test_rates_divide_the_physics_rate(sim_app):
    every = sim_app.strides(240.0, {'render': 30.0, 'camera': 15.0, 'imu': 100.0,
                                    'lidar': 10.0, 'fast': 1000.0})
    assert every == {'render': 8, 'camera': 16, 'imu': 2, 'lidar': 24, 'fast': 1}


# --- composition ------------------------------------------------------------

JAZZY_IMAGES = {'racecarneo', 'nvblox', 'depthanything', 'pantiltros', 'ros2'}


def composition():
    return yaml.safe_load(COMPOSITION.read_text())['services']


def test_composition_runs_every_node_on_simulation_time():
    services = composition()
    commands = {name: ' '.join(s['command']) if isinstance(s['command'], list) else s['command']
                for name, s in services.items() if 'command' in s}
    for name in ('racecar_turret_slam', 'racecar_turret_nav2', 'racecar_turret_nvblox',
                 'racecar_turret_depthanything', 'racecar_turret_cmd_vel_to_ackermann',
                 'racecar_turret_odom_tf'):
        assert 'use_sim_time' in commands[name], name
    assert "s/use_sim_time: false/use_sim_time: true/" in commands['racecar_turret_nav2']
    assert 'use_sim_time: true' in commands['racecar_turret_nvblox']
    tracker = yaml.safe_load((SYSTEM / 'config/turret_tracker_isaac.yaml').read_text())
    assert tracker['pantilt_yolo_tracker']['ros__parameters']['use_sim_time'] is True
    assert 'turret_tracker_isaac.yaml' in commands['racecar_turret_tracker']


def test_composition_drives_the_hardware_car():
    services = composition()
    command = services['racecar_turret_cmd_vel_to_ackermann']['command']
    assert 'wheelbase' not in command and 'max_steering_angle' not in command
    params = yaml.safe_load((REPO / 'systems/racecarneo/config/cmd_vel_to_ackermann.yaml').read_text())
    params = params['racecarneo_cmd_vel_to_ackermann']['ros__parameters']
    import racecar_turret_isaac as app_module
    assert (params['wheelbase'], params['max_steering_angle']) == (
        app_module.WHEELBASE, app_module.MAX_STEERING)
    rsp = services['racecar_turret_robot_state_publisher']['command']
    assert 'racecar_turret.urdf.xacro' in rsp and '/turret/joint_states' in rsp
    assert 'sim_geometry:=false' in rsp


def test_composition_names_and_open_file_limits():
    services = composition()
    names = [s['container_name'] for s in services.values() if 'container_name' in s]
    assert len(names) == len(set(names))
    others = set()
    for other in ('racecarneo.yaml', 'turret_sim.yaml', 'racecarneo_hardware.yaml'):
        others |= {s.get('container_name') for s in
                   yaml.safe_load((REPO / 'compositions' / other).read_text())['services'].values()}
    assert not set(names) & others
    for name, service in services.items():
        match = re.match(r'salsicha/cyberphysics:([a-z0-9_]+)-', service.get('image', ''))
        if match and match.group(1) in JAZZY_IMAGES and name != 'racecar_turret_assets':
            assert service['ulimits']['nofile'] == {'soft': 1024, 'hard': 524288}, name
    assert services['racecar_turret_evaluation']['profiles'] == ['evaluation']


def test_thresholds_are_known_to_the_scorer():
    import json

    scorer = load_script(REPO / 'tools/racecar_turret_scorer.py', 'racecar_turret_scorer_t')
    thresholds = json.loads((SYSTEM / 'validation/acceptance_thresholds.json').read_text())
    known = set(scorer.CHECKS) | {'on_target_deg', 'min_loss_s', 'aiming_runs', 'notes',
                                   'measured'}
    assert set(thresholds) <= known
    assert thresholds['aiming_runs'] == ['compensated']


def test_people_give_way_to_the_car_and_walk_on():
    tracks = Tracks(resolve_tracks({
        'sample_spacing_m': 0.05,
        'models': {'walking_person': {'tilt_with_ground': False}},
        'targets': [{'name': 'p', 'class': 'person', 'model': 'walking_person', 'speed_mps': 1.0,
                     'route_enu_m': [[0.0, -0.1], [10.0, 0.0], [0.0, 0.1]]}]},
        lambda e, n: np.zeros_like(np.asarray(e, float)), [0, 0, 0]))
    walkers = rts.Walkers(tracks, radius=0.22, clearance=0.25)
    car = ((3.0, 0.0), 0.0, 0.5, 0.34)
    for _ in range(400):
        walkers.step(0.01, *car)
    pose = walkers.poses()[0]
    # Stopped just short of the car's footprint (x from 2.75), not in it.
    gap = rts.rectangle_distance((pose.east, pose.north), *car)
    assert 0.22 + 0.25 - 0.011 <= gap < 0.22 + 0.25 + 0.011
    assert walkers.waited[0] > 1.0
    # The car moves on: so does the person.
    for _ in range(100):
        walkers.step(0.01, (3.0, 3.0), 0.0, 0.5, 0.34)
    assert walkers.poses()[0].east > pose.east + 0.9
    # Walking away from a car behind is never held up.
    away = rts.Walkers(tracks, radius=0.22)
    for _ in range(100):
        away.step(0.01, (-0.3, 0.0), 0.0, 0.5, 0.34)
    assert away.waited[0] == 0.0
