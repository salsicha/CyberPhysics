"""Aerodrone object tracker (systems/aerodrone/tracking.md): geolocation
against known geometry, the target filter, the follow modes' set-points and
limits, command parsing, the status JSON and the state machine
(applications/aerostack2/scripts/tracking_core.py), and the ground-truth
detector's projection (tracking_truth_detector.py)."""
import json
import math
import sys

import numpy as np
import pytest
from conftest import REPO, load_script

sys.path.insert(0, str(REPO / 'applications/aerostack2/scripts'))
import tracking_core as tc  # noqa: E402

truth = load_script(REPO / 'applications/aerostack2/scripts/tracking_truth_detector.py',
                    'tracking_truth_detector_review')
tracker_node = load_script(REPO / 'applications/aerostack2/scripts/aerodrone_tracker.py',
                           'aerodrone_tracker_review')

CONFIG = tc.TrackerConfig()
# The Gazebo hd_camera (60 deg) and the OAK-1 (69 deg) at 640x480.
GAZEBO = tc.Intrinsics(554.3, 554.3, 320.5, 240.5, 640, 480)
OAK1 = tc.Intrinsics(320.0 / math.tan(math.radians(34.5)),
                     320.0 / math.tan(math.radians(34.5)), 319.5, 239.5, 640, 480)
CONTRACT_KEYS = {'state', 'mode', 'target_id', 'target', 'slant_range_m',
                 'depression_deg', 'message'}


def quaternion(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    """(x, y, z, w) of ZYX Euler angles (FLU body)."""
    roll, pitch, yaw = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def pose(position=(0.0, 0.0, 40.0), **angles):
    return tc.Pose(tuple(position), quaternion(**angles), 0, 0.0)


def flat(up=0.0):
    return lambda east, north: up


# --- geolocation ------------------------------------------------------------
def test_centre_pixel_of_a_level_camera_is_the_point_below():
    geo = tc.geolocate(GAZEBO.cx, GAZEBO.cy, (10, 10), pose((5.0, -3.0, 40.0)),
                       GAZEBO, flat(2.0), 0.0, False, CONFIG)
    assert geo.position == pytest.approx((5.0, -3.0, 2.0), abs=0.05)
    assert geo.range_m == pytest.approx(38.0, abs=0.05)
    assert math.degrees(geo.depression_rad) == pytest.approx(90.0)


@pytest.mark.parametrize('mount_deg', [0.0, 90.0])
@pytest.mark.parametrize('yaw_deg', [0.0, 35.0, -120.0])
def test_image_axes_on_flat_ground(mount_deg, yaw_deg):
    """100 px right of centre is height * 100 / f to the image right, which
    is mount - 90 deg from the nose (camera_geometry.camera_mount)."""
    height = 40.0
    geo = tc.geolocate(GAZEBO.cx + 100.0, GAZEBO.cy, (10, 10),
                       pose((0.0, 0.0, height), yaw_deg=yaw_deg), GAZEBO,
                       flat(), math.radians(mount_deg), False, CONFIG)
    heading = math.radians(yaw_deg + mount_deg - 90.0)
    distance = height * 100.0 / GAZEBO.fx
    assert geo.position[:2] == pytest.approx(
        (distance * math.cos(heading), distance * math.sin(heading)), abs=0.05)


def test_a_target_ahead_of_the_nose_is_at_the_image_top_in_gazebo():
    rotation = tc.camera_world_rotation(quaternion(yaw_deg=0.0), 0.0)
    u, v, _ = tc.project_point((10.0, 0.0, 0.0), (0.0, 0.0, 40.0), rotation, GAZEBO)
    assert u == pytest.approx(GAZEBO.cx)
    assert v < GAZEBO.cy


@pytest.mark.parametrize('roll,pitch,yaw,mount,stabilized', [
    (0.0, 0.0, 0.0, 0.0, False), (8.0, -5.0, 40.0, 0.0, False),
    (-12.0, 9.0, 200.0, 90.0, False), (10.0, 10.0, 75.0, 0.0, True)])
def test_projection_and_geolocation_round_trip(roll, pitch, yaw, mount, stabilized):
    """The truth detector's projection and the tracker's ray agree."""
    camera = (12.0, -7.0, 55.0)
    target = (20.0, 5.0, 3.0)
    rotation = tc.camera_world_rotation(quaternion(roll, pitch, yaw),
                                        math.radians(mount), stabilized)
    u, v, _ = tc.project_point(target, camera, rotation, GAZEBO)
    geo = tc.geolocate(u, v, (10, 10), pose(camera, roll_deg=roll, pitch_deg=pitch,
                                            yaw_deg=yaw),
                       GAZEBO, flat(3.0), math.radians(mount), stabilized, CONFIG)
    assert geo.position == pytest.approx(target, abs=0.05)


def test_ray_meets_a_sloped_plane_where_the_algebra_says():
    """Ground up = 0.3 e - 0.2 n + 5: the ray o + t d meets it at
    t = (o_z - g(o)) / (-(d_z) + 0.3 d_e - 0.2 d_n)."""
    ground = lambda e, n: 0.3 * e - 0.2 * n + 5.0  # noqa: E731
    origin = np.array([4.0, -6.0, 80.0])
    direction = np.array([0.35, 0.2, -1.0])
    direction /= np.linalg.norm(direction)
    t = (origin[2] - ground(origin[0], origin[1])) / (
        -direction[2] + 0.3 * direction[0] - 0.2 * direction[1])
    point = tc.intersect_terrain(origin, direction, ground)
    assert point == pytest.approx(origin + t * direction, abs=0.05)
    assert point[2] == pytest.approx(ground(point[0], point[1]), abs=0.05)


def test_ray_stops_at_the_first_ridge_it_meets():
    """A 30 m wall 20 m east of a camera looking 45 deg down-east at 50 m:
    the ray hits the wall (at 30 m up), not the ground behind it."""
    ground = lambda e, n: 30.0 if e >= 20.0 else 0.0  # noqa: E731
    direction = np.array([1.0, 0.0, -1.0]) / math.sqrt(2.0)
    point = tc.intersect_terrain((0.0, 0.0, 50.0), direction, ground)
    assert point[0] == pytest.approx(20.0, abs=0.3)
    assert point[2] == pytest.approx(30.0, abs=0.3)


@pytest.mark.parametrize('origin,direction,ground', [
    ((0.0, 0.0, 40.0), (1.0, 0.0, 0.0), flat()),        # horizontal
    ((0.0, 0.0, 40.0), (0.0, 0.0, 1.0), flat()),        # up
    ((0.0, 0.0, -1.0), (0.0, 0.0, -1.0), flat()),       # below ground
    ((0.0, 0.0, 40.0), (0.7, 0.0, -0.7), lambda e, n: None if e > 10 else 0.0),
])
def test_rays_that_do_not_meet_the_ground(origin, direction, ground):
    direction = np.asarray(direction) / np.linalg.norm(direction)
    assert tc.intersect_terrain(origin, direction, ground) is None


class FakeHeightMap:
    """HeightMapCache's interface over a plane (MSL heights)."""
    row_spacing_m = col_spacing_m = 10.0

    def __init__(self, origin):
        self.origin = origin

    def elevation_at(self, lat, lon):
        east, north = tc.latlon_to_local(lat, lon, *self.origin)
        if abs(east) > 2000 or abs(north) > 2000:
            return float('nan')
        return 700.0 + 0.1 * east + 0.05 * north

    def edge_distance_m(self, lat, lon):
        east, north = tc.latlon_to_local(lat, lon, *self.origin)
        return 2000.0 - max(abs(east), abs(north))


class FakeGeoid:
    def height(self, lat, lon):
        return -32.0


def test_dem_heights_are_put_on_the_ellipsoid_about_the_origin():
    """MSL DEM + geoid height - origin ellipsoid altitude (+ offset)."""
    origin = (37.9234, -122.5967, 700.0 - 32.0 - 10.0)  # 10 m below the DEM
    dem = tc.DemTerrain(FakeHeightMap(origin[:2]), origin, FakeGeoid())
    assert dem.height(0.0, 0.0) == pytest.approx(10.0, abs=1e-3)
    assert dem.height(100.0, -40.0) == pytest.approx(10.0 + 10.0 - 2.0, abs=1e-3)
    assert dem.with_offset(-1.5).height(0.0, 0.0) == pytest.approx(8.5, abs=1e-3)
    assert dem.height(5000.0, 0.0) is None
    assert dem.contains(0.0, 0.0, 1000.0) and not dem.contains(1500.0, 0.0, 1000.0)


def test_geolocation_on_the_dem_matches_the_slope():
    origin = (37.9234, -122.5967, 700.0 - 32.0)
    dem = tc.DemTerrain(FakeHeightMap(origin[:2]), origin, FakeGeoid())
    camera, target = (0.0, 0.0, 60.0), (25.0, 10.0, dem.height(25.0, 10.0))
    rotation = tc.camera_world_rotation(quaternion(yaw_deg=30.0), 0.0)
    u, v, _ = tc.project_point(target, camera, rotation, OAK1)
    geo = tc.geolocate(u, v, (8, 8), pose(camera, yaw_deg=30.0), OAK1,
                       dem.height, 0.0, False, CONFIG)
    assert geo.position == pytest.approx(target, abs=0.05)


def test_cached_dems_are_found_by_name(tmp_path):
    for name in ('dem_srtm_37.9234_-122.5967_8.0_10.npy',
                 'dem_37.7694_-122.4862_10.0_30.npy',
                 'dem_synthetic_37.0000_-122.0000_37.0000_-122.0000_8.0_10.npy',
                 'notes.txt'):
        (tmp_path / name).write_bytes(b'')
    found = {entry[1:] for entry in tc.dem_files(str(tmp_path))}
    assert found == {(37.9234, -122.5967, 8.0, 10.0), (37.7694, -122.4862, 10.0, 30.0)}
    best = tc.best_dem(str(tmp_path), 37.93, -122.60)
    assert best[0].endswith('dem_srtm_37.9234_-122.5967_8.0_10.npy')
    assert tc.best_dem(str(tmp_path), 40.0, -100.0) is None


def test_heightmap_loads_a_cached_dem_without_downloading(tmp_path):
    grid = np.full((801, 801), 512.0, dtype=np.float32)
    np.save(tmp_path / 'dem_srtm_37.9234_-122.5967_8.0_10.npy', grid)
    entry = tc.best_dem(str(tmp_path), 37.9234, -122.5967)
    heightmap = tc.load_heightmap(entry)
    assert heightmap.elevation_at(37.9234, -122.5967) == pytest.approx(512.0)


def test_geolocation_noise_grows_with_range_and_obliquity():
    near = tc.geolocate(GAZEBO.cx, GAZEBO.cy, (10, 10), pose((0, 0, 20.0)),
                        GAZEBO, flat(), 0.0, False, CONFIG)
    far = tc.geolocate(GAZEBO.cx, GAZEBO.cy, (10, 10), pose((0, 0, 100.0)),
                       GAZEBO, flat(), 0.0, False, CONFIG)
    oblique = tc.geolocate(GAZEBO.cx + 280, GAZEBO.cy, (10, 10), pose((0, 0, 100.0)),
                           GAZEBO, flat(), 0.0, False, CONFIG)
    assert near.covariance[0, 0] < far.covariance[0, 0] < oblique.covariance[0, 0]


# --- poses at image stamps ----------------------------------------------------
def test_pose_history_interpolates_position_and_attitude():
    history = tc.PoseHistory()
    second = 1_000_000_000
    history.add(10 * second, (0.0, 0.0, 40.0), quaternion(yaw_deg=0.0))
    history.add(10 * second + second // 5, (2.0, 0.0, 41.0), quaternion(yaw_deg=20.0))
    middle = history.at(10 * second + second // 10)
    assert middle.position == pytest.approx((1.0, 0.0, 40.5))
    assert math.degrees(tc.yaw_of(middle.quaternion)) == pytest.approx(10.0, abs=1e-6)
    assert history.at(10 * second + second) is None  # beyond max_offset_s


def test_pose_history_maps_simulator_stamps_through_the_clock():
    """Gazebo stamps frames in simulation time, MAVROS odometry in wall
    time; /clock pairs map one onto the other."""
    history = tc.PoseHistory()
    wall0, sim0 = 1_700_000_000_000_000_000, 120_000_000_000
    for i in range(11):
        history.add(wall0 + i * 100_000_000, (float(i), 0.0, 40.0), quaternion())
        history.add_clock(sim0 + i * 100_000_000, wall0 + i * 100_000_000)
    frame = history.at(sim0 + 450_000_000)
    assert frame.position[0] == pytest.approx(4.5, abs=1e-4)
    assert frame.stamp_ns == pytest.approx(wall0 + 450_000_000, abs=1000)


def test_image_stamps_are_put_on_the_trackers_clock():
    """The tracker's clock is simulation time under use_sim_time, wall time
    otherwise; a simulator's frames are stamped in simulation time. A stamp
    already on the clock is kept, one on the other clock goes through the
    /clock pairs (here a simulator at 0.2x real time, as Isaac)."""
    history = tc.PoseHistory()
    wall0, sim0 = 1_700_000_000_000_000_000, 120_000_000_000
    for i in range(11):
        history.add_clock(sim0 + i * 20_000_000, wall0 + i * 100_000_000)
    frame = sim0 + 100_000_000
    assert history.to_clock(frame, sim0 + 200_000_000) == frame           # use_sim_time
    assert history.to_clock(frame, wall0 + 900_000_000) == pytest.approx(
        wall0 + 500_000_000, abs=10)                                        # wall clock
    assert history.to_clock(wall0 + 500_000_000, sim0 + 200_000_000) == pytest.approx(
        sim0 + 100_000_000, abs=10)
    assert history.to_clock(frame, 5_000_000_000_000) is None               # neither clock
    assert tc.PoseHistory().to_clock(wall0, wall0 + 300_000_000) == wall0   # the aircraft


def test_timeouts_are_seconds_of_the_trackers_clock():
    """lost_timeout_s and handover_window_s count the clock the tracker is
    given, the simulation's under use_sim_time: in a simulator at 0.2x real
    time, 3 s of simulation are 15 s of wall time (the first tracker took
    them as 3 s of wall time, 0.6 s of Isaac's simulation)."""
    core = tc.TrackerCore(CONFIG)
    core.handle_command(TRACK, 100.0, flying())
    core.measurement(tc.Measurement(np.array([5.0, 5.0, 0.0]), COV, 100.0), 100.0)
    core.target_class = 'car'
    assert core.handover_open(102.4) and not core.handover_open(102.6)
    assert core.tick(102.9) == [] and core.state == 'tracking'
    assert core.tick(103.1) == [] and core.state == 'lost'


# --- the filter ---------------------------------------------------------------
def test_filter_converges_on_a_constant_velocity_target():
    """5 Hz fixes with 1 m of noise (attitude and box jitter at 50 m)."""
    rng = np.random.default_rng(3)
    covariance = np.diag([1.0, 1.0, 1.0])
    start = np.array([10.0, -5.0, 2.0])
    velocity = np.array([4.0, 1.5, 0.0])
    kf = tc.ConstantVelocityFilter(start, covariance, 0.0, CONFIG)
    errors, velocities = [], []
    for k in range(1, 151):
        t = 0.2 * k
        truth_position = start + velocity * t
        assert kf.update(truth_position + rng.normal(0.0, 1.0, 3), covariance, t)
        errors.append(np.linalg.norm(kf.position[:2] - truth_position[:2]))
        velocities.append(kf.velocity[:2])
    assert np.mean(velocities[-50:], axis=0) == pytest.approx(velocity[:2], abs=0.3)
    assert np.median(errors[50:]) < 0.8  # below the measurement noise
    state, _ = kf.predicted(kf.t + 1.0)
    assert state[:2] == pytest.approx((start + velocity * (kf.t + 1.0))[:2], abs=2.0)


def test_filter_gates_an_outlier_and_restarts_after_repeated_ones():
    covariance = np.diag([1.0, 1.0, 1.0])
    kf = tc.ConstantVelocityFilter((0.0, 0.0, 0.0), covariance, 0.0, CONFIG)
    for k in range(1, 11):
        assert kf.update((0.0, 0.0, 0.0), covariance, 0.2 * k)
    assert not kf.update((80.0, 0.0, 0.0), covariance, 2.2)
    assert kf.position == pytest.approx((0.0, 0.0, 0.0), abs=0.3)
    core = tc.TrackerCore(CONFIG, state='tracking', command=tc.Command('track', '7', 'centered', 40.0),
                          filter=kf)
    for k in range(CONFIG.max_rejections):
        core.measurement(tc.Measurement(np.array([80.0, 0.0, 0.0]), covariance, 2.4 + 0.2 * k), 3.0)
    assert core.filter.position == pytest.approx((80.0, 0.0, 0.0), abs=1e-6)


# --- set-points and limits ----------------------------------------------------
def test_centered_setpoint_is_above_the_target_within_the_height_limits():
    ground = lambda e, n: 12.0  # noqa: E731
    point = tc.centered_setpoint((30.0, -4.0, 12.0), 40.0, 0.5, ground, CONFIG)
    assert point.position == pytest.approx((30.0, -4.0, 52.0))
    assert point.yaw == 0.5 and point.clamped == ''
    low = tc.centered_setpoint((30.0, -4.0, 12.0), 5.0, 0.0, ground, CONFIG)
    high = tc.centered_setpoint((30.0, -4.0, 12.0), 300.0, 0.0, ground, CONFIG)
    assert low.position[2] == pytest.approx(12.0 + CONFIG.min_agl_m)
    assert high.position[2] == pytest.approx(12.0 + CONFIG.max_agl_m)
    assert 'minimum' in low.clamped and 'maximum' in high.clamped


@pytest.mark.parametrize('mount_deg', [0.0, 90.0])
@pytest.mark.parametrize('depression', [60.0, 70.0, 85.0])
def test_slant_setpoint_keeps_range_angle_bearing_and_long_axis(mount_deg, depression):
    target = (100.0, 50.0, 8.0)
    drone = (40.0, -10.0, 70.0)
    point = tc.slant_setpoint(target, drone, 60.0, depression, None,
                              math.radians(mount_deg), 0.3, flat(8.0), CONFIG, OAK1)
    slant, angle = tc.geometry(point.position, target)
    assert slant == pytest.approx(60.0)
    assert angle == pytest.approx(depression)
    bearing = math.atan2(target[1] - drone[1], target[0] - drone[0])
    assert point.bearing == pytest.approx(bearing)
    offset = np.subtract(target[:2], point.position[:2])
    assert math.atan2(offset[1], offset[0]) == pytest.approx(bearing)
    # From the set-point with that yaw, the target is on the image's
    # horizontal (long) axis, (90 - depression) off the centre.
    rotation = tc.camera_world_rotation(quaternion(yaw_deg=math.degrees(point.yaw)),
                                        math.radians(mount_deg))
    u, v, _ = tc.project_point(target, point.position, rotation, OAK1)
    assert v == pytest.approx(OAK1.cy, abs=0.5)
    assert abs(u - OAK1.cx) == pytest.approx(
        OAK1.fx * math.tan(math.radians(90.0 - depression)), abs=0.5)


def test_slant_yaw_takes_the_closer_end_of_the_long_axis():
    yaws = [tc.long_axis_yaw(0.0, 0.0, current, OAK1) for current in (1.4, -1.4)]
    assert yaws == pytest.approx([math.pi / 2, -math.pi / 2])


def test_slant_above_the_target_keeps_the_previous_bearing():
    point = tc.slant_setpoint((0.0, 0.0, 0.0), (0.2, 0.1, 50.0), 60.0, 70.0, 1.0,
                              0.0, 0.0, flat(), CONFIG, OAK1)
    assert point.bearing == pytest.approx(1.0)


def test_slant_height_stays_above_the_ground_under_the_setpoint():
    """A ridge 25 m above the target under the stand-off point."""
    ground = lambda e, n: 25.0 if e < 0.0 else 0.0  # noqa: E731
    point = tc.slant_setpoint((5.0, 0.0, 0.0), (-50.0, 0.0, 40.0), 18.0, 60.0, None,
                              0.0, 0.0, ground, CONFIG, OAK1)
    assert point.position[2] == pytest.approx(25.0 + CONFIG.min_agl_m)
    assert 'minimum' in point.clamped


def test_field_of_view_limits_the_depression():
    """60 deg keeps a target 4.5 deg inside the OAK-1's 69 deg image; the
    Gazebo camera's 60 deg needs 64.5."""
    assert tc.min_depression_for_fov(OAK1, 4.5) == pytest.approx(60.0, abs=1e-6)
    assert tc.min_depression_for_fov(GAZEBO, 4.5) == pytest.approx(64.5, abs=0.05)
    assert tc.min_depression_for_fov(None, 4.5) == tc.MIN_DEPRESSION_DEG
    core = tracking_core_at((0.0, 0.0, 0.0), '{"action": "track", "target_id": "3", '
                            '"mode": "slant", "slant_range_m": 60, "depression_deg": 60}')
    drone = tc.Drone((-40.0, 0.0, 50.0), 0.0, 50.0)
    point = core.setpoint(10.0, drone, flat(), 0.0, GAZEBO)
    assert point.depression_deg == pytest.approx(64.5, abs=0.05)
    status = core.status(10.0, drone.position)
    assert 'depression held at 64.5 deg' in status['message']


def test_tilt_toward_a_bearing():
    """The body's up axis leaning toward the bearing: a nose-down pitch
    leans it ahead, a right roll to the right."""
    assert tc.tilt_toward_deg(quaternion(pitch_deg=10.0), 0.0) == pytest.approx(10.0)
    assert tc.tilt_toward_deg(quaternion(pitch_deg=10.0), math.pi) == pytest.approx(-10.0)
    assert tc.tilt_toward_deg(quaternion(pitch_deg=10.0), math.pi / 2) == pytest.approx(0.0)
    assert tc.tilt_toward_deg(quaternion(roll_deg=8.0, yaw_deg=90.0), 0.0) == pytest.approx(
        8.0, abs=1e-6)


SLANT70 = ('{"action": "track", "target_id": "3", "mode": "slant", '
           '"slant_range_m": 60, "depression_deg": 70}')


def test_aircraft_tilt_toward_the_target_raises_the_depression():
    """Gazebo's body-fixed camera, a car 20.5 m east of the aircraft that
    chases it, nose-down 12 deg (the Gazebo test flights' tilt where the
    car left the image): the floor rises from 64.5 deg to 76.5, which
    keeps the car 4.5 deg inside the image through the tilted camera; it
    comes down over tilt_release_s once level, and a stabilised camera
    (no camera_tilt_quaternion) never raises it."""
    tilted = quaternion(pitch_deg=12.0)
    core = tracking_core_at((20.5, 0.0, 0.0), SLANT70, now=0.0)
    drone = tc.Drone((0.0, 0.0, 56.4), 0.0, 56.4, camera_tilt_quaternion=tilted)
    point = core.setpoint(0.1, drone, flat(), 0.0, GAZEBO)
    assert point.depression_deg == pytest.approx(76.5, abs=0.1)
    assert 'aircraft tilt 12 deg' in core.status(0.1, drone.position)['message']
    # The car through the tilted camera at that geometry: inside the edge.
    standoff = 60.0 * math.cos(math.radians(point.depression_deg))
    height = 60.0 * math.sin(math.radians(point.depression_deg))
    pose_at = tc.Pose((20.5 - standoff, 0.0, height),
                      quaternion(pitch_deg=12.0, yaw_deg=math.degrees(point.yaw)), 0, 0.0)
    u, v, _ = tc.project_point((20.5, 0.0, 0.0), pose_at.position,
                               tc.camera_world_rotation(pose_at.quaternion, 0.0), GAZEBO)
    edge = min(u, GAZEBO.width - u)
    assert math.degrees(math.atan((GAZEBO.cx - edge) / GAZEBO.fx)) <= 30.0 - 4.4
    # Level again: down to 70 (the command) within a few release times.
    level = tc.Drone((0.0, 0.0, 56.4), 0.0, 56.4, camera_tilt_quaternion=quaternion())
    for k in range(1, 300):
        point = core.setpoint(0.1 + 0.1 * k, level, flat(), 0.0, GAZEBO)
    assert point.depression_deg == pytest.approx(70.0, abs=0.05)
    assert core.tilt_raise < 0.05
    # A stabilised camera does not turn with the aircraft.
    core = tracking_core_at((20.5, 0.0, 0.0), SLANT70, now=0.0)
    steady = tc.Drone((0.0, 0.0, 56.4), 0.0, 56.4)
    assert core.setpoint(0.1, steady, flat(), 0.0, GAZEBO).depression_deg == pytest.approx(70.0)


def test_reference_shaper_limits_speed_lead_and_turn_rate():
    config = tc.TrackerConfig(max_speed_mps=5.0, max_vertical_speed_mps=1.0,
                              max_yaw_rate_dps=30.0, max_reference_lead_m=12.0)
    shaper = tc.ReferenceShaper(config)
    shaper.reset((0.0, 0.0, 40.0), 0.0)
    position, yaw = shaper.step((100.0, 0.0, 60.0), math.pi, (0.0, 0.0, 40.0), 0.1)
    assert position == pytest.approx((0.5, 0.0, 40.1))
    assert yaw == pytest.approx(math.radians(3.0))
    for _ in range(100):
        position, _ = shaper.step((100.0, 0.0, 60.0), math.pi, (0.0, 0.0, 40.0), 0.1)
    assert position[0] == pytest.approx(12.0)  # held max_reference_lead_m ahead
    shaper.reset((0.0, 0.0, 40.0), 0.0)
    position, _ = shaper.step((0.3, 0.4, 40.0), 0.0, (0.0, 0.0, 40.0), 0.1)
    assert position == pytest.approx((0.3, 0.4, 40.0))  # within a step: exact


# --- commands -----------------------------------------------------------------
@pytest.mark.parametrize('text,expected', [
    ('{"action": "stop"}', tc.Command('stop')),
    ('{"action": "track", "target_id": "7", "mode": "centered", "altitude_agl_m": 40}',
     tc.Command('track', '7', 'centered', altitude_agl_m=40.0)),
    ('{"action": "track", "target_id": "7"}', tc.Command('track', '7', 'centered')),
    ('{"action": "track", "target_id": 7, "mode": "slant", "slant_range_m": 60, '
     '"depression_deg": 70}', tc.Command('track', '7', 'slant', slant_range_m=60.0,
                                         depression_deg=70.0)),
    ('{"action": "track", "target_id": "car_1", "mode": "slant"}',
     tc.Command('track', 'car_1', 'slant', slant_range_m=60.0, depression_deg=70.0)),
    ('{"action": "track", "target_id": "7", "mode": "slant", "slant_range_m": 30, '
     '"depression_deg": 90}', tc.Command('track', '7', 'slant', slant_range_m=30.0,
                                         depression_deg=90.0)),
])
def test_valid_commands(text, expected):
    assert tc.parse_command(text, CONFIG) == (expected, '')


@pytest.mark.parametrize('text,reason', [
    ('not json', 'not valid JSON'),
    ('[1, 2]', 'not a JSON object'),
    ('{"action": "follow"}', 'unknown action'),
    ('{"action": "track"}', 'target_id'),
    ('{"action": "track", "target_id": ""}', 'target_id'),
    ('{"action": "track", "target_id": true}', 'target_id'),
    ('{"action": "track", "target_id": "7", "mode": "orbit"}', 'unknown mode'),
    ('{"action": "track", "target_id": "7", "altitude_agl_m": "40"}', 'must be a number'),
    ('{"action": "track", "target_id": "7", "altitude_agl_m": 10}', '[15, 120]'),
    ('{"action": "track", "target_id": "7", "altitude_agl_m": 121}', '[15, 120]'),
    ('{"action": "track", "target_id": "7", "mode": "slant", "depression_deg": 45}',
     '[60, 90]'),
    ('{"action": "track", "target_id": "7", "mode": "slant", "depression_deg": 91}',
     '[60, 90]'),
    ('{"action": "track", "target_id": "7", "mode": "slant", "slant_range_m": -5}',
     'positive'),
    ('{"action": "track", "target_id": "7", "mode": "slant", "slant_range_m": 200, '
     '"depression_deg": 70}', 'height above the target'),
    ('{"action": "track", "target_id": "7", "mode": "slant", "slant_range_m": 10, '
     '"depression_deg": 70}', 'height above the target'),
    ('{"action": "track", "target_id": "7", "mode": "slant", "depression_deg": NaN}',
     'must be a number'),
])
def test_invalid_commands_are_refused_with_a_reason(text, reason):
    command, error = tc.parse_command(text, CONFIG)
    assert command is None and reason in error


# --- state machine and status ---------------------------------------------------
TRACK = '{"action": "track", "target_id": "7", "mode": "centered", "altitude_agl_m": 40}'
COV = np.diag([1.0, 1.0, 1.0])


def flying(position=(0.0, 0.0, 50.0), agl=50.0, **fields):
    return tc.Drone(tuple(position), 0.0, agl, **fields)


def tracking_core_at(target, text=TRACK, now=1.0):
    core = tc.TrackerCore(CONFIG)
    core.handle_command(text, now, flying())
    core.measurement(tc.Measurement(np.asarray(target, dtype=float), COV, now), now)
    core.following = True
    return core


def test_track_acquire_lose_reacquire_and_stop():
    core = tc.TrackerCore(CONFIG)
    assert core.status(0.0)['state'] == 'idle'
    assert core.handle_command(TRACK, 0.0, flying()) == ['preempt']
    assert core.state == 'acquiring' and core.command.altitude_agl_m == 40.0
    assert core.measurement(tc.Measurement(np.array([5.0, 5.0, 0.0]), COV, 0.5), 0.5) == ['follow']
    assert core.state == 'tracking'
    core.following = True
    assert core.tick(3.0) == []                     # seen 2.5 s ago
    assert core.tick(3.6) == ['hover']              # lost_timeout_s 3
    assert core.state == 'lost' and 'not seen' in core.message
    core.following = False
    assert core.status(3.6)['target']['east'] == pytest.approx(5.0)
    assert core.measurement(tc.Measurement(np.array([9.0, 5.0, 0.0]), COV, 4.0), 4.0) == ['follow']
    assert core.state == 'tracking'
    assert core.filter.position == pytest.approx((9.0, 5.0, 0.0))  # restarted
    core.following = True
    assert core.handle_command('{"action": "stop"}', 5.0, flying()) == ['hover']
    assert core.status(5.0) == {'state': 'idle', 'mode': None, 'target_id': None,
                                'target': None, 'slant_range_m': None,
                                'depression_deg': None, 'message': 'stopped: hovering'}


def test_a_target_never_seen_is_lost_after_the_timeout():
    core = tc.TrackerCore(CONFIG)
    core.handle_command(TRACK, 10.0, flying())
    assert core.tick(12.9) == [] and core.state == 'acquiring'
    assert core.tick(13.1) == [] and core.state == 'lost'


def test_a_target_that_cannot_be_placed_is_an_error():
    core = tc.TrackerCore(CONFIG)
    core.handle_command(TRACK, 10.0, flying())
    core.geolocation_failed('no camera_info', 10.5)
    core.tick(13.5)
    assert core.state == 'error' and 'no camera_info' in core.message


def test_stop_when_idle_commands_nothing():
    core = tc.TrackerCore(CONFIG)
    assert core.handle_command('{"action": "stop"}', 0.0) == []
    assert core.state == 'idle'


def test_preemption_ends_tracking_and_stops_following():
    core = tracking_core_at((0.0, 0.0, 0.0))
    assert core.preempt('GoToBehavior', 2.0) == ['hover']
    status = core.status(2.0)
    assert status['state'] == 'idle' and 'pre-empted by GoToBehavior' in status['message']
    assert core.preempt('GoToBehavior', 3.0) == []


def test_refused_commands_while_idle_and_while_tracking():
    core = tc.TrackerCore(CONFIG)
    core.handle_command('{"action": "track"}', 0.0, flying())
    assert core.state == 'error' and core.message.startswith('rejected command')
    core = tracking_core_at((0.0, 0.0, 0.0))
    core.handle_command('{"action": "track", "target_id": "7", "mode": "slant", '
                        '"depression_deg": 30}', 2.0, flying())
    assert core.state == 'tracking' and '[60, 90]' in core.message
    assert core.command.mode == 'centered'


@pytest.mark.parametrize('drone,reason', [
    (flying(flying=False), 'not flying'),
    (flying(busy='LandBehavior'), 'LandBehavior is running')])
def test_tracking_is_refused_on_the_ground_and_during_takeoff_or_landing(drone, reason):
    core = tc.TrackerCore(CONFIG)
    assert core.handle_command(TRACK, 0.0, drone) == []
    assert core.state == 'error' and reason in core.message


def test_centered_altitude_defaults_to_the_current_height_within_limits():
    core = tc.TrackerCore(CONFIG)
    core.handle_command('{"action": "track", "target_id": "7"}', 0.0, flying(agl=150.0))
    assert core.command.altitude_agl_m == CONFIG.max_agl_m
    core.handle_command('{"action": "track", "target_id": "7"}', 0.0, flying(agl=33.0))
    assert core.command.altitude_agl_m == 33.0


def test_a_new_target_hovers_until_it_is_placed_and_a_new_mode_keeps_the_filter():
    core = tracking_core_at((0.0, 0.0, 0.0))
    kf = core.filter
    actions = core.handle_command('{"action": "track", "target_id": "7", "mode": "slant"}',
                                  2.0, flying())
    assert actions == ['preempt', 'follow'] and core.filter is kf
    assert core.state == 'tracking' and core.command.mode == 'slant'
    actions = core.handle_command('{"action": "track", "target_id": "12"}', 3.0, flying())
    assert actions == ['preempt', 'hover'] and core.state == 'acquiring'
    assert core.filter is None and not core.following


def test_a_refused_follow_is_retried_while_the_target_is_seen():
    core = tracking_core_at((0.0, 0.0, 0.0))
    core.follow_failed('FollowReference refused the goal', 2.0, retry_s=2.0)
    assert core.state == 'error' and not core.following
    measurement = tc.Measurement(np.zeros(3), COV, 2.5)
    assert core.measurement(measurement, 2.5) == []
    assert core.measurement(tc.Measurement(np.zeros(3), COV, 4.2), 4.2) == ['follow']
    assert core.state == 'tracking'


def test_status_json_carries_the_contract_fields():
    core = tracking_core_at((30.0, 40.0, 2.0), '{"action": "track", "target_id": "7", '
                            '"mode": "slant", "slant_range_m": 60, "depression_deg": 70}')
    status = json.loads(tc.status_json(core.status(1.0, (0.0, 0.0, 52.0))))
    assert set(status) == CONTRACT_KEYS
    assert status['state'] == 'tracking' and status['mode'] == 'slant'
    assert status['target_id'] == '7'
    assert set(status['target']) == {'east', 'north', 'up', 'speed_mps'}
    assert status['target']['east'] == pytest.approx(30.0)
    assert status['slant_range_m'] == pytest.approx(math.hypot(50.0, 50.0), abs=0.01)
    assert status['depression_deg'] == pytest.approx(45.0, abs=0.01)
    assert isinstance(status['message'], str)


def test_setpoint_aims_at_the_target_predicted_lead_s_ahead():
    core = tracking_core_at((0.0, 0.0, 0.0), now=0.0)
    core.filter.x[3:] = (4.0, 0.0, 0.0)
    point = core.setpoint(0.0, flying(), flat(), 0.0, OAK1)
    assert point.position[0] == pytest.approx(4.0 * CONFIG.lead_s)
    assert point.position[2] == pytest.approx(40.0)


@pytest.mark.parametrize('mount_deg', [0.0, 90.0])
def test_centered_yaw_lays_the_targets_travel_along_the_long_axis(mount_deg):
    """A car driving north-east, followed centered: the set-point's yaw
    puts its direction of travel along the image's long axis (the end
    nearer the current yaw), so the projection of a point ahead of it lies
    on that axis; a person walking keeps the yaw."""
    mount = math.radians(mount_deg)
    core = tracking_core_at((0.0, 0.0, 0.0), now=0.0)
    core.filter.x[3:] = (2.5, 2.5, 0.0)
    point = core.setpoint(0.0, flying(), flat(), mount, GAZEBO)
    above = tc.Pose((0.0, 0.0, 40.0), quaternion(yaw_deg=math.degrees(point.yaw)), 0, 0.0)
    rotation = tc.camera_world_rotation(above.quaternion, mount)
    u, v, _ = tc.project_point((10.0, 10.0, 0.0), above.position, rotation, GAZEBO)
    assert v == pytest.approx(GAZEBO.cy, abs=1e-6) and abs(u - GAZEBO.cx) > 100.0
    assert abs(tc.wrap_angle(point.yaw)) <= math.pi / 2 + 1e-9  # nearer yaw 0
    # Walking pace (1.5 m/s), under centered_yaw_min_speed_mps: the yaw stays.
    core.filter.x[3:] = (1.2, 0.9, 0.0)
    assert core.setpoint(0.1, flying(), flat(), mount, GAZEBO).yaw == pytest.approx(point.yaw)
    core = tracking_core_at((0.0, 0.0, 0.0), now=0.0)
    core.filter.x[3:] = (1.2, 0.9, 0.0)
    assert core.setpoint(0.0, flying(), flat(), mount, GAZEBO).yaw == pytest.approx(0.0)


# --- the ground-truth detector --------------------------------------------------
def test_routes_are_continuous_at_their_speed():
    for spec in ('car_1,car,stadium,60,0,120,30,4.0', 'c,car,circle,0,0,25,0,3.0',
                 'p,person,line,0,0,30,40,1.5'):
        route = truth.parse_spec(spec)[0]
        points = [truth.route_position(route, 0.05 * k)[:2] for k in range(4000)]
        steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
        assert steps.max() == pytest.approx(route[-1] * 0.05, rel=0.02)
    with pytest.raises(ValueError):
        truth.parse_spec('x,car,zigzag,0,0,1,1,1')


def test_footprint_box_is_centred_on_the_target_and_sized_by_class():
    rotation = tc.camera_world_rotation(quaternion(), 0.0)
    target = {'east': 0.0, 'north': 0.0, 'up': 0.0, 'class': 'car', 'yaw': 0.0}
    cx, cy, size_x, size_y = truth.footprint_box(target, (0.0, 0.0, 40.0), rotation, GAZEBO)
    assert (cx, cy) == pytest.approx((GAZEBO.cx, GAZEBO.cy))
    # Heading east = toward the nose = image up: length along v.
    assert size_y == pytest.approx(4.5 * GAZEBO.fy / 40.0)
    assert size_x == pytest.approx(1.9 * GAZEBO.fx / 40.0)
    behind = {'east': 0.0, 'north': 200.0, 'up': 0.0, 'class': 'car'}
    assert truth.footprint_box(behind, (0.0, 0.0, 40.0), rotation, GAZEBO) is None


def test_truth_boxes_geolocate_onto_the_truth():
    q = quaternion(4.0, -3.0, 50.0)
    camera = (10.0, 5.0, 45.0)
    target = {'east': 18.0, 'north': 12.0, 'up': 1.0, 'class': 'person'}
    rotation = tc.camera_world_rotation(q, 0.0)
    cx, cy, sx, sy = truth.footprint_box(target, camera, rotation, GAZEBO)
    geo = tc.geolocate(cx, cy, (sx, sy), tc.Pose(camera, q, 0, 0.0), GAZEBO,
                       flat(1.0), 0.0, False, CONFIG)
    assert geo.position == pytest.approx((18.0, 12.0, 1.0), abs=0.05)


def test_truth_targets_are_interpolated_at_the_frame_time():
    history = [(10.0, {'car_1': {'east': 0.0, 'north': 0.0, 'up': 1.0}}),
               (10.1, {'car_1': {'east': 0.5, 'north': 0.0, 'up': 1.0}})]
    assert truth.interpolate_targets(history, 10.04)['car_1']['east'] == pytest.approx(0.2)
    assert truth.interpolate_targets(history, 12.0) == {}


def test_wide_images_are_scaled_to_the_detector_width():
    wide = tc.Intrinsics(1000.0, 1000.0, 639.5, 359.5, 1280, 720)
    scaled, scale = truth.scaled_intrinsics(wide, 640)
    assert scale == 0.5 and (scaled.width, scaled.height) == (640, 360)
    assert (scaled.fx, scaled.cx, scaled.cy) == pytest.approx((500.0, 319.5, 179.5))


# --- the ROS node's terrain choice ----------------------------------------------
def test_tracker_terrain_prefers_the_dem_then_the_rangefinder_then_flat():
    terrain = tracker_node.Terrain()
    assert terrain.source(0.0, 30.0) == 'flat'
    assert terrain.height_fn(0.0, 30.0)(5.0, 5.0) == 0.0
    terrain.range_ground = (-12.0, 0.0)
    assert terrain.source(1.0, 30.0) == 'rangefinder'
    assert terrain.height_fn(1.0, 30.0)(5.0, 5.0) == -12.0
    origin = (37.9234, -122.5967, 700.0 - 32.0)
    terrain.dem = tc.DemTerrain(FakeHeightMap(origin[:2]), origin, FakeGeoid())
    terrain.bias, terrain.bias_time = 1.5, 1.0
    assert terrain.source(1.0, 30.0) == 'dem'
    assert terrain.height_fn(1.0, 30.0)(0.0, 0.0) == pytest.approx(1.5)
    assert terrain.height_fn(40.0, 30.0)(0.0, 0.0) == pytest.approx(0.0)  # bias expired
    assert terrain.height_fn(1.0, 30.0)(9000.0, 0.0) == -12.0  # off the DEM


def test_every_tracker_parameter_has_a_config_field_or_a_node_default():
    assert set(tracker_node.CONFIG_FIELDS) == set(vars(tc.TrackerConfig()))
    assert not set(tracker_node.CONFIG_FIELDS) & set(tracker_node.PARAMETERS)


def test_lead_is_learnt_from_the_lag_behind_a_steady_target():
    """An aircraft 3 m behind a centered set-point while the target drives
    east at 3 m/s needs 1 s more lead; one ahead, less; a fast target or a
    catch-up teaches nothing."""
    def run(offset_e, speed=3.0, seconds=30.0, following=True):
        core = tracking_core_at((0.0, 0.0, 0.0), now=0.0)
        core.following = following
        core.filter.x[3:] = (speed, 0.0, 0.0)
        t = 0.0
        while t < seconds:
            target_e = speed * t
            core.filter.x[:3] = (target_e, 0.0, 0.0)
            core.filter.t = t
            lag = offset_e * max(0.0, 1.0 - core.lead_extra_s / 1.0)
            drone = flying((target_e + lag, 0.0, 40.0))
            core.setpoint(t, drone, flat(), 0.0, OAK1)
            t += 0.1
        return core.lead_extra_s

    assert run(-3.0) == pytest.approx(1.0, abs=0.1)        # trailing 3 m at 3 m/s
    assert -CONFIG.lead_s <= run(+2.0) < -0.5              # ahead: less lead
    assert run(-3.0, speed=4.8) == 0.0                     # beyond 0.9 x the limit
    assert run(-30.0) == 0.0                               # catching up
    assert run(-3.0, following=False) == 0.0
    assert run(-3.0, speed=0.5) == 0.0                     # (nearly) still


def test_a_turning_target_is_predicted_along_its_arc():
    """A car on a 15 m circle at 4 m/s, seen at 5 Hz: the filter learns the
    turn rate and predicts 3 s ahead on the circle, where a straight line
    would put it 6.6 m outside."""
    radius, speed = 15.0, 4.0
    omega = speed / radius

    def on_circle(t):
        return np.array([radius * math.cos(omega * t), radius * math.sin(omega * t), 0.0])

    covariance = np.diag([0.04, 0.04, 0.04])
    kf = tc.ConstantVelocityFilter(on_circle(0.0), covariance, 0.0, tc.TrackerConfig())
    for k in range(1, 151):
        kf.update(on_circle(0.2 * k), covariance, 0.2 * k)
    assert kf.turn_rate == pytest.approx(omega, rel=0.1)
    ahead = kf.predicted_turning(kf.t + 3.0)
    assert np.linalg.norm(ahead[:2] - on_circle(kf.t + 3.0)[:2]) < 1.5
    straight, _ = kf.predicted(kf.t + 3.0)
    assert np.linalg.norm(straight[:2] - on_circle(kf.t + 3.0)[:2]) > 5.0


def test_a_straight_target_turns_nothing():
    covariance = np.diag([0.04, 0.04, 0.04])
    kf = tc.ConstantVelocityFilter((0.0, 0.0, 0.0), covariance, 0.0, tc.TrackerConfig())
    for k in range(1, 51):
        kf.update((0.8 * k, 0.0, 0.0), covariance, 0.2 * k)
    assert abs(kf.turn_rate) < 0.01
    assert kf.predicted_turning(kf.t + 2.0)[:2] == pytest.approx((48.0, 0.0), abs=0.3)


# --- track handover -----------------------------------------------------------
def handover_core(target_class='car'):
    """Tracking '7' (a car) east at 4 m/s, last placed at t = 2.0 s."""
    core = tc.TrackerCore(CONFIG)
    core.handle_command(TRACK, 0.0, flying())
    for k in range(11):
        t = 0.2 * k
        core.measurement(tc.Measurement(np.array([4.0 * t, 0.0, 0.0]), COV, t), t)
        core.target_class, core.target_side_px = target_class, 50.0
    core.following = True
    return core


def candidate(east, north, t, cls='car', side=50.0):
    return (tc.Measurement(np.array([east, north, 0.0]), COV, t), cls, side)


def test_a_new_id_on_the_target_takes_over_the_selected_one():
    core = handover_core()
    # 0.4 s later the detector's '7' is gone; '12' is where the car is
    # predicted (9.6 m east), a person stands beside it.
    candidates = {'12': candidate(9.7, 0.3, 2.4), '15': candidate(10.5, 1.0, 2.4, 'person')}
    assert core.handover(candidates, 2.4) == '12'
    core.hand_over('12', 2.4)
    assert core.measurement(candidates['12'][0], 2.4) == []
    status = core.status(2.4)
    assert status['state'] == 'tracking' and status['target_id'] == '12'
    assert 'handed over from 7' in status['message']
    # A van is the same kind of object as a car (the detector hesitates).
    assert handover_core().handover({'3': candidate(9.6, 0.0, 2.4, 'van')}, 2.4) == '3'


@pytest.mark.parametrize('candidates,now,why', [
    ({'12': candidate(9.6, 0.0, 2.4, 'person')}, 2.4, 'another kind'),
    ({'12': candidate(15.0, 0.0, 2.4)}, 2.4, 'too far from the prediction'),
    ({'12': candidate(9.6, 0.0, 2.4), '13': candidate(10.6, 1.0, 2.4)}, 2.4, 'two cars there'),
    ({'12': candidate(17.6, 0.0, 4.4)}, 4.6, 'gone longer than handover_window_s'),
    ({'7': candidate(9.6, 0.0, 2.4)}, 2.4, 'the selected id itself'),
    ({'12': candidate(9.6, 0.0, 2.4, side=12.0)}, 2.4, 'a box a quarter of the size'),
])
def test_no_handover_when_it_is_not_clearly_the_target(candidates, now, why):
    assert handover_core().handover(candidates, now) is None, why


def test_no_handover_before_the_target_was_placed_or_after_a_new_command():
    core = tc.TrackerCore(CONFIG)
    core.handle_command(TRACK, 0.0, flying())
    assert not core.handover_open(0.5)                 # never placed
    core = handover_core()
    core.handle_command(TRACK.replace('"7"', '"8"'), 2.2, flying())
    assert core.target_class is None and not core.handover_open(2.3)


def test_tracker_node_hands_a_vanished_id_over_on_its_detections():
    """aerodrone_tracker._on_detections: the selected id missing from a
    frame, a car box at the target's predicted ground point takes over."""
    import threading
    from types import SimpleNamespace as NS

    import tracking_fakes as fakes

    node = tracker_node.Tracker.__new__(tracker_node.Tracker)
    core = handover_core()
    node.core, node.config = core, CONFIG
    node.lock = threading.RLock()
    node.intrinsics, node.mount_yaw, node.stabilized = GAZEBO, 0.0, False
    node.terrain, node.bias_hold_s = tracker_node.Terrain(), 30.0
    node.origin = None
    node.poses = NS(at=lambda stamp: pose((9.6, 0.0, 40.0)),   # straight above the car
                    to_clock=lambda stamp, reference: stamp)
    logs, published = [], []
    node.node = NS(get_logger=lambda: NS(info=logs.append, warn=logs.append))
    node.now = lambda: 2.4
    node._execute = lambda actions: None
    node._publish_status = lambda: published.append(core.status(2.4))

    def detection(track_id, u, v, cls):
        item = fakes.Detection2D(id=track_id)
        item.bbox.center.position.x, item.bbox.center.position.y = u, v
        item.bbox.size_x = item.bbox.size_y = 30.0
        item.results = [fakes.ObjectHypothesisWithPose()]
        item.results[0].hypothesis.class_id = cls
        return item

    frame = fakes.Detection2DArray(detections=[detection('12', 320.5, 240.5, 'car'),
                                               detection('15', 400.0, 240.5, 'person')])
    node._on_detections(frame)
    assert core.command.target_id == '12' and core.state == 'tracking'
    assert published and published[-1]['target_id'] == '12'
    assert any('now track 12' in str(line) for line in logs)
    # A frame with neither the id nor a car on the target changes nothing.
    node._on_detections(fakes.Detection2DArray(detections=[detection('15', 400.0, 240.5,
                                                                     'person')]))
    assert core.command.target_id == '12'
    # The id moved onto a person beside the car is not the target any more.
    seen = core.last_seen
    node.now = lambda: 2.6
    node._on_detections(fakes.Detection2DArray(detections=[detection('12', 400.0, 240.5,
                                                                     'person')]))
    assert core.last_seen == seen and core.target_class == 'car'


def test_an_occluded_person_is_not_handed_to_another_person_nearby():
    """A walking person (1.5 m/s) hidden: a standing person 6 m from where
    the target is predicted is not it."""
    core = tc.TrackerCore(CONFIG)
    core.handle_command(TRACK, 0.0, flying())
    for k in range(11):
        t = 0.2 * k
        core.measurement(tc.Measurement(np.array([1.5 * t, 0.0, 0.0]), COV, t), t)
    core.target_class, core.target_side_px = 'person', 12.0
    assert core.handover({'5': candidate(3.75, 6.0, 2.5, 'person', 12.0)}, 2.5) is None
    assert core.handover({'5': candidate(3.9, 0.5, 2.5, 'person', 14.0)}, 2.5) == '5'
