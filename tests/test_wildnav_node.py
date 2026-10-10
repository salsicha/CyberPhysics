"""WildNav search prior, EKF-origin anchoring, jump gating and the
vehicle position under a tilted camera, driven through the node's real
__init__ and matcher with ROS stubbed out.

The tile cache is replaced by one whose keypoints are the query's moved by
a known transform (a similarity, or the exact ground projection of a
tilted camera), so the geometry is exact and SIFT-free."""
import importlib
import math
import sys
import time
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import cv2
import numpy as np
import pytest

import synthetic_world as sw
from conftest import ROS_MODULES
from wildnav_pkg.camera_geometry import (
    camera_rotation, ground_projection, horizontal_sigma)
from wildnav_pkg.satellite_cache import (
    ground_resolution_m, latlon_to_tile, tile_pixel_to_latlon)

ORIGIN = (37.9234, -122.5967)
ZOOM = 17
# Image and odometry stamp (both wall time, as on the aircraft).
STAMP = NS(sec=1_791_600_000, nanosec=0)
# The Isaac camera: 640 x 480, 60 deg across.
FOCAL, CX, CY = 554.67, 319.5, 239.5
CAMERA_INFO = NS(k=[FOCAL, 0.0, CX, 0.0, FOCAL, CY, 0.0, 0.0, 1.0],
                 d=[0.0] * 5, width=640, height=480)


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.params, self.subs, self.pubs = {}, {}, {}
        self.logger = MagicMock()

    def declare_parameter(self, name, value):
        self.params[name] = self.overrides.get(name, value)

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subs[topic] = callback

    def create_publisher(self, msg_type, topic, qos):
        self.pubs[topic] = MagicMock()
        return self.pubs[topic]

    def create_timer(self, period, callback):
        return MagicMock()

    def get_logger(self):
        return self.logger


class NavSatStatus:
    STATUS_NO_FIX, STATUS_FIX, STATUS_SBAS_FIX, STATUS_GBAS_FIX = -1, 0, 1, 2
    SERVICE_GPS = 1


class NavSatFix:
    COVARIANCE_TYPE_UNKNOWN, COVARIANCE_TYPE_APPROXIMATED = 0, 1
    COVARIANCE_TYPE_DIAGONAL_KNOWN, COVARIANCE_TYPE_KNOWN = 2, 3

    def __init__(self, latitude=0.0, longitude=0.0, altitude=0.0,
                 status=0, variance=None):
        self.header = NS(stamp=None, frame_id='')
        self.status = NS(status=status, service=0)
        self.latitude, self.longitude = latitude, longitude
        self.altitude = altitude
        self.position_covariance = [0.0] * 9
        self.position_covariance_type = 0
        if variance is not None:
            self.position_covariance = [variance, 0.0, 0.0,
                                        0.0, variance, 0.0,
                                        0.0, 0.0, 4.0]
            self.position_covariance_type = 2


@pytest.fixture(scope='module')
def wn():
    import wildnav_pkg
    previous = getattr(wildnav_pkg, 'wildnav_node', None)
    with patch.dict(sys.modules):
        for name in ROS_MODULES + ('geographic_msgs', 'geographic_msgs.msg'):
            sys.modules[name] = MagicMock()
        sys.modules['rclpy.node'].Node = FakeNode
        sensor = sys.modules['sensor_msgs.msg']
        sensor.NavSatFix, sensor.NavSatStatus = NavSatFix, NavSatStatus
        std = sys.modules['std_msgs.msg']
        std.Bool = std.Float32 = std.Int32 = lambda data: NS(data=data)
        sys.modules.pop('wildnav_pkg.wildnav_node', None)
        module = importlib.import_module('wildnav_pkg.wildnav_node')
    # Leave the package as found: other tests import the real module.
    if previous is not None:
        wildnav_pkg.wildnav_node = previous
    elif hasattr(wildnav_pkg, 'wildnav_node'):
        del wildnav_pkg.wildnav_node
    return module


class FakeCache:
    """A single tile whose features are the query keypoints moved so the
    image centre lands on (lat, lon)."""
    synthetic = False
    zoom = ZOOM
    feature_backend = 'SIFT'

    def __init__(self, lat, lon, yaw=0.7, scale=0.5):
        rng = np.random.default_rng(1)
        self.query = rng.uniform([0, 0], [640, 480], (80, 2)).astype(
            np.float32)
        self.descriptors = rng.uniform(0, 255, (80, 128)).astype(np.float32)
        x, y = latlon_to_tile(lat, lon, ZOOM)
        self.tile = (int(x), int(y))
        target = np.array([(x - int(x)) * 256.0, (y - int(y)) * 256.0])
        rotation = scale * np.array([[math.cos(yaw), -math.sin(yaw)],
                                     [math.sin(yaw), math.cos(yaw)]])
        self.points = ((self.query - [320.0, 240.0]) @ rotation.T +
                       target).astype(np.float32)
        self.searches = []

    def describe(self, image):
        return self.query, self.descriptors

    def candidate_tiles(self, lat, lon, radius, max_tiles):
        self.searches.append((lat, lon, radius))
        return [self.tile]

    def is_cached(self, x, y):
        return True

    def get_features(self, x, y, allow_download=True):
        return self.points, self.descriptors, (256, 256)

    def matcher(self):
        return cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)

    def set_origin(self, lat, lon):
        pass


@pytest.fixture
def make_node(wn, tmp_path):
    def make(target=(120.0, -80.0), **overrides):
        FakeNode.overrides = dict(
            initial_lat=ORIGIN[0], initial_lon=ORIGIN[1],
            cache_dir=str(tmp_path), zoom=ZOOM)
        FakeNode.overrides.update(overrides)
        node = wn.WildNavNode()
        node.cache = FakeCache(*sw.local_to_latlon(*target, *ORIGIN))
        node.bridge = MagicMock()
        return node
    return make


def euler_quaternion(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    """(x, y, z, w) of ZYX Euler angles in degrees."""
    roll, pitch, yaw = (math.radians(v) for v in (roll_deg, pitch_deg,
                                                    yaw_deg))
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def shifted(stamp, seconds):
    total = stamp.sec * 1_000_000_000 + stamp.nanosec + int(seconds * 1e9)
    return NS(sec=total // 1_000_000_000, nanosec=total % 1_000_000_000)


def odometry(east=0.0, north=0.0, z=35.0, quaternion=(0.0, 0.0, 0.0, 1.0),
             stamp=STAMP):
    x, y, qz, w = quaternion
    zero = NS(x=0.0, y=0.0, z=0.0)
    return NS(header=NS(stamp=stamp),
              pose=NS(pose=NS(position=NS(x=east, y=north, z=z),
                              orientation=NS(x=x, y=y, z=qz, w=w)),
                      covariance=[0.0] * 14 + [2.0] + [0.0] * 21),
              twist=NS(twist=NS(linear=zero, angular=zero)),
              child_frame_id='base_link')


def prior_at(east, north, altitude=300.0, **kwargs):
    return NavSatFix(*sw.local_to_latlon(east, north, *ORIGIN), altitude,
                     **kwargs)


def match(node, prior, odom=None, camera_info=None, feed_odom=True):
    """Match one image stamped STAMP; odometry with the same stamp is fed
    to the node first (its attitude history) unless feed_odom is False."""
    for publisher in node.pubs.values():
        publisher.reset_mock()
    odom = odom or odometry()
    if feed_odom:
        node.subs['/navigation/odometry'](odom)
    node._run_match(np.zeros((480, 640, 3), np.uint8), NS(stamp=STAMP),
                    odom, prior, camera_info)

    def sent(topic):
        return [c.args[0] for c in node.pubs[topic].publish.call_args_list]
    valid = sent('/wildnav/valid')
    fixes = sent('/wildnav/fix')
    return NS(valid=valid[-1].data if valid else None,
              fix=fixes[-1] if fixes else None,
              odometry=sent('/wildnav/odometry'))


def set_origin(node, lat, lon, alt=250.0):
    node.subs['/navigation/origin'](
        NS(position=NS(latitude=lat, longitude=lon, altitude=alt)))


def test_fix_published_before_origin_but_local_output_waits(make_node):
    node = make_node(target=(120.0, -80.0))
    result = match(node, prior_at(150.0, -60.0))
    assert result.fix is not None and result.valid is False
    assert result.odometry == []
    east, north = sw.latlon_to_local(result.fix.latitude,
                                     result.fix.longitude, *ORIGIN)
    assert (east, north) == pytest.approx((120.0, -80.0), abs=0.5)
    assert result.fix.altitude == 300.0


def test_local_output_is_relative_to_ekf_origin(make_node):
    node = make_node(target=(120.0, -80.0))
    # The EKF origin differs from initial_lat/lon (the old anchor) and the
    # odometry position is irrelevant to the search.
    origin = sw.local_to_latlon(-700.0, 300.0, *ORIGIN)
    set_origin(node, *origin)
    result = match(node, prior_at(150.0, -60.0),
                   odom=odometry(east=5000.0, north=-5000.0))
    assert result.valid is True
    odom = result.odometry[-1]
    assert odom.pose.pose.position.x == pytest.approx(820.0, abs=0.5)
    assert odom.pose.pose.position.y == pytest.approx(-380.0, abs=0.5)
    assert odom.pose.pose.position.z == 35.0


def test_search_centres_on_prior_and_widens_with_its_sigma(make_node):
    node = make_node()
    prior = prior_at(150.0, -60.0, variance=100.0 ** 2)
    match(node, prior)
    lat, lon, radius = node.cache.searches[-1]
    assert (lat, lon) == (prior.latitude, prior.longitude)
    assert radius == pytest.approx(250.0 + 3.0 * 100.0)
    match(node, prior_at(150.0, -60.0, variance=1e6))
    assert node.cache.searches[-1][2] == 750.0


@pytest.mark.parametrize('variance,accepted', [(None, False),
                                               (100.0 ** 2, True)])
def test_jump_gate_widens_with_prior_uncertainty(make_node, variance,
                                                 accepted):
    # The match is 400 m from the prior: beyond the 250 m gate for a
    # confident prior, inside it once the prior's sigma widens the search.
    node = make_node(target=(400.0, 0.0))
    set_origin(node, *ORIGIN)
    result = match(node, prior_at(0.0, 0.0, variance=variance))
    assert (result.fix is not None) == accepted
    assert result.valid is accepted


def test_image_snapshots_the_prior(make_node):
    node = make_node()
    fused = prior_at(10.0, 10.0)
    gps = prior_at(500.0, 500.0)
    node.subs['/navigation/fix'](fused)
    node.subs['/drone0/sensor_measurements/gps'](gps)
    node._on_image(NS(header=NS(stamp=1)))
    assert node.latest_image_prior is fused
    node.latest_prior_received = time.monotonic() - 10.0
    node._on_image(NS(header=NS(stamp=2)))
    assert node.latest_image_prior is gps
    node.latest_gps_received = time.monotonic() - 10.0
    node._on_image(NS(header=NS(stamp=3)))
    assert node.latest_image_prior is None


def test_vertical_variance_falls_back_to_odometry(make_node):
    node = make_node()
    known = match(node, prior_at(150.0, -60.0, variance=25.0))
    assert known.fix.position_covariance[8] == 4.0
    unknown = match(node, prior_at(150.0, -60.0))
    assert unknown.fix.position_covariance[8] == 2.0


def test_synthetic_world_anchor(wn, tmp_path):
    FakeNode.overrides = dict(cache_dir=str(tmp_path), zoom=ZOOM,
                              tile_url_template='synthetic://world')
    node = wn.WildNavNode()
    # No anchor yet: the synthetic tiles cannot be generated.
    node.latest_image = np.zeros((4, 4, 3), np.uint8)
    node.latest_image_odom = odometry()
    node.latest_image_prior = prior_at(0.0, 0.0)
    node.latest_image_received = time.monotonic()
    node._match_latest()
    assert node.pubs['/wildnav/valid'].publish.call_args.args[0].data is False
    origin = sw.local_to_latlon(30.0, 40.0, *ORIGIN)
    set_origin(node, *origin)
    assert (node.cache.origin_lat, node.cache.origin_lon) == origin
    # A configured anchor is never replaced by the EKF origin.
    FakeNode.overrides.update(initial_lat=ORIGIN[0], initial_lon=ORIGIN[1])
    node = wn.WildNavNode()
    set_origin(node, *origin)
    assert (node.cache.origin_lat, node.cache.origin_lon) == ORIGIN


def test_prior_selection_and_radius(wn):
    now = 100.0
    fused, gps = NavSatFix(1.0, 2.0), NavSatFix(3.0, 4.0)
    assert wn.select_prior(fused, 99.0, gps, 99.0, now, 2.0) is fused
    assert wn.select_prior(fused, 90.0, gps, 99.0, now, 2.0) is gps
    no_fix = NavSatFix(3.0, 4.0, status=NavSatStatus.STATUS_NO_FIX)
    assert wn.select_prior(fused, 90.0, no_fix, 99.0, now, 2.0) is None
    assert wn.search_radius(NavSatFix(), 250.0, 3.0, 750.0) == 250.0
    assert wn.search_radius(NavSatFix(variance=50.0 ** 2),
                            250.0, 3.0, 750.0) == pytest.approx(400.0)


def test_tile_pixel_round_trip():
    lat, lon = sw.local_to_latlon(120.0, -80.0, *ORIGIN)
    x, y = latlon_to_tile(lat, lon, ZOOM)
    back = tile_pixel_to_latlon(int(x), int(y), (x - int(x)) * 256,
                                (y - int(y)) * 256, ZOOM)
    assert back == pytest.approx((lat, lon), abs=1e-9)


class ProjectedCache(FakeCache):
    """Keypoints placed where a camera with this rotation, height above
    flat ground and the Isaac intrinsics sees them: an exact homography.
    The tile is the one holding the image centre's ground point."""

    def __init__(self, vehicle, rotation, height):
        rng = np.random.default_rng(2)
        self.query = rng.uniform([0, 0], [640, 480], (120, 2)).astype(
            np.float32)
        self.descriptors = rng.uniform(0, 255, (120, 128)).astype(np.float32)
        projection = ground_projection(rotation, (FOCAL, FOCAL, CX, CY),
                                       height)
        pixels = np.column_stack((self.query, np.ones(len(self.query))))
        mapped = pixels @ projection.T
        east = vehicle[0] + mapped[:, 0] / mapped[:, 2]
        north = vehicle[1] + mapped[:, 1] / mapped[:, 2]
        centre = projection @ np.array([CX, CY, 1.0])
        self.centre = (vehicle[0] + centre[0] / centre[2],
                       vehicle[1] + centre[1] / centre[2])
        x, y = latlon_to_tile(*sw.local_to_latlon(*self.centre, *ORIGIN),
                              ZOOM)
        self.tile = (int(x), int(y))
        tiles = np.array([
            latlon_to_tile(*sw.local_to_latlon(e, n, *ORIGIN), ZOOM)
            for e, n in zip(east, north)])
        self.points = ((tiles - self.tile) * 256.0).astype(np.float32)
        self.searches = []


def fix_error(result, east, north):
    got = sw.latlon_to_local(result.fix.latitude, result.fix.longitude,
                             *ORIGIN)
    return math.hypot(got[0] - east, got[1] - north)


@pytest.mark.parametrize('roll,pitch,yaw,mount,height', [
    (0.0, 10.0, 30.0, 90.0, 250.0),
    (-12.0, 6.0, -120.0, 0.0, 250.0),
    (8.0, -15.0, 200.0, 90.0, 400.0),
    (18.0, 0.0, 75.0, 0.0, 180.0)])
def test_tilted_camera_fixes_the_point_below_the_vehicle(
        make_node, roll, pitch, yaw, mount, height):
    """The fix is where the nadir pixel lands, not the image centre's
    ground point h * tan(tilt) away; its covariance adds the attitude
    uncertainty times the height (recovered from the matched scale)."""
    vehicle = (300.0, -150.0)
    q = euler_quaternion(roll, pitch, yaw)
    rotation = camera_rotation(q, math.radians(mount))
    node = make_node(camera_mount_yaw_deg=mount)
    node.cache = ProjectedCache(vehicle, rotation, height)
    set_origin(node, *ORIGIN)
    result = match(node, prior_at(*vehicle), odom=odometry(quaternion=q),
                   camera_info=CAMERA_INFO)
    assert result.valid is True
    assert fix_error(result, *vehicle) < 0.5
    assert math.hypot(node.cache.centre[0] - vehicle[0],
                      node.cache.centre[1] - vehicle[1]) > 0.1 * height
    odom = result.odometry[-1]
    assert (odom.pose.pose.position.x, odom.pose.pose.position.y) == \
        pytest.approx(vehicle, abs=0.5)
    expected = horizontal_sigma(3.0, height, math.radians(1.0)) ** 2
    assert result.fix.position_covariance[0] == pytest.approx(expected,
                                                              rel=0.03)


def test_jump_gate_compares_vehicle_positions(make_node):
    """800 m up and pitched 17 deg, the image centre is ~245 m behind the
    vehicle; with a 200 m jump gate the old image-centre fix was rejected
    against a prior at the vehicle, the vehicle position is accepted."""
    vehicle = (100.0, 50.0)
    q = euler_quaternion(pitch_deg=17.0, yaw_deg=10.0)
    node = make_node(max_position_jump_m=200.0)
    node.cache = ProjectedCache(vehicle, camera_rotation(q, 0.0), 800.0)
    assert math.hypot(node.cache.centre[0] - vehicle[0],
                      node.cache.centre[1] - vehicle[1]) > 200.0
    set_origin(node, *ORIGIN)
    result = match(node, prior_at(*vehicle), odom=odometry(quaternion=q),
                   camera_info=CAMERA_INFO)
    assert result.valid is True
    assert fix_error(result, *vehicle) < 0.5


def test_stabilized_camera_ignores_vehicle_tilt(make_node):
    vehicle = (-200.0, 400.0)
    level = camera_rotation(euler_quaternion(yaw_deg=-35.0), 0.0)
    tilted = euler_quaternion(roll_deg=9.0, pitch_deg=-11.0, yaw_deg=-35.0)
    node = make_node(camera_stabilized=True)
    node.cache = ProjectedCache(vehicle, level, 250.0)
    result = match(node, prior_at(*vehicle), odom=odometry(quaternion=tilted),
                   camera_info=CAMERA_INFO)
    assert fix_error(result, *vehicle) < 0.5


def test_attitude_is_taken_at_the_image_stamp(make_node):
    vehicle = (50.0, 60.0)
    q = euler_quaternion(roll_deg=-8.0, pitch_deg=12.0, yaw_deg=140.0)
    node = make_node()
    node.cache = ProjectedCache(vehicle, camera_rotation(q, 0.0), 250.0)
    # Odometry around the image stamp has the image's attitude; the latest
    # sample, 2 s later, is level.
    for offset, attitude in ((-0.05, q), (0.05, q),
                             (2.0, (0.0, 0.0, 0.0, 1.0))):
        node.subs['/navigation/odometry'](
            odometry(quaternion=attitude, stamp=shifted(STAMP, offset)))
    result = match(node, prior_at(*vehicle), camera_info=CAMERA_INFO,
                   odom=node.latest_odom, feed_odom=False)
    assert fix_error(result, *vehicle) < 0.5

    # No odometry within max_attitude_offset_s of the image: no fix.
    node = make_node()
    node.cache = ProjectedCache(vehicle, camera_rotation(q, 0.0), 250.0)
    node.subs['/navigation/odometry'](
        odometry(quaternion=q, stamp=shifted(STAMP, -1.0)))
    result = match(node, prior_at(*vehicle), camera_info=CAMERA_INFO,
                   odom=node.latest_odom, feed_odom=False)
    assert result.fix is None and result.valid is False


def test_simulator_stamped_image_uses_the_clock(make_node):
    """Isaac stamps images in simulator time (0.2x real time here) while
    MAVROS stamps odometry in wall time: /clock maps one onto the other."""
    vehicle = (0.0, 0.0)
    q = euler_quaternion(pitch_deg=10.0, yaw_deg=60.0)
    node = make_node()
    assert '/clock' in node.subs
    node.cache = ProjectedCache(vehicle, camera_rotation(q, 0.0), 250.0)
    sim_stamp = NS(sec=120, nanosec=0)
    wall_ns = STAMP.sec * 1_000_000_000
    for i in range(20):
        node.attitude.add_clock(int((119.0 + 0.1 * i) * 1e9),
                                wall_ns + int(0.5e9 * i))
    # The image's sim time 120 s is wall STAMP + 5 s.
    for offset, attitude in ((4.9, q), (5.1, q), (8.0, (0.0, 0.0, 0.0, 1.0))):
        node.subs['/navigation/odometry'](
            odometry(quaternion=attitude, stamp=shifted(STAMP, offset)))
    for publisher in node.pubs.values():
        publisher.reset_mock()
    node._run_match(np.zeros((480, 640, 3), np.uint8), NS(stamp=sim_stamp),
                    node.latest_odom, prior_at(*vehicle), CAMERA_INFO)
    fix = node.pubs['/wildnav/fix'].publish.call_args.args[0]
    got = sw.latlon_to_local(fix.latitude, fix.longitude, *ORIGIN)
    assert math.hypot(*got) < 0.5


def test_gray_intrinsics_follow_the_matched_image(wn):
    # Calibrated 1280 x 720, matched at 800 x 450.
    info = NS(k=[1000.0, 0, 639.5, 0, 1000.0, 359.5, 0, 0, 1],
              width=1280, height=720)
    fx, fy, cx, cy = wn.gray_intrinsics(info, 800, 450, 0.625, 69.0)
    assert (fx, fy) == pytest.approx((625.0, 625.0))
    assert (cx, cy) == pytest.approx((399.5, 224.5))
    # No calibration: a centred pinhole across fov_deg.
    fx, _, cx, cy = wn.gray_intrinsics(None, 640, 480, 1.0, 60.0)
    assert fx == pytest.approx(320.0 / math.tan(math.radians(30.0)))
    assert (cx, cy) == (320.0, 240.0)


def test_height_from_the_matched_scale(wn):
    """A level camera at h sees h / f metres per pixel."""
    rotation = camera_rotation(euler_quaternion(yaw_deg=20.0), 0.0)
    resolution = ground_resolution_m(ORIGIN[0], ZOOM)
    pixels_per_px = 300.0 / FOCAL / resolution
    homography = np.diag([pixels_per_px, pixels_per_px, 1.0])
    assert wn.height_above_ground(homography, (CX, CY), rotation,
                                  (FOCAL, FOCAL, CX, CY), resolution) == \
        pytest.approx(300.0)
