"""WildNav search prior, EKF-origin anchoring and jump gating, driven
through the node's real __init__ and matcher with ROS stubbed out.

The tile cache is replaced by one whose keypoints are the query's moved by
a known similarity transform, so the geometry is exact and SIFT-free."""
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
from wildnav_pkg.satellite_cache import latlon_to_tile, tile_pixel_to_latlon

ORIGIN = (37.9234, -122.5967)
ZOOM = 17


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


def odometry(east=0.0, north=0.0, z=35.0):
    return NS(pose=NS(pose=NS(position=NS(x=east, y=north, z=z),
                              orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
                      covariance=[0.0] * 14 + [2.0] + [0.0] * 21),
              twist=NS(), child_frame_id='base_link')


def prior_at(east, north, altitude=300.0, **kwargs):
    return NavSatFix(*sw.local_to_latlon(east, north, *ORIGIN), altitude,
                     **kwargs)


def match(node, prior, odom=None):
    for publisher in node.pubs.values():
        publisher.reset_mock()
    node._run_match(np.zeros((480, 640, 3), np.uint8), NS(stamp=1),
                    odom or odometry(), prior, None)

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
