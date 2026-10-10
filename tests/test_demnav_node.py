"""DemNav node behaviour on the synthetic world, driven through its real
__init__ and callbacks with ROS stubbed out.

Level depth frames are rendered with the airplane camera sim's nadir
camera model (pinned against the physical model in test_camera_sim_yaw);
banked and pitched ones by ray casting the synthetic terrain from a
body-fixed camera.
"""
import importlib
import math
import sys
import time
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

import synthetic_world as sw
from conftest import REPO, ROS_MODULES, install_ros_stubs, load_script

ORIGIN = (37.9234, -122.5967)
FOV_DEG = 69.0
WIDTH, HEIGHT = 160, 120
# Frame and odometry stamp (MAVROS and camera both on wall time).
STAMP = NS(sec=1_791_600_000, nanosec=0)


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
            self.position_covariance[0] = variance
            self.position_covariance[4] = variance
            self.position_covariance_type = 2


@pytest.fixture(scope='module')
def dn():
    import demnav_pkg
    previous = getattr(demnav_pkg, 'demnav_node', None)
    with patch.dict(sys.modules):
        for name in ROS_MODULES + ('geographic_msgs', 'geographic_msgs.msg'):
            sys.modules[name] = MagicMock()
        sys.modules['rclpy.node'].Node = FakeNode
        sensor = sys.modules['sensor_msgs.msg']
        sensor.NavSatFix, sensor.NavSatStatus = NavSatFix, NavSatStatus
        std = sys.modules['std_msgs.msg']
        std.Bool = std.Float32 = lambda data: NS(data=data)
        sys.modules.pop('demnav_pkg.demnav_node', None)
        module = importlib.import_module('demnav_pkg.demnav_node')
    # Leave the package as found: other tests import the real module.
    if previous is not None:
        demnav_pkg.demnav_node = previous
    elif hasattr(demnav_pkg, 'demnav_node'):
        del demnav_pkg.demnav_node
    return module


@pytest.fixture(scope='module')
def camera_sim():
    install_ros_stubs()
    return load_script(REPO / 'systems/airplane/scripts/satellite_camera_sim.py',
                       'camera_sim_for_demnav')


@pytest.fixture(scope='module')
def cache_dir(tmp_path_factory):
    return str(tmp_path_factory.mktemp('dem'))


@pytest.fixture
def make_node(dn, cache_dir):
    def make(**overrides):
        FakeNode.overrides = dict(
            initial_lat=ORIGIN[0], initial_lon=ORIGIN[1], area_km=8.0,
            dem_resolution_m=10.0, cache_dir=cache_dir,
            synthetic_terrain=True, gps_altitude_datum='msl',
            fov_deg=FOV_DEG, min_correlation=0.35)
        FakeNode.overrides.update(overrides)
        node = dn.DemNavNode()
        if node._heightmap_thread is not None:
            node._heightmap_thread.join()
        node.bridge = MagicMock()
        return node
    return make


def render(camera_sim, east, north, yaw, agl=180.0, mount=0.0):
    """Depth seen by the sim's body-fixed nadir camera, and the camera's
    MSL altitude."""
    stub = NS(east_m=east, north_m=north, yaw_rad=yaw,
              camera_mount_yaw=math.radians(mount),
              fov=math.radians(FOV_DEG))
    altitude = float(sw.terrain_height(east, north)) + agl
    grid_east, grid_north = camera_sim.SatelliteCameraSim._ground_grid(
        stub, WIDTH, HEIGHT, agl)
    depth = altitude - sw.terrain_height(grid_east, grid_north)
    return depth.astype(np.float32), altitude


def camera_info(width=WIDTH, height=HEIGHT, fov_deg=FOV_DEG):
    focal = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    return NS(width=width, height=height,
              k=[focal, 0, width / 2, 0, focal, height / 2, 0, 0, 1])


def euler_quaternion(roll, pitch, yaw):
    """(x, y, z, w) of ZYX Euler angles in radians."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def odometry(yaw, roll=0.0, pitch=0.0, stamp=STAMP):
    x, y, z, w = euler_quaternion(roll, pitch, yaw)
    zero = NS(x=0.0, y=0.0, z=0.0)
    return NS(header=NS(stamp=stamp),
              pose=NS(pose=NS(orientation=NS(x=x, y=y, z=z, w=w))),
              twist=NS(twist=NS(linear=zero, angular=zero)))


def fix_at(east, north, altitude, **kwargs):
    return NavSatFix(*sw.local_to_latlon(east, north, *ORIGIN), altitude,
                     **kwargs)


def frame(node, depth, yaw, prior=None, gps=None, info=True, roll=0.0,
          pitch=0.0, odom=True):
    """Feed one depth frame; return what the node published for it."""
    for publisher in node.pubs.values():
        publisher.reset_mock()
    node.bridge.reset_mock()
    if odom:
        node.subs['/odom'](odometry(yaw, roll, pitch))
    if info:
        node.subs['/oak1/camera_info'](camera_info())
    if prior is not None:
        node.subs['/navigation/fix'](prior)
    if gps is not None:
        node.subs['/fix'](gps)
    node.bridge.imgmsg_to_cv2.return_value = depth
    node.subs['/oak1/relative_depth'](NS(header=NS(stamp=STAMP)))

    def sent(topic):
        return [c.args[0] for c in node.pubs[topic].publish.call_args_list]
    valid = sent('/demnav/valid')
    fixes = sent('/demnav/fix')
    metric = node.bridge.cv2_to_imgmsg.call_args_list
    confidence = sent('/demnav/confidence')
    return NS(valid=valid[-1].data if valid else None,
              fix=fixes[-1] if fixes else None,
              odometry=sent('/demnav/odometry'),
              confidence=confidence[-1].data if confidence else None,
              metric_depth=metric[0].args[0] if metric else None)


def error_m(result, east, north):
    got_east, got_north = sw.latlon_to_local(
        result.fix.latitude, result.fix.longitude, *ORIGIN)
    return got_east - east, got_north - north


def set_origin(node, lat=ORIGIN[0], lon=ORIGIN[1], alt=250.0):
    node.subs['/navigation/origin'](
        NS(position=NS(latitude=lat, longitude=lon, altitude=alt)))


@pytest.mark.parametrize('yaw_deg,mount_deg', [
    (0.0, 0.0), (90.0, 0.0), (180.0, 0.0), (-90.0, 0.0), (30.0, 0.0),
    (-135.0, 0.0), (0.0, 90.0), (-60.0, -90.0)])
def test_fix_at_any_heading_and_camera_mount(make_node, camera_sim,
                                              yaw_deg, mount_deg):
    node = make_node(camera_mount_yaw_deg=mount_deg)
    set_origin(node)
    east, north, yaw = 500.0, 200.0, math.radians(yaw_deg)
    depth, altitude = render(camera_sim, east, north, yaw, mount=mount_deg)
    result = frame(node, depth, yaw,
                   prior=fix_at(east + 60.0, north - 45.0, altitude))
    assert result.valid is True
    assert math.hypot(*error_m(result, east, north)) < 8.0


def test_no_systematic_offset(make_node, camera_sim):
    """int() truncation of the prior pixel plus a w//2 template centre
    biased every fix ~1 DEM cell south-east (+9.5 m E, -9.3 m N)."""
    node = make_node()
    rng = np.random.default_rng(4)
    errors = []
    for _ in range(16):
        east, north = rng.uniform(-600, 600, 2)
        yaw = rng.uniform(-math.pi, math.pi)
        depth, altitude = render(camera_sim, east, north, yaw)
        result = frame(node, depth, yaw, prior=fix_at(east, north, altitude))
        if result.fix is not None:
            errors.append(error_m(result, east, north))
    errors = np.array(errors)
    assert len(errors) >= 12
    assert abs(errors[:, 0].mean()) < 2.0
    assert abs(errors[:, 1].mean()) < 2.0
    assert np.median(np.hypot(*errors.T)) < 5.0


def test_terrain_lookalikes_are_rejected(make_node, camera_sim):
    """Away from its markers the synthetic terrain repeats: with a wide
    search, near-perfect look-alikes hundreds of metres away used to win
    at NCC ~1.0. The second-peak test must refuse those fixes."""
    rng = np.random.default_rng(0)
    cases = []
    # Repetitive terrain out to 1.5 km, and the distinctive centre.
    for extent, count in ((1500.0, 16), (500.0, 16)):
        for _ in range(count):
            east, north = rng.uniform(-extent, extent, 2)
            yaw = rng.uniform(-math.pi, math.pi)
            cases.append((east, north, yaw) +
                          render(camera_sim, east, north, yaw))

    def wrong_and_right(node):
        wrong = right = 0
        for east, north, yaw, depth, altitude in cases:
            result = frame(node, depth, yaw,
                           prior=fix_at(east, north, altitude))
            if result.fix is not None:
                if math.hypot(*error_m(result, east, north)) > 50.0:
                    wrong += 1
                else:
                    right += 1
        return wrong, right

    # Without the uniqueness test the look-alikes produce wrong fixes...
    wrong, _ = wrong_and_right(make_node(search_radius_m=1200.0,
                                         min_peak_uniqueness=0.0))
    assert wrong >= 3
    # ...with it none get through, while distinctive terrain still fixes.
    wrong, right = wrong_and_right(make_node(search_radius_m=1200.0))
    assert wrong == 0
    assert right >= 4


def test_peak_uniqueness_scale(dn):
    assert dn.peak_uniqueness(1.0, 0.999) < 0.2   # near-perfect fits tie
    assert dn.peak_uniqueness(0.9999, 0.95) > 0.8
    assert dn.peak_uniqueness(0.8, 0.8) == 0.0
    assert dn.peak_uniqueness(0.8, 0.0) > 0.6


def test_local_outputs_wait_for_ekf_origin(make_node, camera_sim):
    node = make_node()
    east, north, yaw = -220.0, 95.0, 0.4
    depth, altitude = render(camera_sim, east, north, yaw)
    result = frame(node, depth, yaw, prior=fix_at(east, north, altitude))
    # The lat/lon fix does not need the origin; local odometry does.
    assert result.fix is not None
    assert result.valid is False
    assert result.odometry == []

    # The EKF origin, not the first GPS fix or initial_lat/lon, anchors
    # the local frame.
    origin = sw.local_to_latlon(-1000.0, 500.0, *ORIGIN)
    set_origin(node, *origin, alt=200.0)
    result = frame(node, depth, yaw, prior=fix_at(east, north, altitude))
    assert result.valid is True
    odom = result.odometry[-1]
    expected = sw.latlon_to_local(result.fix.latitude, result.fix.longitude,
                                  *origin)
    assert odom.pose.pose.position.x == pytest.approx(expected[0])
    assert odom.pose.pose.position.y == pytest.approx(expected[1])
    assert odom.pose.pose.position.z == pytest.approx(altitude - 200.0)
    assert expected[0] == pytest.approx(east + 1000.0, abs=8.0)


def test_prior_selection(dn):
    now = 100.0
    fused = NavSatFix(1.0, 2.0)
    gps = NavSatFix(3.0, 4.0)
    assert dn.select_prior(fused, 99.0, gps, 99.5, now, 2.0) is fused
    assert dn.select_prior(fused, 97.0, gps, 99.5, now, 2.0) is gps
    assert dn.select_prior(None, 0.0, gps, 99.5, now, 2.0) is gps
    assert dn.select_prior(fused, 97.0, gps, 97.0, now, 2.0) is None
    no_fix = NavSatFix(3.0, 4.0, status=NavSatStatus.STATUS_NO_FIX)
    assert dn.select_prior(None, 0.0, no_fix, 99.5, now, 2.0) is None
    nan_fused = NavSatFix(float('nan'), 2.0)
    assert dn.select_prior(nan_fused, 99.0, gps, 99.5, now, 2.0) is gps


def test_search_radius_grows_with_prior_uncertainty(dn):
    assert dn.search_radius(NavSatFix(), 200.0, 3.0, 600.0) == 200.0
    assert dn.search_radius(NavSatFix(variance=50.0 ** 2),
                            200.0, 3.0, 600.0) == pytest.approx(350.0)
    assert dn.search_radius(NavSatFix(variance=1e6),
                            200.0, 3.0, 600.0) == 600.0
    unknown = NavSatFix(variance=1e4)
    unknown.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    assert dn.search_radius(unknown, 200.0, 3.0, 600.0) == 200.0
    assert dn.search_radius(NavSatFix(variance=float('nan')),
                            200.0, 3.0, 600.0) == 200.0


def test_search_follows_the_fused_prior_and_skips_stale_inputs(
        make_node, camera_sim):
    node = make_node()
    east, north, yaw = 150.0, 260.0, 1.1
    depth, altitude = render(camera_sim, east, north, yaw)
    # GPS 2 km off (e.g. spoofed); the fresh fused prior wins.
    result = frame(node, depth, yaw, prior=fix_at(east, north, altitude),
                   gps=fix_at(east + 2000.0, north, altitude))
    assert result.fix is not None
    assert math.hypot(*error_m(result, east, north)) < 8.0

    # Stale fused prior and stale GPS: no search at all.
    node.latest_prior_received = time.monotonic() - 10.0
    node.latest_gps_received = time.monotonic() - 10.0
    node.subs['/oak1/relative_depth'](NS(header=NS(stamp=STAMP)))
    assert node.pubs['/demnav/valid'].publish.call_args.args[0].data is False
    node.pubs['/demnav/fix'].publish.reset_mock()
    node.subs['/oak1/relative_depth'](NS(header=NS(stamp=STAMP)))
    node.pubs['/demnav/fix'].publish.assert_not_called()


def test_search_window_off_the_map_is_rejected_then_map_recentres(
        make_node, camera_sim):
    """A clamped DEM window at the map edge used to return confident fixes
    hundreds of metres off; the map was also never reloaded."""
    node = make_node(area_km=4.0)
    old_map = node.heightmap
    east, north, yaw = 2600.0, 100.0, 0.3
    depth, altitude = render(camera_sim, east, north, yaw)
    result = frame(node, depth, yaw, prior=fix_at(east, north, altitude))
    assert result.fix is None and result.valid is False
    node._heightmap_thread.join()
    assert node.heightmap is not old_map
    assert node.heightmap.edge_distance_m(
        *sw.local_to_latlon(east, north, *ORIGIN)) > 1900.0

    result = frame(node, depth, yaw, prior=fix_at(east, north, altitude))
    assert result.fix is not None
    assert math.hypot(*error_m(result, east, north)) < 8.0


def test_window_crossing_the_edge_is_rejected_without_reload(
        make_node, camera_sim):
    node = make_node(area_km=4.0, search_radius_m=200.0)
    old_map = node.heightmap
    # 1.9 km east: on the map, but the search window crosses its edge.
    east, north, yaw = 1900.0, 0.0, 0.0
    depth, altitude = render(camera_sim, east, north, yaw)
    result = frame(node, depth, yaw, prior=fix_at(east, north, altitude))
    assert result.fix is None
    if node._heightmap_thread is not None:
        node._heightmap_thread.join()
    # Close enough to the edge for the widest search to need a new map.
    assert node.heightmap is not old_map


@pytest.fixture
def scale_spy(dn, monkeypatch):
    """Records the metres-per-pixel below the camera the template was built
    with (height above ground over the focal lengths)."""
    calls = []
    original = dn.ground_template

    def spy(depth, valid, rotation, intrinsics, height_m, cell_m):
        calls.append((height_m / intrinsics[0], height_m / intrinsics[1]))
        return original(depth, valid, rotation, intrinsics, height_m, cell_m)
    monkeypatch.setattr(dn, 'ground_template', spy)
    return calls


def write_constant_geoid(path, height_m):
    # A whole-earth GeographicLib grid with one constant geoid height.
    sample = int(round((height_m + 108.0) / 0.003))
    path.write_bytes(b'P5\n# Offset -108\n# Scale 0.003\n4 3\n65535\n' +
                     np.full((3, 4), sample, '>u2').tobytes())
    return str(path)


def test_ellipsoid_altitude_is_converted_to_msl(make_node, camera_sim,
                                                tmp_path, scale_spy):
    """SRTM is MSL while MAVROS reports ellipsoid height: at Mount
    Tamalpais the geoid is ~-32 m, so AGL came out ~32 m low."""
    geoid = write_constant_geoid(tmp_path / 'geoid.pgm', -32.0)
    east, north, yaw = -80.0, 40.0, 2.0
    depth, altitude_msl = render(camera_sim, east, north, yaw, agl=180.0)
    ellipsoid = altitude_msl - 32.0
    focal = camera_info().k[0]

    node = make_node(gps_altitude_datum='ellipsoid', geoid_file=geoid)
    result = frame(node, depth, yaw, prior=fix_at(east, north, ellipsoid))
    assert scale_spy[-1][0] == pytest.approx(180.0 / focal, rel=0.01)
    # The affine depth fit lands on the true range, and the fix keeps the
    # input altitude and datum.
    assert np.nanmedian(np.abs(result.metric_depth - depth)) < 1.0
    assert result.fix.altitude == ellipsoid

    # Treating the same ellipsoid altitude as MSL reproduces the bug.
    node = make_node(gps_altitude_datum='msl')
    frame(node, depth, yaw, prior=fix_at(east, north, ellipsoid))
    assert scale_spy[-1][0] == pytest.approx(148.0 / focal, rel=0.01)


def test_missing_geoid_warns_once_and_assumes_zero(make_node, camera_sim,
                                                   scale_spy):
    node = make_node(gps_altitude_datum='ellipsoid',
                     geoid_file='/nonexistent/egm96-5.pgm')
    warnings = [c for c in node.logger.warn.call_args_list
                if 'geoid' in str(c.args[0])]
    assert len(warnings) == 1
    east, north, yaw = -80.0, 40.0, 2.0
    depth, altitude = render(camera_sim, east, north, yaw, agl=180.0)
    frame(node, depth, yaw, prior=fix_at(east, north, altitude))
    frame(node, depth, yaw, prior=fix_at(east, north, altitude))
    assert scale_spy[-1][0] == pytest.approx(180.0 / camera_info().k[0],
                                             rel=0.01)
    assert len([c for c in node.logger.warn.call_args_list
                if 'geoid' in str(c.args[0])]) == 1


def test_unknown_altitude_datum_is_refused(make_node):
    with pytest.raises(ValueError):
        make_node(gps_altitude_datum='wgs84')


def test_rangefinder_sets_height_above_ground(make_node, camera_sim,
                                              scale_spy):
    node = make_node()
    east, north, yaw = 120.0, -60.0, -0.7
    depth, altitude = render(camera_sim, east, north, yaw, agl=180.0)
    focal = camera_info().k[0]
    # Altitude prior 50 m high (e.g. barometric drift).
    prior = fix_at(east, north, altitude + 50.0)

    node.subs['/aerodrone/rangefinder'](
        NS(range=180.0, min_range=0.3, max_range=400.0))
    result = frame(node, depth, yaw, prior=prior)
    assert scale_spy[-1][0] == pytest.approx(180.0 / focal, rel=0.01)
    assert math.hypot(*error_m(result, east, north)) < 8.0

    # Stale or out-of-range readings fall back to the altitude.
    node.latest_range_received = time.monotonic() - 1.0
    frame(node, depth, yaw, prior=prior)
    assert scale_spy[-1][0] == pytest.approx(230.0 / focal, rel=0.01)
    node.subs['/aerodrone/rangefinder'](
        NS(range=180.0, min_range=0.3, max_range=40.0))
    frame(node, depth, yaw, prior=prior)
    assert scale_spy[-1][0] == pytest.approx(230.0 / focal, rel=0.01)

    disabled = make_node(rangefinder_topic='')
    assert '/aerodrone/rangefinder' not in disabled.subs


def test_camera_info_sets_ground_sampling(make_node, camera_sim, scale_spy):
    """GSD came from fov_deg even with calibration available: an OAK-1
    256x192 preview is a 4:3 crop (~52 deg), not the 69 deg default."""
    node = make_node(fov_deg=69.0)
    east, north, yaw = 0.0, 0.0, 0.0
    depth, altitude = render(camera_sim, east, north, yaw, agl=180.0)
    prior = fix_at(east, north, altitude)

    frame(node, depth, yaw, prior=prior, info=False)
    assert scale_spy[-1][0] == pytest.approx(
        180.0 * 2 * math.tan(math.radians(69.0) / 2) / WIDTH, rel=0.01)

    # Calibration for a 52 deg camera, published at twice the resolution.
    node.subs['/oak1/camera_info'](camera_info(2 * WIDTH, 2 * HEIGHT, 52.0))
    frame(node, depth, yaw, prior=prior, info=False)
    expected = 180.0 * 2 * math.tan(math.radians(52.0) / 2) / WIDTH
    assert scale_spy[-1][0] == pytest.approx(expected, rel=0.01)
    assert scale_spy[-1][1] == pytest.approx(expected, rel=0.01)


def test_widest_search_never_below_base_radius(make_node):
    # A stack raising search_radius_m above the max keeps the base radius
    # for map-edge planning too.
    node = make_node(search_radius_m=900.0, max_search_radius_m=600.0)
    assert node.max_search_radius_m == 900.0


def shifted(stamp, seconds):
    total = stamp.sec * 1_000_000_000 + stamp.nanosec + int(seconds * 1e9)
    return NS(sec=total // 1_000_000_000, nanosec=total % 1_000_000_000)


def render_tilted(east, north, roll, pitch, yaw, mount_deg=0.0, agl=180.0):
    """Metric z-depth seen by a body-fixed camera at this attitude (ray cast
    over the synthetic terrain), the camera's MSL altitude, and where the
    optical axis meets the ground."""
    from demnav_pkg.camera_geometry import camera_rotation
    rotation = camera_rotation(euler_quaternion(roll, pitch, yaw),
                               math.radians(mount_deg))
    k = camera_info().k
    u, v = np.meshgrid(np.arange(WIDTH, dtype=float),
                       np.arange(HEIGHT, dtype=float))
    rays = np.stack(((u - k[2]) / k[0], (v - k[5]) / k[4], np.ones_like(u)),
                    axis=-1) @ rotation.T
    altitude = float(sw.terrain_height(east, north)) + agl
    depth = np.full(u.shape, agl)
    for _ in range(60):
        ground = sw.terrain_height(east + depth * rays[..., 0],
                                   north + depth * rays[..., 1])
        gap = altitude + depth * rays[..., 2] - ground
        depth = depth + 0.7 * gap / np.maximum(-rays[..., 2], 1e-3)
    row, col = int(k[5]), int(k[2])
    centre = (east + depth[row, col] * rays[row, col, 0],
              north + depth[row, col] * rays[row, col, 1])
    return depth.astype(np.float32), altitude, centre


@pytest.mark.parametrize('roll_deg,pitch_deg,yaw_deg,mount_deg', [
    (0.0, 12.0, 30.0, 0.0), (-10.0, 0.0, -100.0, 90.0),
    (8.0, -8.0, 160.0, 90.0), (14.0, 6.0, 45.0, 0.0)])
def test_tilted_camera_fixes_the_vehicle_not_the_image_centre(
        make_node, roll_deg, pitch_deg, yaw_deg, mount_deg):
    """A banked or pitched body-fixed camera looks h * tan(tilt) away from
    the vehicle; the fix must still be the point below the vehicle."""
    node = make_node(camera_mount_yaw_deg=mount_deg, depth_is_metric=True)
    set_origin(node)
    east, north = 500.0, 200.0
    roll, pitch, yaw = (math.radians(v) for v in (roll_deg, pitch_deg,
                                                    yaw_deg))
    depth, altitude, centre = render_tilted(east, north, roll, pitch, yaw,
                                            mount_deg)
    result = frame(node, depth, yaw, roll=roll, pitch=pitch,
                   prior=fix_at(east + 60.0, north - 45.0, altitude))
    assert result.valid is True
    assert math.hypot(*error_m(result, east, north)) < 8.0
    # The old answer, where the optical axis meets the ground, is far off.
    assert math.hypot(centre[0] - east, centre[1] - north) > 25.0


def test_stabilized_camera_ignores_vehicle_tilt(make_node, camera_sim):
    """Genesis and satellite_camera_sim hold the camera level: their frames
    must not be shifted by the vehicle's roll and pitch."""
    east, north, yaw = 500.0, 200.0, 0.9
    depth, altitude = render(camera_sim, east, north, yaw)
    tilt = {'roll': math.radians(10.0), 'pitch': math.radians(-12.0)}
    stabilised = frame(make_node(camera_stabilized=True), depth, yaw,
                       prior=fix_at(east, north, altitude), **tilt)
    assert math.hypot(*error_m(stabilised, east, north)) < 8.0
    # Treated as body-fixed, the same frame lands ~h * tan(tilt) away.
    body_fixed = frame(make_node(), depth, yaw,
                       prior=fix_at(east, north, altitude), **tilt)
    assert (body_fixed.fix is None or
            math.hypot(*error_m(body_fixed, east, north)) > 25.0)


def test_the_attitude_at_the_frame_stamp_is_used(make_node):
    east, north = 500.0, 200.0
    roll, pitch, yaw = 0.0, math.radians(12.0), math.radians(-60.0)
    depth, altitude, _ = render_tilted(east, north, roll, pitch, yaw)
    prior = fix_at(east, north, altitude)

    # Odometry brackets the frame with its attitude; the latest sample,
    # seconds later, is level.
    node = make_node(depth_is_metric=True)
    for offset, attitude in ((-0.1, (roll, pitch)), (0.1, (roll, pitch)),
                             (3.0, (0.0, 0.0))):
        node.subs['/odom'](odometry(yaw, *attitude,
                                    stamp=shifted(STAMP, offset)))
    result = frame(node, depth, yaw, prior=prior, odom=False)
    assert math.hypot(*error_m(result, east, north)) < 8.0

    # No odometry within max_attitude_offset_s of the frame: no fix.
    node = make_node(depth_is_metric=True)
    node.subs['/odom'](odometry(yaw, roll, pitch, stamp=shifted(STAMP, -1.0)))
    result = frame(node, depth, yaw, prior=prior, odom=False)
    assert result.fix is None and result.valid is False


def test_fix_covariance_adds_attitude_error_times_height(make_node,
                                                        camera_sim):
    node = make_node(search_radius_m=50.0, attitude_sigma_deg=5.0)
    east, north, yaw = 100.0, -50.0, 0.3
    depth, altitude = render(camera_sim, east, north, yaw, agl=180.0)
    result = frame(node, depth, yaw, prior=fix_at(east, north, altitude))
    match = max(10.0, 50.0 * (1.0 - result.confidence))
    expected = match ** 2 + (180.0 * math.radians(5.0)) ** 2
    assert result.fix.position_covariance[0] == pytest.approx(expected,
                                                              rel=0.03)
    assert result.fix.position_covariance[4] == \
        result.fix.position_covariance[0]


def test_tilted_range_becomes_height_below_the_camera():
    """Metric z-depth of flat ground from a tilted camera ramps across the
    frame (a false terrain slope); as height below the camera it is flat,
    and level frames are unchanged."""
    from demnav_pkg.camera_geometry import camera_rotation
    from demnav_pkg.terrain_match import height_below_camera
    k = camera_info().k
    intrinsics = (k[0], k[4], k[2], k[5])
    u, v = np.meshgrid(np.arange(WIDTH, dtype=float),
                       np.arange(HEIGHT, dtype=float))
    for roll, pitch, expect_flat in ((0.0, 0.0, True), (9.0, -13.0, True)):
        rotation = camera_rotation(euler_quaternion(
            math.radians(roll), math.radians(pitch), 0.7), 0.0)
        rays = np.stack(((u - k[2]) / k[0], (v - k[5]) / k[4],
                         np.ones_like(u)), axis=-1) @ rotation.T
        z_depth = (150.0 / -rays[..., 2]).astype(np.float32)
        height = height_below_camera(z_depth, rotation, intrinsics)
        assert np.allclose(height, 150.0, atol=1e-3)
        assert (np.ptp(z_depth) < 1e-3) == (roll == 0.0 and pitch == 0.0)
