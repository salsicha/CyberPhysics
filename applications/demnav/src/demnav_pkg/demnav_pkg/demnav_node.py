#!/usr/bin/env python3
import math
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import (
    CameraInfo,
    Image,
    NavSatFix,
    NavSatStatus,
    PointCloud2,
    PointField,
    Range,
)
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge

from synthetic_world import latlon_to_local

from .camera_geometry import (
    AttitudeHistory,
    camera_rotation,
    horizontal_sigma,
    quaternion_matrix,
    roll_pitch_rad,
    stamp_ns,
)
from .geoid import GeoidGrid
from .heightmap import HeightMapCache
from .terrain_match import (
    correlate,
    ground_template,
    heading_sensitivity,
    match_sigma,
    terrain_template,
    view_trend,
)

# Unexplained-variance floor of the peak uniqueness test: two fits both
# better than NCC ~0.995 are a tie, not a win for resampling noise.
PEAK_RESIDUAL_FLOOR = 0.01
# Steepest view angle the relative-depth relief term counts (80 deg).
MAX_VIEW_RAD = math.radians(80.0)
# Camera tilt beyond which relative depth also loses the curvature a
# tilted view of flat ground puts along the view's heading (inverse or log
# depth; under half a metre below 5 deg at 200 m with a 69 deg lens).
QUADRATIC_TILT_RAD = math.radians(5.0)


def fix_is_usable(fix, received, now, timeout_s) -> bool:
    return (fix is not None and now - received <= timeout_s and
            fix.status.status >= NavSatStatus.STATUS_FIX and
            math.isfinite(fix.latitude) and math.isfinite(fix.longitude))


def select_prior(prior, prior_received, gps, gps_received, now, timeout_s):
    """Search centre: the fused navigation fix while fresh, else a fresh
    raw GPS fix, else None (skip the frame)."""
    if fix_is_usable(prior, prior_received, now, timeout_s):
        return prior
    if fix_is_usable(gps, gps_received, now, timeout_s):
        return gps
    return None


def search_radius(fix, base_m, sigma_scale, maximum_m) -> float:
    """clamp(base + k * sigma, base, maximum) with sigma the larger
    horizontal standard deviation of the fix; an unknown or invalid
    covariance adds nothing."""
    sigma = 0.0
    if fix.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        variance = max(fix.position_covariance[0], fix.position_covariance[4])
        if math.isfinite(variance) and variance > 0.0:
            sigma = math.sqrt(variance)
    return min(max(base_m + sigma_scale * sigma, base_m),
               max(maximum_m, base_m))


def _flag(value) -> bool:
    """A boolean parameter that launch files may pass as a string."""
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def peak_uniqueness(best: float, second: float) -> float:
    """1 - (unexplained variance of the best fit) / (that of the runner-up
    outside the peak neighbourhood): 0 when a look-alike fits as well,
    near 1 when the best fit is much better."""
    best_residual = 1.0 - best * best + PEAK_RESIDUAL_FLOOR
    second_residual = 1.0 - second * second + PEAK_RESIDUAL_FLOOR
    return float(np.clip(1.0 - best_residual / second_residual, 0.0, 1.0))


class DemNavNode(Node):
    def __init__(self):
        super().__init__('demnav_node')

        self.declare_parameter('gps_topic',        '/fix')
        self.declare_parameter('depth_topic',      '/oak1/relative_depth')
        self.declare_parameter('camera_info_topic', '/oak1/camera_info')
        self.declare_parameter('odom_topic',       '/odom')
        self.declare_parameter('output_topic',     '/demnav/fix')
        self.declare_parameter('odometry_output_topic', '/demnav/odometry')
        self.declare_parameter('pose_output_topic', '/demnav/pose')
        self.declare_parameter('metric_depth_topic', '/demnav/metric_depth')
        self.declare_parameter('pointcloud_topic', '/demnav/points')
        self.declare_parameter('confidence_topic', '/demnav/confidence')
        self.declare_parameter('scale_topic', '/demnav/depth_scale')
        self.declare_parameter('valid_topic', '/demnav/valid')
        self.declare_parameter('origin_topic',     '/navigation/origin')
        self.declare_parameter('prior_fix_topic',  '/navigation/fix')
        self.declare_parameter('prior_timeout_s',  2.0)
        self.declare_parameter('rangefinder_topic', '/aerodrone/rangefinder')
        self.declare_parameter('rangefinder_timeout_s', 0.5)
        self.declare_parameter('initial_lat',      0.0)
        self.declare_parameter('initial_lon',      0.0)
        self.declare_parameter('area_km',          10.0)
        self.declare_parameter('search_radius_m',  200.0)
        self.declare_parameter('search_radius_sigma_scale', 3.0)
        self.declare_parameter('max_search_radius_m', 600.0)
        self.declare_parameter('fov_deg',          90.0)
        self.declare_parameter('camera_mount_yaw_deg', 0.0)
        self.declare_parameter('camera_stabilized', False)
        self.declare_parameter('depth_is_metric', False)
        self.declare_parameter('attitude_sigma_deg', 1.0)
        self.declare_parameter('heading_sigma_deg', 1.0)
        self.declare_parameter('match_sigma_scale', 2.0)
        self.declare_parameter('match_sigma_floor_cells', 0.1)
        self.declare_parameter('stamp_sigma_s', 0.05)
        self.declare_parameter('max_attitude_offset_s', 0.25)
        self.declare_parameter('sim_clock_topic', '/clock')
        self.declare_parameter('gps_altitude_datum', 'ellipsoid')
        self.declare_parameter(
            'geoid_file', '/usr/share/GeographicLib/geoids/egm96-5.pgm')
        self.declare_parameter('dem_resolution_m', 30.0)
        self.declare_parameter('min_correlation',  0.3)
        self.declare_parameter('min_peak_uniqueness', 0.5)
        self.declare_parameter('cache_dir',        '/data/demnav_cache')
        self.declare_parameter('synthetic_terrain', False)
        self.declare_parameter('pointcloud_stride', 4)
        self.declare_parameter('min_dem_footprint_pixels', 5)
        self.declare_parameter('min_metric_depth_m', 0.2)
        self.declare_parameter('max_metric_depth_m', 500.0)
        self.declare_parameter('max_attitude_deg', 20.0)

        gps_topic      = self.get_parameter('gps_topic').value
        depth_topic    = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        odom_topic     = self.get_parameter('odom_topic').value
        output_topic   = self.get_parameter('output_topic').value
        initial_lat    = self.get_parameter('initial_lat').value
        initial_lon    = self.get_parameter('initial_lon').value

        self.fov_deg          = self.get_parameter('fov_deg').value
        self.search_radius_m  = self.get_parameter('search_radius_m').value
        self.search_radius_sigma_scale = float(
            self.get_parameter('search_radius_sigma_scale').value)
        self.max_search_radius_m = max(
            float(self.get_parameter('max_search_radius_m').value),
            float(self.search_radius_m))
        self.prior_timeout_s = float(
            self.get_parameter('prior_timeout_s').value)
        self.rangefinder_timeout_s = float(
            self.get_parameter('rangefinder_timeout_s').value)
        self.camera_mount_yaw_rad = math.radians(
            float(self.get_parameter('camera_mount_yaw_deg').value))
        self.camera_stabilized = _flag(
            self.get_parameter('camera_stabilized').value)
        self.depth_is_metric = _flag(
            self.get_parameter('depth_is_metric').value)
        self.attitude_sigma_rad = math.radians(
            float(self.get_parameter('attitude_sigma_deg').value))
        self.stamp_sigma_s = float(self.get_parameter('stamp_sigma_s').value)
        self.heading_sigma_rad = math.radians(
            float(self.get_parameter('heading_sigma_deg').value))
        self.match_sigma_scale = float(
            self.get_parameter('match_sigma_scale').value)
        self.match_sigma_floor_cells = float(
            self.get_parameter('match_sigma_floor_cells').value)
        self.min_correlation  = self.get_parameter('min_correlation').value
        self.min_peak_uniqueness = float(
            self.get_parameter('min_peak_uniqueness').value)
        # Local outputs are anchored at the autopilot's EKF origin, received
        # on origin_topic; nothing local is published before it arrives.
        self.origin = None
        # initial_lat/initial_lon only pre-load the map area; they are also
        # the fixed anchor of the synthetic world (the simulator renders it
        # about the same point), which falls back to the EKF origin.
        initial_set = initial_lat != 0.0 or initial_lon != 0.0
        self.synthetic_anchor = (
            (initial_lat, initial_lon) if initial_set else None)
        self.pointcloud_stride = max(
            1, int(self.get_parameter('pointcloud_stride').value))
        self.min_dem_footprint_pixels = max(
            3, int(self.get_parameter('min_dem_footprint_pixels').value))
        self.min_metric_depth = self.get_parameter('min_metric_depth_m').value
        self.max_metric_depth = self.get_parameter('max_metric_depth_m').value
        self.max_attitude_rad = math.radians(
            float(self.get_parameter('max_attitude_deg').value))
        self.synthetic_terrain = _flag(
            self.get_parameter('synthetic_terrain').value)

        self.geoid = None
        datum = str(self.get_parameter('gps_altitude_datum').value).lower()
        if datum not in ('ellipsoid', 'msl'):
            raise ValueError(
                f"gps_altitude_datum must be 'ellipsoid' or 'msl', not {datum!r}")
        if datum == 'ellipsoid':
            geoid_file = self.get_parameter('geoid_file').value
            try:
                self.geoid = GeoidGrid(geoid_file)
            except (OSError, ValueError) as e:
                # SRTM heights are EGM96 MSL, so without the geoid every
                # height above ground is off by the local geoid height
                # (about -32 m at Mount Tamalpais, within +-110 m worldwide).
                self.get_logger().warn(
                    f'No geoid model ({e}); treating ellipsoid altitudes as '
                    'MSL — height above ground will be off by the local '
                    'geoid height (about 30 m in California)')

        self.bridge       = CvBridge()
        self.latest_gps   = None
        self.latest_gps_received = 0.0
        self.latest_prior = None
        self.latest_prior_received = 0.0
        self.latest_range = None
        self.latest_range_received = 0.0
        self.latest_odom  = None
        self.camera_info  = None
        # Vehicle attitude by odometry stamp: each depth frame is projected
        # with the attitude at its own timestamp.
        self.attitude = AttitudeHistory(max_offset_s=float(
            self.get_parameter('max_attitude_offset_s').value))
        self.heightmap    = None
        self._heightmap_thread = None

        if 500.0 * float(self.get_parameter('area_km').value) < (
                1.2 * self.max_search_radius_m):
            self.get_logger().warn(
                'area_km is too small to hold max_search_radius_m searches; '
                'wide searches will leave the map and be rejected')
        if initial_set:
            self._load_heightmap(initial_lat, initial_lon)
        else:
            self.get_logger().warn(
                'initial_lat/initial_lon not set — height map loads around '
                'the first search prior')

        self.create_subscription(
            NavSatFix, gps_topic, self._on_fix, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, self.get_parameter('prior_fix_topic').value,
            self._on_prior, qos_profile_sensor_data)
        self.create_subscription(
            GeoPointStamped, self.get_parameter('origin_topic').value,
            self._on_origin,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        rangefinder_topic = self.get_parameter('rangefinder_topic').value
        if rangefinder_topic:
            self.create_subscription(
                Range, rangefinder_topic, self._on_range,
                qos_profile_sensor_data)
        self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        clock_topic = str(self.get_parameter('sim_clock_topic').value or '')
        if clock_topic:
            self.create_subscription(
                Clock, clock_topic, self._on_clock, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info,
            qos_profile_sensor_data)
        self.fix_publisher = self.create_publisher(
            NavSatFix, output_topic, 10)
        self.odom_publisher = self.create_publisher(
            Odometry, self.get_parameter('odometry_output_topic').value, 10)
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('pose_output_topic').value, 10)
        self.metric_depth_publisher = self.create_publisher(
            Image, self.get_parameter('metric_depth_topic').value, 10)
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, self.get_parameter('pointcloud_topic').value, 10)
        self.confidence_publisher = self.create_publisher(
            Float32, self.get_parameter('confidence_topic').value, 10)
        self.scale_publisher = self.create_publisher(
            Float32, self.get_parameter('scale_topic').value, 10)
        self.valid_publisher = self.create_publisher(
            Bool, self.get_parameter('valid_topic').value, 10)
        self.get_logger().info(
            f'DemNav ready | gps={gps_topic} depth={depth_topic} out={output_topic}')

    def _load_heightmap(self, lat: float, lon: float):
        if (self._heightmap_thread is not None and
                self._heightmap_thread.is_alive()):
            return
        if self.synthetic_terrain and self.synthetic_anchor is None:
            # The synthetic world must be built about the same fixed point
            # the simulator renders it about; wait for the EKF origin.
            return
        self.get_logger().info(
            f'Loading height map at ({lat:.5f}, {lon:.5f}) in the background ...')
        self._heightmap_thread = threading.Thread(
            target=self._build_heightmap, args=(lat, lon), daemon=True)
        self._heightmap_thread.start()

    def _build_heightmap(self, lat: float, lon: float):
        anchor_lat, anchor_lon = self.synthetic_anchor or (lat, lon)
        try:
            heightmap = HeightMapCache(
                lat, lon,
                self.get_parameter('area_km').value,
                self.get_parameter('dem_resolution_m').value,
                self.get_parameter('cache_dir').value,
                self.synthetic_terrain,
                origin_lat=anchor_lat,
                origin_lon=anchor_lon)
        except Exception as e:
            self.get_logger().error(f'Failed to load height map: {e}')
            return
        self.heightmap = heightmap
        self.get_logger().info('Height map ready.')

    def _maybe_recentre_heightmap(self, heightmap, lat: float, lon: float,
                                  footprint_m: float):
        """Reload the map about the prior before the widest search window
        would cross its edge. Requiring a real move from the map centre
        keeps an undersized map from reloading every frame."""
        if heightmap.edge_distance_m(lat, lon) >= (
                self.max_search_radius_m + footprint_m):
            return
        east, north = heightmap.latlon_to_local(lat, lon)
        if math.hypot(east, north) < 0.25 * heightmap.half_extent_m:
            return
        self._load_heightmap(lat, lon)

    def _on_origin(self, msg: GeoPointStamped):
        origin = (msg.position.latitude, msg.position.longitude,
                  msg.position.altitude)
        if not all(math.isfinite(value) for value in origin):
            return
        if origin != self.origin:
            self.get_logger().info(
                f'EKF origin: {origin[0]:.7f}, {origin[1]:.7f}, '
                f'{origin[2]:.2f} m')
        self.origin = origin
        if self.synthetic_terrain and self.synthetic_anchor is None:
            self.synthetic_anchor = origin[:2]

    def _on_fix(self, msg: NavSatFix):
        self.latest_gps = msg
        self.latest_gps_received = time.monotonic()

    def _on_prior(self, msg: NavSatFix):
        self.latest_prior = msg
        self.latest_prior_received = time.monotonic()

    def _on_range(self, msg: Range):
        self.latest_range = msg
        self.latest_range_received = time.monotonic()

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg
        q = msg.pose.pose.orientation
        angular, linear = msg.twist.twist.angular, msg.twist.twist.linear
        self.attitude.add(
            stamp_ns(msg.header.stamp), (q.x, q.y, q.z, q.w),
            math.sqrt(angular.x ** 2 + angular.y ** 2 + angular.z ** 2),
            math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2))

    def _on_clock(self, msg: Clock):
        # Simulators stamp camera frames in simulator time, MAVROS odometry
        # in wall time.
        self.attitude.add_clock(stamp_ns(msg.clock), time.time_ns())

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _on_depth(self, msg: Image):
        now = time.monotonic()
        prior = select_prior(
            self.latest_prior, self.latest_prior_received,
            self.latest_gps, self.latest_gps_received,
            now, self.prior_timeout_s)
        if prior is None:
            self._publish_valid(False)
            return
        if self.heightmap is None:
            self._load_heightmap(prior.latitude, prior.longitude)
            self._publish_valid(False)
            return
        attitude = self.attitude.at(stamp_ns(msg.header.stamp))
        if attitude is None:
            self.get_logger().warn(
                'No odometry attitude at the depth frame time — skipping',
                throttle_duration_sec=10.0)
            self._publish_valid(False)
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')
            self._publish_valid(False)
            return
        self._run_matching(depth, msg, prior, now, attitude)

    def _intrinsics(self, width: int, height: int):
        """(fx, fy, cx, cy) for a frame of this size: CameraInfo scaled to
        it, else a centred pinhole with fov_deg across the width."""
        info = self.camera_info
        if (info is not None and info.k[0] > 0.0 and info.k[4] > 0.0 and
                info.width > 0 and info.height > 0):
            sx = width / float(info.width)
            sy = height / float(info.height)
            return (info.k[0] * sx, info.k[4] * sy,
                    (info.k[2] + 0.5) * sx - 0.5,
                    (info.k[5] + 0.5) * sy - 0.5)
        fx = width / (2.0 * math.tan(math.radians(self.fov_deg / 2.0)))
        return fx, fx, width / 2.0, height / 2.0

    def _altitude_msl(self, lat: float, lon: float, altitude: float) -> float:
        if self.geoid is None:
            return altitude
        return altitude - self.geoid.height(lat, lon)

    def _rangefinder_agl(self, now: float, attitude):
        """Height above ground from a fresh, in-range rangefinder reading,
        else None. The beam follows the body z axis."""
        msg = self.latest_range
        if (msg is None or
                now - self.latest_range_received > self.rangefinder_timeout_s):
            return None
        reading = float(msg.range)
        if not (math.isfinite(reading) and
                msg.min_range <= reading <= msg.max_range):
            return None
        # Vertical component of the body z axis: cos(roll) * cos(pitch).
        return reading * quaternion_matrix(*attitude.quaternion)[2, 2]

    def _run_matching(self, depth: np.ndarray, source_msg: Image,
                      prior: NavSatFix, now: float, attitude):
        lat = prior.latitude
        lon = prior.longitude
        alt = prior.altitude
        heightmap = self.heightmap

        if not math.isfinite(alt):
            self._publish_valid(False)
            return

        # Steep banks and pitches put a ramp into the depth (range along a
        # tilted view, not height) that swamps the terrain relief the DEM
        # correlation looks for; skip those frames (0 disables).
        roll, pitch = roll_pitch_rad(attitude.quaternion)
        if self.max_attitude_rad > 0.0 and (
                abs(roll) > self.max_attitude_rad or
                abs(pitch) > self.max_attitude_rad):
            self.get_logger().debug(
                'Attitude beyond max_attitude_deg — skipping off-nadir frame')
            self._publish_valid(False)
            return

        img_h, img_w = depth.shape
        raw_depth = depth.astype(np.float32)
        finite = np.isfinite(raw_depth)
        if finite.sum() < raw_depth.size * 0.25:
            self._publish_valid(False)
            return

        # Height above ground sets the template scale: a fresh rangefinder
        # reading, else the altitude above the DEM (SRTM heights are MSL).
        ground_elevation = heightmap.elevation_at(lat, lon)
        if not math.isfinite(ground_elevation):
            self._maybe_recentre_heightmap(heightmap, lat, lon, 0.0)
            self._publish_valid(False)
            return
        alt_msl = self._altitude_msl(lat, lon, alt)
        agl = self._rangefinder_agl(now, attitude)
        if agl is None:
            agl = max(alt_msl - ground_elevation, 1.0)

        # Ground sampling distance from the calibrated focal length.
        fx, fy, cx, cy = self._intrinsics(img_w, img_h)
        cell_m = heightmap.col_spacing_m
        footprint_cells = (img_w * agl / fx / cell_m, img_h * agl / fy / cell_m)
        if min(footprint_cells) < self.min_dem_footprint_pixels:
            self.get_logger().debug(
                'Camera footprint is too small at the configured DEM resolution')
            self._publish_valid(False)
            return
        footprint_m = max(footprint_cells) * cell_m
        self._maybe_recentre_heightmap(heightmap, lat, lon, footprint_m)

        # Project the frame north-up onto the DEM grid with the camera's
        # attitude at the frame time: the template anchor is the point
        # below the vehicle, so the match is the vehicle's position.
        rotation = camera_rotation(
            attitude.quaternion, self.camera_mount_yaw_rad,
            self.camera_stabilized)
        if self.depth_is_metric:
            # Metric depth (simulator depth cameras) puts every pixel on
            # the terrain in 3D: the template is the height below the
            # camera at each point's true ground position, free of the
            # view's tilt and of relief displacement.
            projected = terrain_template(
                raw_depth, finite & (raw_depth > self.min_metric_depth) &
                (raw_depth < self.max_metric_depth),
                rotation, (fx, fy, cx, cy), cell_m)
        else:
            # Relative depth (the OAK-1 network output) has no known scale
            # or form: it goes onto flat ground agl below the camera (the
            # ramp a tilted view adds is left out of the correlation).
            projected = ground_template(
                raw_depth, finite, rotation, (fx, fy, cx, cy), agl, cell_m)
        if projected is None:
            self._publish_valid(False)
            return
        relative_tmpl, valid_tmpl, anchor = projected
        if valid_tmpl.sum() < 9:
            self._publish_valid(False)
            return

        # Normalise (zero mean, unit variance)
        std = float(relative_tmpl[valid_tmpl].std())
        if std < 1e-6:
            self._publish_valid(False)
            return
        depth_tmpl = (relative_tmpl - relative_tmpl[valid_tmpl].mean()) / std

        # DEM window: every anchor position within the search radius,
        # plus the template around it. Windows off the map are rejected.
        radius_m = search_radius(
            prior, self.search_radius_m, self.search_radius_sigma_scale,
            self.max_search_radius_m)
        window_m = 2.0 * (radius_m + max(depth_tmpl.shape) * cell_m)
        dem_patch, dem_res, (center_row, center_col) = heightmap.get_patch(
            lat, lon, window_m, window_m)
        if dem_patch.size == 0:
            self.get_logger().debug('Search window leaves the height map')
            self._publish_valid(False)
            return

        dem = dem_patch.astype(np.float32)
        dem_std = dem.std()
        if dem_std < 1e-6:
            self._publish_valid(False)
            return
        dem = (dem - dem.mean()) / dem_std

        # Relative-depth models may encode either depth or inverse depth;
        # correlate() scores both polarities and also finds the best
        # look-alike outside the peak's neighbourhood (a footprint radius).
        exclusion_cells = 0.5 * max(footprint_cells)
        # A tilted view of flat ground ramps relative depth along the
        # optical axis' heading; any hillside sloping that way would match
        # the ramp. That profile is left out of the template and of every
        # DEM window: only the rest of the relief is matched.
        trend = () if self.depth_is_metric else view_trend(
            rotation, depth_tmpl.shape, QUADRATIC_TILT_RAD)
        match = correlate(
            dem, depth_tmpl.astype(np.float32), valid_tmpl, anchor,
            (center_row, center_col), radius_m / dem_res, exclusion_cells,
            trend)
        if match is None:
            self._publish_valid(False)
            return
        max_val = match['score']
        if max_val < self.min_correlation:
            self.get_logger().debug(f'Low correlation {max_val:.2f} — skipping')
            self._publish_valid(False)
            return
        uniqueness = peak_uniqueness(max_val, match['second'])
        if uniqueness < self.min_peak_uniqueness:
            self.get_logger().debug(
                f'Ambiguous terrain match: peak {max_val:.3f}, look-alike '
                f'{match["second"]:.3f} — skipping')
            self._publish_valid(False)
            return

        # Map the matched point below the vehicle (sub-cell) back through
        # the grid geometry.
        prior_row, prior_col = heightmap.latlon_to_pixel(lat, lon)
        corrected_lat, corrected_lon = heightmap.pixel_to_latlon(
            prior_row + match['position'][0] - center_row,
            prior_col + match['position'][1] - center_col)
        offset_east_m, offset_north_m = latlon_to_local(
            corrected_lat, corrected_lon, lat, lon)

        match_row, match_col = match['loc']
        matched_dem = dem_patch[
            match_row:match_row + depth_tmpl.shape[0],
            match_col:match_col + depth_tmpl.shape[1]]
        target_range = alt_msl - matched_dem
        fit_mask = (
            valid_tmpl &
            np.isfinite(target_range) &
            (target_range > self.min_metric_depth) &
            (target_range < self.max_metric_depth))
        if fit_mask.sum() < 9:
            self._publish_valid(False)
            return

        design = np.column_stack((
            relative_tmpl[fit_mask],
            np.ones(fit_mask.sum(), dtype=np.float32)))
        coefficients, _, _, _ = np.linalg.lstsq(
            design, target_range[fit_mask], rcond=None)
        depth_scale = float(coefficients[0])
        depth_offset = float(coefficients[1])
        fitted = design @ coefficients
        residual = target_range[fit_mask] - fitted
        target_variance = float(np.var(target_range[fit_mask]))
        fit_score = 1.0
        if target_variance > 1e-6:
            fit_score = max(
                0.0, 1.0 - float(np.var(residual)) / target_variance)
        confidence = float(np.clip(
            (0.7 * max_val + 0.3 * fit_score) * uniqueness, 0.0, 1.0))

        metric_depth = raw_depth * depth_scale + depth_offset
        metric_depth[
            (~finite) |
            (metric_depth < self.min_metric_depth) |
            (metric_depth > self.max_metric_depth)] = np.nan

        # Matching error from the score and sharpness of the peak (at most
        # the look-alike exclusion radius) and from a heading error, which
        # turns the template about the point below the vehicle; plus the
        # attitude and timing errors of the projection (they move that
        # point by about height x angle).
        located_m = match_sigma(
            max_val, match['curvature'], int(valid_tmpl.sum()), dem_res,
            self.match_sigma_scale, self.match_sigma_floor_cells,
            exclusion_cells * dem_res)
        if self.heading_sigma_rad > 0.0:
            turning = heading_sensitivity(
                dem, depth_tmpl.astype(np.float32), valid_tmpl, anchor,
                match['position'], trend)
            located_m = min(math.hypot(located_m, (
                turning if turning is not None else float('inf')) *
                dem_res * self.heading_sigma_rad), exclusion_cells * dem_res)
        if not self.depth_is_metric:
            # Projected onto flat ground, relief dh seen at view angle a
            # lands dh x tan(a) off: the template shifts by up to the
            # relief at the edge of the view (half the field of view off
            # the optical axis, plus the camera's tilt).
            tilt = math.acos(min(1.0, max(-1.0, -rotation[2, 2])))
            view = min(math.atan(0.5 * max(img_w / fx, img_h / fy)) + tilt,
                       MAX_VIEW_RAD)
            located_m = math.hypot(located_m, float(
                np.std(matched_dem[fit_mask])) * math.tan(view))
        # A stabilised camera stays level whatever the body's rates (and
        # turning about the vertical does not move the point below it).
        horizontal_variance = horizontal_sigma(
            located_m, agl, self.attitude_sigma_rad,
            0.0 if self.camera_stabilized else attitude.rate,
            attitude.speed, self.stamp_sigma_s, attitude.offset_s) ** 2
        vertical_sigma = max(dem_res, float(np.std(residual)))
        vertical_variance = vertical_sigma ** 2

        # The fix keeps the prior's altitude (and datum); it does not
        # depend on the EKF origin, so it is published regardless.
        fix = NavSatFix()
        fix.header.stamp      = source_msg.header.stamp
        fix.header.frame_id   = 'map'
        fix.status.status     = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service    = NavSatStatus.SERVICE_GPS
        fix.latitude          = corrected_lat
        fix.longitude         = corrected_lon
        fix.altitude          = alt
        fix.position_covariance = [
            horizontal_variance, 0.0, 0.0,
            0.0, horizontal_variance, 0.0,
            0.0, 0.0, vertical_variance]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_publisher.publish(fix)

        metric_msg = self.bridge.cv2_to_imgmsg(metric_depth, encoding='32FC1')
        metric_msg.header = source_msg.header
        self.confidence_publisher.publish(Float32(data=confidence))
        self.scale_publisher.publish(Float32(data=depth_scale))
        self.metric_depth_publisher.publish(metric_msg)
        self.pointcloud_publisher.publish(
            self._make_pointcloud(metric_depth, source_msg.header))

        self.get_logger().debug(
            f'confidence={confidence:.2f} uniqueness={uniqueness:.2f} '
            f'scale={depth_scale:.3f} '
            f'offset=({offset_east_m:.1f}m E, {offset_north_m:.1f}m N)')

        if self.origin is None:
            # Local corrections need the EKF origin.
            self._publish_valid(False)
            return
        origin_lat, origin_lon, origin_alt = self.origin
        east, north = latlon_to_local(
            corrected_lat, corrected_lon, origin_lat, origin_lon)
        odom = Odometry()
        odom.header = fix.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = alt - origin_alt
        if self.latest_odom is not None:
            odom.pose.pose.orientation = self.latest_odom.pose.pose.orientation
            odom.twist = self.latest_odom.twist
        else:
            odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[0] = horizontal_variance
        odom.pose.covariance[7] = horizontal_variance
        odom.pose.covariance[14] = vertical_variance
        odom.pose.covariance[21] = 0.1
        odom.pose.covariance[28] = 0.1
        odom.pose.covariance[35] = 0.2
        pose = PoseWithCovarianceStamped()
        pose.header = odom.header
        pose.pose = odom.pose

        self._publish_valid(True)
        self.odom_publisher.publish(odom)
        self.pose_publisher.publish(pose)

    def _make_pointcloud(self, metric_depth, header):
        height, width = metric_depth.shape
        fx, fy, cx, cy = self._intrinsics(width, height)

        rows, cols = np.mgrid[
            0:height:self.pointcloud_stride,
            0:width:self.pointcloud_stride]
        z = metric_depth[
            0:height:self.pointcloud_stride,
            0:width:self.pointcloud_stride]
        valid = np.isfinite(z)
        z = z[valid]
        x = ((cols[valid] - cx) / fx * z).astype(np.float32)
        y = ((rows[valid] - cy) / fy * z).astype(np.float32)
        points = np.column_stack((x, y, z.astype(np.float32)))

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        cloud.data = points.astype(np.float32, copy=False).tobytes()
        return cloud

    def _publish_valid(self, valid):
        self.valid_publisher.publish(Bool(data=valid))


def main(args=None):
    rclpy.init(args=args)
    node = DemNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
