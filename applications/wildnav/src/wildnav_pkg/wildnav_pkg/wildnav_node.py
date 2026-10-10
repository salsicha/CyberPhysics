#!/usr/bin/env python3
import copy
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, Int32

from synthetic_world import METERS_PER_DEG_LAT, latlon_to_local

from .camera_geometry import (
    AttitudeHistory,
    camera_rotation,
    horizontal_sigma,
    nadir_pixel,
    roll_pitch_rad,
    stamp_ns,
)
from .satellite_cache import SatelliteTileCache
from .satellite_cache import ground_resolution_m, tile_pixel_to_latlon


def _flag(value):
    """A boolean parameter that launch files may pass as a string."""
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def gray_intrinsics(camera_info, width, height, scale, fov_deg):
    """(fx, fy, cx, cy) of the matched (undistorted, scaled) image: the
    calibration scaled to it, else a centred pinhole with fov_deg across
    the width."""
    if (camera_info is not None and camera_info.k[0] > 0.0 and
            camera_info.k[4] > 0.0):
        k = camera_info.k
        sx = sy = scale
        if camera_info.width > 0 and camera_info.height > 0:
            sx = width / float(camera_info.width)
            sy = height / float(camera_info.height)
        return (k[0] * sx, k[4] * sy, (k[2] + 0.5) * sx - 0.5,
                (k[5] + 0.5) * sy - 0.5)
    focal = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    return focal, focal, width / 2.0, height / 2.0


def height_above_ground(homography, pixel, rotation, intrinsics,
                        metres_per_tile_px):
    """Height of the camera above the ground point at pixel (the point
    below the vehicle), from the matched image-to-tile scale there: a
    vertical ray at h metres samples h * cos(a)^1.5 / f metres per pixel
    (geometric mean), a being the ray's angle off the optical axis."""
    u, v = pixel
    h = homography
    w = h[2, 0] * u + h[2, 1] * v + h[2, 2]
    x = (h[0, 0] * u + h[0, 1] * v + h[0, 2]) / w
    y = (h[1, 0] * u + h[1, 1] * v + h[1, 2]) / w
    jacobian = np.array([
        [h[0, 0] - x * h[2, 0], h[0, 1] - x * h[2, 1]],
        [h[1, 0] - y * h[2, 0], h[1, 1] - y * h[2, 1]]]) / w
    metres_per_px = math.sqrt(abs(np.linalg.det(jacobian))) * (
        metres_per_tile_px)
    cos_off_axis = float((rotation.T @ np.array([0.0, 0.0, -1.0]))[2])
    focal = math.sqrt(intrinsics[0] * intrinsics[1])
    return focal * metres_per_px / max(cos_off_axis, 1e-3) ** 1.5


def horizontal_distance_m(lat_a, lon_a, lat_b, lon_b):
    mean_lat = math.radians((lat_a + lat_b) * 0.5)
    east = (lon_b - lon_a) * METERS_PER_DEG_LAT * math.cos(mean_lat)
    north = (lat_b - lat_a) * METERS_PER_DEG_LAT
    return math.hypot(east, north)


def fix_is_usable(fix, received, now, timeout_s):
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


def search_radius(fix, base_m, sigma_scale, maximum_m):
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


class WildNavNode(Node):
    def __init__(self):
        super().__init__('wildnav_node')

        defaults = {
            'image_topic': '/oak1/image_highres',
            'camera_info_topic': '/oak1/image_highres/camera_info',
            'odom_topic': '/navigation/odometry',
            'gps_topic': '/drone0/sensor_measurements/gps',
            'prior_fix_topic': '/navigation/fix',
            'origin_topic': '/navigation/origin',
            'fix_topic': '/wildnav/fix',
            'odometry_topic': '/wildnav/odometry',
            'confidence_topic': '/wildnav/confidence',
            'match_count_topic': '/wildnav/match_count',
            'valid_topic': '/wildnav/valid',
            'initial_lat': 0.0,
            'initial_lon': 0.0,
            'cache_dir': '/data/wildnav_cache',
            'tile_url_template': (
                'https://server.arcgisonline.com/ArcGIS/rest/services/'
                'World_Imagery/MapServer/tile/{z}/{y}/{x}'),
            'zoom': 18,
            'search_radius_m': 250.0,
            'search_radius_sigma_scale': 3.0,
            'max_search_radius_m': 750.0,
            'prior_timeout_s': 2.0,
            'max_tiles': 49,
            'max_tile_downloads_per_cycle': 8,
            'match_rate_hz': 0.2,
            'feature_backend': 'SIFT',
            'max_features': 1500,
            'ratio_threshold': 0.72,
            'min_matches': 16,
            'min_inliers': 10,
            'min_inlier_ratio': 0.25,
            'max_reprojection_error_px': 8.0,
            'max_position_jump_m': 250.0,
            'max_image_age_s': 10.0,
            'request_timeout_s': 5.0,
            'max_attitude_deg': 20.0,
            # Camera extrinsics (camera_geometry.py): image-top heading from
            # the nose (counter-clockwise, degrees) and whether the camera is
            # roll/pitch-stabilised instead of body-fixed.
            'camera_mount_yaw_deg': 0.0,
            'camera_stabilized': False,
            # Horizontal field of view when no camera_info is received.
            'fov_deg': 69.0,
            # Fix covariance: attitude (EKF and camera mount) and camera
            # stamp uncertainty.
            'attitude_sigma_deg': 1.0,
            'stamp_sigma_s': 0.05,
            'max_attitude_offset_s': 0.25,
            'sim_clock_topic': '/clock',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        # Local outputs are anchored at the autopilot's EKF origin, received
        # on origin_topic; nothing local is published before it arrives.
        self.origin = None
        # initial_lat/initial_lon are only the fixed anchor of the synthetic
        # world (the simulator renders it about the same point), which
        # falls back to the EKF origin when unset.
        initial_lat = float(self.get_parameter('initial_lat').value)
        initial_lon = float(self.get_parameter('initial_lon').value)
        self.synthetic_anchor = (
            (initial_lat, initial_lon)
            if initial_lat != 0.0 or initial_lon != 0.0 else None)
        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_odom = None
        # Vehicle attitude by odometry stamp, looked up at each image stamp.
        self.attitude = AttitudeHistory(max_offset_s=float(
            self.get_parameter('max_attitude_offset_s').value))
        self.camera_mount_yaw_rad = math.radians(
            float(self.get_parameter('camera_mount_yaw_deg').value))
        self.camera_stabilized = _flag(
            self.get_parameter('camera_stabilized').value)
        self.latest_gps = None
        self.latest_gps_received = 0.0
        self.latest_prior = None
        self.latest_prior_received = 0.0
        self.latest_image = None
        self.latest_image_header = None
        self.latest_image_odom = None
        self.latest_image_prior = None
        self.latest_image_received = 0.0
        self.busy = threading.Lock()

        self.cache = SatelliteTileCache(
            self.get_parameter('cache_dir').value,
            self.get_parameter('tile_url_template').value,
            int(self.get_parameter('zoom').value),
            self.get_parameter('feature_backend').value,
            int(self.get_parameter('max_features').value),
            float(self.get_parameter('request_timeout_s').value),
            initial_lat,
            initial_lon)

        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, qos_profile_sensor_data)
        clock_topic = str(self.get_parameter('sim_clock_topic').value or '')
        if clock_topic:
            self.create_subscription(
                Clock, clock_topic, self._on_clock, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, self.get_parameter('gps_topic').value,
            self._on_gps, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, self.get_parameter('prior_fix_topic').value,
            self._on_prior, qos_profile_sensor_data)
        self.create_subscription(
            GeoPointStamped, self.get_parameter('origin_topic').value,
            self._on_origin,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.fix_pub = self.create_publisher(
            NavSatFix, self.get_parameter('fix_topic').value, 10)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odometry_topic').value, 10)
        self.confidence_pub = self.create_publisher(
            Float32, self.get_parameter('confidence_topic').value, 10)
        self.match_count_pub = self.create_publisher(
            Int32, self.get_parameter('match_count_topic').value, 10)
        self.valid_pub = self.create_publisher(
            Bool, self.get_parameter('valid_topic').value, 10)

        rate = max(0.02, float(self.get_parameter('match_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self._match_latest)
        self.get_logger().info(
            f'WildNav ready at {rate:.2f} Hz using '
            f'{self.cache.feature_backend} and zoom {self.cache.zoom}')

    def _on_image(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            self.latest_image_header = msg.header
            self.latest_image_odom = self.latest_odom
            # The search prior is taken when the image is, not when the
            # slow matcher gets to it.
            now = time.monotonic()
            self.latest_image_prior = select_prior(
                self.latest_prior, self.latest_prior_received,
                self.latest_gps, self.latest_gps_received, now,
                float(self.get_parameter('prior_timeout_s').value))
            self.latest_image_received = now
        except Exception as exc:
            self.get_logger().warning(f'RGB conversion failed: {exc}')

    def _on_camera_info(self, msg):
        self.camera_info = msg

    def _on_odom(self, msg):
        self.latest_odom = msg
        q = msg.pose.pose.orientation
        angular, linear = msg.twist.twist.angular, msg.twist.twist.linear
        self.attitude.add(
            stamp_ns(msg.header.stamp), (q.x, q.y, q.z, q.w),
            math.sqrt(angular.x ** 2 + angular.y ** 2 + angular.z ** 2),
            math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2))

    def _on_clock(self, msg):
        # Simulators stamp camera frames in simulator time, MAVROS odometry
        # in wall time.
        self.attitude.add_clock(stamp_ns(msg.clock), time.time_ns())

    def _on_gps(self, msg):
        self.latest_gps = msg
        self.latest_gps_received = time.monotonic()

    def _on_prior(self, msg):
        self.latest_prior = msg
        self.latest_prior_received = time.monotonic()

    def _on_origin(self, msg):
        origin = (msg.position.latitude, msg.position.longitude,
                  msg.position.altitude)
        if not all(math.isfinite(value) for value in origin):
            return
        if origin != self.origin:
            self.get_logger().info(
                f'EKF origin: {origin[0]:.7f}, {origin[1]:.7f}, '
                f'{origin[2]:.2f} m')
        self.origin = origin
        if self.synthetic_anchor is None:
            self.synthetic_anchor = origin[:2]
            self.cache.set_origin(*self.synthetic_anchor)

    def _match_latest(self):
        if not self.busy.acquire(blocking=False):
            return
        started = False
        try:
            if (self.latest_image is None or self.latest_image_odom is None or
                    self.latest_image_prior is None or
                    (self.cache.synthetic and self.synthetic_anchor is None)):
                self._publish_invalid(0, 0.0)
                return
            if time.monotonic() - self.latest_image_received > float(
                    self.get_parameter('max_image_age_s').value):
                self._publish_invalid(0, 0.0)
                return
            worker = threading.Thread(
                target=self._match_worker,
                args=(self.latest_image.copy(), self.latest_image_header,
                      self.latest_image_odom, self.latest_image_prior,
                      self.camera_info),
                daemon=True)
            worker.start()
            started = True
        finally:
            if not started:
                self.busy.release()

    def _match_worker(self, image, header, image_odom, prior, camera_info):
        try:
            self._run_match(image, header, image_odom, prior, camera_info)
        except Exception as exc:
            self.get_logger().warning(f'WildNav match failed: {exc}')
            self._publish_invalid(0, 0.0)
        finally:
            self.busy.release()

    def _run_match(self, image, header, image_odom, prior, camera_info):
        # The camera pose at the image's own timestamp.
        attitude = self.attitude.at(stamp_ns(header.stamp))
        if attitude is None:
            self.get_logger().warning(
                'No odometry attitude at the image time — skipping',
                throttle_duration_sec=10.0)
            self._publish_invalid(0, 0.0)
            return
        # A steeply banked or pitched view is too oblique for the tile
        # match; skip those frames (0 disables).
        max_attitude = math.radians(
            float(self.get_parameter('max_attitude_deg').value))
        if max_attitude > 0.0:
            roll, pitch = roll_pitch_rad(attitude.quaternion)
            if abs(roll) > max_attitude or abs(pitch) > max_attitude:
                self.get_logger().debug(
                    'Attitude beyond max_attitude_deg — '
                    'skipping off-nadir frame')
                self._publish_invalid(0, 0.0)
                return
        if camera_info is not None and camera_info.k[0] > 0.0:
            matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(camera_info.d, dtype=np.float64)
            image = cv2.undistort(image, matrix, distortion)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        max_dimension = max(gray.shape)
        scale = 1.0
        if max_dimension > 800:
            scale = 800.0 / max_dimension
            gray = cv2.resize(
                gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)

        # The pixel looking straight down: the homography takes it to the
        # point below the vehicle (yaw and height are not needed for that).
        intrinsics = gray_intrinsics(
            camera_info, gray.shape[1], gray.shape[0], scale,
            float(self.get_parameter('fov_deg').value))
        rotation = camera_rotation(
            attitude.quaternion, self.camera_mount_yaw_rad,
            self.camera_stabilized)
        nadir = nadir_pixel(rotation, intrinsics)
        if nadir is None:
            self._publish_invalid(0, 0.0)
            return

        query_points, query_descriptors = self.cache.describe(gray)
        if query_descriptors is None:
            self._publish_invalid(0, 0.0)
            return

        # Search about the prior, widened by its uncertainty.
        predicted_lat, predicted_lon = prior.latitude, prior.longitude
        base_radius = float(self.get_parameter('search_radius_m').value)
        radius = search_radius(
            prior, base_radius,
            float(self.get_parameter('search_radius_sigma_scale').value),
            float(self.get_parameter('max_search_radius_m').value))
        candidates = self.cache.candidate_tiles(
            predicted_lat, predicted_lon, radius,
            int(self.get_parameter('max_tiles').value))

        matcher = self.cache.matcher()
        best = None
        downloads = 0
        maximum_downloads = int(
            self.get_parameter('max_tile_downloads_per_cycle').value)
        ratio_threshold = float(self.get_parameter('ratio_threshold').value)
        for tile_x, tile_y in candidates:
            cached = self.cache.is_cached(tile_x, tile_y)
            allow_download = cached or downloads < maximum_downloads
            if not cached and allow_download:
                downloads += 1
            features = self.cache.get_features(
                tile_x, tile_y, allow_download=allow_download)
            if features is None:
                continue
            tile_points, tile_descriptors, tile_shape = features
            pairs = matcher.knnMatch(query_descriptors, tile_descriptors, k=2)
            good = [
                pair[0] for pair in pairs
                if len(pair) == 2 and
                pair[0].distance < ratio_threshold * pair[1].distance
            ]
            if len(good) < int(self.get_parameter('min_matches').value):
                continue
            source = np.float32(
                [query_points[match.queryIdx] for match in good])
            target = np.float32(
                [tile_points[match.trainIdx] for match in good])
            homography, mask = cv2.findHomography(
                source, target, cv2.RANSAC,
                float(self.get_parameter('max_reprojection_error_px').value))
            if homography is None or mask is None:
                continue
            inlier_mask = mask.ravel().astype(bool)
            inliers = int(inlier_mask.sum())
            inlier_ratio = inliers / len(good)
            if inliers < int(self.get_parameter('min_inliers').value):
                continue
            if inlier_ratio < float(
                    self.get_parameter('min_inlier_ratio').value):
                continue

            projected = cv2.perspectiveTransform(
                source[inlier_mask].reshape(-1, 1, 2), homography
            ).reshape(-1, 2)
            reprojection = float(np.median(np.linalg.norm(
                projected - target[inlier_mask], axis=1)))
            center = np.asarray(
                [[[gray.shape[1] * 0.5, gray.shape[0] * 0.5]]],
                dtype=np.float32)
            tile_center = cv2.perspectiveTransform(center, homography)[0, 0]
            if not (
                    0.0 <= tile_center[0] < tile_shape[1] and
                    0.0 <= tile_center[1] < tile_shape[0]):
                continue
            confidence = (
                min(1.0, inliers / 40.0) *
                inlier_ratio *
                math.exp(-reprojection / 8.0))
            score = confidence * math.log1p(inliers)
            if best is None or score > best['score']:
                best = {
                    'score': score,
                    'confidence': confidence,
                    'inliers': inliers,
                    'reprojection': reprojection,
                    'tile_x': tile_x,
                    'tile_y': tile_y,
                    'homography': homography,
                }

        if best is None:
            self._publish_invalid(0, 0.0)
            return

        # The vehicle's position: where the matched geometry puts the
        # nadir pixel (not the image centre, which a tilted camera points
        # away from the vehicle).
        vehicle_pixel = cv2.perspectiveTransform(
            np.asarray([[nadir]], dtype=np.float64),
            best['homography'])[0, 0]
        if not np.all(np.isfinite(vehicle_pixel)):
            self._publish_invalid(best['inliers'], best['confidence'])
            return
        matched_lat, matched_lon = tile_pixel_to_latlon(
            best['tile_x'], best['tile_y'],
            vehicle_pixel[0], vehicle_pixel[1], self.cache.zoom)
        # The jump gate is measured from the same prior the search used and
        # widens with it.
        displacement = horizontal_distance_m(
            predicted_lat, predicted_lon, matched_lat, matched_lon)
        if displacement > float(
                self.get_parameter('max_position_jump_m').value) + (
                    radius - base_radius):
            self.get_logger().warning(
                f'Rejected WildNav jump of {displacement:.1f} m')
            self._publish_invalid(best['inliers'], best['confidence'])
            return

        # Matching error, plus the attitude and timing errors, which move
        # the nadir pixel by about height x angle.
        resolution = ground_resolution_m(matched_lat, self.cache.zoom)
        height = height_above_ground(
            best['homography'], nadir, rotation, intrinsics, resolution)
        if not (math.isfinite(height) and height > 0.0):
            self._publish_invalid(best['inliers'], best['confidence'])
            return
        sigma = horizontal_sigma(
            max(3.0, best['reprojection'] * resolution), height,
            math.radians(float(
                self.get_parameter('attitude_sigma_deg').value)),
            attitude.rate, attitude.speed,
            float(self.get_parameter('stamp_sigma_s').value),
            attitude.offset_s)
        variance = sigma * sigma
        vertical_variance = prior.position_covariance[8]
        if (prior.position_covariance_type ==
                NavSatFix.COVARIANCE_TYPE_UNKNOWN or
                not math.isfinite(vertical_variance) or
                vertical_variance <= 0.0):
            vertical_variance = image_odom.pose.covariance[14]
        confidence = float(np.clip(best['confidence'], 0.0, 1.0))

        fix = NavSatFix()
        fix.header = copy.deepcopy(header)
        fix.header.frame_id = 'map'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = matched_lat
        fix.longitude = matched_lon
        # The fix keeps the prior's altitude (and datum); it does not depend
        # on the EKF origin, so it is published regardless.
        fix.altitude = prior.altitude
        fix.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, vertical_variance,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.confidence_pub.publish(Float32(data=confidence))
        self.match_count_pub.publish(Int32(data=best['inliers']))
        self.fix_pub.publish(fix)
        self.get_logger().info(
            f'WildNav fix {displacement:.1f} m from prediction, '
            f'{best["inliers"]} inliers, confidence={confidence:.2f}')
        if self.origin is None:
            # Local corrections need the EKF origin.
            self.valid_pub.publish(Bool(data=False))
            return

        matched_east, matched_north = latlon_to_local(
            matched_lat, matched_lon, self.origin[0], self.origin[1])
        odom = Odometry()
        odom.header = copy.deepcopy(header)
        odom.header.frame_id = 'odom'
        odom.child_frame_id = image_odom.child_frame_id or 'base_link'
        odom.pose = copy.deepcopy(image_odom.pose)
        odom.twist = copy.deepcopy(image_odom.twist)
        odom.pose.pose.position.x = matched_east
        odom.pose.pose.position.y = matched_north
        odom.pose.covariance[0] = variance
        odom.pose.covariance[7] = variance

        self.valid_pub.publish(Bool(data=True))
        self.odom_pub.publish(odom)

    def _publish_invalid(self, matches, confidence):
        self.valid_pub.publish(Bool(data=False))
        self.confidence_pub.publish(Float32(data=float(confidence)))
        self.match_count_pub.publish(Int32(data=int(matches)))


def main(args=None):
    rclpy.init(args=args)
    node = WildNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
