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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, Int32

from .satellite_cache import SatelliteTileCache
from .satellite_cache import ground_resolution_m, tile_pixel_to_latlon


METERS_PER_DEG_LAT = 111320.0


def local_to_latlon(east, north, origin_lat, origin_lon):
    lat = origin_lat + north / METERS_PER_DEG_LAT
    metres_per_deg_lon = (
        METERS_PER_DEG_LAT * math.cos(math.radians(origin_lat)))
    lon = origin_lon + east / metres_per_deg_lon
    return lat, lon


def horizontal_distance_m(lat_a, lon_a, lat_b, lon_b):
    mean_lat = math.radians((lat_a + lat_b) * 0.5)
    east = (lon_b - lon_a) * METERS_PER_DEG_LAT * math.cos(mean_lat)
    north = (lat_b - lat_a) * METERS_PER_DEG_LAT
    return math.hypot(east, north)


class WildNavNode(Node):
    def __init__(self):
        super().__init__('wildnav_node')

        defaults = {
            'image_topic': '/oak1/image_highres',
            'camera_info_topic': '/oak1/image_highres/camera_info',
            'odom_topic': '/navigation/odometry',
            'gps_topic': '/drone0/sensor_measurements/gps',
            'fix_topic': '/wildnav/fix',
            'odometry_topic': '/wildnav/odometry',
            'confidence_topic': '/wildnav/confidence',
            'match_count_topic': '/wildnav/match_count',
            'valid_topic': '/wildnav/valid',
            'initial_lat': 0.0,
            'initial_lon': 0.0,
            'origin_alt': 0.0,
            'cache_dir': '/data/wildnav_cache',
            'tile_url_template': (
                'https://server.arcgisonline.com/ArcGIS/rest/services/'
                'World_Imagery/MapServer/tile/{z}/{y}/{x}'),
            'zoom': 18,
            'search_radius_m': 250.0,
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
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.origin_lat = float(self.get_parameter('initial_lat').value)
        self.origin_lon = float(self.get_parameter('initial_lon').value)
        self.origin_alt = float(self.get_parameter('origin_alt').value)
        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_odom = None
        self.latest_image = None
        self.latest_image_header = None
        self.latest_image_odom = None
        self.latest_image_received = 0.0
        self.busy = threading.Lock()

        self.cache = SatelliteTileCache(
            self.get_parameter('cache_dir').value,
            self.get_parameter('tile_url_template').value,
            int(self.get_parameter('zoom').value),
            self.get_parameter('feature_backend').value,
            int(self.get_parameter('max_features').value),
            float(self.get_parameter('request_timeout_s').value),
            self.origin_lat,
            self.origin_lon)

        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, self.get_parameter('gps_topic').value,
            self._on_gps, qos_profile_sensor_data)

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
            self.latest_image_received = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f'RGB conversion failed: {exc}')

    def _on_camera_info(self, msg):
        self.camera_info = msg

    def _on_odom(self, msg):
        self.latest_odom = msg

    def _on_gps(self, msg):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if self.origin_lat == 0.0 and self.origin_lon == 0.0:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            self.cache.set_origin(self.origin_lat, self.origin_lon)
            self.get_logger().info(
                f'WildNav origin set from GPS: '
                f'{self.origin_lat:.7f}, {self.origin_lon:.7f}')

    def _match_latest(self):
        if not self.busy.acquire(blocking=False):
            return
        started = False
        try:
            if (self.latest_image is None or self.latest_image_odom is None or
                    (self.origin_lat == 0.0 and self.origin_lon == 0.0)):
                self._publish_invalid(0, 0.0)
                return
            if time.monotonic() - self.latest_image_received > float(
                    self.get_parameter('max_image_age_s').value):
                self._publish_invalid(0, 0.0)
                return
            worker = threading.Thread(
                target=self._match_worker,
                args=(self.latest_image.copy(), self.latest_image_header,
                      self.latest_image_odom, self.camera_info),
                daemon=True)
            worker.start()
            started = True
        finally:
            if not started:
                self.busy.release()

    def _match_worker(self, image, header, image_odom, camera_info):
        try:
            self._run_match(image, header, image_odom, camera_info)
        except Exception as exc:
            self.get_logger().warning(f'WildNav match failed: {exc}')
            self._publish_invalid(0, 0.0)
        finally:
            self.busy.release()

    def _run_match(self, image, header, image_odom, camera_info):
        if camera_info is not None and camera_info.k[0] > 0.0:
            matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(camera_info.d, dtype=np.float64)
            image = cv2.undistort(image, matrix, distortion)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        max_dimension = max(gray.shape)
        if max_dimension > 800:
            scale = 800.0 / max_dimension
            gray = cv2.resize(
                gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)

        query_points, query_descriptors = self.cache.describe(gray)
        if query_descriptors is None:
            self._publish_invalid(0, 0.0)
            return

        east = image_odom.pose.pose.position.x
        north = image_odom.pose.pose.position.y
        predicted_lat, predicted_lon = local_to_latlon(
            east, north, self.origin_lat, self.origin_lon)
        candidates = self.cache.candidate_tiles(
            predicted_lat, predicted_lon,
            float(self.get_parameter('search_radius_m').value),
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
                    'pixel': tile_center,
                }

        if best is None:
            self._publish_invalid(0, 0.0)
            return

        matched_lat, matched_lon = tile_pixel_to_latlon(
            best['tile_x'], best['tile_y'],
            best['pixel'][0], best['pixel'][1], self.cache.zoom)
        displacement = horizontal_distance_m(
            predicted_lat, predicted_lon, matched_lat, matched_lon)
        if displacement > float(
                self.get_parameter('max_position_jump_m').value):
            self.get_logger().warning(
                f'Rejected WildNav jump of {displacement:.1f} m')
            self._publish_invalid(best['inliers'], best['confidence'])
            return

        resolution = ground_resolution_m(matched_lat, self.cache.zoom)
        sigma = max(3.0, best['reprojection'] * resolution)
        variance = sigma * sigma
        confidence = float(np.clip(best['confidence'], 0.0, 1.0))

        fix = NavSatFix()
        fix.header = copy.deepcopy(header)
        fix.header.frame_id = 'map'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = matched_lat
        fix.longitude = matched_lon
        fix.altitude = self.origin_alt + image_odom.pose.pose.position.z
        fix.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, image_odom.pose.covariance[14],
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        metres_per_deg_lon = (
            METERS_PER_DEG_LAT * math.cos(math.radians(self.origin_lat)))
        matched_east = (matched_lon - self.origin_lon) * metres_per_deg_lon
        matched_north = (matched_lat - self.origin_lat) * METERS_PER_DEG_LAT
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
        self.confidence_pub.publish(Float32(data=confidence))
        self.match_count_pub.publish(Int32(data=best['inliers']))
        self.fix_pub.publish(fix)
        self.odom_pub.publish(odom)
        self.get_logger().info(
            f'WildNav fix {displacement:.1f} m from prediction, '
            f'{best["inliers"]} inliers, confidence={confidence:.2f}')

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
