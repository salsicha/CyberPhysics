#!/usr/bin/env python3
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from .heightmap import HeightMapCache

METERS_PER_DEG_LAT = 111320.0


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapNavNode(Node):
    def __init__(self):
        super().__init__('mapnav_node')

        self.declare_parameter('gps_topic',        '/fix')
        self.declare_parameter('depth_topic',      '/depth_anything/depth')
        self.declare_parameter('odom_topic',       '/odom')
        self.declare_parameter('output_topic',     '/mapnav/fix')
        self.declare_parameter('initial_lat',      0.0)
        self.declare_parameter('initial_lon',      0.0)
        self.declare_parameter('area_km',          10.0)
        self.declare_parameter('search_radius_m',  200.0)
        self.declare_parameter('fov_deg',          90.0)
        self.declare_parameter('dem_resolution_m', 30.0)
        self.declare_parameter('min_correlation',  0.3)
        self.declare_parameter('cache_dir',        '/data/mapnav_cache')

        gps_topic      = self.get_parameter('gps_topic').value
        depth_topic    = self.get_parameter('depth_topic').value
        odom_topic     = self.get_parameter('odom_topic').value
        output_topic   = self.get_parameter('output_topic').value
        initial_lat    = self.get_parameter('initial_lat').value
        initial_lon    = self.get_parameter('initial_lon').value

        self.fov_deg          = self.get_parameter('fov_deg').value
        self.search_radius_m  = self.get_parameter('search_radius_m').value
        self.min_correlation  = self.get_parameter('min_correlation').value

        self.bridge       = CvBridge()
        self.latest_fix   = None
        self.yaw_rad      = 0.0
        self.heightmap    = None

        if initial_lat != 0.0 or initial_lon != 0.0:
            self.get_logger().info(
                f'Pre-loading height map at ({initial_lat:.5f}, {initial_lon:.5f}) ...')
            self._load_heightmap(initial_lat, initial_lon)
        else:
            self.get_logger().warn(
                'initial_lat/initial_lon not set — height map loads on first GPS fix')

        self.create_subscription(NavSatFix, gps_topic,   self._on_fix,   10)
        self.create_subscription(Image,     depth_topic, self._on_depth, 10)
        self.create_subscription(Odometry,  odom_topic,  self._on_odom,  10)
        self.publisher = self.create_publisher(NavSatFix, output_topic, 10)
        self.get_logger().info(
            f'MapNav ready | gps={gps_topic} depth={depth_topic} out={output_topic}')

    def _load_heightmap(self, lat: float, lon: float):
        try:
            self.heightmap = HeightMapCache(
                lat, lon,
                self.get_parameter('area_km').value,
                self.get_parameter('dem_resolution_m').value,
                self.get_parameter('cache_dir').value)
            self.get_logger().info('Height map ready.')
        except Exception as e:
            self.get_logger().error(f'Failed to load height map: {e}')

    def _on_fix(self, msg: NavSatFix):
        self.latest_fix = msg
        if self.heightmap is None:
            self.get_logger().info(
                f'First GPS fix — loading height map at '
                f'({msg.latitude:.5f}, {msg.longitude:.5f})')
            self._load_heightmap(msg.latitude, msg.longitude)

    def _on_odom(self, msg: Odometry):
        self.yaw_rad = _yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_depth(self, msg: Image):
        if self.latest_fix is None or self.heightmap is None:
            return
        if self.latest_fix.status.status < NavSatStatus.STATUS_FIX:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')
            return
        self._run_matching(depth)

    def _run_matching(self, depth: np.ndarray):
        lat = self.latest_fix.latitude
        lon = self.latest_fix.longitude
        alt = self.latest_fix.altitude

        if alt <= 0.0:
            return

        img_h, img_w = depth.shape

        # Rotate depth image to align north using vehicle yaw
        angle_deg = math.degrees(self.yaw_rad)
        M = cv2.getRotationMatrix2D((img_w / 2.0, img_h / 2.0), -angle_deg, 1.0)
        depth_aligned = cv2.warpAffine(depth, M, (img_w, img_h))

        # Normalise (zero mean, unit variance)
        d = depth_aligned.astype(np.float32)
        std = d.std()
        if std < 1e-6:
            return
        d = (d - d.mean()) / std

        # Ground sampling distance → resample depth to DEM pixel scale
        ground_width_m = 2.0 * alt * math.tan(math.radians(self.fov_deg / 2.0))
        gsd = ground_width_m / img_w
        scale = gsd / self.heightmap.resolution_m
        new_w = max(3, int(img_w * scale))
        new_h = max(3, int(img_h * scale))
        depth_tmpl = cv2.resize(d, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Get DEM search patch
        search_m = self.search_radius_m * 2.0
        dem_patch, dem_res = self.heightmap.get_patch(lat, lon, search_m, search_m)

        if (dem_patch.shape[0] < depth_tmpl.shape[0] + 2 or
                dem_patch.shape[1] < depth_tmpl.shape[1] + 2):
            self.get_logger().warn(
                'DEM patch too small for matching — increase area_km or reduce search_radius_m')
            return

        dem = dem_patch.astype(np.float32)
        dem_std = dem.std()
        if dem_std < 1e-6:
            return
        dem = (dem - dem.mean()) / dem_std

        # Normalised cross-correlation template match
        result = cv2.matchTemplate(dem, depth_tmpl, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(result)

        if max_val < self.min_correlation:
            self.get_logger().debug(f'Low correlation {max_val:.2f} — skipping')
            return

        # Centre of the best-match window relative to DEM patch centre
        match_col = max_loc[0] + depth_tmpl.shape[1] // 2
        match_row = max_loc[1] + depth_tmpl.shape[0] // 2
        offset_col = match_col - dem_patch.shape[1] // 2
        offset_row = match_row - dem_patch.shape[0] // 2

        offset_east_m  =  offset_col * dem_res
        offset_north_m = -offset_row * dem_res  # rows increase southward

        m_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(lat))
        delta_lat = offset_north_m / METERS_PER_DEG_LAT
        delta_lon = offset_east_m  / m_per_deg_lon

        fix = NavSatFix()
        fix.header.stamp      = self.get_clock().now().to_msg()
        fix.header.frame_id   = 'map'
        fix.status.status     = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service    = NavSatStatus.SERVICE_GPS
        fix.latitude          = lat + delta_lat
        fix.longitude         = lon + delta_lon
        fix.altitude          = alt
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.publisher.publish(fix)

        self.get_logger().debug(
            f'corr={max_val:.2f} offset=({offset_east_m:.1f}m E, {offset_north_m:.1f}m N)')


def main(args=None):
    rclpy.init(args=args)
    node = MapNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
