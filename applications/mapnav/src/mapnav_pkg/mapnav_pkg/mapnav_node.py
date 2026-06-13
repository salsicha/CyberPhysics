#!/usr/bin/env python3
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import (
    CameraInfo,
    Image,
    NavSatFix,
    NavSatStatus,
    PointCloud2,
    PointField,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
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
        self.declare_parameter('depth_topic',      '/oak1/relative_depth')
        self.declare_parameter('camera_info_topic', '/oak1/camera_info')
        self.declare_parameter('odom_topic',       '/odom')
        self.declare_parameter('output_topic',     '/mapnav/fix')
        self.declare_parameter('odometry_output_topic', '/mapnav/odometry')
        self.declare_parameter('pose_output_topic', '/mapnav/pose')
        self.declare_parameter('metric_depth_topic', '/mapnav/metric_depth')
        self.declare_parameter('pointcloud_topic', '/mapnav/points')
        self.declare_parameter('confidence_topic', '/mapnav/confidence')
        self.declare_parameter('scale_topic', '/mapnav/depth_scale')
        self.declare_parameter('valid_topic', '/mapnav/valid')
        self.declare_parameter('initial_lat',      0.0)
        self.declare_parameter('initial_lon',      0.0)
        self.declare_parameter('origin_alt',       0.0)
        self.declare_parameter('area_km',          10.0)
        self.declare_parameter('search_radius_m',  200.0)
        self.declare_parameter('fov_deg',          90.0)
        self.declare_parameter('dem_resolution_m', 30.0)
        self.declare_parameter('min_correlation',  0.3)
        self.declare_parameter('cache_dir',        '/data/mapnav_cache')
        self.declare_parameter('pointcloud_stride', 4)
        self.declare_parameter('min_dem_footprint_pixels', 5)
        self.declare_parameter('min_metric_depth_m', 0.2)
        self.declare_parameter('max_metric_depth_m', 500.0)

        gps_topic      = self.get_parameter('gps_topic').value
        depth_topic    = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        odom_topic     = self.get_parameter('odom_topic').value
        output_topic   = self.get_parameter('output_topic').value
        initial_lat    = self.get_parameter('initial_lat').value
        initial_lon    = self.get_parameter('initial_lon').value

        self.fov_deg          = self.get_parameter('fov_deg').value
        self.search_radius_m  = self.get_parameter('search_radius_m').value
        self.min_correlation  = self.get_parameter('min_correlation').value
        self.origin_lat       = initial_lat
        self.origin_lon       = initial_lon
        self.origin_alt       = self.get_parameter('origin_alt').value
        self.pointcloud_stride = max(
            1, int(self.get_parameter('pointcloud_stride').value))
        self.min_dem_footprint_pixels = max(
            3, int(self.get_parameter('min_dem_footprint_pixels').value))
        self.min_metric_depth = self.get_parameter('min_metric_depth_m').value
        self.max_metric_depth = self.get_parameter('max_metric_depth_m').value

        self.bridge       = CvBridge()
        self.latest_fix   = None
        self.latest_odom  = None
        self.camera_info  = None
        self.yaw_rad      = 0.0
        self.heightmap    = None

        if initial_lat != 0.0 or initial_lon != 0.0:
            self.get_logger().info(
                f'Pre-loading height map at ({initial_lat:.5f}, {initial_lon:.5f}) ...')
            self._load_heightmap(initial_lat, initial_lon)
        else:
            self.get_logger().warn(
                'initial_lat/initial_lon not set — height map loads on first GPS fix')

        self.create_subscription(
            NavSatFix, gps_topic, self._on_fix, qos_profile_sensor_data)
        self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
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
        if self.origin_lat == 0.0 and self.origin_lon == 0.0:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
        if self.heightmap is None:
            self.get_logger().info(
                f'First GPS fix — loading height map at '
                f'({msg.latitude:.5f}, {msg.longitude:.5f})')
            self._load_heightmap(msg.latitude, msg.longitude)

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg
        self.yaw_rad = _yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _on_depth(self, msg: Image):
        if self.latest_fix is None or self.heightmap is None:
            self._publish_valid(False)
            return
        if self.latest_fix.status.status < NavSatStatus.STATUS_FIX:
            self._publish_valid(False)
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')
            self._publish_valid(False)
            return
        self._run_matching(depth, msg)

    def _run_matching(self, depth: np.ndarray, source_msg: Image):
        lat = self.latest_fix.latitude
        lon = self.latest_fix.longitude
        alt = self.latest_fix.altitude

        if alt <= 0.0:
            self._publish_valid(False)
            return

        img_h, img_w = depth.shape
        raw_depth = depth.astype(np.float32)
        finite = np.isfinite(raw_depth)
        if finite.sum() < raw_depth.size * 0.25:
            self._publish_valid(False)
            return

        # Rotate depth image to align north using vehicle yaw
        angle_deg = math.degrees(self.yaw_rad)
        M = cv2.getRotationMatrix2D((img_w / 2.0, img_h / 2.0), -angle_deg, 1.0)
        fill_value = float(np.nanmedian(raw_depth[finite]))
        depth_aligned = cv2.warpAffine(
            np.where(finite, raw_depth, fill_value),
            M, (img_w, img_h),
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=fill_value)

        # Normalise (zero mean, unit variance)
        d = depth_aligned
        std = d.std()
        if std < 1e-6:
            self._publish_valid(False)
            return
        d = (d - d.mean()) / std

        # Ground sampling distance → resample depth to DEM pixel scale
        ground_width_m = 2.0 * alt * math.tan(math.radians(self.fov_deg / 2.0))
        gsd = ground_width_m / img_w
        scale = gsd / self.heightmap.resolution_m
        new_w = max(3, int(img_w * scale))
        new_h = max(3, int(img_h * scale))
        if (new_w < self.min_dem_footprint_pixels or
                new_h < self.min_dem_footprint_pixels):
            self.get_logger().debug(
                'Camera footprint is too small at the configured DEM resolution')
            self._publish_valid(False)
            return
        depth_tmpl = cv2.resize(d, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        relative_tmpl = cv2.resize(
            depth_aligned, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Get DEM search patch
        search_m = self.search_radius_m * 2.0
        dem_patch, dem_res = self.heightmap.get_patch(lat, lon, search_m, search_m)

        if (dem_patch.shape[0] < depth_tmpl.shape[0] + 2 or
                dem_patch.shape[1] < depth_tmpl.shape[1] + 2):
            self.get_logger().warn(
                'DEM patch too small for matching — increase area_km or reduce search_radius_m')
            self._publish_valid(False)
            return

        dem = dem_patch.astype(np.float32)
        dem_std = dem.std()
        if dem_std < 1e-6:
            self._publish_valid(False)
            return
        dem = (dem - dem.mean()) / dem_std

        # Relative-depth models may encode either depth or inverse depth.
        positive = cv2.matchTemplate(dem, depth_tmpl, cv2.TM_CCOEFF_NORMED)
        negative = cv2.matchTemplate(dem, -depth_tmpl, cv2.TM_CCOEFF_NORMED)
        _, positive_score, _, positive_loc = cv2.minMaxLoc(positive)
        _, negative_score, _, negative_loc = cv2.minMaxLoc(negative)
        if positive_score >= negative_score:
            max_val, max_loc = positive_score, positive_loc
        else:
            max_val, max_loc = negative_score, negative_loc

        if max_val < self.min_correlation:
            self.get_logger().debug(f'Low correlation {max_val:.2f} — skipping')
            self._publish_valid(False)
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
        corrected_lat = lat + delta_lat
        corrected_lon = lon + delta_lon

        matched_dem = dem_patch[
            max_loc[1]:max_loc[1] + depth_tmpl.shape[0],
            max_loc[0]:max_loc[0] + depth_tmpl.shape[1]]
        target_range = alt - matched_dem
        fit_mask = (
            np.isfinite(relative_tmpl) &
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
        confidence = float(np.clip(0.7 * max_val + 0.3 * fit_score, 0.0, 1.0))

        metric_depth = raw_depth * depth_scale + depth_offset
        metric_depth[
            (~finite) |
            (metric_depth < self.min_metric_depth) |
            (metric_depth > self.max_metric_depth)] = np.nan

        horizontal_sigma = max(
            dem_res, self.search_radius_m * (1.0 - confidence))
        vertical_sigma = max(dem_res, float(np.std(residual)))
        horizontal_variance = horizontal_sigma ** 2
        vertical_variance = vertical_sigma ** 2

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

        east = (
            (corrected_lon - self.origin_lon) * METERS_PER_DEG_LAT *
            math.cos(math.radians(self.origin_lat)))
        north = (corrected_lat - self.origin_lat) * METERS_PER_DEG_LAT
        odom = Odometry()
        odom.header = fix.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = alt - self.origin_alt
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

        metric_msg = self.bridge.cv2_to_imgmsg(metric_depth, encoding='32FC1')
        metric_msg.header = source_msg.header
        self.confidence_publisher.publish(Float32(data=confidence))
        self.scale_publisher.publish(Float32(data=depth_scale))
        self._publish_valid(True)
        self.odom_publisher.publish(odom)
        self.pose_publisher.publish(pose)
        self.metric_depth_publisher.publish(metric_msg)
        self.pointcloud_publisher.publish(
            self._make_pointcloud(metric_depth, source_msg.header))

        self.get_logger().debug(
            f'confidence={confidence:.2f} scale={depth_scale:.3f} '
            f'offset=({offset_east_m:.1f}m E, {offset_north_m:.1f}m N)')

    def _make_pointcloud(self, metric_depth, header):
        height, width = metric_depth.shape
        if self.camera_info is not None and self.camera_info.k[0] > 0.0:
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
        else:
            fx = width / (2.0 * math.tan(math.radians(self.fov_deg / 2.0)))
            fy = fx
            cx = width / 2.0
            cy = height / 2.0

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
