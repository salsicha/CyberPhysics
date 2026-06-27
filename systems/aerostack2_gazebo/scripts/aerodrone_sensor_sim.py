#!/usr/bin/env python3
"""Publish Aerodrone hardware-facing simulated sensor topics from AS2 Gazebo topics."""

import math
import os
import time
from glob import glob

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import (
    BatteryState,
    CameraInfo,
    FluidPressure,
    Image,
    Imu,
    MagneticField,
    NavSatFix,
    NavSatStatus,
    Range,
)
from std_msgs.msg import Bool, Float32

try:
    import cv2
except ImportError:  # pragma: no cover - the runtime image carries OpenCV.
    cv2 = None


METERS_PER_DEG_LAT = 111_320.0
WEB_MERCATOR_LIMIT = 85.05112878


def _env_bool(name, default):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in {'0', 'false', 'no', 'off'}


def _rotate_vector_by_quaternion(vector, quat):
    qx, qy, qz, qw = quat
    vx, vy, vz = vector
    uv = np.cross([qx, qy, qz], [vx, vy, vz])
    uuv = np.cross([qx, qy, qz], uv)
    return np.asarray([vx, vy, vz]) + 2.0 * (qw * uv + uuv)


class AerodroneSensorSim(Node):
    def __init__(self):
        super().__init__('aerodrone_sensor_sim')
        self.declare_parameter(
            'synthetic_wilderness_sensors',
            _env_bool('AERODRONE_SYNTHETIC_WILDERNESS_SENSORS', True))
        self.declare_parameter(
            'georeference_gps_from_odom',
            _env_bool('AERODRONE_GEOREFERENCE_GPS_FROM_ODOM', True))
        self.declare_parameter('gps_noise_std_m', 1.5)
        self.declare_parameter('gps_degraded_noise_std_m', 12.0)
        self.declare_parameter('gps_degraded', False)
        self.declare_parameter('imu_accel_noise_std', 0.08)
        self.declare_parameter('imu_gyro_noise_std', 0.015)
        self.declare_parameter('magnetic_field_strength_ut', 47.0)
        self.declare_parameter('magnetic_inclination_deg', 61.0)
        self.declare_parameter('magnetic_declination_deg', 13.4)
        self.declare_parameter('magnetic_noise_std_ut', 0.8)
        self.declare_parameter('barometer_noise_std_pa', 1.5)
        self.declare_parameter('sea_level_pressure_pa', 101325.0)
        self.declare_parameter('pressure_scale_height_m', 8434.5)
        self.declare_parameter('range_min_m', 0.3)
        self.declare_parameter('range_max_m', 40.0)
        self.declare_parameter('battery_nominal_v', 16.0)
        self.declare_parameter('battery_sag_v_per_mps', 0.08)
        self.declare_parameter('geofence_min_x_m', -1000.0)
        self.declare_parameter('geofence_max_x_m', 1100.0)
        self.declare_parameter('geofence_min_y_m', -700.0)
        self.declare_parameter('geofence_max_y_m', 700.0)
        self.declare_parameter('min_agl_m', 25.0)
        self.declare_parameter('max_agl_m', 120.0)
        self.declare_parameter('random_seed', 401)

        seed = int(self.get_parameter('random_seed').value)
        self.rng = np.random.default_rng(seed)
        self.synthetic_sensors = bool(
            self.get_parameter('synthetic_wilderness_sensors').value)
        self.georeference_gps = bool(
            self.get_parameter('georeference_gps_from_odom').value)
        self.latest_odom = None
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.latest_velocity = None
        self.latest_velocity_time = None
        self.start_time = time.monotonic()
        self.origin_lat = float(os.environ.get('INITIAL_LAT', '37.9234'))
        self.origin_lon = float(os.environ.get('INITIAL_LON', '-122.5967'))
        self.origin_alt_m = float(os.environ.get('ORIGIN_ALT', '850.0'))
        self.area_km = float(os.environ.get('AREA_KM', '8.0'))
        self.dem_resolution_m = float(os.environ.get('DEM_RESOLUTION_M', '10.0'))
        self.fov_deg = float(os.environ.get('FOV_DEG', '69.0'))
        self.synthetic_width = int(os.environ.get('SYNTHETIC_CAMERA_WIDTH', '640'))
        self.synthetic_height = int(os.environ.get('SYNTHETIC_CAMERA_HEIGHT', '480'))
        self.dem_cache_dir = os.environ.get('DEMNAV_CACHE_DIR', '/data/demnav_cache')
        self.satellite_cache_dir = os.environ.get('WILDNAV_CACHE_DIR', '/data/wildnav_cache')
        self.wildnav_zoom = int(os.environ.get('WILDNAV_ZOOM', '18'))
        self.dem_grid = None
        self.dem_path = None

        self.gps_pub = self.create_publisher(
            NavSatFix,
            os.environ.get('GPS_STANDARD_TOPIC', '/navigation/gps_standard'),
            10)
        self.imu_pub = self.create_publisher(
            Imu,
            os.environ.get('FC_IMU_TOPIC', '/aerodrone/imu/data_raw'),
            50)
        self.compass_pub = self.create_publisher(
            MagneticField,
            os.environ.get('COMPASS_TOPIC', '/aerodrone/compass/magnetic_field'),
            20)
        self.barometer_pub = self.create_publisher(
            FluidPressure,
            os.environ.get('BAROMETER_TOPIC', '/aerodrone/barometer'),
            20)
        self.rgb_pub = self.create_publisher(
            Image,
            os.environ.get('OAK_IMAGE_TOPIC', '/oak1/image_highres'),
            5)
        self.rgb_info_pub = self.create_publisher(
            CameraInfo,
            os.environ.get('OAK_IMAGE_INFO_TOPIC', '/oak1/image_highres/camera_info'),
            5)
        self.depth_pub = self.create_publisher(
            Image,
            os.environ.get('OAK_RELATIVE_DEPTH_TOPIC', '/oak1/relative_depth'),
            5)
        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            os.environ.get('OAK_DEPTH_INFO_TOPIC', '/oak1/camera_info'),
            5)
        self.range_pub = self.create_publisher(
            Range,
            os.environ.get('RANGEFINDER_TOPIC', '/aerodrone/rangefinder'),
            10)
        self.battery_pub = self.create_publisher(
            Float32,
            os.environ.get('BATTERY_TOPIC', '/aerodrone/battery_voltage'),
            5)
        self.battery_state_pub = self.create_publisher(
            BatteryState,
            os.environ.get('BATTERY_STATE_TOPIC', '/aerodrone/battery'),
            5)
        self.rc_failsafe_pub = self.create_publisher(
            Bool,
            os.environ.get('RC_FAILSAFE_TOPIC', '/aerodrone/rc_failsafe'),
            5)
        self.geofence_pub = self.create_publisher(
            Bool,
            os.environ.get('GEOFENCE_TOPIC', '/aerodrone/geofence_violation'),
            5)
        self.emergency_land_pub = self.create_publisher(
            Bool,
            os.environ.get('EMERGENCY_LAND_TOPIC', '/aerodrone/emergency_land'),
            5)

        self.create_subscription(
            NavSatFix,
            os.environ.get('GPS_TOPIC', '/drone_sim_0/sensor_measurements/gps'),
            self._gps,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry,
            os.environ.get('ODOM_TOPIC', '/drone_sim_0/sensor_measurements/odom'),
            self._odom,
            qos_profile_sensor_data)
        self.create_subscription(
            Image,
            os.environ.get(
                'WILDNAV_IMAGE_TOPIC',
                '/drone_sim_0/sensor_measurements/downward_rgb/image_raw'),
            self._rgb,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo,
            os.environ.get(
                'WILDNAV_CAMERA_INFO_TOPIC',
                '/drone_sim_0/sensor_measurements/downward_rgb/camera_info'),
            self._rgb_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Image,
            os.environ.get(
                'DEPTH_TOPIC',
                '/drone_sim_0/sensor_measurements/downward_rgbd/depth'),
            self._depth,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo,
            os.environ.get(
                'CAMERA_INFO_TOPIC',
                '/drone_sim_0/sensor_measurements/downward_rgbd/depth/camera_info'),
            self._depth_info,
            qos_profile_sensor_data)
        self.imu_timer = self.create_timer(0.02, self._flight_controller_tick)
        self.timer = self.create_timer(0.2, self._status_tick)
        if self.synthetic_sensors:
            self.synthetic_timer = self.create_timer(0.2, self._synthetic_sensor_tick)
            self.get_logger().info(
                'Synthetic wilderness camera/depth enabled from DEM and satellite caches')

    def _gps(self, msg):
        out = NavSatFix()
        out.header = msg.header
        out.status.status = NavSatStatus.STATUS_FIX
        out.status.service = NavSatStatus.SERVICE_GPS
        std_m = float(
            self.get_parameter('gps_degraded_noise_std_m').value
            if self.get_parameter('gps_degraded').value
            else self.get_parameter('gps_noise_std_m').value)
        north_m, east_m = self.rng.normal(0.0, std_m, size=2)
        if self.georeference_gps and self.latest_odom is not None:
            position = self.latest_odom.pose.pose.position
            lat, lon = self._local_to_latlon(
                float(position.x) + east_m,
                float(position.y) + north_m)
            out.latitude = lat
            out.longitude = lon
            out.altitude = self.origin_alt_m + float(position.z) + float(
                self.rng.normal(0.0, max(0.5, std_m * 0.25)))
            out.header.frame_id = 'map'
        else:
            lat_rad = math.radians(msg.latitude if math.isfinite(msg.latitude) else 0.0)
            lon_scale = max(1e-3, METERS_PER_DEG_LAT * math.cos(lat_rad))
            out.latitude = msg.latitude + north_m / METERS_PER_DEG_LAT
            out.longitude = msg.longitude + east_m / lon_scale
            out.altitude = msg.altitude + float(
                self.rng.normal(0.0, max(0.5, std_m * 0.25)))
        out.position_covariance = [
            std_m ** 2, 0.0, 0.0,
            0.0, std_m ** 2, 0.0,
            0.0, 0.0, max(0.5, std_m * 0.25) ** 2,
        ]
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(out)

    def _odom(self, msg):
        self.latest_odom = msg

    def _flight_controller_tick(self):
        if self.latest_odom is None:
            return
        now = self.get_clock().now()
        odom = self.latest_odom
        velocity = np.asarray([
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
        ], dtype=float)
        if self.latest_velocity is None or self.latest_velocity_time is None:
            accel_world = np.asarray([0.0, 0.0, 9.80665], dtype=float)
        else:
            dt = max(1e-3, (now.nanoseconds - self.latest_velocity_time) / 1e9)
            accel_world = (velocity - self.latest_velocity) / dt
            accel_world[2] += 9.80665
        self.latest_velocity = velocity
        self.latest_velocity_time = now.nanoseconds

        q = odom.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        inverse_quat = [-q.x, -q.y, -q.z, q.w]
        accel_body = _rotate_vector_by_quaternion(accel_world, inverse_quat)

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'navigator_imu_link'
        imu.orientation = q
        gyro_std = float(self.get_parameter('imu_gyro_noise_std').value)
        accel_std = float(self.get_parameter('imu_accel_noise_std').value)
        gyro_noise = self.rng.normal(0.0, gyro_std, size=3)
        accel_noise = self.rng.normal(0.0, accel_std, size=3)
        imu.angular_velocity.x = odom.twist.twist.angular.x + float(gyro_noise[0])
        imu.angular_velocity.y = odom.twist.twist.angular.y + float(gyro_noise[1])
        imu.angular_velocity.z = odom.twist.twist.angular.z + float(gyro_noise[2])
        imu.linear_acceleration.x = float(accel_body[0] + accel_noise[0])
        imu.linear_acceleration.y = float(accel_body[1] + accel_noise[1])
        imu.linear_acceleration.z = float(accel_body[2] + accel_noise[2])
        imu.orientation_covariance = [
            0.0004, 0.0, 0.0,
            0.0, 0.0004, 0.0,
            0.0, 0.0, 0.0009,
        ]
        imu.angular_velocity_covariance = [
            gyro_std ** 2, 0.0, 0.0,
            0.0, gyro_std ** 2, 0.0,
            0.0, 0.0, gyro_std ** 2,
        ]
        imu.linear_acceleration_covariance = [
            accel_std ** 2, 0.0, 0.0,
            0.0, accel_std ** 2, 0.0,
            0.0, 0.0, accel_std ** 2,
        ]
        self.imu_pub.publish(imu)

        self._publish_compass(now, quat)
        self._publish_barometer(now, odom.pose.pose.position.z)

    def _publish_compass(self, now, quat):
        strength_t = float(self.get_parameter('magnetic_field_strength_ut').value) * 1e-6
        inclination = math.radians(float(self.get_parameter('magnetic_inclination_deg').value))
        declination = math.radians(float(self.get_parameter('magnetic_declination_deg').value))
        horizontal = strength_t * math.cos(inclination)
        field_enu = np.asarray([
            horizontal * math.sin(declination),
            horizontal * math.cos(declination),
            -strength_t * math.sin(inclination),
        ], dtype=float)
        inverse_quat = [-quat[0], -quat[1], -quat[2], quat[3]]
        body_field = _rotate_vector_by_quaternion(field_enu, inverse_quat)
        body_field += self.rng.normal(
            0.0,
            float(self.get_parameter('magnetic_noise_std_ut').value) * 1e-6,
            size=3)
        msg = MagneticField()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'navigator_imu_link'
        msg.magnetic_field.x = float(body_field[0])
        msg.magnetic_field.y = float(body_field[1])
        msg.magnetic_field.z = float(body_field[2])
        variance = (float(self.get_parameter('magnetic_noise_std_ut').value) * 1e-6) ** 2
        msg.magnetic_field_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]
        self.compass_pub.publish(msg)

    def _publish_barometer(self, now, local_z_m):
        altitude_m = self.origin_alt_m + float(local_z_m)
        pressure = float(self.get_parameter('sea_level_pressure_pa').value) * math.exp(
            -altitude_m / float(self.get_parameter('pressure_scale_height_m').value))
        pressure += float(self.rng.normal(
            0.0,
            float(self.get_parameter('barometer_noise_std_pa').value)))
        msg = FluidPressure()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'navigator_baro_link'
        msg.fluid_pressure = pressure
        msg.variance = float(self.get_parameter('barometer_noise_std_pa').value) ** 2
        self.barometer_pub.publish(msg)

    def _rgb(self, msg):
        if self.synthetic_sensors:
            return
        msg.header.frame_id = 'oak1_image_highres_optical_frame'
        self.rgb_pub.publish(msg)

    def _rgb_info(self, msg):
        if self.synthetic_sensors:
            return
        msg.header.frame_id = 'oak1_image_highres_optical_frame'
        self.rgb_info_pub.publish(msg)

    def _depth(self, msg):
        if self.synthetic_sensors:
            return
        self.latest_depth = msg
        self.latest_depth_stamp = self.get_clock().now()
        msg.header.frame_id = 'oak1_relative_depth_optical_frame'
        self.depth_pub.publish(msg)
        self._publish_range_from_depth(msg)

    def _depth_info(self, msg):
        if self.synthetic_sensors:
            return
        msg.header.frame_id = 'oak1_relative_depth_optical_frame'
        self.depth_info_pub.publish(msg)

    def _publish_range_from_depth(self, msg):
        if msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.step // 4)[:, :msg.width]
        elif msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.step // 2)[:, :msg.width].astype(np.float32) * 0.001
        else:
            return
        h0 = max(0, msg.height // 2 - max(2, msg.height // 20))
        h1 = min(msg.height, msg.height // 2 + max(2, msg.height // 20))
        w0 = max(0, msg.width // 2 - max(2, msg.width // 20))
        w1 = min(msg.width, msg.width // 2 + max(2, msg.width // 20))
        window = depth[h0:h1, w0:w1]
        valid = window[np.isfinite(window) & (window > 0.0)]
        if valid.size == 0:
            return
        out = Range()
        out.header = msg.header
        out.header.frame_id = 'rangefinder_link'
        out.radiation_type = Range.INFRARED
        out.field_of_view = math.radians(4.0)
        out.min_range = float(self.get_parameter('range_min_m').value)
        out.max_range = float(self.get_parameter('range_max_m').value)
        out.range = float(np.clip(np.nanmedian(valid), out.min_range, out.max_range))
        self.range_pub.publish(out)

    def _synthetic_sensor_tick(self):
        if self.latest_odom is None:
            return
        now = self.get_clock().now().to_msg()
        rgb = self._synthetic_satellite_image()
        depth = self._synthetic_dem_depth()
        if rgb is not None:
            rgb_msg = Image()
            rgb_msg.header.stamp = now
            rgb_msg.header.frame_id = 'oak1_image_highres_optical_frame'
            rgb_msg.height, rgb_msg.width = rgb.shape[:2]
            rgb_msg.encoding = 'bgr8'
            rgb_msg.is_bigendian = False
            rgb_msg.step = int(rgb_msg.width * 3)
            rgb_msg.data = rgb.astype(np.uint8, copy=False).tobytes()
            self.rgb_pub.publish(rgb_msg)
            self.rgb_info_pub.publish(self._camera_info(
                now, rgb_msg.width, rgb_msg.height,
                'oak1_image_highres_optical_frame'))
        if depth is not None:
            depth_msg = Image()
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = 'oak1_relative_depth_optical_frame'
            depth_msg.height, depth_msg.width = depth.shape[:2]
            depth_msg.encoding = '32FC1'
            depth_msg.is_bigendian = False
            depth_msg.step = int(depth_msg.width * 4)
            depth_msg.data = depth.astype(np.float32, copy=False).tobytes()
            self.depth_pub.publish(depth_msg)
            self.depth_info_pub.publish(self._camera_info(
                now, depth_msg.width, depth_msg.height,
                'oak1_relative_depth_optical_frame'))
            self._publish_range_from_depth(depth_msg)

    def _camera_info(self, stamp, width, height, frame_id):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = int(width)
        info.height = int(height)
        focal = width / (2.0 * math.tan(math.radians(self.fov_deg / 2.0)))
        cx = width * 0.5
        cy = height * 0.5
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            focal, 0.0, cx, 0.0,
            0.0, focal, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def _local_to_latlon(self, east_m, north_m):
        metres_per_deg_lon = (
            METERS_PER_DEG_LAT * math.cos(math.radians(self.origin_lat)))
        lat = self.origin_lat + north_m / METERS_PER_DEG_LAT
        lon = self.origin_lon + east_m / metres_per_deg_lon
        return lat, lon

    def _latest_local_position(self):
        position = self.latest_odom.pose.pose.position
        return float(position.x), float(position.y), float(position.z)

    def _latest_yaw(self):
        q = self.latest_odom.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _load_dem_grid(self):
        if self.dem_grid is not None:
            return True
        preferred = os.path.join(
            self.dem_cache_dir,
            f'dem_{self.origin_lat:.4f}_{self.origin_lon:.4f}_'
            f'{self.area_km:.1f}_{self.dem_resolution_m:.0f}.npy')
        candidates = [preferred] + sorted(glob(os.path.join(self.dem_cache_dir, '*.npy')))
        for path in candidates:
            if not os.path.exists(path):
                continue
            try:
                self.dem_grid = np.load(path).astype(np.float32)
                self.dem_path = path
                self.get_logger().info(f'Loaded synthetic DEM source {path}')
                return True
            except Exception as exc:
                self.get_logger().warning(f'Failed to load DEM cache {path}: {exc}')
        return False

    def _synthetic_dem_depth(self):
        if cv2 is None or not self._load_dem_grid():
            return None
        east, north, local_z = self._latest_local_position()
        altitude_m = self.origin_alt_m + local_z
        ground_width_m = 2.0 * altitude_m * math.tan(math.radians(self.fov_deg / 2.0))
        ground_height_m = ground_width_m * self.synthetic_height / self.synthetic_width
        cols = max(3, int(ground_width_m / self.dem_resolution_m))
        rows = max(3, int(ground_height_m / self.dem_resolution_m))
        half_m = self.area_km * 500.0
        row_center = (half_m - north) / self.dem_resolution_m
        col_center = (east + half_m) / self.dem_resolution_m
        patch = self._crop_grid(self.dem_grid, row_center, col_center, rows, cols)
        terrain = cv2.resize(
            patch, (self.synthetic_width, self.synthetic_height),
            interpolation=cv2.INTER_CUBIC)
        yaw = self._latest_yaw()
        if abs(yaw) > 1e-3:
            matrix = cv2.getRotationMatrix2D(
                (self.synthetic_width * 0.5, self.synthetic_height * 0.5),
                math.degrees(yaw),
                1.0)
            terrain = cv2.warpAffine(
                terrain, matrix, (self.synthetic_width, self.synthetic_height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_REFLECT)
        depth = (altitude_m - terrain).astype(np.float32)
        depth += self.rng.normal(0.0, 0.35, size=depth.shape).astype(np.float32)
        depth[(~np.isfinite(depth)) | (depth <= 0.2) | (depth > 500.0)] = np.nan
        return depth

    def _crop_grid(self, grid, row_center, col_center, rows, cols):
        rows = min(max(3, rows), grid.shape[0])
        cols = min(max(3, cols), grid.shape[1])
        r0 = int(round(row_center - rows * 0.5))
        c0 = int(round(col_center - cols * 0.5))
        r1 = r0 + rows
        c1 = c0 + cols
        pad_top = max(0, -r0)
        pad_left = max(0, -c0)
        pad_bottom = max(0, r1 - grid.shape[0])
        pad_right = max(0, c1 - grid.shape[1])
        if pad_top or pad_bottom or pad_left or pad_right:
            grid = np.pad(
                grid,
                ((pad_top, pad_bottom), (pad_left, pad_right)),
                mode='edge')
            r0 += pad_top
            r1 += pad_top
            c0 += pad_left
            c1 += pad_left
        return grid[r0:r1, c0:c1].copy()

    def _synthetic_satellite_image(self):
        if cv2 is None:
            return None
        east, north, _ = self._latest_local_position()
        lat, lon = self._local_to_latlon(east, north)
        tile_x, tile_y, pixel_x, pixel_y = self._latlon_to_tile_pixel(
            lat, lon, self.wildnav_zoom)
        path = os.path.join(
            self.satellite_cache_dir,
            f'{self.wildnav_zoom}_{tile_x}_{tile_y}.jpg')
        tile = cv2.imread(path, cv2.IMREAD_COLOR)
        if tile is None:
            candidates = sorted(glob(os.path.join(
                self.satellite_cache_dir, f'{self.wildnav_zoom}_*.jpg')))
            if not candidates:
                return None
            tile = cv2.imread(candidates[0], cv2.IMREAD_COLOR)
            if tile is None:
                return None
        crop_w, crop_h = 128, 96
        center_x = int(np.clip(pixel_x, crop_w * 0.5, 256 - crop_w * 0.5))
        center_y = int(np.clip(pixel_y, crop_h * 0.5, 256 - crop_h * 0.5))
        x0 = center_x - crop_w // 2
        y0 = center_y - crop_h // 2
        crop = tile[y0:y0 + crop_h, x0:x0 + crop_w]
        image = cv2.resize(
            crop, (self.synthetic_width, self.synthetic_height),
            interpolation=cv2.INTER_CUBIC)
        yaw = self._latest_yaw()
        if abs(yaw) > 1e-3:
            matrix = cv2.getRotationMatrix2D(
                (self.synthetic_width * 0.5, self.synthetic_height * 0.5),
                math.degrees(yaw),
                1.0)
            image = cv2.warpAffine(
                image, matrix, (self.synthetic_width, self.synthetic_height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_REFLECT)
        return image

    def _latlon_to_tile_pixel(self, lat, lon, zoom):
        lat = max(-WEB_MERCATOR_LIMIT, min(WEB_MERCATOR_LIMIT, lat))
        scale = 2 ** zoom
        x = (lon + 180.0) / 360.0 * scale
        lat_rad = math.radians(lat)
        y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * scale
        tile_x = int(x) % scale
        tile_y = max(0, min(scale - 1, int(y)))
        return tile_x, tile_y, (x - int(x)) * 256.0, (y - int(y)) * 256.0

    def _status_tick(self):
        battery = Float32()
        speed = 0.0
        z = 0.0
        geofence = False
        if self.latest_odom is not None:
            vx = self.latest_odom.twist.twist.linear.x
            vy = self.latest_odom.twist.twist.linear.y
            vz = self.latest_odom.twist.twist.linear.z
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            x = self.latest_odom.pose.pose.position.x
            y = self.latest_odom.pose.pose.position.y
            z = self.latest_odom.pose.pose.position.z
            geofence = not (
                float(self.get_parameter('geofence_min_x_m').value)
                <= x
                <= float(self.get_parameter('geofence_max_x_m').value)
                and float(self.get_parameter('geofence_min_y_m').value)
                <= y
                <= float(self.get_parameter('geofence_max_y_m').value)
            )
        battery.data = (
            float(self.get_parameter('battery_nominal_v').value)
            - speed * float(self.get_parameter('battery_sag_v_per_mps').value))
        self.battery_pub.publish(battery)

        battery_state = BatteryState()
        battery_state.header.stamp = self.get_clock().now().to_msg()
        battery_state.header.frame_id = 'battery_link'
        battery_state.voltage = battery.data
        battery_state.current = 8.0 + speed * 1.8
        battery_state.percentage = float(np.clip((battery.data - 13.2) / (16.8 - 13.2), 0.0, 1.0))
        battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        self.battery_state_pub.publish(battery_state)

        flag = Bool()
        flag.data = False
        self.rc_failsafe_pub.publish(flag)
        geo = Bool()
        geo.data = bool(geofence)
        self.geofence_pub.publish(geo)
        emergency = Bool()
        emergency.data = geofence or z > float(self.get_parameter('max_agl_m').value)
        self.emergency_land_pub.publish(emergency)


def main():
    rclpy.init()
    node = AerodroneSensorSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
