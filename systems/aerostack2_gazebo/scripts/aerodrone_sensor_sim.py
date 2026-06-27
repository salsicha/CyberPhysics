#!/usr/bin/env python3
"""Publish Aerodrone hardware-facing simulated sensor topics from AS2 Gazebo topics."""

import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, NavSatFix, Range
from std_msgs.msg import Bool, Float32


class AerodroneSensorSim(Node):
    def __init__(self):
        super().__init__('aerodrone_sensor_sim')
        self.declare_parameter('gps_noise_std_m', 1.5)
        self.declare_parameter('gps_degraded_noise_std_m', 12.0)
        self.declare_parameter('gps_degraded', False)
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
        self.latest_odom = None
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.start_time = time.monotonic()

        self.gps_pub = self.create_publisher(NavSatFix, os.environ.get('GPS_STANDARD_TOPIC', '/navigation/gps_standard'), 10)
        self.rgb_pub = self.create_publisher(Image, os.environ.get('OAK_IMAGE_TOPIC', '/oak1/image_highres'), 5)
        self.rgb_info_pub = self.create_publisher(CameraInfo, os.environ.get('OAK_IMAGE_INFO_TOPIC', '/oak1/image_highres/camera_info'), 5)
        self.depth_pub = self.create_publisher(Image, os.environ.get('OAK_RELATIVE_DEPTH_TOPIC', '/oak1/relative_depth'), 5)
        self.depth_info_pub = self.create_publisher(CameraInfo, os.environ.get('OAK_DEPTH_INFO_TOPIC', '/oak1/camera_info'), 5)
        self.range_pub = self.create_publisher(Range, os.environ.get('RANGEFINDER_TOPIC', '/aerodrone/rangefinder'), 10)
        self.battery_pub = self.create_publisher(Float32, os.environ.get('BATTERY_TOPIC', '/aerodrone/battery_voltage'), 5)
        self.rc_failsafe_pub = self.create_publisher(Bool, os.environ.get('RC_FAILSAFE_TOPIC', '/aerodrone/rc_failsafe'), 5)
        self.geofence_pub = self.create_publisher(Bool, os.environ.get('GEOFENCE_TOPIC', '/aerodrone/geofence_violation'), 5)
        self.emergency_land_pub = self.create_publisher(Bool, os.environ.get('EMERGENCY_LAND_TOPIC', '/aerodrone/emergency_land'), 5)

        self.create_subscription(NavSatFix, os.environ.get('GPS_TOPIC', '/drone_sim_0/sensor_measurements/gps'), self._gps, qos_profile_sensor_data)
        self.create_subscription(Odometry, os.environ.get('ODOM_TOPIC', '/drone_sim_0/sensor_measurements/odom'), self._odom, qos_profile_sensor_data)
        self.create_subscription(Image, os.environ.get('WILDNAV_IMAGE_TOPIC', '/drone_sim_0/sensor_measurements/downward_rgb/image_raw'), self._rgb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, os.environ.get('WILDNAV_CAMERA_INFO_TOPIC', '/drone_sim_0/sensor_measurements/downward_rgb/camera_info'), self._rgb_info, qos_profile_sensor_data)
        self.create_subscription(Image, os.environ.get('DEPTH_TOPIC', '/drone_sim_0/sensor_measurements/downward_rgbd/depth'), self._depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, os.environ.get('CAMERA_INFO_TOPIC', '/drone_sim_0/sensor_measurements/downward_rgbd/depth/camera_info'), self._depth_info, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self._status_tick)

    def _gps(self, msg):
        out = NavSatFix()
        out.header = msg.header
        out.status = msg.status
        std_m = float(self.get_parameter('gps_degraded_noise_std_m').value if self.get_parameter('gps_degraded').value else self.get_parameter('gps_noise_std_m').value)
        north_m, east_m = self.rng.normal(0.0, std_m, size=2)
        lat_rad = math.radians(msg.latitude if math.isfinite(msg.latitude) else 0.0)
        lat_scale = 111_320.0
        lon_scale = max(1e-3, lat_scale * math.cos(lat_rad))
        out.latitude = msg.latitude + north_m / lat_scale
        out.longitude = msg.longitude + east_m / lon_scale
        out.altitude = msg.altitude + float(self.rng.normal(0.0, max(0.5, std_m * 0.25)))
        out.position_covariance = [std_m ** 2, 0.0, 0.0, 0.0, std_m ** 2, 0.0, 0.0, 0.0, max(0.5, std_m * 0.25) ** 2]
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(out)

    def _odom(self, msg):
        self.latest_odom = msg

    def _rgb(self, msg):
        msg.header.frame_id = 'oak1_image_highres_optical_frame'
        self.rgb_pub.publish(msg)

    def _rgb_info(self, msg):
        msg.header.frame_id = 'oak1_image_highres_optical_frame'
        self.rgb_info_pub.publish(msg)

    def _depth(self, msg):
        self.latest_depth = msg
        self.latest_depth_stamp = self.get_clock().now()
        msg.header.frame_id = 'oak1_relative_depth_optical_frame'
        self.depth_pub.publish(msg)
        self._publish_range_from_depth(msg)

    def _depth_info(self, msg):
        msg.header.frame_id = 'oak1_relative_depth_optical_frame'
        self.depth_info_pub.publish(msg)

    def _publish_range_from_depth(self, msg):
        if msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        elif msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width].astype(np.float32) * 0.001
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
                float(self.get_parameter('geofence_min_x_m').value) <= x <= float(self.get_parameter('geofence_max_x_m').value)
                and float(self.get_parameter('geofence_min_y_m').value) <= y <= float(self.get_parameter('geofence_max_y_m').value)
            )
        battery.data = float(self.get_parameter('battery_nominal_v').value) - speed * float(self.get_parameter('battery_sag_v_per_mps').value)
        self.battery_pub.publish(battery)
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
