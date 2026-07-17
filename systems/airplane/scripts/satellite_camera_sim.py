#!/usr/bin/env python3

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
# Installed in /venv of the cyberphysics/aerostack2 image this script runs
# in; renders the same world demnav's DEM and wildnav's tiles are built from.
from synthetic_world import latlon_to_local, satellite_rgb, terrain_height


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SatelliteCameraSim(Node):
    def __init__(self) -> None:
        super().__init__("satellite_camera_sim")
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("depth_width", 160)
        self.declare_parameter("depth_height", 120)
        self.declare_parameter("image_topic", "/airplane/downward/image_highres")
        self.declare_parameter(
            "camera_info_topic", "/airplane/downward/image_highres/camera_info"
        )
        self.declare_parameter("preview_image_topic", "/airplane/downward/image_raw")
        self.declare_parameter(
            "preview_camera_info_topic", "/airplane/downward/camera_info"
        )
        self.declare_parameter("depth_topic", "/airplane/downward/relative_depth")
        self.declare_parameter(
            "depth_camera_info_topic", "/airplane/downward/camera_info"
        )
        self.declare_parameter("gps_topic", "/plane0/sensor_measurements/gps")
        self.declare_parameter("odom_topic", "/plane0/sensor_measurements/odom")
        self.declare_parameter("frame_id", "airplane/downward_optical_frame")
        self.declare_parameter("origin_lat", 37.9234)
        self.declare_parameter("origin_lon", -122.5967)
        self.declare_parameter("altitude_agl_m", 180.0)
        self.declare_parameter("groundspeed_mps", 24.0)
        self.declare_parameter("horizontal_fov_deg", 69.0)
        self.declare_parameter("fallback_route_radius_m", 650.0)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.depth_width = int(self.get_parameter("depth_width").value)
        self.depth_height = int(self.get_parameter("depth_height").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.origin_lat = float(self.get_parameter("origin_lat").value)
        self.origin_lon = float(self.get_parameter("origin_lon").value)
        self.altitude_agl = float(self.get_parameter("altitude_agl_m").value)
        self.groundspeed = float(self.get_parameter("groundspeed_mps").value)
        self.fov = math.radians(float(self.get_parameter("horizontal_fov_deg").value))
        self.route_radius = float(self.get_parameter("fallback_route_radius_m").value)
        self.east_m = 0.0
        self.north_m = 0.0
        self.yaw_rad = 0.0
        self.altitude_amsl = None
        self.last_external_update = 0.0
        self.start_time = time.monotonic()

        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("image_topic").value),
            qos_profile_sensor_data,
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            qos_profile_sensor_data,
        )
        self.preview_pub = self.create_publisher(
            Image,
            str(self.get_parameter("preview_image_topic").value),
            qos_profile_sensor_data,
        )
        self.preview_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("preview_camera_info_topic").value),
            qos_profile_sensor_data,
        )
        self.depth_pub = self.create_publisher(
            Image,
            str(self.get_parameter("depth_topic").value),
            qos_profile_sensor_data,
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("depth_camera_info_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter("gps_topic").value),
            self._gps,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom,
            qos_profile_sensor_data,
        )

        rate_hz = max(0.1, float(self.get_parameter("rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info("Synthetic satellite-map camera publishing")

    def _gps(self, msg: NavSatFix) -> None:
        self.east_m, self.north_m = latlon_to_local(
            float(msg.latitude), float(msg.longitude),
            self.origin_lat, self.origin_lon)
        if math.isfinite(float(msg.altitude)) and float(msg.altitude) > 0.0:
            self.altitude_amsl = float(msg.altitude)
        self.last_external_update = time.monotonic()

    def _odom(self, msg: Odometry) -> None:
        self.yaw_rad = _yaw_from_quaternion(msg.pose.pose.orientation)
        if time.monotonic() - self.last_external_update < 1.0:
            return
        self.east_m = float(msg.pose.pose.position.x)
        self.north_m = float(msg.pose.pose.position.y)
        self.last_external_update = time.monotonic()

    def _camera_altitude(self) -> float:
        """Camera altitude in the same datum as the GPS fix DemNav uses."""
        if self.altitude_amsl is not None:
            return self.altitude_amsl
        terrain = float(terrain_height(self.east_m, self.north_m))
        return terrain + self.altitude_agl

    def _tick(self) -> None:
        if time.monotonic() - self.last_external_update > 2.0:
            elapsed = time.monotonic() - self.start_time
            omega = self.groundspeed / max(1.0, self.route_radius)
            self.east_m = self.route_radius * math.sin(omega * elapsed)
            self.north_m = 0.55 * self.route_radius * math.sin(0.5 * omega * elapsed)

        altitude = self._camera_altitude()
        nadir_terrain = float(terrain_height(self.east_m, self.north_m))
        view_height = max(1.0, altitude - nadir_terrain)

        stamp = self.get_clock().now().to_msg()
        image = self._make_image(self.width, self.height, view_height, stamp)
        info = self._camera_info(self.width, self.height, stamp)
        self.image_pub.publish(image)
        self.camera_info_pub.publish(info)

        preview = self._make_image(
            self.depth_width, self.depth_height, view_height, stamp)
        preview_info = self._camera_info(self.depth_width, self.depth_height, stamp)
        self.preview_pub.publish(preview)
        self.preview_info_pub.publish(preview_info)

        depth = self._make_depth(altitude, view_height, stamp)
        self.depth_pub.publish(depth)
        self.depth_info_pub.publish(preview_info)

    def _ground_grid(self, width: int, height: int, view_height: float):
        """East/north metres seen by each pixel of the body-fixed nadir
        camera (image up = vehicle forward). DemNav de-rotates frames by
        -yaw before DEM correlation, so the rendered frame must be yawed
        the same way the aerodrone sensor sim yaws its crops.

        Only yaw is modelled: the camera is assumed roll/pitch-stabilised
        at nadir. The matchers gate on max_attitude_deg, so banked-turn
        frames are skipped rather than rendered off-nadir."""
        ground_width = 2.0 * view_height * math.tan(self.fov * 0.5)
        gsd = ground_width / max(1, width)
        dx = np.arange(width, dtype=np.float64) - width * 0.5
        dy = np.arange(height, dtype=np.float64) - height * 0.5
        dx_grid, dy_grid = np.meshgrid(dx, dy)
        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)
        east = self.east_m + (cos_yaw * dx_grid - sin_yaw * dy_grid) * gsd
        north = self.north_m - (sin_yaw * dx_grid + cos_yaw * dy_grid) * gsd
        return east, north

    def _make_image(self, width: int, height: int,
                    view_height: float, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = width * 3
        east, north = self._ground_grid(width, height, view_height)
        msg.data = satellite_rgb(east, north).tobytes()
        return msg

    def _make_depth(self, altitude: float, view_height: float, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = self.depth_height
        msg.width = self.depth_width
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = self.depth_width * 4
        east, north = self._ground_grid(
            self.depth_width, self.depth_height, view_height)
        depth = np.maximum(1.0, altitude - terrain_height(east, north))
        msg.data = depth.astype(np.float32).tobytes()
        return msg

    def _camera_info(self, width: int, height: int, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height
        focal = width / (2.0 * math.tan(self.fov * 0.5))
        info.k = [
            focal,
            0.0,
            width / 2.0,
            0.0,
            focal,
            height / 2.0,
            0.0,
            0.0,
            1.0,
        ]
        info.p = [
            focal,
            0.0,
            width / 2.0,
            0.0,
            0.0,
            focal,
            height / 2.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return info


def main() -> None:
    rclpy.init()
    node = SatelliteCameraSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
