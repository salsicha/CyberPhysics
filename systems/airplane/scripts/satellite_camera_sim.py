#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from array import array

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, NavSatFix


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
        self.last_external_update = 0.0
        self.start_time = time.monotonic()

        self.image_pub = self.create_publisher(
            Image, str(self.get_parameter("image_topic").value), 10
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), 10
        )
        self.preview_pub = self.create_publisher(
            Image, str(self.get_parameter("preview_image_topic").value), 10
        )
        self.preview_info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("preview_camera_info_topic").value), 10
        )
        self.depth_pub = self.create_publisher(
            Image, str(self.get_parameter("depth_topic").value), 10
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("depth_camera_info_topic").value), 10
        )
        self.create_subscription(
            NavSatFix, str(self.get_parameter("gps_topic").value), self._gps, 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom, 10
        )

        rate_hz = max(0.1, float(self.get_parameter("rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info("Synthetic satellite-map camera publishing")

    def _gps(self, msg: NavSatFix) -> None:
        metres_per_deg_lat = 111_111.0
        metres_per_deg_lon = metres_per_deg_lat * math.cos(
            math.radians(self.origin_lat)
        )
        self.north_m = (float(msg.latitude) - self.origin_lat) * metres_per_deg_lat
        self.east_m = (float(msg.longitude) - self.origin_lon) * metres_per_deg_lon
        self.last_external_update = time.monotonic()

    def _odom(self, msg: Odometry) -> None:
        if time.monotonic() - self.last_external_update < 1.0:
            return
        self.east_m = float(msg.pose.pose.position.x)
        self.north_m = float(msg.pose.pose.position.y)
        self.last_external_update = time.monotonic()

    def _tick(self) -> None:
        if time.monotonic() - self.last_external_update > 2.0:
            elapsed = time.monotonic() - self.start_time
            omega = self.groundspeed / max(1.0, self.route_radius)
            self.east_m = self.route_radius * math.sin(omega * elapsed)
            self.north_m = 0.55 * self.route_radius * math.sin(0.5 * omega * elapsed)

        stamp = self.get_clock().now().to_msg()
        image = self._make_image(self.width, self.height, stamp)
        info = self._camera_info(self.width, self.height, stamp)
        self.image_pub.publish(image)
        self.camera_info_pub.publish(info)

        preview = self._make_image(self.depth_width, self.depth_height, stamp)
        preview_info = self._camera_info(self.depth_width, self.depth_height, stamp)
        self.preview_pub.publish(preview)
        self.preview_info_pub.publish(preview_info)

        depth = self._make_depth(stamp)
        self.depth_pub.publish(depth)
        self.depth_info_pub.publish(preview_info)

    def _make_image(self, width: int, height: int, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = width * 3

        ground_width = 2.0 * self.altitude_agl * math.tan(self.fov * 0.5)
        gsd = ground_width / max(1, width)
        data = bytearray(width * height * 3)
        index = 0
        for py in range(height):
            north = self.north_m + (height * 0.5 - py) * gsd
            for px in range(width):
                east = self.east_m + (px - width * 0.5) * gsd
                r, g, b = self._satellite_rgb(east, north)
                data[index] = r
                data[index + 1] = g
                data[index + 2] = b
                index += 3
        msg.data = bytes(data)
        return msg

    def _make_depth(self, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = self.depth_height
        msg.width = self.depth_width
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = self.depth_width * 4
        ground_width = 2.0 * self.altitude_agl * math.tan(self.fov * 0.5)
        gsd = ground_width / max(1, self.depth_width)
        values = array("f")
        for py in range(self.depth_height):
            north = self.north_m + (self.depth_height * 0.5 - py) * gsd
            for px in range(self.depth_width):
                east = self.east_m + (px - self.depth_width * 0.5) * gsd
                terrain = self._terrain_height(east, north)
                values.append(max(1.0, self.altitude_agl - terrain))
        msg.data = values.tobytes()
        return msg

    def _satellite_rgb(self, east: float, north: float) -> tuple[int, int, int]:
        coast = north < -420.0 + 45.0 * math.sin(east / 210.0)
        road_a = abs((north + 0.22 * east + 35.0) % 180.0 - 90.0) < 4.5
        road_b = abs((east - 0.35 * north - 20.0) % 240.0 - 120.0) < 3.5
        trail = abs((north - 90.0 * math.sin(east / 180.0)) % 130.0 - 65.0) < 2.0
        parcel = (math.floor(east / 95.0) + math.floor(north / 85.0)) % 2
        noise = int((math.sin(east * 0.19 + north * 0.11) + 1.0) * 9.0)

        if coast:
            return 34, 82 + noise, 128 + noise
        if road_a or road_b:
            return 128 + noise, 124 + noise, 112 + noise
        if trail:
            return 132 + noise, 104 + noise, 76
        if parcel:
            return 84 + noise, 126 + noise, 72
        return 57 + noise, 108 + noise, 64

    @staticmethod
    def _terrain_height(east: float, north: float) -> float:
        ridge = 48.0 * math.exp(-((north - 0.18 * east - 120.0) / 420.0) ** 2)
        hill_shape = ((east + 240.0) / 380.0) ** 2
        hill_shape += ((north - 80.0) / 300.0) ** 2
        hill = 36.0 * math.exp(-hill_shape)
        coast_line = -420.0 + 45.0 * math.sin(east / 210.0)
        coast_drop = -22.0 if north < coast_line else 0.0
        texture = 4.0 * math.sin(east / 95.0) * math.cos(north / 120.0)
        return ridge + hill + coast_drop + texture

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
