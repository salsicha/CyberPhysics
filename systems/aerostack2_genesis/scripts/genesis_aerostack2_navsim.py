#!/usr/bin/env python3
import argparse
import math
import os
import time

import numpy as np

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Header


def env_float(name, default):
    return float(os.environ.get(name, default))


def env_int(name, default):
    return int(os.environ.get(name, default))


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(0.5 * yaw)
    q.z = math.sin(0.5 * yaw)
    return q


def latlon_from_offsets(origin_lat, origin_lon, north_m, east_m):
    lat = origin_lat + north_m / 111_320.0
    lon = origin_lon + east_m / (111_320.0 * math.cos(math.radians(origin_lat)))
    return lat, lon


def to_numpy(value):
    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    if hasattr(value, 'numpy'):
        value = value.numpy()
    return np.asarray(value, dtype=np.float64)


class GenesisNavSim(Node):
    def __init__(self, args):
        super().__init__('genesis_aerostack2_navsim')
        self.args = args
        self.cmd_vx = args.speed_mps
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_yaw_rate = 0.0
        self.last_stamp = self.get_clock().now()

        self.gps_pub = self.create_publisher(NavSatFix, args.gps_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, args.odom_topic, 10)
        self.imu_pub = self.create_publisher(Imu, args.imu_topic, 10)
        self.rgb_pub = self.create_publisher(Image, args.rgb_topic, 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, args.rgb_info_topic, 10)
        self.depth_pub = self.create_publisher(Image, args.depth_topic, 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, args.depth_info_topic, 10)

        self.get_logger().info(
            'Genesis headless navsim publishing gps=%s odom=%s rgb=%s depth=%s'
            % (args.gps_topic, args.odom_topic, args.rgb_topic, args.depth_topic)
        )

    def publish_all(self, pos, quat, vel, ang_vel):
        stamp = self.get_clock().now().to_msg()
        x, y, z = [float(v) for v in pos[:3]]
        vx, vy, vz = [float(v) for v in vel[:3]]
        wx, wy, wz = [float(v) for v in ang_vel[:3]]
        yaw = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                         1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]))
        lat, lon = latlon_from_offsets(self.args.lat, self.args.lon, y, x)
        altitude = self.args.origin_alt + z

        self._publish_gps(stamp, lat, lon, altitude)
        self._publish_odom(stamp, x, y, z, quat, vx, vy, vz, wx, wy, wz)
        self._publish_imu(stamp, quat, wx, wy, wz)
        self._publish_cameras(stamp, x, y, max(z, 0.1), yaw)

    def _publish_gps(self, stamp, lat, lon, altitude):
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = altitude
        msg.position_covariance = [0.8, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.5]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gps_pub.publish(msg)

    def _publish_odom(self, stamp, x, y, z, quat, vx, vy, vz, wx, wy, wz):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = float(quat[0])
        msg.pose.pose.orientation.x = float(quat[1])
        msg.pose.pose.orientation.y = float(quat[2])
        msg.pose.pose.orientation.z = float(quat[3])
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = vz
        msg.twist.twist.angular.x = wx
        msg.twist.twist.angular.y = wy
        msg.twist.twist.angular.z = wz
        self.odom_pub.publish(msg)

    def _publish_imu(self, stamp, quat, wx, wy, wz):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu'
        msg.orientation.w = float(quat[0])
        msg.orientation.x = float(quat[1])
        msg.orientation.y = float(quat[2])
        msg.orientation.z = float(quat[3])
        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz
        msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(msg)

    def _camera_info(self, stamp, frame_id, width, height):
        fx = fy = width / (2.0 * math.tan(math.radians(self.args.fov_deg) / 2.0))
        cx = width / 2.0
        cy = height / 2.0
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = width
        msg.height = height
        msg.distortion_model = 'plumb_bob'
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _publish_cameras(self, stamp, east, north, altitude, yaw):
        width = self.args.width
        height = self.args.height
        xs = np.linspace(-1.0, 1.0, width, dtype=np.float32)
        ys = np.linspace(-1.0, 1.0, height, dtype=np.float32)
        xx, yy = np.meshgrid(xs, ys)
        scale = max(altitude, 1.0) * math.tan(math.radians(self.args.fov_deg) / 2.0)
        world_x = east + scale * (math.cos(yaw) * xx - math.sin(yaw) * yy)
        world_y = north + scale * (math.sin(yaw) * xx + math.cos(yaw) * yy)
        terrain = 0.8 * np.sin(world_x * 0.08) + 0.6 * np.cos(world_y * 0.07)
        grid = (((np.floor(world_x / 8.0) + np.floor(world_y / 8.0)) % 2) * 70).astype(np.uint8)
        rgb = np.empty((height, width, 3), dtype=np.uint8)
        rgb[..., 0] = np.clip(80 + grid + 35 * np.sin(world_x * 0.11), 0, 255)
        rgb[..., 1] = np.clip(120 + 45 * np.cos(world_y * 0.09), 0, 255)
        rgb[..., 2] = np.clip(90 + 55 * np.sin((world_x + world_y) * 0.05), 0, 255)
        depth = np.maximum(0.1, altitude - terrain).astype(np.float32)

        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = 'downward_rgb_optical_frame'
        rgb_msg.height = height
        rgb_msg.width = width
        rgb_msg.encoding = 'rgb8'
        rgb_msg.is_bigendian = False
        rgb_msg.step = width * 3
        rgb_msg.data = rgb.tobytes()
        self.rgb_pub.publish(rgb_msg)
        self.rgb_info_pub.publish(self._camera_info(stamp, rgb_msg.header.frame_id, width, height))

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'downward_rgbd_optical_frame'
        depth_msg.height = height
        depth_msg.width = width
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = False
        depth_msg.step = width * 4
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(self._camera_info(stamp, depth_msg.header.frame_id, width, height))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', default=os.environ.get('GENESIS_BACKEND', 'cpu'), choices=['cpu', 'gpu'])
    parser.add_argument('--lat', type=float, default=env_float('INITIAL_LAT', 37.8044))
    parser.add_argument('--lon', type=float, default=env_float('INITIAL_LON', -122.4661))
    parser.add_argument('--origin-alt', type=float, default=env_float('ORIGIN_ALT', 0.0))
    parser.add_argument('--rate-hz', type=float, default=env_float('GENESIS_RATE_HZ', 20.0))
    parser.add_argument('--speed-mps', type=float, default=env_float('GENESIS_SPEED_MPS', 1.5))
    parser.add_argument('--radius-m', type=float, default=env_float('GENESIS_FLIGHT_RADIUS_M', 20.0))
    parser.add_argument('--altitude-m', type=float, default=env_float('GENESIS_FLIGHT_ALTITUDE_M', 2.0))
    parser.add_argument('--width', type=int, default=env_int('GENESIS_CAMERA_WIDTH', 640))
    parser.add_argument('--height', type=int, default=env_int('GENESIS_CAMERA_HEIGHT', 480))
    parser.add_argument('--fov-deg', type=float, default=env_float('FOV_DEG', 90.0))
    parser.add_argument('--gps-topic', default=os.environ.get('GPS_TOPIC', '/drone_sim_0/sensor_measurements/gps'))
    parser.add_argument('--odom-topic', default=os.environ.get('ODOM_TOPIC', '/drone_sim_0/sensor_measurements/odom'))
    parser.add_argument('--imu-topic', default=os.environ.get('IMU_TOPIC', '/drone_sim_0/sensor_measurements/imu'))
    parser.add_argument('--rgb-topic', default=os.environ.get('WILDNAV_IMAGE_TOPIC', '/drone_sim_0/sensor_measurements/downward_rgb/image_raw'))
    parser.add_argument('--rgb-info-topic', default=os.environ.get('WILDNAV_CAMERA_INFO_TOPIC', '/drone_sim_0/sensor_measurements/downward_rgb/camera_info'))
    parser.add_argument('--depth-topic', default=os.environ.get('DEPTH_TOPIC', '/drone_sim_0/sensor_measurements/downward_rgbd/depth'))
    parser.add_argument('--depth-info-topic', default=os.environ.get('CAMERA_INFO_TOPIC', '/drone_sim_0/sensor_measurements/downward_rgbd/depth/camera_info'))
    return parser.parse_args()


def main():
    args = parse_args()
    import genesis as gs

    rclpy.init()
    node = GenesisNavSim(args)
    backend = gs.cpu if args.backend == 'cpu' else gs.gpu
    gs.init(backend=backend)
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=1.0 / args.rate_hz, gravity=(0, 0, -9.81)),
        show_viewer=False,
    )
    scene.add_entity(gs.morphs.Plane())
    drone = scene.add_entity(
        morph=gs.morphs.Drone(file='urdf/drones/cf2x.urdf', pos=(args.radius_m, 0.0, args.altitude_m))
    )
    scene.build()

    base_rpm = 14468.429183500699
    dt = 1.0 / args.rate_hz
    tick = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            t = tick * dt
            omega = args.speed_mps / max(args.radius_m, 1.0)
            # Keep the drone airborne and gently perturbed so physics/state updates.
            rpm = base_rpm * (1.0 + 0.015 * math.sin(omega * t))
            if hasattr(drone, 'set_propellers_rpm'):
                drone.set_propellers_rpm([rpm, rpm, rpm, rpm])
            else:
                drone.set_propellels_rpm([rpm, rpm, rpm, rpm])
            scene.step()

            pos = to_numpy(drone.get_pos())
            quat = to_numpy(drone.get_quat())
            vel = to_numpy(drone.get_vel()) if hasattr(drone, 'get_vel') else np.zeros(3)
            ang = to_numpy(drone.get_ang()) if hasattr(drone, 'get_ang') else np.zeros(3)
            node.publish_all(pos, quat, vel, ang)
            tick += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
