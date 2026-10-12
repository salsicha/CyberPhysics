#!/usr/bin/env python3
"""Connect ArduPilot JSON SITL directly to Gazebo's physical rotor model."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu

sys.path.insert(0, "/workspace/applications/ardupilot_sitl/scripts")
from ardupilot_json import (  # noqa: E402
    ArduPilotJSON,
    body_velocity_to_enu,
    pwm_to_rotor_speed,
    state_from_enu,
)
from airframe_model import load_airframe


class GazeboArduPilotBackend(Node):
    def __init__(self):
        super().__init__("gazebo_ardupilot_backend")
        self.origin_lat = float(os.environ.get("INITIAL_LAT", "37.9234"))
        self.origin_lon = float(os.environ.get("INITIAL_LON", "-122.5967"))
        self.origin_alt = float(os.environ.get("ORIGIN_ALT", "781.0"))
        manifest_path = Path(os.environ.get(
            "TERRAIN_MANIFEST", "/data/navsim_assets/manifest.json"))
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.home_enu = manifest["home_enu_m"]
        self.maximum_rotor_speed = load_airframe()['max_rotor_speed_rad_s']
        self.pose = None
        self.twist = None
        self.imu = None
        # The ground-truth twist (as2 ground_truth_bridge, from Gazebo's
        # OdometryPublisher) is body-frame FLU. Rotate it to world ENU
        # before state_from_enu converts it to ArduPilot's NED frame.
        # Set false only for a replacement twist source already in ENU.
        self.twist_in_body_frame = os.environ.get(
            "GAZEBO_TWIST_IN_BODY_FRAME", "true").strip().lower() in ("1", "true", "yes")
        self.transport = ArduPilotJSON(
            port=int(os.environ.get("ARDUPILOT_JSON_PORT", "9002")))
        self.actuator_pub = self.create_publisher(
            Actuators,
            os.environ.get("GAZEBO_ACTUATOR_TOPIC", "/gz/aerodrone/cmd_actuators"),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            PoseStamped,
            os.environ.get("POSE_TOPIC", "/aerodrone/ground_truth/pose"),
            lambda msg: setattr(self, "pose", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            os.environ.get("TWIST_TOPIC", "/aerodrone/ground_truth/twist"),
            lambda msg: setattr(self, "twist", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            # Simulator-only Gazebo IMU (drone_bridges_isolated.py). Never the
            # platform's sensor_measurements/imu: that is ArduPilot's own IMU
            # output from MAVROS.
            os.environ.get("GAZEBO_IMU_TOPIC", "/aerodrone/ground_truth/imu"),
            lambda msg: setattr(self, "imu", msg),
            qos_profile_sensor_data,
        )
        self.create_timer(0.0025, self.tick)
        self.get_logger().info(
            "ArduPilot JSON -> Gazebo rotor physics on %s" % self.actuator_pub.topic_name)

    def tick(self):
        frame = self.transport.receive()
        if frame is None:
            return
        speeds = pwm_to_rotor_speed(frame.pwm[:4], maximum_speed=self.maximum_rotor_speed)
        command = Actuators()
        command.header.stamp = self.get_clock().now().to_msg()
        command.velocity = speeds.tolist()
        self.actuator_pub.publish(command)
        if self.pose is None or self.twist is None:
            return

        p = self.pose.pose.position
        q = self.pose.pose.orientation
        linear = self.twist.twist.linear
        angular = self.twist.twist.angular
        if self.imu is None:
            gyro = [angular.x, angular.y, angular.z]
            accel = [0.0, 0.0, 9.80665]
        else:
            gyro = [self.imu.angular_velocity.x, self.imu.angular_velocity.y,
                    self.imu.angular_velocity.z]
            accel = [self.imu.linear_acceleration.x, self.imu.linear_acceleration.y,
                     self.imu.linear_acceleration.z]
        velocity = [linear.x, linear.y, linear.z]
        if self.twist_in_body_frame:
            velocity = body_velocity_to_enu(velocity, [q.w, q.x, q.y, q.z])
        state = state_from_enu(
            [p.x, p.y, p.z],
            velocity,
            [q.w, q.x, q.y, q.z],
            gyro,
            accel,
            self.origin_lat,
            self.origin_lon,
            self.origin_alt,
            timestamp=self.pose.header.stamp.sec + self.pose.header.stamp.nanosec * 1e-9,
            reference_enu=self.home_enu,
        )
        self.transport.send(state)

    def destroy_node(self):
        self.transport.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = GazeboArduPilotBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
