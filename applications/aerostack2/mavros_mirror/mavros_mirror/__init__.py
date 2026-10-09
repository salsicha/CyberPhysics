"""Generic MAVROS mirror bridge shared by every ArduPilot platform.

Republishes the MAVROS telemetry topics under the vehicle's
``/<namespace>/sensor_measurements/*`` prefix that demnav, wildnav, and the
platform autonomy stacks subscribe to, with optional extras:

- ``mirror_pressure`` adds the ArduSub static-pressure mirror;
- ``enable_velocity_setpoints`` forwards Twist commands on
  ``cmd_vel_topic`` to MAVROS velocity setpoints (off by default).
  ``cmd_vel_frame`` says how the Twist is expressed:

  * ``local_enu`` (default): world-frame ENU velocity, forwarded to
    ``setpoint_velocity/cmd_vel`` (MAVLink LOCAL_NED).
  * ``body_flu``: vehicle-frame velocity (linear.x forward, linear.y left,
    linear.z up, angular.z yaw rate), the usual ``cmd_vel`` convention of
    the marine waypoint followers. Forwarded to ``setpoint_raw/local`` as a
    PositionTarget in FRAME_BODY_NED with yaw rate, which ArduSub and
    ArduRover rotate by their own heading. Sending such a Twist as ENU
    would make "forward" always mean "east".

One implementation replaces the previous per-platform copies
(blueboat_bridge.py, bluerov2_bridge.py, airplane_mavros_bridge.py), so a
QoS or topic fix lands on every platform at once. Installed into /venv of
the salsicha/cyberphysics:aerostack2-latest base image; run as ``mavros_mirror`` with a
``--params-file`` keyed ``/**:`` so the shared node name matches.
"""
from __future__ import annotations

from functools import partial

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, NavSatFix

try:  # Only needed for body-frame velocity setpoints.
    from mavros_msgs.msg import PositionTarget
except ImportError:  # pragma: no cover - mavros_msgs ships with MAVROS
    PositionTarget = None

CMD_VEL_FRAMES = ("local_enu", "body_flu")


def normalize_namespace(value: str) -> str:
    cleaned = value.strip("/")
    return f"/{cleaned}" if cleaned else ""


def default_cmd_vel_topic(namespace: str) -> str:
    return f"{normalize_namespace(namespace)}/cmd_vel"


def body_velocity_target(twist, target):
    """Fill a mavros_msgs/PositionTarget with a body-frame (FLU) velocity and
    yaw-rate setpoint. MAVROS converts FLU to MAVLink FRD for FRAME_BODY_NED
    and the yaw rate from ENU to NED."""
    target.coordinate_frame = target.FRAME_BODY_NED
    target.type_mask = (
        target.IGNORE_PX | target.IGNORE_PY | target.IGNORE_PZ
        | target.IGNORE_AFX | target.IGNORE_AFY | target.IGNORE_AFZ
        | target.IGNORE_YAW)
    target.velocity.x = float(twist.linear.x)
    target.velocity.y = float(twist.linear.y)
    target.velocity.z = float(twist.linear.z)
    target.yaw_rate = float(twist.angular.z)
    return target


class MavrosMirror(Node):
    def __init__(self) -> None:
        super().__init__("mavros_mirror")
        self.declare_parameter("namespace", "vehicle0")
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("mirror_pressure", False)
        self.declare_parameter("pressure_topic", "/mavros/imu/static_pressure")
        self.declare_parameter("enable_velocity_setpoints", False)
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter(
            "mavros_velocity_topic", "/mavros/setpoint_velocity/cmd_vel"
        )
        self.declare_parameter("velocity_frame_id", "map")
        self.declare_parameter("cmd_vel_frame", "local_enu")
        self.declare_parameter(
            "mavros_setpoint_raw_topic", "/mavros/setpoint_raw/local"
        )

        namespace = normalize_namespace(str(self.get_parameter("namespace").value))
        mavros_namespace = normalize_namespace(
            str(self.get_parameter("mavros_namespace").value)
        )
        output_prefix = f"{namespace}/sensor_measurements"

        self._mirror(
            NavSatFix,
            f"{mavros_namespace}/global_position/global",
            f"{output_prefix}/gps",
        )
        self._mirror(
            Odometry,
            f"{mavros_namespace}/local_position/odom",
            f"{output_prefix}/odom",
        )
        self._mirror(
            Imu,
            f"{mavros_namespace}/imu/data",
            f"{output_prefix}/imu",
        )
        self._mirror(
            BatteryState,
            f"{mavros_namespace}/battery",
            f"{output_prefix}/battery",
        )
        if bool(self.get_parameter("mirror_pressure").value):
            self._mirror(
                FluidPressure,
                str(self.get_parameter("pressure_topic").value),
                f"{output_prefix}/pressure",
            )

        self.velocity_frame_id = str(
            self.get_parameter("velocity_frame_id").value
        )
        self.velocity_pub = None
        self.cmd_vel_frame = str(self.get_parameter("cmd_vel_frame").value)
        if self.cmd_vel_frame not in CMD_VEL_FRAMES:
            raise ValueError(
                f"cmd_vel_frame must be one of {CMD_VEL_FRAMES}, "
                f"not {self.cmd_vel_frame!r}")
        if bool(self.get_parameter("enable_velocity_setpoints").value):
            cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
            if not cmd_vel_topic:
                cmd_vel_topic = default_cmd_vel_topic(namespace)
            if self.cmd_vel_frame == "body_flu":
                if PositionTarget is None:
                    raise RuntimeError("body_flu velocity needs mavros_msgs")
                self.velocity_pub = self.create_publisher(
                    PositionTarget,
                    str(self.get_parameter("mavros_setpoint_raw_topic").value),
                    10,
                )
            else:
                self.velocity_pub = self.create_publisher(
                    TwistStamped,
                    str(self.get_parameter("mavros_velocity_topic").value),
                    10,
                )
            self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel, 10)
            self.get_logger().warn(
                "Velocity setpoints are enabled. Verify autopilot mode, "
                "arming, failsafes, and manual override before operating."
            )

        self.get_logger().info(
            f"MAVROS mirror publishing telemetry under {output_prefix}"
        )

    def _mirror(self, msg_type, input_topic: str, output_topic: str) -> None:
        publisher = self.create_publisher(
            msg_type, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            msg_type,
            input_topic,
            partial(self._publish, publisher),
            qos_profile_sensor_data,
        )

    @staticmethod
    def _publish(publisher, msg) -> None:
        publisher.publish(msg)

    def _cmd_vel(self, msg: Twist) -> None:
        if self.velocity_pub is None:
            return
        if self.cmd_vel_frame == "body_flu":
            target = body_velocity_target(msg, PositionTarget())
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = "base_link"
            self.velocity_pub.publish(target)
            return
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.velocity_frame_id
        out.twist = msg
        self.velocity_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = MavrosMirror()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
