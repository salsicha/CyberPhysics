"""Generic MAVROS mirror bridge shared by every ArduPilot platform.

Republishes the MAVROS telemetry topics under the vehicle's
``/<namespace>/sensor_measurements/*`` prefix that demnav, wildnav, and the
platform autonomy stacks subscribe to, with optional extras:

- ``mirror_pressure`` adds the ArduSub pressure topic,
  ``sensor_measurements/pressure``. ``pressure_source`` says where it
  comes from:

  * ``pressure_topic`` (default): mirror ``pressure_topic``, MAVROS's
    ``imu/static_pressure``. MAVROS 2.14 makes that from SCALED_PRESSURE,
    the autopilot's barometer 0 only.
  * ``ardusub_depth_sensor``: the water pressure of ArduSub's external depth
    sensor. On a Navigator the board's own barometer is barometer 0 and a
    Bar30 or Bar100 the next one, which ArduSub sends as SCALED_PRESSURE2
    (``depth_sensor_message``; SCALED_PRESSURE3 on a board with two
    barometers of its own) and MAVROS publishes on no topic: it is decoded
    from MAVROS's raw MAVLink stream (``mavlink_source_topic``). It is used
    while ArduSub's SYS_STATUS does not deny having a depth sensor;
    otherwise, or when no such message has arrived for
    ``depth_sensor_timeout_s``, barometer 0 is mirrored with a warning
    (DepthPressureSelector).
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

import struct
import time
from functools import partial

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, NavSatFix

try:  # Only needed for body-frame velocity setpoints and the depth sensor.
    from mavros_msgs.msg import Mavlink, PositionTarget, SysStatus
except ImportError:  # pragma: no cover - mavros_msgs ships with MAVROS
    Mavlink = PositionTarget = SysStatus = None

CMD_VEL_FRAMES = ("local_enu", "body_flu")
PRESSURE_SOURCES = ("pressure_topic", "ardusub_depth_sensor")
# ArduPilot sends barometer n as SCALED_PRESSURE (n = 0, the one MAVROS
# publishes), SCALED_PRESSURE2 (1) and SCALED_PRESSURE3 (2; ArduSub sends a
# temperature sensor's reading there instead when it has one). One payload
# layout.
DEPTH_SENSOR_MESSAGES = {"SCALED_PRESSURE2": (137, 1), "SCALED_PRESSURE3": (143, 2)}
# ArduSub sets this SYS_STATUS sensor bit only for a water-pressure (depth)
# sensor (GCS_Sub.cpp), not for the board's own barometer.
MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 0x08
MAVLINK_FRAMING_OK = 1
# MAVROS's router publishes every MAVLink message best-effort to its UAS
# (/uas<tgt_system>/mavlink_source); sys_status is latched, reliable.
MAVLINK_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
LATCHED_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


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


def mavlink_payload(message) -> bytes:
    """Payload bytes of a mavros_msgs/Mavlink message: MAVROS copies
    mavlink_message_t's payload into uint64 words (mavlink_convert.hpp),
    `len` bytes of them used. MAVLink 2 drops trailing zero bytes, so it can
    be shorter than the message's full payload."""
    words = list(message.payload64)
    return struct.pack(f"<{len(words)}Q", *words)[: int(message.len)]


def decode_scaled_pressure(payload: bytes):
    """(time_boot_ms, press_abs hPa, press_diff hPa, temperature cdegC) of a
    SCALED_PRESSURE, SCALED_PRESSURE2 or SCALED_PRESSURE3 payload (MAVLink
    common.xml: uint32, float, float, int16, then the int16
    temperature_press_diff extension, not needed here)."""
    return struct.unpack_from("<Iffh", payload.ljust(14, b"\0"))


class DepthPressureSelector:
    """Which barometer feeds the submarine's pressure topic.

    ArduSub picks the first water-pressure barometer as its depth sensor
    and primary barometer (Sub::init_ardupilot): on a Navigator, whose own
    BMP280/BMP388 is barometer 0, a Bar30 or Bar100 on the I2C6 header
    (BARO_EXT_BUS 6) is barometer 1, sent as SCALED_PRESSURE2. Barometer 0
    (MAVROS's imu/static_pressure) is then the air inside the enclosure.
    ArduSub's SYS_STATUS reports an absolute-pressure sensor only when it
    has such a depth sensor.

    The external message is used while it keeps arriving (within
    `timeout_s`) and SYS_STATUS does not deny a depth sensor. Otherwise
    barometer 0 is used, once `timeout_s` has passed since its first
    message without an external one (or since the last external one), and
    the reason is logged: a warning, unless ArduSub says it has
    a depth sensor and no external message has arrived at all, in which case
    barometer 0 is that sensor (ArduSub SITL, whose barometers are all water
    sensors, or a board without one of its own). select() returns the
    source and a (level, text) to log when the choice or what ArduSub
    reported changed.
    """

    EXTERNAL = "external"
    BAROMETER0 = "barometer0"

    def __init__(self, message_name="SCALED_PRESSURE2", timeout_s=5.0,
                 fallback="/mavros/imu/static_pressure"):
        if message_name not in DEPTH_SENSOR_MESSAGES:
            raise ValueError(
                f"depth_sensor_message must be one of {sorted(DEPTH_SENSOR_MESSAGES)}, "
                f"not {message_name!r}")
        self.message_name = message_name
        self.message_id, self.instance = DEPTH_SENSOR_MESSAGES[message_name]
        self.timeout_s = float(timeout_s)
        self.fallback = fallback
        self.depth_sensor = None    # SYS_STATUS: True, False or not seen yet
        self.last_external = None   # time of the latest external message
        self.first_barometer0 = None
        self.source = None
        self.reported = None

    def sys_status(self, sensors_present):
        self.depth_sensor = bool(int(sensors_present) & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)

    def external(self, now):
        self.last_external = now
        return self.select(now)

    def barometer0(self, now):
        if self.first_barometer0 is None:
            self.first_barometer0 = now
        return self.select(now)

    def select(self, now):
        fresh = (self.last_external is not None
                 and now - self.last_external <= self.timeout_s)
        if fresh and self.depth_sensor is not False:
            source = self.EXTERNAL
        elif self.first_barometer0 is None or (
                self.source is None and now - self.first_barometer0 < self.timeout_s):
            return self.source, None  # still waiting to hear from the depth sensor
        else:
            source = self.BAROMETER0
        previous, self.source = self.source, source
        state = (source, self.depth_sensor)
        if state == self.reported:
            return source, None
        self.reported = state
        return source, self._report(source, previous, fresh)

    def _report(self, source, previous, fresh):
        name = f"{self.message_name} (barometer {self.instance})"
        if source == self.EXTERNAL:
            known = ("ArduSub reports a depth sensor" if self.depth_sensor
                     else "ArduSub has not reported its sensors yet")
            return "info", f"Pressure from the external depth sensor, {name}: {known}"
        if fresh:
            why = f"{name} arrives"
        elif previous == self.EXTERNAL:
            why = f"{name} stopped for {self.timeout_s:g} s"
        else:
            why = f"no {name} in {self.timeout_s:g} s"
        target = f"publishing barometer 0 ({self.fallback})"
        if self.depth_sensor and self.last_external is None:
            return "info", (f"{why}; ArduSub reports a depth sensor, so barometer 0 "
                            f"is it: {target}")
        if self.depth_sensor is False:
            joiner = ", but" if fresh else ";"
            return "warning", (f"{why}{joiner} ArduSub's SYS_STATUS reports no depth "
                               f"sensor: {target}, the autopilot's own barometer (air "
                               "pressure inside the enclosure), not water depth")
        # No SYS_STATUS yet, or the depth sensor's messages stopped.
        known = ("no SYS_STATUS from ArduSub yet" if self.depth_sensor is None
                 else "ArduSub reports a depth sensor")
        return "warning", (f"{why} ({known}): {target}, which may be the autopilot's "
                           "own barometer (air pressure inside the enclosure), not "
                           "depth")


class MavrosMirror(Node):
    def __init__(self) -> None:
        super().__init__("mavros_mirror")
        self.declare_parameter("namespace", "vehicle0")
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("mirror_pressure", False)
        self.declare_parameter("pressure_topic", "/mavros/imu/static_pressure")
        self.declare_parameter("pressure_source", "pressure_topic")
        self.declare_parameter("depth_sensor_message", "SCALED_PRESSURE2")
        self.declare_parameter("depth_sensor_timeout_s", 5.0)
        self.declare_parameter("mavlink_source_topic", "/uas1/mavlink_source")
        self.declare_parameter("sys_status_topic", "/mavros/sys_status")
        self.declare_parameter("autopilot_system_id", 1)
        self.declare_parameter("autopilot_component_id", 1)
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
        self.depth_selector = None
        if bool(self.get_parameter("mirror_pressure").value):
            pressure_source = str(self.get_parameter("pressure_source").value)
            if pressure_source not in PRESSURE_SOURCES:
                raise ValueError(
                    f"pressure_source must be one of {PRESSURE_SOURCES}, "
                    f"not {pressure_source!r}")
            if pressure_source == "pressure_topic":
                self._mirror(
                    FluidPressure,
                    str(self.get_parameter("pressure_topic").value),
                    f"{output_prefix}/pressure",
                )
            else:
                self._depth_sensor_pressure(f"{output_prefix}/pressure")

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

    def _depth_sensor_pressure(self, output_topic: str) -> None:
        """pressure_source ardusub_depth_sensor: publish the external depth
        sensor's message, or barometer 0 (pressure_topic) when there is none
        (DepthPressureSelector). Only subscribes: nothing goes to MAVROS."""
        if Mavlink is None or SysStatus is None:
            raise RuntimeError("pressure_source ardusub_depth_sensor needs mavros_msgs")
        fallback = str(self.get_parameter("pressure_topic").value)
        self.depth_selector = DepthPressureSelector(
            str(self.get_parameter("depth_sensor_message").value).strip().upper(),
            float(self.get_parameter("depth_sensor_timeout_s").value),
            fallback)
        self.autopilot_ids = (int(self.get_parameter("autopilot_system_id").value),
                              int(self.get_parameter("autopilot_component_id").value))
        # MAVROS stamps its pressure topic in its imu frame ("base_link").
        self.pressure_frame_id = "base_link"
        self.pressure_pub = self.create_publisher(
            FluidPressure, output_topic, qos_profile_sensor_data)
        self.create_subscription(
            FluidPressure, fallback, self._barometer0, qos_profile_sensor_data)
        self.create_subscription(
            Mavlink, str(self.get_parameter("mavlink_source_topic").value),
            self._mavlink, MAVLINK_QOS)
        self.create_subscription(
            SysStatus, str(self.get_parameter("sys_status_topic").value),
            self._sys_status, LATCHED_QOS)
        self.get_logger().info(
            f"Pressure: ArduSub's external depth sensor "
            f"({self.depth_selector.message_name} from "
            f"{self.get_parameter('mavlink_source_topic').value}), else {fallback}")

    def _select(self, choice):
        source, report = choice
        if report:
            level, text = report
            # One logging call site per severity: rclpy refuses a second
            # severity from the same line ("Logger severity cannot be
            # changed between calls", seen on the Pi).
            if level == "warning":
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
        return source

    def _barometer0(self, msg) -> None:
        self.pressure_frame_id = msg.header.frame_id or self.pressure_frame_id
        if self._select(self.depth_selector.barometer0(time.monotonic())) == \
                DepthPressureSelector.BAROMETER0:
            self.pressure_pub.publish(msg)

    def _sys_status(self, msg) -> None:
        self.depth_selector.sys_status(msg.sensors_present)
        self._select(self.depth_selector.select(time.monotonic()))

    def _mavlink(self, msg) -> None:
        if (msg.msgid != self.depth_selector.message_id
                or (msg.sysid, msg.compid) != self.autopilot_ids
                or msg.framing_status != MAVLINK_FRAMING_OK):
            return
        _, press_abs_hpa, _, _ = decode_scaled_pressure(mavlink_payload(msg))
        if self._select(self.depth_selector.external(time.monotonic())) != \
                DepthPressureSelector.EXTERNAL:
            return
        out = FluidPressure()
        # The router's receive time (mavros_router.cpp); FluidPressure in Pa.
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.pressure_frame_id
        out.fluid_pressure = float(press_abs_hpa) * 100.0
        out.variance = 0.0
        self.pressure_pub.publish(out)

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
