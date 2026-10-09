#!/usr/bin/env python3
"""Serial bridge between the PanTiltROS joint contract and turret firmware.

Line protocol (ASCII, newline terminated, radians):

  host -> firmware  ``SET pan_joint <pan> tilt_joint <tilt>``  move to position
  host -> firmware  ``STOP``                                    hold / stop motion
  firmware -> host  ``POS pan_joint <pan> tilt_joint <tilt>``  measured position

/turret/joint_states is published only from ``POS`` lines; commands are never
echoed back as joint states. Firmware that does not report positions leaves
the topic silent, and the tracker then relies on /turret/actuator_ready, which
is true only while commands are actually being written to the serial port.
"""

from __future__ import annotations

import math
import os
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    import serial
except ImportError:  # pragma: no cover - depends on container build contents.
    serial = None

JOINT_NAMES = ("pan_joint", "tilt_joint")
STOP_LINE = "STOP\n"
MAX_LINE_BYTES = 256


def env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def format_command(pan: float, tilt: float) -> str:
    return f"SET pan_joint {pan:.6f} tilt_joint {tilt:.6f}\n"


def parse_position_line(line: str) -> Optional[tuple[float, float]]:
    """Parse ``POS pan_joint <rad> tilt_joint <rad>``; None if malformed."""
    tokens = line.split()
    if len(tokens) != 5 or tokens[0] != "POS":
        return None
    values = {}
    for name, raw in zip(tokens[1::2], tokens[2::2]):
        try:
            value = float(raw)
        except ValueError:
            return None
        if not math.isfinite(value):
            return None
        values[name] = value
    if set(values) != set(JOINT_NAMES):
        return None
    return values["pan_joint"], values["tilt_joint"]


class TurretArduinoBridge(Node):
    def __init__(self) -> None:
        super().__init__("turret_arduino_bridge")
        self.declare_parameter("joint_command_topic", "/turret/joint_commands")
        self.declare_parameter("joint_state_topic", "/turret/joint_states")
        self.declare_parameter("diagnostics_topic", "/turret/diagnostics")
        self.declare_parameter("emergency_stop_topic", "/turret/emergency_stop")
        self.declare_parameter("actuator_ready_topic", "/turret/actuator_ready")
        self.declare_parameter(
            "serial_port", os.environ.get("TURRET_ARDUINO_PORT", "/dev/ttyACM0")
        )
        self.declare_parameter(
            "baud_rate", int(os.environ.get("TURRET_ARDUINO_BAUD", "115200"))
        )
        self.declare_parameter(
            "dry_run", env_bool("TURRET_ARDUINO_DRY_RUN", False)
        )
        # Nothing is written until an explicit emergency-stop release (False).
        self.declare_parameter("require_estop_release", True)
        self.declare_parameter("diagnostic_rate_hz", 1.0)
        self.declare_parameter("poll_rate_hz", 50.0)
        self.declare_parameter("ready_rate_hz", 5.0)
        # Defaults match the shared PanTiltROS turret contract: tilt is restricted
        # to keep the payload clear of the base.
        self.declare_parameter("pan_min_rad", -1.5708)
        self.declare_parameter("pan_max_rad", 1.5708)
        self.declare_parameter("tilt_min_rad", -0.65)
        self.declare_parameter("tilt_max_rad", 0.75)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.pan_min_rad = float(self.get_parameter("pan_min_rad").value)
        self.pan_max_rad = float(self.get_parameter("pan_max_rad").value)
        self.tilt_min_rad = float(self.get_parameter("tilt_min_rad").value)
        self.tilt_max_rad = float(self.get_parameter("tilt_max_rad").value)
        self.serial_handle = None
        self.last_error = ""
        self.last_write_time: Optional[float] = None
        self.last_feedback_time: Optional[float] = None
        self.connect_settle_s = 2.0
        self.reconnect_interval_s = 2.0
        self.serial_ready_time = 0.0
        self.last_connect_attempt = float("-inf")
        self.rx_buffer = b""
        self.estop_engaged = bool(self.get_parameter("require_estop_release").value)
        self.stop_sent = False
        self.dropped_commands = 0
        self.last_ready: Optional[bool] = None

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.joint_state_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_state_topic").value), 10
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), 10
        )
        self.ready_pub = self.create_publisher(
            Bool, str(self.get_parameter("actuator_ready_topic").value), latched_qos
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_command_topic").value),
            self._joint_command,
            10,
        )
        # Latched for transient-local publishers, volatile for default-QoS ones.
        for qos in (latched_qos, 10):
            self.create_subscription(
                Bool,
                str(self.get_parameter("emergency_stop_topic").value),
                self._emergency_stop,
                qos,
            )

        rate_hz = max(0.2, float(self.get_parameter("diagnostic_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._publish_diagnostics)
        poll_hz = max(1.0, float(self.get_parameter("poll_rate_hz").value))
        self.create_timer(1.0 / poll_hz, self._poll)
        ready_hz = max(1.0, float(self.get_parameter("ready_rate_hz").value))
        self.create_timer(1.0 / ready_hz, self._publish_ready)
        self._connect_serial()
        self.get_logger().info(
            "Turret Arduino bridge ready on "
            f"{self.serial_port} at {self.baud_rate} baud"
        )

    def _connect_serial(self) -> None:
        if self.dry_run:
            self.last_error = "dry_run enabled"
            return
        if serial is None:
            self.last_error = "pyserial is not installed in this image"
            return
        if self.serial_handle and self.serial_handle.is_open:
            return
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_interval_s:
            return
        self.last_connect_attempt = now
        try:
            self.serial_handle = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.05,
                write_timeout=0.1,
            )
            # Non-blocking settle: skip writes until the Arduino has rebooted.
            self.serial_ready_time = now + self.connect_settle_s
            self.rx_buffer = b""
            self.stop_sent = False
            self.last_error = ""
        except Exception as exc:  # pragma: no cover - hardware dependent.
            self.serial_handle = None
            self.last_error = str(exc)

    def _connected(self) -> bool:
        return bool(self.serial_handle and self.serial_handle.is_open)

    def _settling(self) -> bool:
        return self._connected() and time.monotonic() < self.serial_ready_time

    def actuator_ready(self) -> bool:
        """True only while SET commands would actually be written."""
        return (
            not self.dry_run
            and not self.estop_engaged
            and self._connected()
            and not self._settling()
        )

    def _write(self, line: str) -> bool:
        try:
            self.serial_handle.write(line.encode("ascii"))
            self.serial_handle.flush()
        except Exception as exc:  # pragma: no cover - hardware dependent.
            self.last_error = str(exc)
            self._close_serial()
            return False
        self.last_error = ""
        return True

    def _joint_command(self, msg: JointState) -> None:
        pan = self._joint_position(msg, "pan_joint")
        tilt = self._joint_position(msg, "tilt_joint")
        if pan is None or tilt is None:
            self.last_error = "joint command must include pan_joint and tilt_joint"
            return
        if not (math.isfinite(pan) and math.isfinite(tilt)):
            self.last_error = f"rejected non-finite joint command ({pan}, {tilt})"
            self.get_logger().warning(self.last_error, throttle_duration_sec=5.0)
            return
        if not self.actuator_ready():
            # Never queue: a command held through an outage or e-stop would be
            # executed as a full-speed jump when the link comes back.
            self.dropped_commands += 1
            return
        pan = min(max(pan, self.pan_min_rad), self.pan_max_rad)
        tilt = min(max(tilt, self.tilt_min_rad), self.tilt_max_rad)
        if self._write(format_command(pan, tilt)):
            self.last_write_time = time.monotonic()

    def _joint_position(self, msg: JointState, name: str) -> Optional[float]:
        if name in msg.name:
            index = msg.name.index(name)
            if index < len(msg.position):
                return float(msg.position[index])
        return None

    def _emergency_stop(self, msg: Bool) -> None:
        engaged = bool(msg.data)
        if engaged != self.estop_engaged:
            self.get_logger().warning(
                "emergency stop " + ("engaged" if engaged else "released")
            )
        self.estop_engaged = engaged
        if engaged:
            self.stop_sent = False
            self._send_stop_if_needed()
        self._publish_ready()

    def _send_stop_if_needed(self) -> None:
        if (
            self.estop_engaged
            and not self.stop_sent
            and not self.dry_run
            and self._connected()
            and not self._settling()
        ):
            self.stop_sent = self._write(STOP_LINE)

    def _poll(self) -> None:
        if self.dry_run:
            return
        self._connect_serial()
        if not self._connected():
            return
        try:
            waiting = self.serial_handle.in_waiting
            data = self.serial_handle.read(waiting) if waiting else b""
        except Exception as exc:  # pragma: no cover - hardware dependent.
            self.last_error = str(exc)
            self._close_serial()
            return
        self._handle_serial_data(data)
        self._send_stop_if_needed()
        if self.actuator_ready() != self.last_ready:
            self._publish_ready()

    def _handle_serial_data(self, data: bytes) -> None:
        self.rx_buffer += data
        *lines, self.rx_buffer = self.rx_buffer.split(b"\n")
        if len(self.rx_buffer) > MAX_LINE_BYTES:
            self.rx_buffer = b""
        for raw in lines:
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            position = parse_position_line(line)
            if position is not None:
                self._publish_joint_state(*position)
            elif line.startswith("ERR"):
                self.last_error = f"firmware: {line[3:].strip()}"

    def _publish_joint_state(self, pan: float, tilt: float) -> None:
        self.last_feedback_time = time.monotonic()
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(JOINT_NAMES)
        state.position = [pan, tilt]
        self.joint_state_pub.publish(state)

    def _publish_ready(self) -> None:
        ready = self.actuator_ready()
        self.last_ready = ready
        self.ready_pub.publish(Bool(data=ready))

    def _age(self, stamp: Optional[float]) -> str:
        return "never" if stamp is None else f"{time.monotonic() - stamp:.2f}"

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name = "turret_arduino_bridge"
        status.hardware_id = self.serial_port
        connected = self._connected()
        feedback = (
            self.last_feedback_time is not None
            and time.monotonic() - self.last_feedback_time < 1.0
        )
        if self.dry_run:
            status.level = DiagnosticStatus.WARN
            status.message = "dry run: commands are not sent"
        elif not connected or self.last_error:
            status.level = DiagnosticStatus.ERROR
            status.message = self.last_error or "serial disconnected"
        elif self.estop_engaged:
            status.level = DiagnosticStatus.WARN
            status.message = "emergency stop engaged (publish false to release)"
        elif self._settling():
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for the Arduino to reset"
        else:
            status.level = DiagnosticStatus.OK
            status.message = (
                "serial connected"
                if feedback
                else "serial connected, no position feedback from firmware"
            )
        status.values = [
            KeyValue(key="port", value=self.serial_port),
            KeyValue(key="baud_rate", value=str(self.baud_rate)),
            KeyValue(key="connected", value=str(connected).lower()),
            KeyValue(key="ready", value=str(self.actuator_ready()).lower()),
            KeyValue(key="emergency_stop", value=str(self.estop_engaged).lower()),
            KeyValue(key="position_feedback", value=str(feedback).lower()),
            KeyValue(key="last_feedback_age_s", value=self._age(self.last_feedback_time)),
            KeyValue(key="last_write_age_s", value=self._age(self.last_write_time)),
            KeyValue(key="dropped_commands", value=str(self.dropped_commands)),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diagnostics_pub.publish(msg)

    def _close_serial(self) -> None:
        if self.serial_handle:
            try:
                self.serial_handle.close()
            finally:
                self.serial_handle = None
        self.stop_sent = False

    def destroy_node(self) -> bool:
        if not self.dry_run and self._connected():
            self._write(STOP_LINE)
        self._close_serial()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TurretArduinoBridge()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
