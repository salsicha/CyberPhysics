#!/usr/bin/env python3

from __future__ import annotations

import os
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError:  # pragma: no cover - depends on container build contents.
    serial = None


def env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


class TurretArduinoBridge(Node):
    def __init__(self) -> None:
        super().__init__("turret_arduino_bridge")
        self.declare_parameter("joint_command_topic", "/turret/joint_commands")
        self.declare_parameter("joint_state_topic", "/turret/joint_states")
        self.declare_parameter("diagnostics_topic", "/turret/diagnostics")
        self.declare_parameter(
            "serial_port", os.environ.get("TURRET_ARDUINO_PORT", "/dev/ttyACM0")
        )
        self.declare_parameter(
            "baud_rate", int(os.environ.get("TURRET_ARDUINO_BAUD", "115200"))
        )
        self.declare_parameter(
            "dry_run", env_bool("TURRET_ARDUINO_DRY_RUN", False)
        )
        self.declare_parameter("diagnostic_rate_hz", 1.0)
        # Defaults match the turret contract in
        # systems/turret/scenarios/warehouse_tracking.json: tilt is restricted
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
        self.last_command = JointState()
        self.last_error = ""
        self.last_write_time = time.time()
        self.connect_settle_s = 2.0
        self.reconnect_interval_s = 2.0
        self.serial_ready_time = 0.0
        self.last_connect_attempt = float("-inf")

        self.joint_state_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_state_topic").value), 10
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), 10
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_command_topic").value),
            self._joint_command,
            10,
        )

        rate_hz = max(0.2, float(self.get_parameter("diagnostic_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._publish_diagnostics)
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
            self.last_error = ""
        except Exception as exc:  # pragma: no cover - hardware dependent.
            self.serial_handle = None
            self.last_error = str(exc)

    def _joint_command(self, msg: JointState) -> None:
        pan = self._joint_position(msg, "pan_joint")
        tilt = self._joint_position(msg, "tilt_joint")
        if pan is None or tilt is None:
            self.last_error = "joint command must include pan_joint and tilt_joint"
            return

        self.last_command = msg
        pan = min(max(pan, self.pan_min_rad), self.pan_max_rad)
        tilt = min(max(tilt, self.tilt_min_rad), self.tilt_max_rad)
        line = f"SET pan_joint {pan:.6f} tilt_joint {tilt:.6f}\n"
        if not self.dry_run:
            self._connect_serial()
            if (
                self.serial_handle
                and self.serial_handle.is_open
                and time.monotonic() >= self.serial_ready_time
            ):
                try:
                    self.serial_handle.write(line.encode("ascii"))
                    self.serial_handle.flush()
                    self.last_error = ""
                except Exception as exc:  # pragma: no cover - hardware dependent.
                    self.last_error = str(exc)
                    self._close_serial()
        self.last_write_time = time.time()
        self._publish_joint_state(pan, tilt)

    def _joint_position(self, msg: JointState, name: str) -> Optional[float]:
        if name in msg.name:
            index = msg.name.index(name)
            if index < len(msg.position):
                return float(msg.position[index])
        return None

    def _publish_joint_state(self, pan: float, tilt: float) -> None:
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = ["pan_joint", "tilt_joint"]
        state.position = [pan, tilt]
        state.velocity = [0.0, 0.0]
        state.effort = [0.0, 0.0]
        self.joint_state_pub.publish(state)

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name = "turret_arduino_bridge"
        status.hardware_id = self.serial_port
        connected = bool(self.serial_handle and self.serial_handle.is_open)
        if self.dry_run:
            status.level = DiagnosticStatus.WARN
            status.message = "dry run"
        elif connected and not self.last_error:
            status.level = DiagnosticStatus.OK
            status.message = "serial connected"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = self.last_error or "serial disconnected"
        status.values = [
            KeyValue(key="port", value=self.serial_port),
            KeyValue(key="baud_rate", value=str(self.baud_rate)),
            KeyValue(key="connected", value=str(connected).lower()),
            KeyValue(
                key="last_write_age_s",
                value=f"{time.time() - self.last_write_time:.2f}",
            ),
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

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TurretArduinoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
