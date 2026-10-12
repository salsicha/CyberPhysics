#!/usr/bin/env python3
"""Keep the autopilot sending the MAVLink messages a vehicle stack needs.

ArduPilot streams telemetry at its SRn_* stream rates, and on BlueOS's link
those of a fresh ArduPlane or ArduRover are about 1 Hz (measured on a Pi 5
Navigator: /mavros/imu/data at 1 Hz); MAVROS asks for nothing more. This
node asks for every message of a platform's rates file with
MAV_CMD_SET_MESSAGE_INTERVAL, through MAVROS's set_message_interval service
and the request function of the Aerodrone's flight_ready
(wait_for_mavros_navigation.py), and asks again whenever the autopilot may
have dropped the rates:

- MAVROS connects or reconnects: the stack started, the link came back, or
  the autopilot rebooted for longer than MAVROS's heartbeat timeout;
- the attitude stream (MAVROS imu/data, made from ATTITUDE or
  ATTITUDE_QUATERNION) runs below half its requested rate while MAVROS stays
  connected: an autopilot restart shorter than that timeout, or a ground
  station resetting the streams with REQUEST_DATA_STREAM. Repeated
  re-requests back off from --check-seconds up to MAX_CHECK_SECONDS.

ArduPilot keeps message intervals in RAM for the link a request came in on
(here BlueOS's router, so every router client gets them) and forgets them
on reboot. They are not parameters: this node writes no parameter, changes
no mode and never arms; set_message_interval is the only service it calls.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
import yaml
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from wait_for_mavros_navigation import request_message_interval

# MAVLink (common) messages a rates file may name; any other by its id.
MESSAGE_IDS = {
    "SYS_STATUS": 1,
    "GPS_RAW_INT": 24,
    "RAW_IMU": 27,
    "SCALED_PRESSURE": 29,
    "ATTITUDE": 30,
    "ATTITUDE_QUATERNION": 31,
    "LOCAL_POSITION_NED": 32,
    "GLOBAL_POSITION_INT": 33,
    "VFR_HUD": 74,
    "HIGHRES_IMU": 105,
    "DISTANCE_SENSOR": 132,
    "SCALED_PRESSURE2": 137,
    "BATTERY_STATUS": 147,
    "EXTENDED_SYS_STATE": 245,
}
MESSAGE_NAMES = {message_id: name for name, message_id in MESSAGE_IDS.items()}
# MAVROS 2 publishes imu/data from ATTITUDE_QUATERNION once it has seen one
# and from ATTITUDE until then.
ATTITUDE_IDS = (MESSAGE_IDS["ATTITUDE"], MESSAGE_IDS["ATTITUDE_QUATERNION"])
# A sanity limit, not an autopilot one: a larger value is more likely an
# interval typed in milliseconds than a rate these stacks need.
MAX_RATE_HZ = 50.0
MAX_CHECK_SECONDS = 60.0


def message_name(identifier):
    return MESSAGE_NAMES.get(identifier, f"message {identifier}")


def message_id(key):
    """MAVLink id of a rates-file key: a name in MESSAGE_IDS or an id."""
    if isinstance(key, int) and not isinstance(key, bool):
        return key
    text = str(key).strip()
    if text.isdigit():
        return int(text)
    if text.upper() not in MESSAGE_IDS:
        raise ValueError(
            f"unknown MAVLink message {key!r}: give its numeric id or one of "
            + ", ".join(sorted(MESSAGE_IDS)))
    return MESSAGE_IDS[text.upper()]


def parse_rates(data, source="rates"):
    """{MAVLink id: Hz} from a rates file's `message_rates_hz` mapping."""
    entries = (data or {}).get("message_rates_hz") or {}
    if not isinstance(entries, dict) or not entries:
        raise ValueError(f"{source}: no message_rates_hz mapping of message: Hz")
    rates = {}
    for key, value in entries.items():
        identifier = message_id(key)
        rate = float(value)
        if not (math.isfinite(rate) and 0.0 < rate <= MAX_RATE_HZ):
            raise ValueError(
                f"{source}: {key} at {value} Hz; rates must be in (0, {MAX_RATE_HZ:g}] Hz")
        if identifier in rates:
            raise ValueError(f"{source}: {message_name(identifier)} is listed twice")
        rates[identifier] = rate
    return rates


def load_rates(path):
    with open(path, encoding="utf-8") as stream:
        return parse_rates(yaml.safe_load(stream), path)


def describe(rates):
    return ", ".join(f"{message_name(identifier)} {rate:g} Hz"
                     for identifier, rate in rates.items())


class MessageRateKeeper(Node):
    """Requests the rates on (re)connection and when the attitude stream
    shows the autopilot dropped them. Call step() from the main loop, not a
    callback: each request spins the node until MAVROS answers."""

    def __init__(self, namespace, rates, check_seconds=5.0, retry_seconds=2.0,
                 max_attempts=3, min_fraction=0.5):
        super().__init__("mavros_message_rates")
        self.rates = dict(rates)
        self.check_seconds = float(check_seconds)
        self.retry_seconds = float(retry_seconds)
        self.max_attempts = int(max_attempts)
        self.min_fraction = float(min_fraction)
        self.connected = False
        self.pending = {}       # message id -> failed attempts, still to request
        self.refused = set()    # given up on until the next request_all()
        self.next_attempt = 0.0
        self.window_start = None
        self.imu_count = 0
        self.backoff = 1.0
        self.confirmed = False
        self.create_subscription(State, f"{namespace}/state", self._state, 10)
        self.create_subscription(
            Imu, f"{namespace}/imu/data", self._imu, qos_profile_sensor_data)
        self.interval_client = self.create_client(
            MessageInterval, f"{namespace}/set_message_interval")

    def _state(self, message):
        connected = bool(message.connected)
        if connected and not self.connected:
            self.backoff = 1.0
            self.request_all("MAVROS connected to the autopilot")
        elif self.connected and not connected:
            self.get_logger().warning(
                "MAVROS lost the autopilot; requesting the message rates again "
                "when it reconnects")
        self.connected = connected

    def _imu(self, message):
        self.imu_count += 1

    def attitude_rate_hz(self):
        """The imu/data rate the requests should give (0: not watched)."""
        return max((self.rates[identifier] for identifier in ATTITUDE_IDS
                    if identifier in self.rates and identifier not in self.refused),
                   default=0.0)

    def request_all(self, reason):
        self.refused.clear()
        self.pending = {identifier: 0 for identifier in self.rates}
        self.get_logger().info(f"{reason}: requesting {describe(self.rates)}")
        self.next_attempt = 0.0
        self.window_start = None
        self.confirmed = False

    def step(self, now):
        if not self.connected:
            self.window_start = None
            return
        if self.pending:
            self._request_pending(now)
        else:
            self._check_attitude_rate(now)

    def _request_pending(self, now):
        if now < self.next_attempt or not self.interval_client.service_is_ready():
            return
        for identifier in list(self.pending):
            if not self.connected:
                return  # lost while asking (each request spins the node)
            if request_message_interval(
                    self, self.interval_client, identifier, self.rates[identifier]):
                del self.pending[identifier]
                continue
            self.pending[identifier] += 1
            if self.pending[identifier] >= self.max_attempts:
                del self.pending[identifier]
                self.refused.add(identifier)
                self.get_logger().warning(
                    f"The autopilot did not accept {message_name(identifier)} at "
                    f"{self.rates[identifier]:g} Hz after {self.max_attempts} attempts; "
                    "not asking again until the rates are requested anew")
        if self.pending:
            # Seen once right after each autopilot restart on the Pi 5
            # Navigator: the first request, sent on the first heartbeat, went
            # unanswered; the next one was accepted.
            self.next_attempt = now + self.retry_seconds
            self.get_logger().info(
                "Not accepted yet: "
                + ", ".join(message_name(identifier) for identifier in self.pending)
                + f"; asking again in {self.retry_seconds:g} s")

    def _check_attitude_rate(self, now):
        expected = self.attitude_rate_hz()
        if expected <= 0.0:
            return
        if self.window_start is None:
            self.window_start, self.imu_count = now, 0
            return
        elapsed = now - self.window_start
        if elapsed < self.check_seconds * self.backoff:
            return
        measured = self.imu_count / elapsed
        self.window_start, self.imu_count = now, 0
        if measured >= self.min_fraction * expected:
            if not self.confirmed:
                self.get_logger().info(
                    f"imu/data at {measured:.1f} Hz (requested {expected:g} Hz)")
            self.confirmed = True
            self.backoff = 1.0
            return
        self.backoff = min(2.0 * self.backoff, MAX_CHECK_SECONDS / self.check_seconds)
        self.get_logger().warning(
            f"imu/data at {measured:.1f} Hz, below the requested {expected:g} Hz "
            "(autopilot restarted, or a ground station reset its streams)")
        self.request_all("Message rates lost")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--rates", required=True,
        help="YAML file whose message_rates_hz maps MAVLink message names or ids to Hz")
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument(
        "--check-seconds", type=float, default=5.0,
        help="window over which the imu/data rate is checked")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rates = load_rates(args.rates)
    rclpy.init()
    node = MessageRateKeeper(args.namespace.rstrip("/"), rates,
                             check_seconds=args.check_seconds)
    node.get_logger().info(f"Keeping MAVLink message rates: {describe(rates)}")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            node.step(time.monotonic())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # SIGINT, or docker stop's SIGTERM (rclpy handles both)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
