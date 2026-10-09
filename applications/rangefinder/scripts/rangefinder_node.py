#!/usr/bin/env python3
"""Benewake TFmini-S / TF-Luna UART rangefinder -> sensor_msgs/Range.

Reads the sensor's standard 9-byte frames from a serial port and publishes
every frame on `topic` (default /aerodrone/rangefinder, the downward
height-above-ground input DemNav reads). Readings follow REP 117: a finite
range when valid, +inf for no return or beyond max_range, -inf below
min_range, NaN for saturated/erroneous readings (`publish_invalid:=false`
drops those instead). The node reopens the port after serial errors and
warns when no frame has arrived for `stale_timeout_s`.

The beam is the +x axis of `frame_id` (rangefinder_link, as in the
simulator); mounted downward that frame is base_link pitched +90 deg.
"""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from benewake import MODELS, OK, FrameParser, interpret


def open_serial(port, baud_rate):
    import serial  # pyserial; imported here so tests need no serial module
    # timeout=0: non-blocking reads from the poll timer. exclusive: refuse a
    # port another process (e.g. a second driver) already holds.
    return serial.Serial(port, baud_rate, timeout=0, exclusive=True)


class BenewakeRangefinder(Node):

    def __init__(self, serial_factory=open_serial):
        super().__init__('rangefinder')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('model', 'tfmini_s')
        self.declare_parameter('topic', '/aerodrone/rangefinder')
        self.declare_parameter('frame_id', 'rangefinder_link')
        # 0 selects the datasheet value of `model`.
        self.declare_parameter('min_range', 0.0)
        self.declare_parameter('max_range', 0.0)
        self.declare_parameter('field_of_view', 0.0)
        self.declare_parameter('publish_invalid', True)
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('reconnect_period_s', 1.0)
        self.declare_parameter('poll_period_s', 0.005)

        model = str(self.get_parameter('model').value).lower().replace('-', '_')
        if model not in MODELS:
            raise ValueError(
                f'Unknown rangefinder model {model!r}; expected one of {sorted(MODELS)}')
        spec = MODELS[model]
        self.model = model
        self.ambient_strength = spec.ambient_strength
        self.port_name = str(self.get_parameter('port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.min_range = float(self.get_parameter('min_range').value) or spec.min_range_m
        self.max_range = float(self.get_parameter('max_range').value) or spec.max_range_m
        self.field_of_view = (float(self.get_parameter('field_of_view').value) or
                              spec.field_of_view_rad)
        if not 0.0 <= self.min_range < self.max_range:
            raise ValueError(f'Invalid range limits {self.min_range}..{self.max_range} m')
        self.publish_invalid = bool(self.get_parameter('publish_invalid').value)
        self.stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        self.reconnect_period_s = float(self.get_parameter('reconnect_period_s').value)

        self._serial_factory = serial_factory
        self._serial = None
        self._next_open_time = 0.0
        self._opened_time = None
        self._last_frame_time = None
        self._stale = False
        self._last_status = None
        self.parser = FrameParser()

        topic = str(self.get_parameter('topic').value)
        self.publisher = self.create_publisher(Range, topic, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter('poll_period_s').value), self._poll)
        self.create_timer(0.25, self._check_stale)
        self.get_logger().info(
            f'{model} on {self.port_name} @ {self.baud_rate} baud -> {topic} '
            f'[{self.frame_id}, {self.min_range:.2f}-{self.max_range:.2f} m, '
            f'FOV {math.degrees(self.field_of_view):.1f} deg]')

    # Serial port ---------------------------------------------------------
    def _open(self, now):
        self._next_open_time = now + self.reconnect_period_s
        try:
            self._serial = self._serial_factory(self.port_name, self.baud_rate)
        except (OSError, ValueError) as exc:  # serial.SerialException is an OSError
            self.get_logger().warn(
                f'Cannot open {self.port_name}: {exc}; retrying every '
                f'{self.reconnect_period_s:.1f} s', throttle_duration_sec=10.0)
            return
        self.parser.reset()
        self._opened_time = now
        self._stale = False
        self.get_logger().info(f'Opened {self.port_name}')

    def close(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except OSError:
                pass
            self._serial = None

    def _poll(self):
        now = time.monotonic()
        if self._serial is None:
            if now >= self._next_open_time:
                self._open(now)
            return
        try:
            data = self._serial.read(self._serial.in_waiting or 1)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Serial error on {self.port_name}: {exc}; reconnecting')
            self.close()
            self._next_open_time = now + self.reconnect_period_s
            return
        if data:
            frames = self.parser.feed(data)
            if frames:
                self._last_frame_time = now
                for frame in frames:
                    self._publish(frame)

    # Publishing ----------------------------------------------------------
    def make_range(self, frame):
        """(Range message, status) for one frame."""
        value, status = interpret(frame, self.min_range, self.max_range,
                                  self.ambient_strength)
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = float(self.field_of_view)
        msg.min_range = float(self.min_range)
        msg.max_range = float(self.max_range)
        msg.range = float(value)
        return msg, status

    def _publish(self, frame):
        msg, status = self.make_range(frame)
        if status != self._last_status:
            if status != OK:
                self.get_logger().info(
                    f'Reading {status} (dist {frame.distance_cm} cm, strength '
                    f'{frame.strength}) -> range {msg.range}',
                    throttle_duration_sec=5.0)
            self._last_status = status
        if status != OK and not self.publish_invalid:
            return
        self.publisher.publish(msg)

    def _check_stale(self):
        if self._serial is None:
            return
        now = time.monotonic()
        last = self._last_frame_time
        if last is None or (self._opened_time is not None and last < self._opened_time):
            last = self._opened_time
        age = now - last
        if age > self.stale_timeout_s:
            self._stale = True
            self.get_logger().warn(
                f'No valid rangefinder frame from {self.port_name} for {age:.1f} s '
                f'(checksum errors {self.parser.checksum_errors}, discarded bytes '
                f'{self.parser.discarded_bytes}); check wiring, baud rate and that '
                f'the sensor outputs the 9-byte cm format', throttle_duration_sec=5.0)
        elif self._stale:
            self._stale = False
            self.get_logger().info('Rangefinder data resumed')


def main(args=None):
    rclpy.init(args=args)
    node = BenewakeRangefinder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
