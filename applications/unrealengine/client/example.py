#!/usr/bin/env python3
"""Minimal ROS 2 client for an Unreal Engine project.

Subscribes to a camera image and a pose topic published by the UE project
(natively through rclUE, or through rosbridge with ROSIntegration), logs
their frame rate and latency, and publishes a geometry_msgs/Twist command at
a fixed rate. Topic names depend on the UE project, so they are parameters:

    python3 example.py --ros-args -p image_topic:=/robot/camera/image_raw \\
        -p pose_topic:=/robot/odom -p cmd_topic:=/robot/cmd_vel -p linear_x:=0.2

Parameters:
    image_topic      sensor_msgs/Image topic        (/ue/camera/image_raw)
    pose_topic       pose topic                     (/ue/odom)
    pose_type        'odometry' (nav_msgs/Odometry) or
                     'pose' (geometry_msgs/PoseStamped)   (odometry)
    cmd_topic        geometry_msgs/Twist topic      (/ue/cmd_vel)
    cmd_rate_hz      command rate, 0 disables it    (10.0)
    linear_x         commanded forward speed, m/s   (0.0)
    angular_z        commanded yaw rate, rad/s      (0.0)
    report_period_s  statistics log period          (2.0)
    run_duration_s   exit after this long, 0 = run until stopped (0.0)

Latency is this node's clock minus the message header stamp, so it is only
meaningful when the UE project stamps messages with the same clock: wall
time, or simulation time published on /clock with use_sim_time:=true here.
The client publishes one zero command when it exits.
"""
import math
import signal

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

POSE_TYPES = ('odometry', 'pose')


def stamp_to_s(stamp):
    """builtin_interfaces/Time to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class StreamStats:
    """Message rate and header-stamp latency over one reporting period.

    A zero stamp (header left unset) counts as a message without a latency
    sample. A mean latency beyond max_plausible_latency_s means the stamps
    are on a different clock than this node (e.g. UE game time against wall
    time), so it is flagged instead of being reported as a delay.
    """

    def __init__(self, name, now_s, max_plausible_latency_s=10.0):
        self.name = name
        self.max_plausible_latency_s = max_plausible_latency_s
        self.total = 0
        self.reset(now_s)

    def reset(self, now_s):
        self.period_start_s = now_s
        self.count = 0
        self.unstamped = 0
        self.latency_count = 0
        self.latency_sum_s = 0.0
        self.latency_min_s = math.inf
        self.latency_max_s = -math.inf

    def record(self, now_s, stamp_s=None):
        self.count += 1
        self.total += 1
        if not stamp_s:
            self.unstamped += 1
            return
        latency = now_s - stamp_s
        self.latency_count += 1
        self.latency_sum_s += latency
        self.latency_min_s = min(self.latency_min_s, latency)
        self.latency_max_s = max(self.latency_max_s, latency)

    def report(self, now_s):
        """Summary of the period ending at now_s; starts a new period."""
        elapsed = now_s - self.period_start_s
        summary = {
            'name': self.name,
            'period_s': elapsed,
            'count': self.count,
            'total': self.total,
            'rate_hz': self.count / elapsed if elapsed > 0 else 0.0,
            'unstamped': self.unstamped,
            'latency_mean_s': None,
            'latency_min_s': None,
            'latency_max_s': None,
            'clock_mismatch': False,
        }
        if self.latency_count:
            mean = self.latency_sum_s / self.latency_count
            summary.update(latency_mean_s=mean,
                           latency_min_s=self.latency_min_s,
                           latency_max_s=self.latency_max_s,
                           clock_mismatch=abs(mean) > self.max_plausible_latency_s)
        self.reset(now_s)
        return summary


def format_report(summary):
    """One log line for a StreamStats.report() summary."""
    name, period = summary['name'], summary['period_s']
    if not summary['count']:
        return f'{name}: no messages in the last {period:.1f} s ({summary["total"]} total)'
    line = f'{name}: {summary["rate_hz"]:.1f} Hz ({summary["count"]} msgs in {period:.1f} s)'
    if summary['latency_mean_s'] is None:
        return line + ', latency n/a (header stamps unset)'
    if summary['clock_mismatch']:
        return (line + f', latency {summary["latency_mean_s"]:.3g} s: header stamps are not '
                'on this node\'s clock (UE game time? publish /clock and set use_sim_time:=true)')
    return line + (f', latency mean {summary["latency_mean_s"] * 1e3:.1f} ms, '
                   f'min {summary["latency_min_s"] * 1e3:.1f} ms, '
                   f'max {summary["latency_max_s"] * 1e3:.1f} ms')


def pose_position(msg, pose_type):
    """geometry_msgs/Point of an Odometry or PoseStamped message."""
    return msg.pose.pose.position if pose_type == 'odometry' else msg.pose.position


class UEClient(Node):
    def __init__(self):
        super().__init__('ue_client')
        self.image_topic = self.param('image_topic', '/ue/camera/image_raw')
        self.pose_topic = self.param('pose_topic', '/ue/odom')
        self.pose_type = self.param('pose_type', 'odometry')
        self.cmd_topic = self.param('cmd_topic', '/ue/cmd_vel')
        cmd_rate_hz = float(self.param('cmd_rate_hz', 10.0))
        self.linear_x = float(self.param('linear_x', 0.0))
        self.angular_z = float(self.param('angular_z', 0.0))
        report_period_s = float(self.param('report_period_s', 2.0))
        self.run_duration_s = float(self.param('run_duration_s', 0.0))
        if self.pose_type not in POSE_TYPES:
            raise ValueError(f'pose_type must be one of {POSE_TYPES}, got {self.pose_type!r}')

        now = self.now_s()
        self.start_s = now
        self.image_stats = StreamStats(self.image_topic, now)
        self.pose_stats = StreamStats(self.pose_topic, now)
        self.last_image = None
        self.last_position = None
        self.cmd_count = 0

        # Best effort matches both best-effort and reliable publishers
        self.create_subscription(Image, self.image_topic, self.on_image,
                                 qos_profile_sensor_data)
        pose_msg = Odometry if self.pose_type == 'odometry' else PoseStamped
        self.create_subscription(pose_msg, self.pose_topic, self.on_pose,
                                 qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        if cmd_rate_hz > 0:
            self.create_timer(1.0 / cmd_rate_hz, self.publish_cmd)
        self.create_timer(report_period_s, self.report)
        self.get_logger().info(
            f'image {self.image_topic}, {self.pose_type} {self.pose_topic}; '
            f'publishing Twist(linear.x={self.linear_x}, angular.z={self.angular_z}) '
            f'on {self.cmd_topic} at {cmd_rate_hz} Hz')

    def param(self, name, default):
        # Dynamically typed so that e.g. linear_x:=1 works as well as 1.0
        descriptor = ParameterDescriptor(dynamic_typing=True)
        return self.declare_parameter(name, default, descriptor).value

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_image(self, msg):
        self.image_stats.record(self.now_s(), stamp_to_s(msg.header.stamp))
        self.last_image = (msg.width, msg.height, msg.encoding)

    def on_pose(self, msg):
        self.pose_stats.record(self.now_s(), stamp_to_s(msg.header.stamp))
        self.last_position = pose_position(msg, self.pose_type)

    def publish_cmd(self, linear_x=None, angular_z=None):
        cmd = Twist()
        cmd.linear.x = self.linear_x if linear_x is None else linear_x
        cmd.angular.z = self.angular_z if angular_z is None else angular_z
        self.cmd_pub.publish(cmd)
        self.cmd_count += 1

    def report(self):
        now = self.now_s()
        image = format_report(self.image_stats.report(now))
        if self.last_image:
            image += ' [%dx%d %s]' % self.last_image
        pose = format_report(self.pose_stats.report(now))
        if self.last_position is not None:
            p = self.last_position
            pose += f' [x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}]'
        log = self.get_logger()
        log.info(image)
        log.info(pose)
        log.info(f'{self.cmd_topic}: {self.cmd_count} commands published')

    def done(self):
        return self.run_duration_s > 0 and self.now_s() - self.start_s >= self.run_duration_s


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Ctrl-C and `docker stop` are handled here instead of by rclpy, so the
    # context is still up to send the final zero command
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _interrupt)
    node = UEClient()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_cmd(0.0, 0.0)
            node.get_logger().info(f'stopped: zero command sent on {node.cmd_topic}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
