"""Unreal Engine example client: rate/latency bookkeeping, parameters,
callbacks and the final zero command, with the ROS modules stubbed out."""
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, load_script

ROS_MODULES = ('rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.executors', 'rclpy.signals',
               'rcl_interfaces', 'rcl_interfaces.msg', 'geometry_msgs', 'geometry_msgs.msg',
               'nav_msgs', 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg')


class FakeNode:
    """Just enough of rclpy.node.Node for UEClient.__init__."""
    overrides = {}

    def __init__(self, name):
        self.node_name = name
        self.subscriptions, self.publishers, self.timers = [], [], []
        self.clock_s = 1000.0
        self.logger = MagicMock()

    def declare_parameter(self, name, default, descriptor=None):
        return NS(value=self.overrides.get(name, default))

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((msg_type, topic, callback))

    def create_publisher(self, msg_type, topic, depth):
        publisher = MagicMock()
        self.publishers.append((msg_type, topic, publisher))
        return publisher

    def create_timer(self, period_s, callback):
        self.timers.append((period_s, callback))

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=round(self.clock_s * 1e9)))

    def get_logger(self):
        return self.logger


@pytest.fixture
def module():
    with patch.dict(sys.modules):
        for name in ROS_MODULES:
            sys.modules[name] = MagicMock()
        sys.modules['rclpy.node'].Node = FakeNode
        sys.modules['rclpy.executors'].ExternalShutdownException = type(
            'ExternalShutdownException', (Exception,), {})
        FakeNode.overrides = {}
        mod = load_script(REPO / 'applications/unrealengine/client/example.py',
                          'unrealengine_client_example')
        mod.Twist = lambda: NS(linear=NS(x=None), angular=NS(z=None))
        yield mod


def stamped(t, **fields):
    """A message whose header stamp is t seconds (0: unset)."""
    sec = int(t)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=round((t - sec) * 1e9))), **fields)


def info_lines(node):
    return [call.args[0] for call in node.logger.info.call_args_list]


def test_rate_and_latency_over_one_period(module):
    stats = module.StreamStats('/cam', now_s=10.0)
    for i in range(20):  # 10 Hz for 2 s, 50 ms then 150 ms behind
        now = 10.1 + 0.1 * i
        stats.record(now, now - (0.05 if i % 2 else 0.15))
    summary = stats.report(12.0)
    assert summary['count'] == 20 and summary['rate_hz'] == pytest.approx(10.0)
    assert summary['latency_mean_s'] == pytest.approx(0.1)
    assert summary['latency_min_s'] == pytest.approx(0.05)
    assert summary['latency_max_s'] == pytest.approx(0.15)
    assert not summary['clock_mismatch']
    line = module.format_report(summary)
    assert line.startswith('/cam: 10.0 Hz (20 msgs in 2.0 s)')
    assert 'latency mean 100.0 ms, min 50.0 ms, max 150.0 ms' in line


def test_report_starts_a_new_period(module):
    stats = module.StreamStats('/cam', now_s=0.0)
    stats.record(0.5, 0.4)
    stats.report(1.0)
    summary = stats.report(3.0)
    assert summary['count'] == 0 and summary['total'] == 1
    assert summary['period_s'] == pytest.approx(2.0)
    assert summary['latency_mean_s'] is None
    assert module.format_report(summary) == '/cam: no messages in the last 2.0 s (1 total)'


def test_unset_header_stamps_count_without_latency(module):
    stats = module.StreamStats('/odom', now_s=0.0)
    for now in (0.5, 1.0):
        stats.record(now, 0.0)
    stats.record(1.5)
    summary = stats.report(2.0)
    assert summary['count'] == 3 and summary['unstamped'] == 3
    assert summary['latency_mean_s'] is None
    assert module.format_report(summary).endswith('latency n/a (header stamps unset)')


def test_stamps_on_another_clock_are_flagged_not_reported_as_latency(module):
    # UE game time (seconds since the level started) against wall time
    stats = module.StreamStats('/cam', now_s=1.7e9)
    stats.record(1.7e9 + 0.5, 42.0)
    summary = stats.report(1.7e9 + 1.0)
    assert summary['clock_mismatch']
    line = module.format_report(summary)
    assert 'not on this node\'s clock' in line and 'use_sim_time' in line
    assert 'latency mean' not in line


def test_pose_position_reads_odometry_and_pose_stamped(module):
    point = NS(x=1.0, y=2.0, z=3.0)
    assert module.pose_position(NS(pose=NS(pose=NS(position=point))), 'odometry') is point
    assert module.pose_position(NS(pose=NS(position=point)), 'pose') is point


def test_default_parameters_wire_topics_and_rates(module):
    node = module.UEClient()
    assert node.node_name == 'ue_client'
    (image_type, image_topic, _), (pose_type, pose_topic, _) = node.subscriptions
    assert (image_type, image_topic) == (module.Image, '/ue/camera/image_raw')
    assert (pose_type, pose_topic) == (module.Odometry, '/ue/odom')
    [(cmd_type, cmd_topic, _)] = node.publishers
    assert (cmd_type, cmd_topic) == (module.Twist, '/ue/cmd_vel')
    assert [period for period, _ in node.timers] == [pytest.approx(0.1), 2.0]
    assert (node.linear_x, node.angular_z) == (0.0, 0.0)
    assert not node.done()


def test_parameters_select_topics_pose_type_and_integer_values(module):
    FakeNode.overrides = {'image_topic': '/robot/cam', 'pose_topic': '/robot/pose',
                          'pose_type': 'pose', 'cmd_topic': '/robot/cmd_vel',
                          'cmd_rate_hz': 0, 'linear_x': 1, 'report_period_s': 5}
    node = module.UEClient()
    assert [(t, topic) for t, topic, _ in node.subscriptions] == [
        (module.Image, '/robot/cam'), (module.PoseStamped, '/robot/pose')]
    assert node.publishers[0][1] == '/robot/cmd_vel'
    # cmd_rate_hz 0: no command timer, only the report timer
    assert [period for period, _ in node.timers] == [5.0]
    assert node.linear_x == 1.0 and isinstance(node.linear_x, float)


def test_unknown_pose_type_is_rejected(module):
    FakeNode.overrides = {'pose_type': 'tf'}
    with pytest.raises(ValueError, match='pose_type'):
        module.UEClient()


def test_callbacks_feed_the_report(module):
    node = module.UEClient()
    on_image, on_pose = (callback for _, _, callback in node.subscriptions)
    for i in range(4):
        node.clock_s = 1000.5 + 0.5 * i
        on_image(stamped(node.clock_s - 0.12, width=640, height=480, encoding='rgb8'))
        on_pose(stamped(0, pose=NS(pose=NS(position=NS(x=1.0, y=-2.0, z=0.5)))))
    node.clock_s = 1002.0
    node.report()
    image_line, pose_line, cmd_line = info_lines(node)[-3:]
    assert image_line.startswith('/ue/camera/image_raw: 2.0 Hz (4 msgs in 2.0 s)')
    assert 'latency mean 120.0 ms' in image_line and image_line.endswith('[640x480 rgb8]')
    assert pose_line.startswith('/ue/odom: 2.0 Hz (4 msgs in 2.0 s), latency n/a')
    assert pose_line.endswith('[x=1.00 y=-2.00 z=0.50]')
    assert cmd_line == '/ue/cmd_vel: 0 commands published'


def test_command_timer_publishes_the_configured_twist(module):
    FakeNode.overrides = {'linear_x': 0.5, 'angular_z': -0.25}
    node = module.UEClient()
    publisher = node.publishers[0][2]
    command_timer = node.timers[0][1]
    command_timer()
    command_timer()
    cmd = publisher.publish.call_args.args[0]
    assert (cmd.linear.x, cmd.angular.z) == (0.5, -0.25)
    node.publish_cmd(0.0, 0.0)
    cmd = publisher.publish.call_args.args[0]
    assert (cmd.linear.x, cmd.angular.z) == (0.0, 0.0)
    assert node.cmd_count == 3


def test_run_duration_ends_the_run(module):
    FakeNode.overrides = {'run_duration_s': 3}
    node = module.UEClient()
    node.clock_s = 1002.9
    assert not node.done()
    node.clock_s = 1003.0
    assert node.done()


@pytest.mark.parametrize('stop', ['run_duration', 'interrupt'])
def test_main_sends_one_zero_command_when_it_stops(module, stop):
    node = MagicMock()
    if stop == 'run_duration':
        node.done.side_effect = [False, False, True]
    else:
        node.done.return_value = False
        module.rclpy.spin_once.side_effect = [None, KeyboardInterrupt]
    module.rclpy.ok.return_value = True
    with patch.object(module, 'UEClient', return_value=node), \
            patch.object(module.signal, 'signal') as install_handler:
        module.main([])
    install_handler.assert_called_once_with(module.signal.SIGTERM, module._interrupt)
    assert module.rclpy.spin_once.call_count == 2
    node.publish_cmd.assert_called_once_with(0.0, 0.0)
    node.destroy_node.assert_called_once()
    module.rclpy.try_shutdown.assert_called_once()
