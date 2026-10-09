"""RACECAR Neo NiceGUI: the displayed pose is in the frame goals are sent in.

nicegui, OpenCV, tf2_ros and the ROS packages are stubbed, so importing the
script only builds the page definition.
"""
import math
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, install_ros_stubs, load_script


class TransformException(Exception):
    pass


@pytest.fixture
def gui():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('rclpy.action', 'rclpy.executors', 'rclpy.time', 'nav2_msgs',
                     'nav2_msgs.action', 'tf2_ros', 'nicegui', 'cv2'):
            sys.modules[name] = MagicMock()
        sys.modules['tf2_ros'].TransformException = TransformException
        module = load_script(REPO / 'applications/nicegui/scripts/main.py', 'racecarneo_nicegui')
    node = module.RacecarNeoGuiNode.__new__(module.RacecarNeoGuiNode)
    node.state = module.VehicleState()
    node.tf_buffer = MagicMock()
    node.nav_client = MagicMock()
    node.get_clock = MagicMock()
    return module, node


def map_transform(x, y, yaw):
    return NS(transform=NS(translation=NS(x=x, y=y, z=0.0),
                           rotation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


def odometry(x, y):
    return NS(header=NS(frame_id='odom'),
              pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0),
                              orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))))


def test_pose_comes_from_map_to_base_link(gui):
    module, node = gui
    node.tf_buffer.lookup_transform.return_value = map_transform(3.0, -1.0, 0.5)
    node.update_map_pose()
    assert node.tf_buffer.lookup_transform.call_args.args[:2] == ('map', 'base_link')
    assert (node.state.x, node.state.y, node.state.yaw) == pytest.approx((3.0, -1.0, 0.5))
    assert node.state.pose_time > 0.0

    # After SLAM corrections /odom differs from map; it must not move the display.
    node.handle_odom(odometry(10.0, 10.0))
    assert (node.state.x, node.state.y) == pytest.approx((3.0, -1.0))
    assert node.state.odom_time > 0.0


def test_missing_map_transform_leaves_pose_unset(gui):
    _module, node = gui
    node.tf_buffer.lookup_transform.side_effect = TransformException('no map yet')
    node.update_map_pose()
    assert node.state.pose_time == 0.0


def test_goals_are_sent_in_the_displayed_frame(gui):
    module, node = gui
    node.tf_buffer.lookup_transform.return_value = map_transform(1.0, 2.0, 0.0)
    node.update_map_pose()
    node.nav_client.wait_for_server.return_value = True

    node.send_waypoint(node.state.x, node.state.y, 0.0)

    goal = node.nav_client.send_goal_async.call_args.args[0]
    assert goal.pose.header.frame_id == module.MAP_FRAME
    assert (goal.pose.pose.position.x, goal.pose.pose.position.y) == (1.0, 2.0)
