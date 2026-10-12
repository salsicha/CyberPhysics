"""RACECAR Neo NiceGUI: the displayed pose is in the frame goals are sent in,
and the car and turret commands the page sends (systems/racecar_turret/
interfaces.md).

nicegui, fastapi, OpenCV, tf2_ros and the ROS packages are stubbed, so
importing the script only builds the page definition.
"""
import json
import math
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import numpy  # noqa: F401  (loaded once, outside the patched sys.modules below)
import pytest

from conftest import REPO, install_ros_stubs, load_script


class TransformException(Exception):
    pass


def load_gui(env=None):
    with patch.dict(sys.modules), patch.dict('os.environ', env or {}):
        install_ros_stubs()
        for name in ('rclpy.action', 'rclpy.executors', 'rclpy.time', 'nav2_msgs',
                     'nav2_msgs.action', 'action_msgs', 'action_msgs.srv', 'tf2_ros',
                     'nicegui', 'fastapi', 'cv2'):
            sys.modules[name] = MagicMock()
        sys.modules['tf2_ros'].TransformException = TransformException
        module = load_script(REPO / 'applications/nicegui/scripts/main.py', 'racecarneo_nicegui')
    # Message types that record their fields, so tests can read what was published.
    module.Bool = lambda data: NS(data=data)
    module.String = lambda data: NS(data=data)
    module.Twist = lambda: NS(linear=NS(x=0.0), angular=NS(z=0.0))
    return module


@pytest.fixture
def gui():
    module = load_gui()
    node = module.RacecarNeoGuiNode.__new__(module.RacecarNeoGuiNode)
    node.state = module.VehicleState()
    node.tf_buffer = MagicMock()
    node.nav_client = MagicMock()
    node.get_clock = MagicMock()
    node.turret = False
    return module, node


def published(publisher):
    return [call.args[0] for call in publisher.publish.call_args_list]


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


def test_the_turret_page_is_opt_in():
    assert load_gui().TURRET is False
    turret = load_gui({'UI_TURRET': '1'})
    assert turret.TURRET is True and turret.TITLE == 'RACECAR Turret'


def test_turret_only_layout():
    standalone = load_gui({'UI_TURRET': '1', 'UI_CAR': '0',
                           'UI_TURRET_CAMERA_TOPIC': '/camera/camera/color/image_raw'})
    assert (standalone.TURRET, standalone.CAR, standalone.TITLE) == (True, False, 'PanTiltROS Turret')
    assert standalone.TURRET_CAMERA_TOPIC == '/camera/camera/color/image_raw'
    assert load_gui({'UI_TURRET': '1'}).TURRET_CAMERA_TOPIC == '/turret/camera/image_raw'
    # A page needs something on it: without the turret the car part stays
    assert load_gui({'UI_CAR': '0'}).CAR is True
    # Stop all then only disarms the turret; car commands go nowhere
    node = standalone.RacecarNeoGuiNode.__new__(standalone.RacecarNeoGuiNode)
    node.state = standalone.VehicleState()
    node.turret, node.car = True, False
    node.turret_estop_pub = MagicMock()
    node.stop_all()
    node.request_stop(True)
    assert [msg.data for msg in published(node.turret_estop_pub)] == [True]
    assert not hasattr(node, 'stop_pub') and node.cancel_navigation('x') is False


def test_launcher_commands_are_json_strings(gui):
    module, node = gui
    node.turret = True
    node.turret_command_pub = MagicMock()
    status = module.logic.parse_turret_status(json.dumps({
        'mode': 'auto', 'target_id': '7', 'locked': True, 'launcher': 'ready', 'darts': 2,
        'fire_block': ''}))
    node.send_turret_command(module.logic.fire_command(status))
    node.send_turret_command(module.logic.reload_command(6))
    node.send_turret_command(module.logic.auto_next_command(True))
    node.send_turret_command({'action': 'next'})
    node.send_turret_command({'action': 'clear_tags'})
    assert [json.loads(msg.data) for msg in published(node.turret_command_pub)] == [
        {'action': 'tag', 'target_id': '7'}, {'action': 'reload', 'darts': 6},
        {'action': 'auto_next', 'enabled': True}, {'action': 'next'}, {'action': 'clear_tags'}]
