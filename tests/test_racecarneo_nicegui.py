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


def test_drive_commands_go_to_the_contract_topics(gui):
    module, node = gui
    node.teleop_pub, node.stop_pub = MagicMock(), MagicMock()
    node.publish_teleop(0.2, -0.1)
    [twist] = published(node.teleop_pub)
    assert (twist.linear.x, twist.angular.z) == (0.2, -0.1)
    node.request_stop(True)
    node.request_stop(False)
    assert [msg.data for msg in published(node.stop_pub)] == [True, False]


def test_cancel_reaches_every_ready_nav2_action(gui):
    module, node = gui
    ready, absent = MagicMock(), MagicMock()
    ready.service_is_ready.return_value = True
    absent.service_is_ready.return_value = False
    node.cancel_clients = [ready, absent]
    assert node.cancel_navigation('Cancel') is True
    ready.call_async.assert_called_once()
    absent.call_async.assert_not_called()
    assert 'canceled' in node.state.nav_status
    ready.service_is_ready.return_value = False
    assert node.cancel_navigation('Cancel') is False


def test_stop_all_stops_cancels_and_disarms_the_turret(gui):
    module, node = gui
    node.turret = True
    node.stop_pub, node.turret_estop_pub = MagicMock(), MagicMock()
    client = MagicMock()
    client.service_is_ready.return_value = True
    node.cancel_clients = [client]
    node.stop_all()
    assert [msg.data for msg in published(node.stop_pub)] == [True]
    client.call_async.assert_called_once()
    assert [msg.data for msg in published(node.turret_estop_pub)] == [True]


def test_turret_commands_are_json_strings(gui):
    module, node = gui
    node.turret_command_pub = MagicMock()
    node.send_turret_command({'action': 'release'})
    assert published(node.turret_command_pub) == []          # plain RACECAR page: no turret
    node.turret = True
    node.send_turret_command({'action': 'lock', 'target_id': '3'})
    [msg] = published(node.turret_command_pub)
    assert json.loads(msg.data) == {'action': 'lock', 'target_id': '3'}


def test_status_and_segments_handlers(gui):
    module, node = gui
    node.handle_control_status(NS(data=json.dumps(
        {'source': 'nav2', 'stopped': False, 'speed_mps': 0.2, 'steering_rad': 0.1})))
    assert node.state.control['source'] == 'nav2' and node.state.control_time > 0.0
    node.handle_turret_status(NS(data='{"mode": "auto", "target_id": "3", "locked": true}'))
    assert node.state.turret_status['locked'] is True
    # The size comes from the message, or else from the last turret image.
    node.state.turret_image_size = (320, 240)
    node.handle_segments(NS(data=json.dumps([{'id': 4, 'class': 'person', 'confidence': 0.5,
                                              'bbox_xywh': [160, 120, 32, 48]}])))
    detection = node.state.segments.detections[0]
    assert (detection.id, detection.u, detection.v) == ('4', 0.5, 0.5)
    node.handle_segments(NS(data='not json'))
    assert node.state.segments.detections[0].id == '4'       # bad messages are ignored


def test_plan_in_another_frame_is_not_drawn(gui):
    module, node = gui
    pose = NS(pose=NS(position=NS(x=1.0, y=2.0)))
    node.handle_plan(NS(header=NS(frame_id='map'), poses=[pose, pose]))
    assert node.state.path == ((1.0, 2.0), (1.0, 2.0))
    node.handle_plan(NS(header=NS(frame_id='odom'), poses=[]))
    assert node.state.path == ((1.0, 2.0), (1.0, 2.0))
