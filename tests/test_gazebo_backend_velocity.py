"""The Gazebo ArduPilot backend's velocity frame: Gazebo's ground-truth twist
(OdometryPublisher, <dimensions>3</dimensions>) is the body-frame velocity;
ArduPilot's JSON state wants it in ENU (applications/gazebo/scripts/
gazebo_ardupilot_backend.py, GAZEBO_TWIST_IN_BODY_FRAME)."""
import math
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest
from conftest import REPO, load_script

sys.path.insert(0, str(REPO / 'applications/ardupilot_sitl/scripts'))
from ardupilot_json import body_velocity_to_enu, state_from_enu  # noqa: E402


def yaw_quaternion(yaw_deg):
    half = math.radians(yaw_deg) / 2.0
    return [math.cos(half), 0.0, 0.0, math.sin(half)]  # w, x, y, z


@pytest.mark.parametrize('yaw_deg,expected', [
    (0.0, (4.0, 0.0, 0.0)),      # nose east: body x is east
    (90.0, (0.0, 4.0, 0.0)),     # nose north
    (180.0, (-4.0, 0.0, 0.0)),
    (-60.0, (2.0, -2.0 * math.sqrt(3.0), 0.0))])
def test_forward_flight_is_along_the_heading(yaw_deg, expected):
    assert body_velocity_to_enu([4.0, 0.0, 0.0], yaw_quaternion(yaw_deg)) == \
        pytest.approx(expected)


def test_ardupilot_gets_north_east_down_of_the_rotated_velocity():
    velocity = body_velocity_to_enu([4.0, 0.0, -1.0], yaw_quaternion(90.0))
    state = state_from_enu([0.0, 0.0, 10.0], velocity, yaw_quaternion(90.0),
                           [0.0, 0.0, 0.0], [0.0, 0.0, 9.8], 37.9, -122.6, 700.0)
    assert state['velocity'] == pytest.approx([4.0, 0.0, 1.0])


def test_pitch_rotates_vertical_velocity_too():
    # Nose pitched up 30 degrees: FLU forward becomes east and up in ENU.
    q = [math.cos(math.pi / 12), 0, -math.sin(math.pi / 12), 0]
    assert body_velocity_to_enu([4, 0, 0], q) == pytest.approx([2 * math.sqrt(3), 0, 2])


@pytest.mark.parametrize('setting,expected_ned', [
    (None, [4.0, 0.0, 1.0]),  # no env override: production default
    ('true', [4.0, 0.0, 1.0]),
    ('false', [0.0, 4.0, 1.0]),  # alternate source already in ENU
])
def test_backend_default_sends_rotated_velocity_to_ardupilot(
        monkeypatch, tmp_path, setting, expected_ned):
    class FakeNode:
        def __init__(self, name):
            self.create_publisher = MagicMock()
            self.create_subscription = MagicMock()
            self.create_timer = MagicMock()
            self.get_logger = MagicMock()

    modules = {name: MagicMock() for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'actuator_msgs', 'actuator_msgs.msg',
        'geometry_msgs', 'geometry_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg')}
    modules['rclpy.node'].Node = FakeNode
    with patch.dict(sys.modules, modules):
        backend = load_script(REPO / 'applications/gazebo/scripts/gazebo_ardupilot_backend.py',
                              'gazebo_backend_velocity_test')
    manifest = tmp_path / 'manifest.json'
    manifest.write_text('{"home_enu_m": [0, 0, 0]}')
    monkeypatch.setenv('TERRAIN_MANIFEST', str(manifest))
    if setting is None:
        monkeypatch.delenv('GAZEBO_TWIST_IN_BODY_FRAME', raising=False)
    else:
        monkeypatch.setenv('GAZEBO_TWIST_IN_BODY_FRAME', setting)
    transport = MagicMock()
    transport.receive.return_value = SimpleNamespace(pwm=[1500] * 4)
    monkeypatch.setattr(backend, 'ArduPilotJSON', lambda **kwargs: transport)
    node = backend.GazeboArduPilotBackend()
    node.get_clock = MagicMock()
    q = yaw_quaternion(90)
    node.pose = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
        pose=SimpleNamespace(position=SimpleNamespace(x=0, y=0, z=10),
                             orientation=SimpleNamespace(w=q[0], x=q[1], y=q[2], z=q[3])))
    node.twist = SimpleNamespace(twist=SimpleNamespace(
        linear=SimpleNamespace(x=4, y=0, z=-1), angular=SimpleNamespace(x=0, y=0, z=0)))
    node.tick()
    transport.send.assert_called_once()
    assert transport.send.call_args.args[0]['velocity'] == pytest.approx(expected_ned)
