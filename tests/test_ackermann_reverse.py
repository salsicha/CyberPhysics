"""Exercise the command callback without requiring a running ROS graph."""
import math
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, install_ros_stubs, load_script

WHEELBASE, MAX_SPEED, MAX_STEER = 0.33, 0.35, 0.42


def convert(speed, yaw_rate):
    """Run one Twist through the converter; return the published drive or None."""
    with patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['ackermann_msgs'] = MagicMock()
        sys.modules['ackermann_msgs.msg'] = MagicMock()
        module = load_script(REPO / 'applications/racecarneo/scripts/cmd_vel_to_ackermann.py',
                             'ackermann_review')
        node = module.CmdVelToAckermann.__new__(module.CmdVelToAckermann)
        node.wheelbase = WHEELBASE
        node.max_speed = MAX_SPEED
        node.max_steering_angle = MAX_STEER
        node.pub = MagicMock()
        node.get_clock = MagicMock()
        node.get_logger = MagicMock()
        node._cmd_vel(SimpleNamespace(linear=SimpleNamespace(x=speed),
                                      angular=SimpleNamespace(z=yaw_rate)))
        if not node.pub.publish.called:
            return None
        return node.pub.publish.call_args.args[0].drive


@pytest.mark.parametrize("speed,yaw_rate", [
    (-0.2, 0.0), (-0.2, 0.02), (-0.2, -0.02),
    (0.2, 0.02), (0.2, -0.02), (0.0, 0.02), (-0.2, 3.0),
])
def test_forward_and_reverse_bicycle_steering(speed, yaw_rate):
    drive = convert(speed, yaw_rate)
    # ackermann_msgs units: m/s and rad.
    steering = drive.steering_angle
    assert drive.speed == pytest.approx(speed)
    if speed == 0.0:
        assert steering == 0.0
    elif abs(yaw_rate) < 1:
        assert speed * math.tan(steering) / WHEELBASE == pytest.approx(yaw_rate)
    else:
        assert steering == -MAX_STEER


@pytest.mark.parametrize("speed,yaw_rate", [(0.7, 0.4), (-0.7, 0.4), (1.4, -0.2)])
def test_speed_limit_keeps_requested_curvature(speed, yaw_rate):
    drive = convert(speed, yaw_rate)
    assert drive.speed == pytest.approx(math.copysign(MAX_SPEED, speed))
    # Clamping speed must slow the car along the requested path, not tighten it.
    assert math.tan(drive.steering_angle) / WHEELBASE == pytest.approx(yaw_rate / speed)


@pytest.mark.parametrize("speed,yaw_rate", [
    (math.nan, 0.0), (0.2, math.nan), (math.inf, 0.0), (0.2, -math.inf)])
def test_non_finite_twist_is_not_forwarded(speed, yaw_rate):
    # min/max would have turned NaN into a full-speed command.
    assert convert(speed, yaw_rate) is None
