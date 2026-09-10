"""Exercise the command callback without requiring a running ROS graph."""
import math
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, install_ros_stubs, load_script


@pytest.mark.parametrize("speed,yaw_rate", [
    (-0.2, 0.0), (-0.2, 0.02), (-0.2, -0.02),
    (0.2, 0.02), (0.2, -0.02), (0.0, 0.02), (-0.2, 3.0),
])
def test_forward_and_reverse_bicycle_steering(speed, yaw_rate):
    with patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['ackermann_msgs'] = MagicMock()
        sys.modules['ackermann_msgs.msg'] = MagicMock()
        module = load_script(REPO / 'applications/racecarneo/scripts/cmd_vel_to_ackermann.py',
                             'ackermann_review')
        node = module.CmdVelToAckermann.__new__(module.CmdVelToAckermann)
        node.wheelbase = 0.33
        node.max_speed = 0.35
        node.max_steering_angle = 0.42
        node.pub = MagicMock()
        node.get_clock = MagicMock()
        node._cmd_vel(SimpleNamespace(linear=SimpleNamespace(x=speed),
                                      angular=SimpleNamespace(z=yaw_rate)))
        output = node.pub.publish.call_args.args[0]
        steering = output.drive.steering_angle * node.max_steering_angle
        assert output.drive.speed == pytest.approx(speed / node.max_speed)
        if speed == 0.0:
            assert steering == 0.0
        elif abs(yaw_rate) < 1:
            assert speed * math.tan(steering) / node.wheelbase == pytest.approx(yaw_rate)
        else:
            assert steering == -node.max_steering_angle
