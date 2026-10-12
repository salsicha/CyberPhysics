"""The Gazebo ArduPilot backend's velocity frame: Gazebo's ground-truth twist
(OdometryPublisher, <dimensions>3</dimensions>) is the body-frame velocity;
ArduPilot's JSON state wants it in ENU (applications/gazebo/scripts/
gazebo_ardupilot_backend.py, GAZEBO_TWIST_IN_BODY_FRAME)."""
import math
import sys

import pytest
from conftest import REPO

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


def test_backend_rotates_the_twist_only_when_asked():
    backend = (REPO / 'applications/gazebo/scripts/gazebo_ardupilot_backend.py').read_text()
    assert '"GAZEBO_TWIST_IN_BODY_FRAME", "false"' in backend
    assert 'body_velocity_to_enu(velocity, [q.w, q.x, q.y, q.z])' in backend
    layer = (REPO / 'tools/tracking_sim_test_gazebo.yaml').read_text()
    assert 'GAZEBO_TWIST_IN_BODY_FRAME: "true"' in layer
