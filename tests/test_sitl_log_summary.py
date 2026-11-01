"""tools/sitl_log_summary.py reports ArduPilot's IMU sample-rate estimate:
near the nominal 1000 Hz at arming behind Gazebo's 100 Hz JSON backend, the
gyro filter runs ten times too slow and the attitude loop shakes
(tests/test_sitl_clock.py)."""
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from conftest import REPO, load_script


class Message(SimpleNamespace):
    def get_type(self):
        return self.kind


def summarise(messages):
    log = MagicMock()
    log.recv_match.side_effect = list(messages) + [None]
    mavutil = MagicMock()
    mavutil.mavlink_connection.return_value = log
    pymavlink = MagicMock(mavutil=mavutil)
    with patch.dict(sys.modules, {'pymavlink': pymavlink, 'pymavlink.mavutil': mavutil}):
        tool = load_script(REPO / 'tools/sitl_log_summary.py', 'sitl_log_summary_test')
        return tool.summarise('x.BIN')


def imu(t_us, instance, rate_hz):
    return Message(kind='IMU', TimeUS=t_us, I=instance, AccZ=-9.81, GHz=rate_hz)


def test_reports_the_first_imus_rate_estimate_at_start_and_end():
    out = summarise([imu(1, 0, 841), imu(1, 1, 639), imu(2, 0, 700), imu(3, 0, 100)])
    assert out['imu_rate_estimate_hz'] == {'at_start': 841, 'at_end': 100}


def test_no_rate_field_no_estimate():
    out = summarise([Message(kind='IMU', TimeUS=1, I=0, AccZ=-9.81)])
    assert out['imu_rate_estimate_hz'] is None
