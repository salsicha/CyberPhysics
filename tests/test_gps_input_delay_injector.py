"""tools/gps_input_delay_injector.py: the delay line that makes corrections
late and bursty for the GPS_INPUT reproduction (tools/gps_input_delay.yaml)."""
import sys
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, install_ros_stubs, load_script


@pytest.fixture
def module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['rcl_interfaces'] = MagicMock()
        sys.modules['rcl_interfaces.msg'] = MagicMock()
        yield load_script(REPO / 'tools/gps_input_delay_injector.py', 'gps_input_delay_injector_test')


def test_delay_and_jitter_keep_arrival_order(module):
    line = module.DelayLine(delay_s=0.3, jitter_s=2.0, seed=3)
    dues = [line.push(100.0 + 0.1 * k, k) for k in range(50)]
    assert all(b >= a for a, b in zip(dues, dues[1:]))     # first in, first out
    assert all(0.3 <= due - (100.0 + 0.1 * k) <= 2.3 + 0.1 * 50 for k, due in enumerate(dues))
    assert line.pop_due(100.2) == []
    released = line.pop_due(1000.0)
    assert released == list(range(50))


def test_bursts_release_everything_together(module):
    line = module.DelayLine(delay_s=0.0, burst_s=8.0)
    for k in range(16):
        line.push(800.0 + 0.5 * k, k)              # 800.0 .. 807.5
    assert line.pop_due(807.99) == []
    assert line.pop_due(808.0) == list(range(16))  # one burst at 808 s


def test_drop_share_and_counters(module):
    line = module.DelayLine(drop=0.5, seed=1)
    for k in range(1000):
        line.push(0.0, k)
    assert 400 < line.dropped < 600
    assert len(line.pop_due(0.0)) == 1000 - line.dropped
    assert line.received == 1000 and line.released == 1000 - line.dropped


def test_undelayed_line_relays_at_once(module):
    line = module.DelayLine()
    line.push(5.0, 'x')
    assert line.pop_due(5.0) == ['x']
