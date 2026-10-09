"""Fallback OAK-1 intrinsics DemNav scales its depth template with."""
import math
import sys
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, install_ros_stubs, load_script


@pytest.fixture(scope='module')
def megadepth():
    with patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['depthai'] = MagicMock()
        yield load_script(REPO / 'applications/oak1/scripts/megadepth_ros.py',
                          'megadepth_review')


def hfov_deg(k, width):
    return math.degrees(2.0 * math.atan(width / 2.0 / k[0]))


def test_preview_is_a_four_by_three_crop(megadepth):
    k = megadepth.approximate_intrinsics(256, 192)
    # 69 deg would make DemNav's template ~40% too large.
    assert hfov_deg(k, 256) == pytest.approx(52.0, abs=0.5)
    assert k[4] == k[0]
    assert (k[2], k[5]) == (128.0, 96.0)


def test_sixteen_by_nine_still_is_uncropped(megadepth):
    k = megadepth.approximate_intrinsics(1280, 720)
    assert hfov_deg(k, 1280) == pytest.approx(66.1, abs=0.5)
