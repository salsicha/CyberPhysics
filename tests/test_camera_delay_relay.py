"""tools/camera_delay_relay.py: frames released a fixed delay (plus
jitter) after they arrive, in order, and the layer feeds the detector."""
import random
import sys
from unittest.mock import patch

import pytest
import yaml

from conftest import REPO, install_ros_stubs, load_script


@pytest.fixture(scope='module')
def relay():
    with patch.dict(sys.modules):
        install_ros_stubs()
        yield load_script(REPO / 'tools/camera_delay_relay.py', 'camera_delay_relay_review')


def test_frames_come_out_the_delay_later_in_order(relay):
    queue = relay.DelayQueue(0.15)
    queue.put(10.00, 'a')
    queue.put(10.05, 'b')
    assert queue.due(10.149) == []
    assert queue.due(10.15) == ['a']
    assert queue.due(10.201) == ['b']


def test_jitter_never_reorders(relay):
    queue = relay.DelayQueue(0.1, jitter_s=0.2, rng=random.Random(3))
    for k in range(200):
        queue.put(k * 0.01, k)
    released = queue.due(100.0)
    assert released == list(range(200))
    queue = relay.DelayQueue(0.1, jitter_s=0.2, rng=random.Random(3))
    queue.put(0.0, 'x')
    assert queue.due(0.099) == [] and queue.due(0.301) == ['x']


def test_the_layer_feeds_the_detector_the_delayed_camera():
    layer = yaml.safe_load((REPO / 'tools/camera_delay.yaml').read_text())['services']
    detector = layer['tracking_detector']['command']
    assert '-p image_topic:=/camera_delay/image_raw' in detector
    assert '-p camera_info_topic:=/camera_delay/camera_info' in detector
    relay = layer['camera_delay']['command']
    assert '-p delay_s:=${CAMERA_DELAY_S:-0.15}' in relay
    stack = yaml.safe_load((REPO / 'compositions/aerodrone_tracking_sim.yaml').read_text())
    original = stack['services']['tracking_detector']['command']
    # The same detector but for its two input topics.
    strip = lambda text: [arg for arg in text.split() if 'image_topic' not in arg
                          and 'camera_info_topic' not in arg]
    assert strip(detector) == strip(original)


def test_stamp_offset_shifts_the_header_only(relay):
    class Time:
        def __init__(self, sec=0, nanosec=0):
            self.sec, self.nanosec = sec, nanosec
    stamp = Time(10, 990_000_000)
    later = relay.shifted(stamp, 50_000_000)
    assert (later.sec, later.nanosec) == (11, 40_000_000)
    earlier = relay.shifted(stamp, -1_000_000_000)
    assert (earlier.sec, earlier.nanosec) == (9, 990_000_000)
    assert (stamp.sec, stamp.nanosec) == (10, 990_000_000)  # the original kept
    layer = yaml.safe_load((REPO / 'tools/camera_delay.yaml').read_text())['services']
    assert '-p stamp_offset_s:=${CAMERA_STAMP_OFFSET_S:-0.0}' in layer['camera_delay']['command']


def test_the_truth_boxes_come_from_the_same_frames():
    """The driver pairs detections with truth boxes by stamp: both from the
    relayed frames, the truth detector otherwise as the test layer runs it."""
    layer = yaml.safe_load((REPO / 'tools/camera_delay.yaml').read_text())['services']
    test = yaml.safe_load((REPO / 'tools/tracking_sim_test.yaml').read_text())['services']
    ours, theirs = layer['tracking_truth_boxes']['command'], test['tracking_truth_boxes']['command']
    assert '-p camera_info_topic:=/camera_delay/camera_info' in ours
    strip = lambda text: [arg for arg in text.split() if 'camera_info_topic' not in arg]
    assert strip(ours) == strip(theirs)
