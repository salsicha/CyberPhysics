from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import MagicMock
import sys
import threading

import numpy as np
import pytest

from conftest import REPO, install_ros_stubs

install_ros_stubs()
sys.path.insert(0, str(REPO / 'applications/so101/scripts'))
from groot_so101_bridge import SO101GrootBridge
from so101_common import JOINT_NAMES
from wildnav_pkg.navigation_fusion import Correction, NavigationFusionNode


def odometry(x=0., stamp=1):
    vector = lambda: NS(x=0., y=0., z=0.)
    return NS(header=NS(stamp=NS(sec=stamp, nanosec=0), frame_id='odom'),
              pose=NS(pose=NS(position=NS(x=x, y=0., z=0.),
                             orientation=NS(x=0., y=0., z=0., w=1.)), covariance=[0.] * 36),
              twist=NS(twist=NS(linear=vector(), angular=vector()), covariance=[0.] * 36))


@pytest.fixture
def bridge():
    node = SO101GrootBridge.__new__(SO101GrootBridge)
    node.action_keys = ['joint_positions']
    node.action_key = 'joint_positions'
    node.max_joint_step = .08
    node.latest_positions = np.zeros(6)
    node.joint_history = deque(maxlen=4)
    node.have_joint_state = True
    node.get_logger = MagicMock()
    return node


@pytest.mark.parametrize('invalid', [np.nan, np.inf, -np.inf])
def test_bridge_rejects_invalid_policy_actions_and_feedback(bridge, invalid):
    vector = np.zeros(6)
    vector[0] = invalid
    with pytest.raises(ValueError, match='finite'):
        bridge._extract_action({'joint_positions': vector}, np.zeros(6))
    with pytest.raises(ValueError, match='finite'):
        bridge._extract_action({'joint_positions': np.zeros(6)}, vector)
    bridge._joint_cb(NS(name=JOINT_NAMES, position=vector))
    assert not bridge.have_joint_state
    assert not bridge.joint_history
    bridge._joint_cb(NS(name=JOINT_NAMES, position=np.zeros(6)))
    assert bridge.have_joint_state
    assert np.all(np.isfinite(bridge.joint_history[-1]))


def test_bridge_rejects_incomplete_feedback_and_preserves_step_limit(bridge):
    bridge._joint_cb(NS(name=JOINT_NAMES, position=[0.]))
    assert not bridge.have_joint_state
    target = bridge._extract_action({'joint_positions': [1.] * 6}, np.zeros(6))
    assert np.all(np.isfinite(target))
    assert np.max(np.abs(target)) <= bridge.max_joint_step
    with pytest.raises(ValueError, match='outside joint limits'):
        bridge._extract_action({'joint_positions': [0.] * 6}, [2.1, 0, 0, 0, 0, 0])


def test_policy_worker_drops_invalid_reply_and_recovers(bridge):
    bridge._modality_configured = True
    bridge._policy_lock = threading.Lock()
    bridge._observation = MagicMock(return_value={})
    bridge.client = MagicMock()
    bridge.command_pub = MagicMock()
    bridge.client.get_action.side_effect = [
        {'joint_positions': [np.nan] * 6}, {'joint_positions': [.01] * 6}]
    bridge._policy_worker()
    bridge.command_pub.publish.assert_not_called()
    assert not bridge._policy_busy
    bridge._policy_worker()
    bridge.command_pub.publish.assert_called_once()
    assert np.all(np.isfinite(bridge.command_pub.publish.call_args.args[0].data))


@pytest.fixture
def fusion():
    node = NavigationFusionNode.__new__(NavigationFusionNode)
    parameters = dict(minimum_demnav_confidence=.35, minimum_wildnav_confidence=.2,
                      maximum_correction_m=300., correction_timeout_s=30.,
                      raw_history_s=20., correction_time_constant_s=2.)
    node.get_parameter = lambda name: NS(value=parameters[name])
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=2_000_000_000))
    node.get_logger = MagicMock()
    node.demnav = Correction(valid=True, confidence=1.)
    node.wildnav = Correction()
    node.latest_raw = odometry()
    node.raw_history = deque([(1_000_000_000, 0., 0.)])
    node.offset_east = node.offset_north = 0.
    node.last_publish_ns = None
    node.global_reference = None
    node.output_pub = MagicMock()
    node.source_pub = MagicMock()
    node.confidence_pub = MagicMock()
    return node


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -float('inf')])
def test_fusion_rejects_invalid_correction_and_recovers(fusion, invalid):
    fusion._on_correction('demnav', fusion.demnav, odometry(invalid))
    assert fusion.demnav.received_ns == 0
    fusion._on_raw(odometry(stamp=2))
    assert np.isfinite(fusion.output_pub.publish.call_args.args[0].pose.pose.position.x)
    fusion._on_correction('demnav', fusion.demnav, odometry(10., stamp=3))
    fusion._on_raw(odometry(stamp=3))
    assert fusion.demnav.received_ns > 0
    assert 0 < fusion.offset_east <= 10


@pytest.mark.parametrize('field', ['covariance', 'confidence', 'raw_position', 'raw_velocity'])
def test_fusion_rejects_invalid_covariance_confidence_and_raw_data(fusion, field):
    msg = odometry()
    if field == 'covariance':
        msg.pose.covariance[0] = float('nan')
        fusion._on_correction('demnav', fusion.demnav, msg)
        assert fusion.demnav.received_ns == 0
    elif field == 'confidence':
        fusion._on_confidence('demnav', fusion.demnav, NS(data=float('inf')))
        fusion._on_correction('demnav', fusion.demnav, msg)
        assert fusion.demnav.received_ns == 0
    else:
        if field == 'raw_position':
            msg.pose.pose.position.z = float('nan')
        else:
            msg.twist.twist.linear.x = float('nan')
        fusion._on_raw(msg)
        fusion.output_pub.publish.assert_not_called()
        assert len(fusion.raw_history) == 1


def test_fusion_recovers_invalid_internal_offset_without_restart(fusion):
    fusion.offset_east = float('nan')
    fusion.offset_north = float('inf')
    fusion._on_correction('demnav', fusion.demnav, odometry(10.))
    fusion._on_raw(odometry(stamp=2))
    assert fusion.offset_east == 10.
    assert fusion.offset_north == 0.
    assert fusion.output_pub.publish.call_args.args[0].pose.pose.position.x == 10.


def test_fusion_ignores_corrupted_source_and_extreme_variance(fusion):
    fusion.demnav = Correction(valid=True, confidence=1., east=float('nan'), received_ns=1)
    assert fusion._target_offset()[2] == ''
    fusion.demnav.east = 1.
    fusion.demnav.confidence = .35
    fusion.demnav.variance = 1.7e308
    assert fusion._target_offset()[2] == ''
