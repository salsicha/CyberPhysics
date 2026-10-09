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

# Navigation fusion invalid-data tests live in test_navigation_fusion.py.


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
