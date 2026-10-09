"""SO-101 GR00T bridge against the policy server contract.

Covers the observation dtypes GR00T's strict mode enforces, joint feedback
that quantizes to just past a limit, and schema detection against the mock
and demo policy servers.
"""
from collections import deque
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import MagicMock
import sys
import threading

import numpy as np
import pytest

from conftest import REPO, install_ros_stubs, load_script

install_ros_stubs()
sys.path.insert(0, str(REPO / 'applications/so101/scripts'))
sys.path.insert(0, str(REPO / 'applications/groot/scripts'))
from groot_so101_bridge import SO101GrootBridge, pack, unpack
from so101_common import (
    JOINT_NAMES, JOINT_QUANTIZATION, LOWER_LIMITS, UPPER_LIMITS, quantize)


def install_gr00t_stubs():
    """Let the policy server import without the GR00T package."""
    class BasePolicy:
        def __init__(self, strict=True):
            self.strict = strict

    modules = {
        'gr00t': ModuleType('gr00t'),
        'gr00t.policy': ModuleType('gr00t.policy'),
        'gr00t.policy.policy': ModuleType('gr00t.policy.policy'),
        'gr00t.policy.server_client': ModuleType('gr00t.policy.server_client'),
    }
    modules['gr00t.policy.policy'].BasePolicy = BasePolicy
    modules['gr00t.policy.server_client'].PolicyServer = MagicMock()
    for name, module in modules.items():
        sys.modules.setdefault(name, module)


install_gr00t_stubs()
policy_server = load_script(
    REPO / 'applications/groot/scripts/so101_policy_server.py', 'groot_so101_policy_server')

SO101_SCHEMA = dict(
    video_keys=['front', 'wrist'], state_keys=['single_arm', 'gripper'],
    action_keys=['single_arm', 'gripper'],
    language_key='annotation.human.task_description', video_horizon=1, state_horizon=1)


def make_bridge(client=None, width=16, height=12, history=4):
    """A bridge with the attributes __init__ sets, minus ROS and ZMQ."""
    node = SO101GrootBridge.__new__(SO101GrootBridge)
    node.get_logger = MagicMock()
    node.client = client
    node.command_pub = MagicMock()
    node.video_key, node.depth_key = 'front', 'front_depth'
    node.state_key = node.action_key = 'joint_positions'
    node.instruction = 'pick the red block'
    node.image_width, node.image_height = width, height
    node.joint_history_size = history
    node.max_joint_step = .08
    node.scenario = {}
    node.latest_image = node.latest_wrist_image = None
    node.latest_depth = node.latest_camera_info = None
    node.latest_positions = np.zeros(len(JOINT_NAMES), dtype=np.float32)
    node.joint_history = deque(maxlen=history)
    node.have_joint_state = False
    node._policy_busy = False
    node._policy_lock = threading.Lock()
    node._missing_wrist_warned = False
    node._modality_configured = False
    node.video_keys = [node.video_key]
    node.state_keys = [node.state_key]
    node.action_keys = [node.action_key]
    node.language_key = 'task'
    node.video_horizon = 1
    node.state_horizon = history
    return node


def joint_state(positions):
    # ROS delivers JointState.position as Python floats (float64).
    return NS(name=JOINT_NAMES, position=[float(p) for p in positions])


def check_strict_observation(bridge, observation):
    """The dtype/shape rules of upstream Gr00tPolicy.check_observation."""
    for key in bridge.video_keys:
        video = observation['video'][key]
        assert isinstance(video, np.ndarray)
        assert video.dtype == np.uint8, f'video {key} is {video.dtype}'
        assert video.ndim == 5 and video.shape[1] == bridge.video_horizon
        assert video.shape[-1] == 3
    for key in bridge.state_keys:
        state = observation['state'][key]
        assert isinstance(state, np.ndarray)
        assert state.dtype == np.float32, f'state {key} is {state.dtype}'
        assert state.ndim == 3 and state.shape[1] == bridge.state_horizon
    language = observation['language'][bridge.language_key]
    assert isinstance(language, list)
    assert all(isinstance(item, list) and len(item) == 1 and isinstance(item[0], str)
               for item in language)


@pytest.mark.parametrize('schema', ['legacy', 'so101'])
def test_observation_satisfies_strict_gr00t_dtypes(schema):
    bridge = make_bridge()
    if schema == 'so101':
        bridge._set_policy_modalities(**SO101_SCHEMA)
    positions = np.array([.1, -.2, .3, -.4, .5, .02])
    bridge._joint_cb(joint_state(positions))
    assert bridge.have_joint_state
    image = np.arange(bridge.image_height * bridge.image_width * 3, dtype=np.uint8)
    bridge._image_cb(NS(encoding='bgr8', height=bridge.image_height, width=bridge.image_width,
                        step=bridge.image_width * 3, data=image.tobytes()))

    observation = bridge._observation()

    check_strict_observation(bridge, observation)
    # The float32 cast must not reorder or truncate the joint values.
    state = np.concatenate([observation['state'][key] for key in bridge.state_keys], axis=-1)
    np.testing.assert_allclose(state[0, -1], positions, rtol=1e-6)


@pytest.mark.parametrize('limit', ['lower', 'upper'])
def test_bridge_keeps_commanding_at_quantized_joint_limits(limit):
    limits = UPPER_LIMITS if limit == 'upper' else LOWER_LIMITS
    # What the Isaac adapter publishes for an arm sitting on its limits.
    measured = quantize(limits, JOINT_QUANTIZATION).astype(np.float64)
    assert np.any(measured > UPPER_LIMITS) or np.any(measured < LOWER_LIMITS), \
        'quantization no longer rounds past a limit; this test lost its edge case'
    beyond = limits + np.sign(limits) * 1.0
    client = NS(get_action=MagicMock(return_value=[{'joint_positions': beyond}, {}]),
                reset=MagicMock())
    bridge = make_bridge(client)
    bridge._modality_configured = True

    bridge._joint_cb(joint_state(measured))
    assert bridge.have_joint_state
    bridge._policy_worker()

    bridge.command_pub.publish.assert_called_once()
    command = np.asarray(bridge.command_pub.publish.call_args.args[0].data)
    assert np.all(command >= LOWER_LIMITS) and np.all(command <= UPPER_LIMITS)


def test_feedback_beyond_measurement_tolerance_is_still_rejected():
    bridge = make_bridge()
    bridge._joint_cb(joint_state(UPPER_LIMITS + 2 * JOINT_QUANTIZATION))
    assert not bridge.have_joint_state


@pytest.mark.parametrize('policy_factory', [
    lambda: policy_server.MockSO101Policy(),
    lambda: policy_server.DemoSO101Policy(),
], ids=['mock', 'demo'])
def test_bridge_configures_from_mock_and_demo_servers(policy_factory):
    policy = policy_factory()
    # Round-trip through the bridge's wire encoding, as the ZMQ server would.
    client = NS(get_modality_config=lambda: unpack(pack(policy.get_modality_config())))
    bridge = make_bridge(client)

    bridge._configure_from_policy()

    assert bridge._modality_configured
    assert bridge.video_keys == ['front']
    assert bridge.state_keys == ['joint_positions']
    assert bridge.action_keys == ['joint_positions']
    assert bridge.language_key == 'annotation.human.task_description'


def test_bridge_commands_from_mock_server_end_to_end():
    policy = policy_server.MockSO101Policy()
    client = NS(
        get_modality_config=lambda: unpack(pack(policy.get_modality_config())),
        get_action=lambda obs: unpack(pack(list(policy._get_action(unpack(pack(obs)))))),
        reset=lambda: unpack(pack(policy.reset())),
    )
    bridge = make_bridge(client)
    bridge._joint_cb(joint_state(np.zeros(len(JOINT_NAMES))))

    bridge._policy_worker()

    bridge.get_logger().warn.assert_not_called()
    bridge.command_pub.publish.assert_called_once()
    command = np.asarray(bridge.command_pub.publish.call_args.args[0].data)
    assert command.shape == (len(JOINT_NAMES),)
    assert np.all(np.isfinite(command))
    assert np.max(np.abs(command)) <= bridge.max_joint_step
