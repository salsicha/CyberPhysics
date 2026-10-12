"""SO-101 policy bridge: episode resets, camera preprocessing, command reports.

The bridge must reset the policy server (dropping any queued SmolVLA action
chunk) when it starts, when joint feedback resumes after a simulator restart
and when the task changes; send frames at the resolution the policy was
trained on; and report latency, observation age, saturation and ground-truth
use for the task metrics.
"""
import json
from types import SimpleNamespace as NS
from unittest.mock import MagicMock

import numpy as np
import pytest

from test_so101_groot_bridge import joint_state, make_bridge, pack, policy_server, unpack
from groot_so101_bridge import letterbox_nearest
from so101_common import CAMERA_HEIGHT, CAMERA_WIDTH, JOINT_NAMES


class RecordingClient:
    """In-process stand-in for the ZMQ client that records endpoint calls."""

    def __init__(self, policy):
        self.policy = policy
        self.calls = []

    def get_modality_config(self):
        self.calls.append('get_modality_config')
        return unpack(pack(self.policy.get_modality_config()))

    def reset(self):
        self.calls.append('reset')
        return unpack(pack(self.policy.reset()))

    def get_action(self, observation):
        self.calls.append('get_action')
        return unpack(pack(list(self.policy._get_action(unpack(pack(observation))))))


def ready_bridge(policy=None, **kwargs):
    client = RecordingClient(policy or policy_server.MockSO101Policy())
    bridge = make_bridge(client, **kwargs)
    # The stubbed std_msgs.String() returns one shared mock, so decode each
    # report as it is published.
    bridge.reports = []
    bridge.stats_pub = NS(publish=lambda msg: bridge.reports.append(json.loads(msg.data)))
    bridge._joint_cb(joint_state(np.zeros(len(JOINT_NAMES))))
    return bridge, client


def test_policy_is_reset_once_before_the_first_request():
    bridge, client = ready_bridge()
    bridge._policy_worker()
    bridge._policy_worker()
    assert client.calls == ['get_modality_config', 'reset', 'get_action', 'get_action']
    assert bridge.command_pub.publish.call_count == 2


def test_feedback_gap_resets_policy_before_next_command(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr('groot_so101_bridge.time.monotonic', lambda: clock[0])
    bridge, client = ready_bridge()
    bridge._policy_worker()
    client.calls.clear()

    # The simulator restarts: no joint feedback for longer than the gap.
    clock[0] += bridge.episode_reset_gap_s + 5.0
    bridge._joint_cb(joint_state(np.zeros(len(JOINT_NAMES))))
    bridge._policy_worker()
    assert client.calls == ['reset', 'get_action']

    # Steady feedback does not reset again.
    client.calls.clear()
    clock[0] += 0.05
    bridge._joint_cb(joint_state(np.zeros(len(JOINT_NAMES))))
    bridge._policy_worker()
    assert client.calls == ['get_action']


def test_invalid_feedback_and_task_change_reset_policy():
    bridge, client = ready_bridge()
    bridge._policy_worker()
    client.calls.clear()

    bridge._joint_cb(NS(name=JOINT_NAMES, position=[np.nan] * 6))
    bridge._joint_cb(joint_state(np.zeros(len(JOINT_NAMES))))
    bridge._policy_worker()
    assert client.calls == ['reset', 'get_action']

    client.calls.clear()
    bridge._set_task(instruction='place the green block in the right bin')
    bridge._policy_worker()
    assert client.calls == ['reset', 'get_action']
    assert bridge.instruction == 'place the green block in the right bin'


def test_failed_reset_blocks_commands_until_it_succeeds():
    bridge, client = ready_bridge()
    bridge._modality_configured = True
    client.reset = MagicMock(side_effect=[RuntimeError('server restarting'), {'reset': True}])
    bridge._policy_worker()
    bridge.command_pub.publish.assert_not_called()
    bridge._policy_worker()
    assert client.reset.call_count == 2
    bridge.command_pub.publish.assert_called_once()


def test_action_computed_across_an_episode_boundary_is_dropped():
    bridge, client = ready_bridge()
    bridge._modality_configured = True
    bridge._reset_pending = False

    def get_action(observation):
        bridge._request_reset('simulator restarted')  # arrives mid-request
        return [{'joint_positions': np.zeros((1, 1, 6))}, {}]

    client.get_action = get_action
    bridge._policy_worker()
    bridge.command_pub.publish.assert_not_called()
    assert bridge._reset_pending


def test_native_resolution_frames_are_sent_unchanged():
    # The camera is the D435i's 16:9 colour stream (640x360), sim and real.
    assert (CAMERA_WIDTH, CAMERA_HEIGHT) == (640, 360)
    h, w = CAMERA_HEIGHT, CAMERA_WIDTH
    bridge, _ = ready_bridge(width=0, height=0)
    image = np.random.default_rng(0).integers(0, 255, (h, w, 3), dtype=np.uint8)
    bridge._image_cb(NS(encoding='rgb8', height=h, width=w, step=w * 3,
                        data=image.tobytes()))
    depth = np.full((h, w), 0.7, dtype=np.float32)
    bridge._depth_cb(NS(encoding='32FC1', height=h, width=w, step=w * 4,
                        data=depth.tobytes()))

    observation = bridge._observation()

    assert observation['video']['front'].shape == (1, 1, h, w, 3)
    np.testing.assert_array_equal(observation['video']['front'][0, 0], image)
    assert observation['depth']['front_depth'].shape == (1, 1, h, w, 1)


def test_placeholder_frame_before_the_first_image_has_the_camera_shape():
    bridge, _ = ready_bridge(width=0, height=0)
    assert bridge._observation()['video']['front'].shape == (1, 1, CAMERA_HEIGHT, CAMERA_WIDTH, 3)


def test_resize_preserves_aspect_ratio_instead_of_squashing():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:, 320:] = 255  # right half white
    boxed = letterbox_nearest(image, 224, 224)
    assert boxed.shape == (224, 224, 3)
    # 640x480 -> 224x168 content, centred with 28-row bands top and bottom.
    assert not boxed[:28].any() and not boxed[-28:].any()
    content = boxed[28:-28]
    assert content.shape[:2] == (168, 224)
    assert not content[:, :111].any() and content[:, 113:].all()
    depth = letterbox_nearest(np.ones((480, 640), np.float32), 224, 224, fill=np.nan)
    assert np.isnan(depth[0]).all() and np.all(depth[28:-28] == 1.0)
    # The 16:9 camera: 640x360 -> 224x126 content, 49-row bands.
    wide = letterbox_nearest(np.full((360, 640, 3), 255, np.uint8), 224, 224)
    assert not wide[:49].any() and not wide[-49:].any() and wide[49:-49].all()


def published_stats(bridge):
    return bridge.reports


def test_command_reports_latency_age_saturation_and_ground_truth(monkeypatch):
    clock = [50.0]
    monkeypatch.setattr('groot_so101_bridge.time.monotonic', lambda: clock[0])
    bridge, client = ready_bridge()
    bridge._modality_configured = True
    bridge._image_cb(NS(encoding='rgb8', height=12, width=16, step=48,
                        data=np.zeros(12 * 16 * 3, np.uint8).tobytes()))

    def slow_action(target, info):
        def get_action(observation):
            clock[0] += 0.04
            return [{'joint_positions': np.asarray(target).reshape(1, 1, 6)}, info]
        return get_action

    client.get_action = slow_action([0.01] * 5 + [0.0], {})
    bridge._policy_worker()
    client.get_action = slow_action([1.0] * 6, {'policy_used_ground_truth': True})
    bridge._policy_worker()

    first, second = published_stats(bridge)
    assert first['policy_latency_ms'] == pytest.approx(40.0)
    assert first['observation_age_ms'] == pytest.approx(40.0)
    assert first['command_saturated'] is False and first['policy_used_ground_truth'] is False
    assert second['observation_age_ms'] == pytest.approx(80.0)
    assert second['command_saturated'] is True  # 1 rad request clipped to the step limit
    assert second['policy_used_ground_truth'] is True


def test_demo_policy_reports_ground_truth_and_mock_does_not():
    scenario = json.loads(
        (policy_server.Path(__file__).resolve().parents[1]
         / 'systems/so101/scenarios/picking_table.json').read_text())
    scenario['selected_task'] = scenario['tasks'][0]
    bridge, _ = ready_bridge(policy_server.DemoSO101Policy(scenario))
    bridge._policy_worker()
    bridge.command_pub.publish.assert_called_once()
    assert published_stats(bridge)[0]['policy_used_ground_truth'] is True

    bridge, _ = ready_bridge(policy_server.MockSO101Policy())
    bridge._policy_worker()
    assert published_stats(bridge)[0]['policy_used_ground_truth'] is False


def test_object_poses_are_only_sent_when_ground_truth_is_enabled():
    bridge, _ = ready_bridge()
    bridge.scenario = {'name': 'table', 'selected_task': {'id': 't'},
                       'objects': [{'id': 'red', 'xyz': [0.3, -0.1, 0.48]}]}
    metadata = bridge._observation()['metadata']['scenario']
    assert 'objects' not in metadata and metadata['selected_task'] == {'id': 't'}
    bridge.send_ground_truth = True
    assert bridge._observation()['metadata']['scenario']['objects']
    assert bridge._policy_used_ground_truth([{}, {}]) is True
