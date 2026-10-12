import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

# GR00T has a script with the same filename. Import this adapter by its path
# so collecting both applications' tests cannot silently select the other one.
spec = importlib.util.spec_from_file_location(
    "smolvla_so101_policy_server",
    Path(__file__).resolve().parents[1] / "scripts" / "so101_policy_server.py",
)
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)
_latest_joint_state = server._latest_joint_state
_latest_rgb = server._latest_rgb
_task_text = server._task_text
isaac_to_so101_units = server.isaac_to_so101_units
so101_to_isaac_units = server.so101_to_isaac_units


class SO101AdapterTest(unittest.TestCase):
    def test_unit_conversion_round_trip(self):
        isaac = np.array([0.2, -0.7, 1.1, -0.3, 0.8, 0.025], dtype=np.float32)
        converted = isaac_to_so101_units(isaac)
        self.assertAlmostEqual(float(converted[-1]), 62.5, places=4)
        np.testing.assert_allclose(so101_to_isaac_units(converted), isaac, atol=1e-6)

    def test_observation_extractors_use_latest_values(self):
        state = np.arange(24, dtype=np.float32).reshape(1, 4, 6)
        image = np.zeros((1, 1, 8, 10, 3), dtype=np.uint8)
        image[..., 2] = 17
        observation = {
            "state": {"joint_positions": state},
            "video": {"front": image},
            "language": {"task": [["pick the red block"]]},
        }
        np.testing.assert_array_equal(_latest_joint_state(observation), state.reshape(-1, 6)[-1])
        self.assertEqual(_latest_rgb(observation).shape, (8, 10, 3))
        self.assertEqual(int(_latest_rgb(observation)[0, 0, 2]), 17)
        self.assertEqual(_task_text(observation), "pick the red block")

    def test_joint_mapping_applies_offsets_signs_and_gripper_range(self):
        mapping = server.load_joint_mapping(None)
        mapping = {
            "joints": {name: dict(entry) for name, entry in mapping["joints"].items()},
            "gripper": {"closed": 95.0, "open": 5.0},
        }
        mapping["joints"]["shoulder_lift"] = {"sign": -1.0, "offset_deg": 90.0}
        # LeRobot reports 0 deg at its calibration mid-range; here that pose is
        # URDF +90 deg and the joint turns the other way.
        lerobot = np.array([10.0, 0.0, -20.0, 0.0, 0.0, 95.0], dtype=np.float32)
        isaac = server.so101_to_isaac_units(lerobot, mapping)
        np.testing.assert_allclose(isaac[:5], np.deg2rad([10.0, 90.0, -20.0, 0.0, 0.0]), atol=1e-6)
        self.assertAlmostEqual(float(isaac[5]), 0.0, places=6)  # 95 is "closed"
        np.testing.assert_allclose(server.isaac_to_so101_units(isaac, mapping), lerobot, atol=1e-4)

    def test_repository_joint_mapping_file_is_valid(self):
        path = Path(__file__).resolve().parents[3] / "systems/so101/config/lerobot_joint_mapping.json"
        if not path.is_file():
            self.skipTest("repository systems/ directory is not mounted")
        mapping = server.load_joint_mapping(str(path))
        self.assertEqual(set(mapping["joints"]), set(server.ARM_JOINT_NAMES))
        # Nominal calibration derived from so101_new_calib.urdf, not the identity placeholder.
        self.assertFalse(server.is_identity_mapping(mapping))
        self.assertEqual(mapping["joints"]["shoulder_pan"]["sign"], -1.0)
        self.assertEqual(mapping["joints"]["elbow_flex"]["offset_deg"], 73.9)

    def test_checkpoint_range_is_checked_against_the_urdf_limits(self):
        repo = Path(__file__).resolve().parents[3]
        mapping_path = repo / "systems/so101/config/lerobot_joint_mapping.json"
        limits = server.load_urdf_arm_limits()
        if limits is None or not mapping_path.is_file():
            self.skipTest("repository systems/ directory is not mounted")
        mapping = server.load_joint_mapping(str(mapping_path))
        # LeRobot degrees at the ends of so101_new_calib.urdf's range (zero =
        # calibration mid-range) fit the URDF through the nominal mapping.
        low = np.degrees([-1.91986, -1.74533, -1.69, -1.65806, -2.74385])
        high = np.degrees([1.91986, 1.74533, 1.69, 1.65806, 2.84121])
        self.assertEqual(server.joints_outside_urdf_limits(low, high, mapping, limits), [])
        # The identity mapping puts the elbow 97 deg backwards, which the arm cannot reach.
        identity = server.load_joint_mapping(None)
        self.assertEqual(server.joints_outside_urdf_limits(low, high, identity, limits),
                         ["shoulder_lift", "elbow_flex"])
        policy = server.SmolVLASO101Policy.__new__(server.SmolVLASO101Policy)
        policy.joint_mapping, policy.joint_limits = identity, limits
        normalizer = SimpleNamespace(stats={"observation.state": {
            "min": np.append(low, 0.0), "max": np.append(high, 100.0)}})
        policy.preprocessor = SimpleNamespace(steps=[normalizer])
        with patch("builtins.print") as printed:
            policy._check_checkpoint_units(skip=False)
        self.assertIn("elbow_flex", printed.call_args.args[0])

    def test_lerobot_range_statistics_are_detected(self):
        # use_degrees=False data saturates at exactly +/-100 and never exceeds it.
        self.assertTrue(server.looks_like_lerobot_range_values(
            [-100.0, -98.2, -100.0, -97.0, -100.0, 0.0], [100.0, 99.1, 87.0, 100.0, 100.0, 100.0]))
        # Degree data exceeding 100 (e.g. wrist roll) or not saturating is not flagged.
        self.assertFalse(server.looks_like_lerobot_range_values(
            [-110.0, -95.0, -90.0, -96.0, -175.0, 0.0], [110.0, 95.0, 90.0, 96.0, 175.0, 100.0]))
        self.assertFalse(server.looks_like_lerobot_range_values(
            [-60.0, -45.0, -30.0, -50.0, -20.0, 0.0], [60.0, 45.0, 30.0, 50.0, 20.0, 100.0]))

    def test_checkpoint_state_range_reads_normalizer_stats(self):
        normalizer = SimpleNamespace(stats={"observation.state": {
            "min": np.full(6, -100.0), "max": np.full(6, 100.0), "mean": np.zeros(6)}})
        preprocessor = SimpleNamespace(steps=[SimpleNamespace(), normalizer])
        low, high = server.checkpoint_state_range(preprocessor)
        self.assertTrue(server.looks_like_lerobot_range_values(low, high))
        self.assertIsNone(server.checkpoint_state_range(SimpleNamespace(steps=[normalizer.__class__()])))

    def test_checkpoint_range_reads_loaded_tensors_instead_of_empty_constructor_stats(self):
        normalizer = SimpleNamespace(stats={}, _tensor_stats={'observation.state': {
            'min': np.full(6, -100.0), 'max': np.full(6, 100.0)}})
        low, high = server.checkpoint_state_range(SimpleNamespace(steps=[normalizer]))
        self.assertTrue(server.looks_like_lerobot_range_values(low, high))

    def test_required_normalization_cannot_silently_become_identity(self):
        mode = SimpleNamespace(value='MEAN_STD')
        norm_map = {'STATE': mode, 'ACTION': mode}
        state = SimpleNamespace(type='STATE', shape=(6,))
        action = SimpleNamespace(type='ACTION', shape=(6,))
        config = SimpleNamespace(input_features={'observation.state': state},
                                 output_features={'action': action}, normalization_mapping=norm_map)
        pre_step = SimpleNamespace(features=config.input_features, norm_map=norm_map,
                                   stats={}, _tensor_stats={'observation.state': {
                                       'mean': np.zeros(6), 'std': np.ones(6)}})
        post_step = SimpleNamespace(features=config.output_features, norm_map=norm_map,
                                    stats={}, _tensor_stats={'action': {
                                        'mean': np.zeros(6), 'std': np.ones(6)}})
        pre, post = SimpleNamespace(steps=[pre_step]), SimpleNamespace(steps=[post_step])
        server.require_checkpoint_statistics(config, pre, post)
        for broken in ({}, {'so100.buffer.action': {'mean': np.zeros(6), 'std': np.ones(6)}},
                       {'action': {'mean': np.zeros(6)}},
                       {'action': {'mean': np.zeros(6), 'std': np.full(6, np.inf)}},
                       {'action': {'mean': np.zeros(5), 'std': np.ones(6)}},
                       {'action': {'mean': np.zeros(6), 'std': -np.ones(6)}}):
            with self.subTest(statistics=broken):
                post_step._tensor_stats = broken
                with self.assertRaisesRegex(ValueError, 'statistics for action'):
                    server.require_checkpoint_statistics(config, pre, post)

    def test_invalid_joint_vector_is_rejected(self):
        with self.assertRaises(ValueError):
            isaac_to_so101_units(np.zeros(5, dtype=np.float32))

    def test_malformed_request_does_not_stop_next_request(self):
        socket = MagicMock()
        socket.recv.side_effect = [b'\xc1', server.pack({'endpoint': 'health'}), KeyboardInterrupt]
        policy = SimpleNamespace(model_path='test', device='cpu')
        with patch.object(server.zmq, 'Context') as context:
            context.return_value.socket.return_value = socket
            server.PolicyServer(policy, '127.0.0.1', 5556).run()
        replies = [server.unpack(call.args[0]) for call in socket.send.call_args_list]
        self.assertIn('error', replies[0])
        self.assertEqual(replies[1]['status'], 'ok')
        socket.close.assert_called_once()

    def test_unserializable_response_does_not_stop_next_request(self):
        socket = MagicMock()
        socket.recv.side_effect = [server.pack({'endpoint': 'reset'}),
                                  server.pack({'endpoint': 'health'}), KeyboardInterrupt]
        policy = SimpleNamespace(model_path='test', device='cpu', reset=lambda: object())
        with patch.object(server.zmq, 'Context') as context:
            context.return_value.socket.return_value = socket
            server.PolicyServer(policy, '127.0.0.1', 5556).run()
        replies = [server.unpack(call.args[0]) for call in socket.send.call_args_list]
        self.assertIn('error', replies[0])
        self.assertEqual(replies[1]['status'], 'ok')


if __name__ == "__main__":
    unittest.main()
