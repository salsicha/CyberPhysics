#!/usr/bin/env python3

import unittest
import importlib.util
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
