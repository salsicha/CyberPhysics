"""Reject mismatched or unusable observations before invoking the learned policy."""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("act3d_observations", ROOT / "applications/act3d/scripts/observations.py")
observations = importlib.util.module_from_spec(spec)
previous_path = sys.path.copy()
try:
    sys.path.insert(0, str(ROOT / "applications/peract/scripts"))
    spec.loader.exec_module(observations)
finally:
    sys.path[:] = previous_path
CONFIG = json.loads((ROOT / "systems/franka/config/act3d_reach.json").read_text())


def observation():
    return {"rgb": np.zeros((128, 128, 3), np.uint8),
            "points": np.full((128, 128, 3), [.5, 0, .1], np.float32),
            "gripper_pose": np.array([.4, 0, .3, 1, 0, 0, 0, 1], np.float32),
            "instruction": CONFIG["instructions"][0],
            "config_digest": observations.config_digest(CONFIG)}


@pytest.mark.parametrize("change,match", [
    ({"config_digest": "other-camera"}, "configurations differ"),
    ({"instruction": "pick up the block"}, "vocabulary"),
    ({"gripper_pose": np.zeros(8)}, "quaternion"),
    ({"gripper_pose": np.zeros(4)}, "shape"),
    ({"points": np.full((128, 128, 3), 200., np.float32)}, "metric world-space"),
    ({"points": np.full((128, 128, 3), np.nan, np.float32)}, "Invalid RGB-D"),
    ({"rgb": np.zeros((128, 128, 3), np.float32)}, "Invalid RGB-D"),
])
def test_bad_observations_fail_closed(change, match):
    with pytest.raises(ValueError, match=match):
        observations.checked_observation(observation() | change, CONFIG)


def test_runtime_observation_needs_no_object_poses_or_action_labels():
    obs = observation()
    rgb, points, gripper, instruction = observations.checked_observation(obs, CONFIG)
    np.testing.assert_array_equal(rgb, obs["rgb"])
    np.testing.assert_array_equal(points, obs["points"])
    np.testing.assert_array_equal(gripper, obs["gripper_pose"])
    assert instruction == obs["instruction"]


def test_sparse_finite_points_do_not_hide_bad_depth_calibration():
    obs = observation()
    obs["points"][:] = 100
    obs["points"][0, :31] = [.5, 0, .1]
    with pytest.raises(ValueError, match="Too few"):
        observations.checked_observation(obs, CONFIG)
