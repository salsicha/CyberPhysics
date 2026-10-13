"""Guard the physical executor boundary without requiring Isaac or CUDA in CI."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("peract_contracts", ROOT / "applications/peract/scripts/contracts.py")
contracts = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contracts)
CONFIG = json.loads((ROOT / "systems/franka/config/peract_reach.json").read_text())


def action(**changes):
    return dict(position=[0.5, 0.0, 0.2], euler_degrees=[-180, 0, 0],
                gripper_open=True, ignore_collisions=False) | changes


@pytest.mark.parametrize("changes", [
    {"position": [float("nan"), 0, 0.2]},
    {"position": [0.5, 0, 0.02]},
    {"position": [0.5, 0, 1]},
    {"position": [0.5, 0]},
    {"euler_degrees": [0, float("inf"), 0]},
    {"ignore_collisions": True},
    {"gripper_open": False},
])
def test_unsafe_prediction_is_rejected_not_repaired(changes):
    with pytest.raises(ValueError):
        contracts.checked_action(action(**changes), CONFIG)


def test_valid_prediction_is_not_replaced_by_scene_truth():
    predicted = action(position=[0.43, -0.08, 0.21])
    xyz, _ = contracts.checked_action(predicted, CONFIG)
    np.testing.assert_array_equal(xyz, predicted["position"])


def test_randomized_scene_keeps_both_targets_in_reach_and_separated():
    rng = np.random.default_rng(23)
    for _ in range(100):
        positions = contracts.sample_scene(rng, CONFIG)
        assert np.linalg.norm(positions[0] - positions[1]) > 0.15
        assert (positions[:, :2] >= CONFIG["spawn_bounds"][:2]).all()
        assert (positions[:, :2] <= CONFIG["spawn_bounds"][2:]).all()


def test_geometry_change_invalidates_checkpoint_contract():
    changed = dict(CONFIG, reach_height=0.3)
    assert contracts.config_digest(changed) != contracts.config_digest(CONFIG)
