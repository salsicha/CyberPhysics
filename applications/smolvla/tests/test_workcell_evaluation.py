"""Regression checks for false-positive grasp/placement and policy scoring."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

SPEC = importlib.util.spec_from_file_location(
    "workcell_evaluation",
    Path(__file__).resolve().parents[1] / "scripts/evaluate_workcell.py",
)
workcell = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(workcell)


def placed(**changes):
    values = dict(
        position=[0.0, 0.0, 0.018],
        rotation=np.eye(3),
        half_size=[0.012] * 3,
        tray_xy=[0.0, 0.0],
        tray_half=0.037,
        speed=0.0,
        tcp_z=0.1,
        jaw=1.4,
    )
    values.update(changes)
    return workcell.stable_placement(**values)


def test_supported_released_block_is_placed():
    assert placed()


@pytest.mark.parametrize(
    "changes",
    [
        {"position": [0, 0, 0.04]},  # upstream counted this airborne block as success
        {"position": [0, 0, 0.012]},  # resting on floor, not tray
        {"position": [0.03, 0, 0.018]},  # center is inside, one edge is outside
        {"speed": 0.1},
        {"tcp_z": 0.04},
        {"jaw": 0.2},
    ],
)
def test_airborne_overhanging_moving_or_unreleased_block_is_not_placed(changes):
    assert not placed(**changes)


def test_rotated_corners_must_also_fit():
    c = np.sqrt(0.5)
    assert placed(position=[0.022, 0, 0.018])
    assert not placed(
        position=[0.022, 0, 0.018], rotation=[[c, -c, 0], [c, c, 0], [0, 0, 1]]
    )


@pytest.mark.parametrize(
    "action",
    [np.zeros(5), np.zeros((1, 6)), [0, 0, 0, 0, 0, np.nan], [0, 0, 0, 0, 0, np.inf]],
)
def test_bad_policy_output_is_rejected(action):
    with pytest.raises(ValueError, match="six finite"):
        workcell.checked_action(action, np.tile([-1, 1], (6, 1)))


def test_finite_commands_are_clamped_and_reported():
    bounded, clipped = workcell.checked_action(
        [2, -2, 0.1, 0, 0, 0], np.tile([-1, 1], (6, 1))
    )
    assert clipped
    np.testing.assert_equal(bounded, [1, -1, 0.1, 0, 0, 0])


def test_placement_cannot_hide_environment_collisions():
    report = workcell.summarize_episodes(
        [{"scenario": "nominal", "success": True, "collision_free_success": False}]
    )
    assert report["nominal_task_acceptance_passed"]
    assert not report["collision_free_acceptance_passed"]
    assert not report["passed"]


def test_robustness_failure_and_missing_nominal_cannot_pass():
    assert not workcell.summarize_episodes([])["passed"]
    report = workcell.summarize_episodes(
        [
            {"scenario": "nominal", "success": True, "collision_free_success": True},
            {"scenario": "visual", "success": False, "collision_free_success": False},
        ]
    )
    assert report["nominal_task_acceptance_passed"]
    assert not report["evaluated_robustness_acceptance_passed"]
    assert not report["passed"]
