#!/usr/bin/env python3
import argparse
import json
import math
import os
from pathlib import Path
from typing import Any

import numpy as np
from gr00t.policy.policy import BasePolicy
from gr00t.policy.server_client import PolicyServer

from so101_task_policy import ScriptedPickPlaceController

JOINT_LIMITS = np.array([
    [-1.9199, 1.9199],
    [-1.7453, 1.7453],
    [-1.7453, 1.7453],
    [-1.6581, 1.6581],
    [-2.7439, 2.7439],
    [0.0, 0.04],
], dtype=np.float32)


class MockSO101Policy(BasePolicy):
    def __init__(self, amplitude: float = 0.35):
        super().__init__(strict=False)
        self.step = 0
        self.amplitude = amplitude

    def check_observation(self, observation: dict[str, Any]) -> None:
        pass

    def check_action(self, action: dict[str, Any]) -> None:
        pass

    def reset(self, options: dict[str, Any] | None = None) -> dict[str, Any]:
        self.step = 0
        return {"reset": True}

    def get_modality_config(self) -> dict[str, Any]:
        return {
            "video": {"modality_keys": ["front"], "delta_indices": [0]},
            "state": {"modality_keys": ["joint_positions"], "delta_indices": [0]},
            "action": {"modality_keys": ["joint_positions"], "delta_indices": [0]},
        }

    def _get_action(self, observation: dict[str, Any], options: dict[str, Any] | None = None):
        state = observation.get("state", {}).get("joint_positions")
        if state is not None:
            current = np.asarray(state, dtype=np.float32).reshape(-1, 6)[-1]
        else:
            current = np.zeros(6, dtype=np.float32)

        phase = self.step * 0.18
        target = np.array([
            self.amplitude * math.sin(phase),
            -0.45 + 0.18 * math.sin(phase * 0.7),
            0.75 + 0.22 * math.sin(phase * 0.9 + 0.8),
            -0.25 + 0.18 * math.sin(phase * 1.1),
            0.45 * math.sin(phase * 0.8 + 1.1),
            0.02 + 0.015 * math.sin(phase * 1.3),
        ], dtype=np.float32)
        target = np.clip(target, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        action = current + np.clip(target - current, -0.08, 0.08)
        action = np.clip(action, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        self.step += 1
        return {"joint_positions": action.reshape(1, 1, 6)}, {"mode": "mock", "step": self.step}


class DemoSO101Policy(BasePolicy):
    """GR00T-compatible deterministic baseline for the tabletop task demo."""

    def __init__(self, scenario=None):
        super().__init__(strict=False)
        self.controller = ScriptedPickPlaceController(scenario=scenario)
        self.last_waypoint = None

    def check_observation(self, observation: dict[str, Any]) -> None:
        pass

    def check_action(self, action: dict[str, Any]) -> None:
        pass

    def reset(self, options: dict[str, Any] | None = None) -> dict[str, Any]:
        self.controller.reset()
        self.last_waypoint = None
        return {"reset": True}

    def get_modality_config(self) -> dict[str, Any]:
        return {
            "video": {"modality_keys": ["front"], "delta_indices": [0]},
            "state": {"modality_keys": ["joint_positions"], "delta_indices": [0]},
            "action": {"modality_keys": ["joint_positions"], "delta_indices": [0]},
        }

    def _get_action(self, observation: dict[str, Any], options: dict[str, Any] | None = None):
        action = self.controller.get_action(observation)
        waypoint = self.controller.status
        if waypoint != self.last_waypoint:
            print(f"SO-101 GR00T waypoint: {waypoint}", flush=True)
            self.last_waypoint = waypoint
        return {
            "joint_positions": action.reshape(1, 1, 6)
        }, {"mode": "demo", "waypoint": waypoint}


def build_real_policy(args):
    from gr00t.data.embodiment_tags import EmbodimentTag
    from gr00t.policy.gr00t_policy import Gr00tPolicy

    return Gr00tPolicy(
        embodiment_tag=EmbodimentTag.resolve(args.embodiment_tag),
        model_path=args.model_path,
        device=args.device,
        strict=args.strict,
    )


def load_demo_scenario(path, task_id=""):
    if not path:
        return None
    scenario_path = Path(path)
    try:
        scenario = json.loads(scenario_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Unable to load demo scenario {scenario_path}: {exc}") from exc
    tasks = scenario.get("tasks", [])
    if task_id:
        selected = next((task for task in tasks if task.get("id") == task_id), None)
        if selected is None:
            raise SystemExit(f"Task {task_id!r} not found in demo scenario {scenario_path}")
        scenario["selected_task"] = selected
    elif tasks:
        scenario["selected_task"] = tasks[0]
    return scenario


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument(
        "--mode",
        choices=["demo", "mock", "real"],
        default=os.environ.get("SO101_GROOT_MODE", "demo"),
    )
    parser.add_argument("--model-path", default=os.environ.get("GR00T_MODEL_PATH"))
    parser.add_argument("--embodiment-tag", default=os.environ.get("GR00T_EMBODIMENT_TAG", "new_embodiment"))
    parser.add_argument("--device", default=os.environ.get("GR00T_DEVICE", "cuda"))
    parser.add_argument("--scenario-file", default=os.environ.get("SO101_SCENARIO_FILE", ""))
    parser.add_argument("--task-id", default=os.environ.get("SO101_TASK_ID", ""))
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--mock-amplitude", type=float, default=0.35)
    args = parser.parse_args()

    if args.mode == "real":
        if not args.model_path:
            raise SystemExit("--model-path or GR00T_MODEL_PATH is required for --mode real")
        policy = build_real_policy(args)
    elif args.mode == "demo":
        policy = DemoSO101Policy(load_demo_scenario(args.scenario_file, args.task_id))
    else:
        policy = MockSO101Policy(amplitude=args.mock_amplitude)

    print(f"Starting SO-101 GR00T policy server mode={args.mode} host={args.host} port={args.port}", flush=True)
    PolicyServer(policy=policy, host=args.host, port=args.port).run()


if __name__ == "__main__":
    main()
