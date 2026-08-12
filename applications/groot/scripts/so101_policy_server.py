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

    model_path = str(args.model_path)
    local_model_path = Path(model_path).expanduser()
    if model_path.startswith("/") and not local_model_path.is_dir():
        raise SystemExit(
            f"SO-101 GR00T checkpoint directory does not exist: {local_model_path}. "
            "Install a fine-tuned SO-100/SO-101 checkpoint there or set "
            "GR00T_MODEL_PATH to its Hugging Face repository ID."
        )

    embodiment_tag = EmbodimentTag.resolve(args.embodiment_tag)
    if model_path.rstrip("/") == "nvidia/GR00T-N1.7-3B" and embodiment_tag.name == "NEW_EMBODIMENT":
        raise SystemExit(
            "nvidia/GR00T-N1.7-3B does not contain an SO-101 NEW_EMBODIMENT "
            "policy. Fine-tune it on an SO-100/SO-101 dataset first, then set "
            "GR00T_MODEL_PATH to that checkpoint."
        )

    hf_home = Path(os.environ.get("HF_HOME", Path.home() / ".cache" / "huggingface"))
    if not os.environ.get("HF_TOKEN") and not (hf_home / "token").is_file():
        print(
            "WARNING: no Hugging Face token was found. GR00T N1.7 checkpoints "
            "load the gated nvidia/Cosmos-Reason2-2B backbone; authenticate or "
            "model loading may fail with GatedRepoError.",
            flush=True,
        )

    if str(args.device).startswith("cuda"):
        import torch

        if not torch.cuda.is_available():
            raise SystemExit(f"GR00T device {args.device!r} requested, but CUDA is unavailable")
        total_gib = torch.cuda.get_device_properties(0).total_memory / (1024**3)
        if total_gib < 16.0 and os.environ.get("SO101_GROOT_ALLOW_LOW_VRAM", "false").lower() not in (
            "1",
            "true",
            "yes",
        ):
            raise SystemExit(
                f"GR00T N1.7 requires at least 16 GiB VRAM for inference; "
                f"this GPU reports {total_gib:.1f} GiB. Use a remote policy server "
                "or set SO101_GROOT_ALLOW_LOW_VRAM=true to attempt it anyway."
            )

    print(
        f"Loading real SO-101 GR00T policy checkpoint={model_path} "
        f"embodiment={embodiment_tag.name} device={args.device}",
        flush=True,
    )
    return Gr00tPolicy(
        embodiment_tag=embodiment_tag,
        model_path=model_path,
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
    parser.add_argument("--embodiment-tag", default=os.environ.get("GR00T_EMBODIMENT_TAG", "NEW_EMBODIMENT"))
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
