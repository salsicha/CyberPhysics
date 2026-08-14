#!/usr/bin/env python3
"""Serve a LeRobot SmolVLA checkpoint to the CyberPhysics SO101 ROS bridge."""

from __future__ import annotations

import argparse
import os
import traceback
from pathlib import Path
from typing import Any

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq

JOINT_COUNT = 6
ARM_JOINT_COUNT = 5
ISAAC_GRIPPER_MAX_M = 0.04
SO101_GRIPPER_MAX = 100.0


def pack(data: Any) -> bytes:
    return msgpack.packb(data, default=mnp.encode, use_bin_type=True)


def unpack(data: bytes) -> Any:
    return msgpack.unpackb(data, object_hook=mnp.decode, raw=False)


def isaac_to_so101_units(state: np.ndarray) -> np.ndarray:
    """Convert Isaac radians/metres to LeRobot SO101 degrees/0-100."""
    state = _joint_vector(state, "Isaac state").copy()
    state[:ARM_JOINT_COUNT] = np.rad2deg(state[:ARM_JOINT_COUNT])
    state[-1] = np.clip(
        state[-1] / ISAAC_GRIPPER_MAX_M * SO101_GRIPPER_MAX,
        0.0,
        SO101_GRIPPER_MAX,
    )
    return state.astype(np.float32)


def so101_to_isaac_units(action: np.ndarray) -> np.ndarray:
    """Convert LeRobot SO101 degrees/0-100 to Isaac radians/metres."""
    action = _joint_vector(action, "SO101 action").copy()
    action[:ARM_JOINT_COUNT] = np.deg2rad(action[:ARM_JOINT_COUNT])
    action[-1] = (
        np.clip(action[-1], 0.0, SO101_GRIPPER_MAX)
        / SO101_GRIPPER_MAX
        * ISAAC_GRIPPER_MAX_M
    )
    return action.astype(np.float32)


def _joint_vector(value: Any, label: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float32).reshape(-1)
    if vector.size != JOINT_COUNT:
        raise ValueError(f"{label} must contain {JOINT_COUNT} values; got shape {np.shape(value)}")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{label} contains a non-finite value")
    return vector


def _latest_joint_state(observation: dict[str, Any]) -> np.ndarray:
    states = observation.get("state")
    if not isinstance(states, dict) or not states:
        raise ValueError("Observation is missing the state modality")
    value = states.get("joint_positions")
    if value is None:
        value = next(iter(states.values()))
    array = np.asarray(value, dtype=np.float32)
    if array.size < JOINT_COUNT or array.shape[-1] != JOINT_COUNT:
        raise ValueError(f"Expected SO101 state with final dimension 6; got {array.shape}")
    return _joint_vector(array.reshape(-1, JOINT_COUNT)[-1], "SO101 state")


def _latest_rgb(observation: dict[str, Any]) -> np.ndarray:
    videos = observation.get("video")
    if not isinstance(videos, dict) or not videos:
        raise ValueError("Observation is missing the video modality")
    value = videos.get("front")
    if value is None:
        value = next(iter(videos.values()))
    array = np.asarray(value)
    if array.ndim < 3 or array.shape[-1] not in (3, 4):
        raise ValueError(f"Expected an RGB image with final dimension 3 or 4; got {array.shape}")
    image = array.reshape((-1, *array.shape[-3:]))[-1, :, :, :3]
    if image.dtype != np.uint8:
        if np.issubdtype(image.dtype, np.floating) and image.size and float(image.max()) <= 1.0:
            image = image * 255.0
        image = np.clip(image, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(image)


def _first_string(value: Any) -> str | None:
    if isinstance(value, str):
        return value
    if isinstance(value, dict):
        for nested in value.values():
            result = _first_string(nested)
            if result is not None:
                return result
    if isinstance(value, (list, tuple)):
        for nested in value:
            result = _first_string(nested)
            if result is not None:
                return result
    return None


def _task_text(observation: dict[str, Any]) -> str:
    return _first_string(observation.get("language", {})) or "move the SO101 arm"


class SmolVLASO101Policy:
    def __init__(
        self,
        model_path: str,
        device: str,
        checkpoint_units: str,
        action_horizon: int,
        revision: str | None,
        local_files_only: bool,
    ) -> None:
        import torch
        from lerobot.configs import PreTrainedConfig
        from lerobot.policies import make_pre_post_processors
        from lerobot.policies.smolvla import SmolVLAPolicy

        self.torch = torch
        self.device = torch.device(device)
        self.model_path = model_path
        self.checkpoint_units = checkpoint_units

        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise SystemExit(f"SmolVLA device {device!r} requested, but CUDA is unavailable")
        if model_path.startswith("/") and not Path(model_path).is_dir():
            raise SystemExit(f"SmolVLA checkpoint directory does not exist: {model_path}")
        if local_files_only:
            os.environ["HF_HUB_OFFLINE"] = "1"

        config = PreTrainedConfig.from_pretrained(
            model_path,
            revision=revision,
            local_files_only=local_files_only,
        )
        if config.type != "smolvla":
            raise SystemExit(f"Checkpoint {model_path!r} is policy type {config.type!r}, not 'smolvla'")
        config.device = str(self.device)

        state_feature = config.robot_state_feature
        action_feature = config.action_feature
        if state_feature is None or tuple(state_feature.shape) != (JOINT_COUNT,):
            raise SystemExit(
                f"SmolVLA checkpoint must use a six-dimensional observation.state; got {state_feature}"
            )
        if action_feature is None or tuple(action_feature.shape) != (JOINT_COUNT,):
            raise SystemExit(
                f"SmolVLA checkpoint must use a six-dimensional action; got {action_feature}"
            )

        self.image_keys = list(config.image_features)
        if not self.image_keys:
            raise SystemExit("SmolVLA checkpoint has no visual input feature")
        config.n_action_steps = max(1, min(int(action_horizon), int(config.chunk_size)))

        print(
            f"Loading SmolVLA checkpoint={model_path} device={self.device} "
            f"units={checkpoint_units} images={self.image_keys} "
            f"action_horizon={config.n_action_steps}",
            flush=True,
        )

        if config.use_peft:
            from peft import PeftConfig, PeftModel

            peft_config = PeftConfig.from_pretrained(model_path, revision=revision)
            base_path = peft_config.base_model_name_or_path
            if not base_path:
                raise SystemExit("PEFT adapter does not identify its base SmolVLA checkpoint")
            base_policy = SmolVLAPolicy.from_pretrained(
                base_path,
                config=config,
                revision=revision,
            )
            self.policy = PeftModel.from_pretrained(
                base_policy,
                model_path,
                config=peft_config,
                is_trainable=False,
            )
            self.policy.to(self.device)
            self.policy.eval()
        else:
            self.policy = SmolVLAPolicy.from_pretrained(
                model_path,
                config=config,
                revision=revision,
                local_files_only=local_files_only,
            )

        self.preprocessor, self.postprocessor = make_pre_post_processors(
            config,
            model_path,
            pretrained_revision=revision,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}},
        )
        self.config = config
        self.reset()

    def reset(self) -> dict[str, Any]:
        self.policy.reset()
        if hasattr(self.preprocessor, "reset"):
            self.preprocessor.reset()
        if hasattr(self.postprocessor, "reset"):
            self.postprocessor.reset()
        return {"reset": True}

    @staticmethod
    def modality_config() -> dict[str, Any]:
        return {
            "video": {"modality_keys": ["front"], "delta_indices": [0]},
            "state": {"modality_keys": ["joint_positions"], "delta_indices": [0]},
            "action": {"modality_keys": ["joint_positions"], "delta_indices": [0]},
            "language": {"modality_keys": ["task"], "delta_indices": [0]},
        }

    def get_action(self, observation: dict[str, Any]) -> dict[str, np.ndarray]:
        from lerobot.policies.utils import prepare_observation_for_inference

        state = _latest_joint_state(observation)
        if self.checkpoint_units == "so101":
            policy_state = isaac_to_so101_units(state)
        else:
            policy_state = state

        image = _latest_rgb(observation)
        task = _task_text(observation)
        frame: dict[str, np.ndarray] = {"observation.state": policy_state}
        for image_key in self.image_keys:
            frame[image_key] = image.copy()

        batch = prepare_observation_for_inference(
            frame,
            device=self.device,
            task=task,
            robot_type="so101_follower",
        )
        batch = self.preprocessor(batch)
        with self.torch.inference_mode():
            action = self.policy.select_action(batch)
            action = self.postprocessor(action)

        target = _joint_vector(action.detach().cpu().numpy(), "SmolVLA action")
        if self.checkpoint_units == "so101":
            target = so101_to_isaac_units(target)
        return {"joint_positions": target.reshape(1, 1, JOINT_COUNT)}


class PolicyServer:
    def __init__(self, policy: SmolVLASO101Policy, host: str, port: int) -> None:
        self.policy = policy
        self.host = host
        self.port = port

    def run(self) -> None:
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.setsockopt(zmq.LINGER, 0)
        socket.bind(f"tcp://{self.host}:{self.port}")
        print(f"SmolVLA SO101 policy server listening on {self.host}:{self.port}", flush=True)
        try:
            while True:
                request = unpack(socket.recv())
                try:
                    response = self._dispatch(request)
                except Exception as exc:
                    traceback.print_exc()
                    response = {"error": f"{type(exc).__name__}: {exc}"}
                socket.send(pack(response))
        except KeyboardInterrupt:
            pass
        finally:
            socket.close()
            context.term()

    def _dispatch(self, request: Any) -> Any:
        if not isinstance(request, dict):
            raise ValueError("Policy request must be a mapping")
        endpoint = request.get("endpoint")
        if endpoint == "health":
            return {
                "status": "ok",
                "model": self.policy.model_path,
                "device": str(self.policy.device),
            }
        if endpoint == "get_modality_config":
            return self.policy.modality_config()
        if endpoint == "reset":
            return self.policy.reset()
        if endpoint == "get_action":
            data = request.get("data")
            if not isinstance(data, dict) or not isinstance(data.get("observation"), dict):
                raise ValueError("get_action requires data.observation")
            return self.policy.get_action(data["observation"])
        raise ValueError(f"Unknown policy endpoint: {endpoint!r}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model-path",
        default=os.environ.get("SMOLVLA_MODEL_PATH", "lerobot/smolvla_base"),
    )
    parser.add_argument("--device", default=os.environ.get("SMOLVLA_DEVICE", "cuda"))
    parser.add_argument(
        "--checkpoint-units",
        choices=["so101", "isaac"],
        default=os.environ.get("SMOLVLA_CHECKPOINT_UNITS", "so101"),
        help="'so101' means arm degrees and gripper 0-100; 'isaac' means radians and metres",
    )
    parser.add_argument(
        "--action-horizon",
        type=int,
        default=int(os.environ.get("SMOLVLA_ACTION_HORIZON", "8")),
    )
    parser.add_argument("--revision", default=os.environ.get("SMOLVLA_REVISION") or None)
    parser.add_argument(
        "--local-files-only",
        action="store_true",
        default=os.environ.get("SMOLVLA_LOCAL_FILES_ONLY", "false").lower()
        in ("1", "true", "yes"),
    )
    parser.add_argument("--host", default=os.environ.get("SMOLVLA_BIND_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("SMOLVLA_PORT", "5556")))
    args = parser.parse_args()

    policy = SmolVLASO101Policy(
        model_path=args.model_path,
        device=args.device,
        checkpoint_units=args.checkpoint_units,
        action_horizon=args.action_horizon,
        revision=args.revision,
        local_files_only=args.local_files_only,
    )
    PolicyServer(policy, args.host, args.port).run()


if __name__ == "__main__":
    main()
