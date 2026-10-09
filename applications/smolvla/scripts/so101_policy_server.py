#!/usr/bin/env python3
"""Serve a LeRobot SmolVLA checkpoint to the CyberPhysics SO101 ROS bridge."""

from __future__ import annotations

import argparse
import json
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
ARM_JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
ISAAC_GRIPPER_MAX_M = 0.04
SO101_GRIPPER_MAX = 100.0
# LeRobot so101_follower with use_degrees=False (its default) reports arm
# joints as -100..100 of the calibrated range, not degrees.
LEROBOT_RANGE_LIMIT = 100.0

# Map LeRobot SO-101 degrees (zero = the calibration mid-range pose) to this
# URDF (zero = arm straight up): urdf_deg = sign * lerobot_deg + offset_deg.
# The identity default is a placeholder: measure offset_deg/sign on your arm
# (see systems/so101/config/lerobot_joint_mapping.json).
IDENTITY_JOINT_MAPPING = {
    "joints": {name: {"sign": 1.0, "offset_deg": 0.0} for name in ARM_JOINT_NAMES},
    "gripper": {"closed": 0.0, "open": SO101_GRIPPER_MAX},
}


def load_joint_mapping(path: str | None) -> dict[str, Any]:
    """Load and validate a LeRobot-degrees to URDF joint mapping file."""
    if not path:
        return IDENTITY_JOINT_MAPPING
    data = json.loads(Path(path).read_text())
    joints = data.get("joints", {})
    mapping = {"joints": {}, "gripper": dict(IDENTITY_JOINT_MAPPING["gripper"])}
    for name in ARM_JOINT_NAMES:
        entry = joints.get(name, {})
        sign = float(entry.get("sign", 1.0))
        offset = float(entry.get("offset_deg", 0.0))
        if sign not in (-1.0, 1.0) or not np.isfinite(offset):
            raise ValueError(f"{path}: {name} needs sign +1/-1 and a finite offset_deg")
        mapping["joints"][name] = {"sign": sign, "offset_deg": offset}
    gripper = data.get("gripper", {})
    closed = float(gripper.get("closed", mapping["gripper"]["closed"]))
    opened = float(gripper.get("open", mapping["gripper"]["open"]))
    if not np.isfinite(closed) or not np.isfinite(opened) or closed == opened:
        raise ValueError(f"{path}: gripper closed/open values must be finite and differ")
    mapping["gripper"] = {"closed": closed, "open": opened}
    return mapping


def is_identity_mapping(mapping: dict[str, Any]) -> bool:
    return all(
        entry["sign"] == 1.0 and entry["offset_deg"] == 0.0
        for entry in mapping["joints"].values()
    )


def _mapping_arrays(mapping: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    joints = mapping["joints"]
    signs = np.asarray([joints[name]["sign"] for name in ARM_JOINT_NAMES], dtype=np.float32)
    offsets = np.asarray([joints[name]["offset_deg"] for name in ARM_JOINT_NAMES], dtype=np.float32)
    return signs, offsets


def looks_like_lerobot_range_values(minimum: Any, maximum: Any) -> bool:
    """True when arm-joint extremes look like LeRobot -100..100 normalised data.

    Normalised values are clipped to exactly +/-100 at the calibrated limits
    and never exceed them; degree data rarely saturates at exactly 100.
    """
    low = np.asarray(minimum, dtype=np.float64).reshape(-1)[:ARM_JOINT_COUNT]
    high = np.asarray(maximum, dtype=np.float64).reshape(-1)[:ARM_JOINT_COUNT]
    if low.size != ARM_JOINT_COUNT or high.size != ARM_JOINT_COUNT:
        return False
    extremes = np.concatenate([low, high])
    if not np.all(np.isfinite(extremes)) or np.any(np.abs(extremes) > LEROBOT_RANGE_LIMIT + 1e-3):
        return False
    return bool(np.any(np.abs(np.abs(extremes) - LEROBOT_RANGE_LIMIT) <= 1e-3))


def pack(data: Any) -> bytes:
    return msgpack.packb(data, default=mnp.encode, use_bin_type=True)


def unpack(data: bytes) -> Any:
    return msgpack.unpackb(data, object_hook=mnp.decode, raw=False)


def isaac_to_so101_units(state: np.ndarray, mapping: dict[str, Any] = IDENTITY_JOINT_MAPPING) -> np.ndarray:
    """Convert Isaac radians/metres to LeRobot SO101 degrees (use_degrees=True) and gripper 0-100."""
    state = _joint_vector(state, "Isaac state").astype(np.float64)
    signs, offsets = _mapping_arrays(mapping)
    state[:ARM_JOINT_COUNT] = (np.rad2deg(state[:ARM_JOINT_COUNT]) - offsets) * signs
    closed, opened = mapping["gripper"]["closed"], mapping["gripper"]["open"]
    fraction = np.clip(state[-1] / ISAAC_GRIPPER_MAX_M, 0.0, 1.0)
    state[-1] = closed + fraction * (opened - closed)
    return state.astype(np.float32)


def so101_to_isaac_units(action: np.ndarray, mapping: dict[str, Any] = IDENTITY_JOINT_MAPPING) -> np.ndarray:
    """Convert LeRobot SO101 degrees (use_degrees=True) and gripper 0-100 to Isaac radians/metres."""
    action = _joint_vector(action, "SO101 action").astype(np.float64)
    signs, offsets = _mapping_arrays(mapping)
    action[:ARM_JOINT_COUNT] = np.deg2rad(signs * action[:ARM_JOINT_COUNT] + offsets)
    closed, opened = mapping["gripper"]["closed"], mapping["gripper"]["open"]
    fraction = np.clip((action[-1] - closed) / (opened - closed), 0.0, 1.0)
    action[-1] = fraction * ISAAC_GRIPPER_MAX_M
    return action.astype(np.float32)


def checkpoint_state_range(preprocessor: Any) -> tuple[np.ndarray, np.ndarray] | None:
    """Return the observation.state min/max a LeRobot normaliser was fitted on."""
    for step in getattr(preprocessor, "steps", None) or []:
        stats = getattr(step, "stats", None)
        if not isinstance(stats, dict):
            continue
        state = stats.get("observation.state")
        if not isinstance(state, dict) or "min" not in state or "max" not in state:
            continue
        values = []
        for key in ("min", "max"):
            value = state[key]
            if hasattr(value, "detach"):
                value = value.detach().cpu().numpy()
            values.append(np.asarray(value, dtype=np.float64))
        return values[0], values[1]
    return None


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
        joint_mapping: dict[str, Any] = IDENTITY_JOINT_MAPPING,
        skip_unit_check: bool = False,
    ) -> None:
        import torch
        from lerobot.configs import PreTrainedConfig
        from lerobot.policies import make_pre_post_processors
        from lerobot.policies.smolvla import SmolVLAPolicy

        self.torch = torch
        self.device = torch.device(device)
        self.model_path = model_path
        self.checkpoint_units = checkpoint_units
        self.joint_mapping = joint_mapping

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
        if self.checkpoint_units == "so101":
            self._check_checkpoint_units(skip_unit_check)
        self.reset()

    def _check_checkpoint_units(self, skip: bool) -> None:
        state_range = checkpoint_state_range(self.preprocessor)
        if state_range is None:
            print(
                "WARNING: could not read the checkpoint's observation.state statistics; "
                "assuming LeRobot SO101 degrees (use_degrees=True).",
                flush=True,
            )
            return
        if looks_like_lerobot_range_values(*state_range):
            message = (
                "SmolVLA checkpoint state statistics look like LeRobot -100..100 "
                "normalised joints (so101_follower use_degrees=False), not degrees: "
                f"min={np.round(state_range[0], 3).tolist()} max={np.round(state_range[1], 3).tolist()}. "
                "Converting them as degrees would command wrong joint angles. Re-record or "
                "fine-tune on use_degrees=True data, or set SMOLVLA_SKIP_UNIT_CHECK=true if "
                "the data really is in degrees."
            )
            if not skip:
                raise SystemExit(message)
            print(f"WARNING: {message}", flush=True)

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
            policy_state = isaac_to_so101_units(state, self.joint_mapping)
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
            target = so101_to_isaac_units(target, self.joint_mapping)
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
                payload = socket.recv()
                try:
                    request = unpack(payload)
                    response = self._dispatch(request)
                    response_payload = pack(response)
                except Exception as exc:
                    traceback.print_exc()
                    response_payload = pack({"error": f"{type(exc).__name__}: {exc}"})
                socket.send(response_payload)
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
        choices=["so101", "so101_range100", "isaac"],
        default=os.environ.get("SMOLVLA_CHECKPOINT_UNITS", "so101"),
        help=(
            "'so101': LeRobot so101_follower recorded with use_degrees=True (arm degrees, "
            "gripper 0-100); 'so101_range100': LeRobot's default use_degrees=False -100..100 "
            "range (rejected: needs the arm's calibration); 'isaac': radians and metres"
        ),
    )
    parser.add_argument(
        "--joint-mapping",
        default=os.environ.get("SMOLVLA_JOINT_MAPPING", ""),
        help="JSON sign/offset mapping from LeRobot degrees to the URDF (default: identity)",
    )
    parser.add_argument(
        "--skip-unit-check",
        action="store_true",
        default=os.environ.get("SMOLVLA_SKIP_UNIT_CHECK", "false").lower() in ("1", "true", "yes"),
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
    if args.checkpoint_units == "so101_range100":
        raise SystemExit(
            "LeRobot use_degrees=False checkpoints use a -100..100 range of each joint's "
            "calibrated travel; mapping it to Isaac radians needs that arm's calibration "
            "file. Record/fine-tune with use_degrees=True and use --checkpoint-units so101."
        )
    joint_mapping = load_joint_mapping(args.joint_mapping)
    if args.checkpoint_units == "so101" and is_identity_mapping(joint_mapping):
        print(
            "WARNING: identity joint mapping: assuming LeRobot's calibration zero equals "
            "the URDF zero for every arm joint. Calibrate SMOLVLA_JOINT_MAPPING "
            "(systems/so101/config/lerobot_joint_mapping.json) for your arm.",
            flush=True,
        )

    policy = SmolVLASO101Policy(
        model_path=args.model_path,
        device=args.device,
        checkpoint_units=args.checkpoint_units,
        action_horizon=args.action_horizon,
        revision=args.revision,
        local_files_only=args.local_files_only,
        joint_mapping=joint_mapping,
        skip_unit_check=args.skip_unit_check,
    )
    PolicyServer(policy, args.host, args.port).run()


if __name__ == "__main__":
    main()
