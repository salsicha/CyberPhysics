#!/usr/bin/env python3
import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import msgpack
import msgpack_numpy as mnp
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray, String
import zmq

from so101_common import (
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    JOINT_NAMES,
    POLICY_STATS_TOPIC,
    safe_hardware_target,
)


def pack(data: Any) -> bytes:
    return msgpack.packb(data, default=mnp.encode, use_bin_type=True)


def unpack(data: bytes) -> Any:
    return msgpack.unpackb(data, object_hook=mnp.decode, raw=False)


def letterbox_nearest(image: np.ndarray, height: int, width: int, fill=0) -> np.ndarray:
    """Resize to height x width without distorting the aspect ratio.

    The image is scaled (nearest neighbour) to fit and centred on a canvas of
    ``fill``, the same resize-with-pad a LeRobot/SmolVLA preprocessor applies,
    instead of squashing a 4:3 frame into a square.
    """
    source_height, source_width = image.shape[:2]
    if (source_height, source_width) == (height, width):
        return image
    scale = min(height / source_height, width / source_width)
    scaled_height = max(1, min(height, int(round(source_height * scale))))
    scaled_width = max(1, min(width, int(round(source_width * scale))))
    rows = np.minimum(
        ((np.arange(scaled_height) + 0.5) / scale).astype(np.int64), source_height - 1
    )
    cols = np.minimum(
        ((np.arange(scaled_width) + 0.5) / scale).astype(np.int64), source_width - 1
    )
    canvas = np.full((height, width) + image.shape[2:], fill, dtype=image.dtype)
    top = (height - scaled_height) // 2
    left = (width - scaled_width) // 2
    canvas[top:top + scaled_height, left:left + scaled_width] = image[rows][:, cols]
    return canvas


class PolicyClient:
    def __init__(self, host: str, port: int, timeout_ms: int):
        print(f"Connecting to GR00T policy server at {host}:{port}", flush=True)
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self.context = zmq.Context.instance()
        self.socket = None
        self._connect()
        print("GR00T policy socket configured", flush=True)

    def _connect(self):
        if self.socket is not None:
            self.socket.close(linger=0)
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.connect(f"tcp://{self.host}:{self.port}")

    def call(self, endpoint: str, data: dict | None = None, requires_input: bool = True):
        request = {"endpoint": endpoint}
        if requires_input:
            request["data"] = data or {}
        try:
            self.socket.send(pack(request))
            response = unpack(self.socket.recv())
        except zmq.error.Again:
            self._connect()
            raise
        if isinstance(response, dict) and "error" in response:
            raise RuntimeError(response["error"])
        return response

    def get_action(self, observation: dict):
        return self.call("get_action", {"observation": observation, "options": None})

    def reset(self):
        # GR00T's PolicyServer calls policy.reset(**data); the SmolVLA server
        # ignores the payload. Both drop any queued action chunk.
        return self.call("reset", {"options": None})

    def get_modality_config(self):
        return self.call("get_modality_config", requires_input=False)


class SO101GrootBridge(Node):
    # Defaults for state that tests and older callers may not initialise.
    stats_pub = None
    send_ground_truth = False
    episode_reset_gap_s = 1.0
    latest_image_time = None
    _last_joint_time = None
    _reset_pending = True

    def __init__(self):
        super().__init__("so101_groot_bridge")
        self.declare_parameter("policy_host", "127.0.0.1")
        self.declare_parameter("policy_port", 5555)
        self.declare_parameter("policy_timeout_ms", 15000)
        # Commands are produced at the rate the policy's training data was
        # recorded at; a chunked policy (SmolVLA) replays its queued actions at
        # this rate. GR00T datasets from convert_rosbags_to_lerobot.py default
        # to --fps 5; LeRobot SO-101 recordings are usually 30 fps.
        self.declare_parameter("command_rate_hz", 5.0)
        self.declare_parameter("instruction", "move the SO-101 arm")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("task_id", "")
        self.declare_parameter("camera_topic", "/so101/camera/image_raw")
        self.declare_parameter("wrist_camera_topic", "")
        self.declare_parameter("depth_topic", "/so101/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/so101/camera/camera_info")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("command_topic", "/so101/joint_commands")
        self.declare_parameter("video_key", "front")
        self.declare_parameter("depth_key", "front_depth")
        self.declare_parameter("state_key", "joint_positions")
        self.declare_parameter("action_key", "joint_positions")
        self.declare_parameter("policy_schema", "auto")
        # 0 sends frames at native resolution so the policy's own processor
        # resizes them exactly as it did the (native 640x480) training videos.
        # Positive values letterbox (aspect-preserving resize + pad).
        self.declare_parameter("image_width", 0)
        self.declare_parameter("image_height", 0)
        self.declare_parameter("joint_history_size", 4)
        self.declare_parameter("max_joint_step", 0.08)
        # Joint feedback that resumes after this gap (e.g. a simulator restart)
        # starts a new episode: the policy server is reset before the next
        # request so no stale queued actions are executed.
        self.declare_parameter("episode_reset_gap_s", 1.0)
        # Include simulator object poses in observation metadata. Any policy
        # given them is reported as having used privileged ground truth.
        self.declare_parameter("send_ground_truth", False)
        self.declare_parameter("stats_topic", POLICY_STATS_TOPIC)

        self.video_key = self.get_parameter("video_key").value
        self.depth_key = self.get_parameter("depth_key").value
        self.state_key = self.get_parameter("state_key").value
        self.action_key = self.get_parameter("action_key").value
        self.policy_schema = self.get_parameter("policy_schema").value
        self.instruction = self.get_parameter("instruction").value
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.joint_history_size = max(1, int(self.get_parameter("joint_history_size").value))
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.episode_reset_gap_s = float(self.get_parameter("episode_reset_gap_s").value)
        self.send_ground_truth = bool(self.get_parameter("send_ground_truth").value)
        self.scenario = self._load_scenario(self.get_parameter("task_id").value)

        host = self.get_parameter("policy_host").value
        port = int(self.get_parameter("policy_port").value)
        timeout_ms = int(self.get_parameter("policy_timeout_ms").value)
        self.client = PolicyClient(host, port, timeout_ms)

        self.latest_image = None
        self.latest_image_time = None
        self.latest_wrist_image = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.latest_positions = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.joint_history = deque(maxlen=self.joint_history_size)
        self.have_joint_state = False
        self._policy_busy = False
        self._policy_lock = threading.Lock()
        self._missing_wrist_warned = False
        self._modality_configured = False
        self.video_keys = [self.video_key]
        self.state_keys = [self.state_key]
        self.action_keys = [self.action_key]
        self.language_key = "task"
        self.video_horizon = 1
        self.state_horizon = self.joint_history_size

        if self.policy_schema == "legacy":
            self._modality_configured = True
        elif self.policy_schema == "so101":
            self._set_policy_modalities(
                video_keys=["front", "wrist"],
                state_keys=["single_arm", "gripper"],
                action_keys=["single_arm", "gripper"],
                language_key="annotation.human.task_description",
                video_horizon=1,
                state_horizon=1,
            )
        elif self.policy_schema != "auto":
            raise ValueError("policy_schema must be one of: auto, legacy, so101")

        self.command_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("command_topic").value, 10
        )
        stats_topic = self.get_parameter("stats_topic").value
        if stats_topic:
            self.stats_pub = self.create_publisher(String, stats_topic, 10)
        self.create_subscription(Image, self.get_parameter("camera_topic").value, self._image_cb, 5)
        wrist_camera_topic = self.get_parameter("wrist_camera_topic").value
        if wrist_camera_topic:
            self.create_subscription(Image, wrist_camera_topic, self._wrist_image_cb, 5)
        depth_topic = self.get_parameter("depth_topic").value
        if depth_topic:
            self.create_subscription(Image, depth_topic, self._depth_cb, 5)
        camera_info_topic = self.get_parameter("camera_info_topic").value
        if camera_info_topic:
            self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 5)
        self.create_subscription(
            JointState, self.get_parameter("joint_state_topic").value, self._joint_cb, 10
        )
        command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        if not np.isfinite(command_rate_hz) or command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        self.timer = self.create_timer(1.0 / command_rate_hz, self._tick)
        self.add_on_set_parameters_callback(self._on_parameters)
        print(f"SO-101 GR00T bridge initialized at {command_rate_hz:g} Hz", flush=True)

    def _on_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult

        updates = {param.name: param.value for param in params}
        if "task_id" in updates or "instruction" in updates:
            self._set_task(updates.get("task_id"), updates.get("instruction"))
        return SetParametersResult(successful=True)

    def _set_task(self, task_id=None, instruction=None):
        """Switch task/instruction at runtime; the next request starts a new episode."""
        if task_id is not None:
            self.scenario = self._load_scenario(task_id)
        if instruction is not None:
            self.instruction = instruction
        self._request_reset("task changed")

    def _request_reset(self, reason):
        if not self._reset_pending:
            self.get_logger().info(f"Resetting the policy before the next request: {reason}")
        self._reset_pending = True

    def _load_scenario(self, task_id=""):
        scenario_file = self.get_parameter("scenario_file").value
        if not scenario_file:
            return {}
        path = Path(scenario_file)
        try:
            scenario = json.loads(path.read_text())
        except OSError as exc:
            self.get_logger().warning(f"Unable to read scenario file {path}: {exc}")
            return {}
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid scenario JSON {path}: {exc}")
            return {}

        tasks = scenario.get("tasks", [])
        selected = None
        if task_id:
            selected = next((task for task in tasks if task.get("id") == task_id), None)
            if selected is None:
                self.get_logger().warning(f"Task {task_id!r} not found in scenario {path}")
        elif tasks:
            selected = tasks[0]
        if selected:
            self.instruction = selected.get("language", self.instruction)
            scenario["selected_task"] = selected
        return scenario

    def _decode_image(self, msg: Image):
        if msg.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
            self.get_logger().warn(f"Unsupported image encoding {msg.encoding}; expected rgb8/bgr8/rgba8/bgra8")
            return None
        channels = 4 if msg.encoding in ("rgba8", "bgra8") else 3
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        arr = arr[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
        arr = arr[:, :, :3]
        if msg.encoding in ("bgr8", "bgra8"):
            arr = arr[:, :, ::-1]
        return self._resize(np.ascontiguousarray(arr))

    def _image_cb(self, msg: Image):
        image = self._decode_image(msg)
        if image is not None:
            self.latest_image_time = time.monotonic()
            self.latest_image = image

    def _wrist_image_cb(self, msg: Image):
        image = self._decode_image(msg)
        if image is not None:
            self.latest_wrist_image = image

    def _depth_cb(self, msg: Image):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        if msg.encoding == "32FC1":
            depth = np.ascontiguousarray(raw[:, :msg.width * 4]).view(np.float32)
        elif msg.encoding == "16UC1":
            depth = np.ascontiguousarray(raw[:, :msg.width * 2]).view(np.uint16).astype(np.float32)
            depth *= 0.001
        else:
            self.get_logger().warn(f"Unsupported depth encoding {msg.encoding}; expected 32FC1/16UC1")
            return
        self.latest_depth = self._resize(depth, fill=np.nan).astype(np.float32)

    def _camera_info_cb(self, msg: CameraInfo):
        self.latest_camera_info = {
            "frame_id": msg.header.frame_id,
            "width": msg.width,
            "height": msg.height,
            "distortion_model": msg.distortion_model,
            "d": list(msg.d),
            "k": list(msg.k),
            "r": list(msg.r),
            "p": list(msg.p),
        }

    def _joint_cb(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        try:
            current = np.asarray([positions[name] for name in JOINT_NAMES], dtype=np.float64)
            safe_hardware_target(current, current, self.max_joint_step)
        except (KeyError, ValueError) as exc:
            self.have_joint_state = False
            self.joint_history.clear()
            self._request_reset("joint feedback became invalid")
            self.get_logger().warning(f"Ignoring invalid SO-101 joint feedback: {exc}")
            return
        now = time.monotonic()
        if self._last_joint_time is not None and now - self._last_joint_time > self.episode_reset_gap_s:
            self.joint_history.clear()
            self._request_reset(
                f"joint feedback resumed after {now - self._last_joint_time:.1f} s (new episode)"
            )
        self._last_joint_time = now
        self.latest_positions = current
        self.joint_history.append(self.latest_positions.copy())
        self.have_joint_state = True

    def _resize(self, image: np.ndarray, fill=0) -> np.ndarray:
        if self.image_width <= 0 or self.image_height <= 0:
            return image
        return letterbox_nearest(image, self.image_height, self.image_width, fill)

    def _joint_history_array(self, horizon=None) -> np.ndarray:
        if not self.joint_history:
            self.joint_history.append(self.latest_positions.copy())
        horizon = max(1, int(horizon or self.joint_history_size))
        history = list(self.joint_history)[-horizon:]
        while len(history) < horizon:
            history.insert(0, history[0].copy())
        return np.stack(history, axis=0).reshape(1, horizon, -1)

    @staticmethod
    def _modality_payload(value):
        if isinstance(value, dict) and (
            value.get("__ModalityConfig__") or value.get("__ModalityConfig_class__")
        ):
            value = value.get("as_json", {})
        if isinstance(value, str):
            value = json.loads(value)
        return value if isinstance(value, dict) else {}

    def _set_policy_modalities(
        self,
        *,
        video_keys,
        state_keys,
        action_keys,
        language_key,
        video_horizon,
        state_horizon,
    ):
        supported_state = len(state_keys) == 1 or state_keys == ["single_arm", "gripper"]
        supported_action = len(action_keys) == 1 or action_keys == ["single_arm", "gripper"]
        if not supported_state or not supported_action:
            raise ValueError(
                "Unsupported GR00T checkpoint schema: expected one six-joint state/action "
                "key or SO-101 keys ['single_arm', 'gripper']; got "
                f"state={state_keys}, action={action_keys}"
            )
        self.video_keys = video_keys
        self.state_keys = state_keys
        self.action_keys = action_keys
        self.language_key = language_key
        self.video_horizon = max(1, int(video_horizon))
        self.state_horizon = max(1, int(state_horizon))
        self._modality_configured = True
        self.get_logger().info(
            "GR00T checkpoint modalities: "
            f"video={video_keys}, state={state_keys}, action={action_keys}, "
            f"language={language_key}"
        )

    def _configure_from_policy(self):
        config = self.client.get_modality_config()
        if not isinstance(config, dict):
            raise ValueError(f"Invalid GR00T modality config response: {type(config)}")

        video = self._modality_payload(config.get("video"))
        state = self._modality_payload(config.get("state"))
        action = self._modality_payload(config.get("action"))
        language = self._modality_payload(config.get("language"))
        video_keys = list(video.get("modality_keys", []))
        state_keys = list(state.get("modality_keys", []))
        action_keys = list(action.get("modality_keys", []))
        language_keys = list(language.get("modality_keys", []))
        if not video_keys or not state_keys or not action_keys or not language_keys:
            raise ValueError(f"Incomplete GR00T modality config: {config}")
        self._set_policy_modalities(
            video_keys=video_keys,
            state_keys=state_keys,
            action_keys=action_keys,
            language_key=language_keys[0],
            video_horizon=len(video.get("delta_indices", [0])),
            state_horizon=len(state.get("delta_indices", [0])),
        )

    def _observation(self) -> dict:
        image = self.latest_image
        if image is None:
            height = self.image_height if self.image_height > 0 else CAMERA_HEIGHT
            width = self.image_width if self.image_width > 0 else CAMERA_WIDTH
            image = np.zeros((height, width, 3), dtype=np.uint8)

        video = {}
        for key in self.video_keys:
            key_image = image
            if key == "wrist":
                if self.latest_wrist_image is not None:
                    key_image = self.latest_wrist_image
                elif not self._missing_wrist_warned:
                    self.get_logger().warning(
                        "Checkpoint expects a wrist camera but wrist_camera_topic is empty or has no frame; "
                        "using the front image for the wrist input"
                    )
                    self._missing_wrist_warned = True
            frame = key_image[np.newaxis, np.newaxis]
            video[key] = np.repeat(frame, self.video_horizon, axis=1)

        # Safety math runs in float64; GR00T's strict observation check
        # requires float32 state arrays.
        joint_history = self._joint_history_array(self.state_horizon).astype(np.float32)
        if self.state_keys == ["single_arm", "gripper"]:
            state = {
                "single_arm": joint_history[..., :5],
                "gripper": joint_history[..., 5:6],
            }
        else:
            state = {self.state_keys[0]: joint_history}
        observation = {
            "video": video,
            "state": state,
            "language": {self.language_key: [[self.instruction]]},
            "metadata": {
                "joint_names": JOINT_NAMES,
                "camera_info": self.latest_camera_info,
                "scenario": self._scenario_metadata(),
                "observation_time": time.time(),
            },
        }
        if self.latest_depth is not None and self.depth_key:
            observation["depth"] = {self.depth_key: self.latest_depth[np.newaxis, np.newaxis, ..., np.newaxis]}
        return observation

    def _scenario_metadata(self):
        if not self.scenario:
            return None
        metadata = {
            "name": self.scenario.get("name"),
            "frame_id": self.scenario.get("frame_id"),
            "selected_task": self.scenario.get("selected_task"),
        }
        if self.send_ground_truth:
            # Privileged simulator state: policies given this are reported as
            # having used ground truth (see _policy_used_ground_truth).
            metadata.update({
                "robot_spawn": self.scenario.get("robot_spawn", {}),
                "static_assets": self.scenario.get("static_assets", []),
                "objects": self.scenario.get("objects", []),
            })
        return metadata

    @staticmethod
    def _response_info(response) -> dict:
        if isinstance(response, (list, tuple)) and len(response) > 1 and isinstance(response[1], dict):
            return response[1]
        return {}

    def _policy_used_ground_truth(self, response) -> bool:
        info = self._response_info(response)
        return bool(self.send_ground_truth or info.get("policy_used_ground_truth", False))

    def _extract_action(self, response, positions) -> np.ndarray:
        return safe_hardware_target(self._requested_action(response), positions, self.max_joint_step)

    def _requested_action(self, response) -> np.ndarray:
        if isinstance(response, (list, tuple)) and response:
            action = response[0]
        else:
            action = response
        if self.action_keys == ["single_arm", "gripper"]:
            if not all(key in action for key in self.action_keys):
                raise ValueError(
                    f"GR00T response is missing SO-101 action keys {self.action_keys}: {list(action)}"
                )
            raw = np.concatenate(
                [np.asarray(action["single_arm"]), np.asarray(action["gripper"])], axis=-1
            )
        elif self.action_keys[0] in action:
            raw = action[self.action_keys[0]]
        elif self.action_key in action:
            raw = action[self.action_key]
        else:
            raw = next(iter(action.values()))
        return np.asarray(raw, dtype=np.float64).reshape(-1, len(JOINT_NAMES))[0]

    def _publish_stats(self, *, latency_s, image_time, requested, target, ground_truth):
        if self.stats_pub is None:
            return
        now = time.monotonic()
        stats = {
            "policy_latency_ms": latency_s * 1000.0,
            # Age of the camera frame (from receipt at this bridge) when the
            # command derived from it is published.
            "observation_age_ms": None if image_time is None else (now - image_time) * 1000.0,
            "command_saturated": bool(np.any(np.abs(target - requested) > 1e-9)),
            "policy_used_ground_truth": bool(ground_truth),
        }
        msg = String()
        msg.data = json.dumps(stats)
        self.stats_pub.publish(msg)

    def _tick(self):
        if not self.have_joint_state:
            return
        with self._policy_lock:
            if self._policy_busy:
                return
            self._policy_busy = True
        threading.Thread(target=self._policy_worker, daemon=True).start()

    def _policy_worker(self):
        try:
            if not self._modality_configured:
                self._configure_from_policy()
                self._request_reset("policy (re)configured")
            if self._reset_pending:
                # Clear before the call so a reset requested meanwhile is kept.
                self._reset_pending = False
                try:
                    self.client.reset()
                except Exception:
                    self._reset_pending = True
                    raise
            image_time = self.latest_image_time
            observation = self._observation()
            started = time.monotonic()
            response = self.client.get_action(observation)
            latency_s = time.monotonic() - started
            if not self.have_joint_state or self._reset_pending:
                raise ValueError("Joint feedback or episode changed while waiting for the policy")
            # Clamp against where the joints are now, not a pre-request
            # snapshot, so a slow policy round-trip cannot command a step
            # larger than max_joint_step from the arm's actual position.
            requested = self._requested_action(response)
            target = safe_hardware_target(requested, self.latest_positions.copy(), self.max_joint_step)
            msg = Float64MultiArray()
            msg.data = target.astype(float).tolist()
            self.command_pub.publish(msg)
            self._publish_stats(
                latency_s=latency_s,
                image_time=image_time,
                requested=requested,
                target=target,
                ground_truth=self._policy_used_ground_truth(response),
            )
        except Exception as exc:
            self.get_logger().warn(f"GR00T policy request failed: {exc}")
        finally:
            with self._policy_lock:
                self._policy_busy = False


def main():
    print("Starting SO-101 GR00T bridge", flush=True)
    rclpy.init()
    print("ROS initialized for SO-101 GR00T bridge", flush=True)
    node = SO101GrootBridge()
    try:
        rclpy.spin(node)
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
