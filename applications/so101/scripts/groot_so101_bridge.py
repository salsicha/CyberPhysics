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
from std_msgs.msg import Float64MultiArray
import zmq

from so101_common import JOINT_NAMES, LOWER_LIMITS, UPPER_LIMITS


def pack(data: Any) -> bytes:
    return msgpack.packb(data, default=mnp.encode, use_bin_type=True)


def unpack(data: bytes) -> Any:
    return msgpack.unpackb(data, object_hook=mnp.decode, raw=False)


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


class SO101GrootBridge(Node):
    def __init__(self):
        super().__init__("so101_groot_bridge")
        self.declare_parameter("policy_host", "127.0.0.1")
        self.declare_parameter("policy_port", 5555)
        self.declare_parameter("policy_timeout_ms", 15000)
        self.declare_parameter("command_rate_hz", 5.0)
        self.declare_parameter("instruction", "move the SO-101 arm")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("task_id", "")
        self.declare_parameter("camera_topic", "/so101/camera/image_raw")
        self.declare_parameter("depth_topic", "/so101/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/so101/camera/camera_info")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("command_topic", "/so101/joint_commands")
        self.declare_parameter("video_key", "front")
        self.declare_parameter("depth_key", "front_depth")
        self.declare_parameter("state_key", "joint_positions")
        self.declare_parameter("action_key", "joint_positions")
        self.declare_parameter("image_width", 224)
        self.declare_parameter("image_height", 224)
        self.declare_parameter("joint_history_size", 4)
        self.declare_parameter("max_joint_step", 0.08)

        self.video_key = self.get_parameter("video_key").value
        self.depth_key = self.get_parameter("depth_key").value
        self.state_key = self.get_parameter("state_key").value
        self.action_key = self.get_parameter("action_key").value
        self.instruction = self.get_parameter("instruction").value
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.joint_history_size = max(1, int(self.get_parameter("joint_history_size").value))
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.scenario = self._load_scenario()

        host = self.get_parameter("policy_host").value
        port = int(self.get_parameter("policy_port").value)
        timeout_ms = int(self.get_parameter("policy_timeout_ms").value)
        self.client = PolicyClient(host, port, timeout_ms)

        self.latest_image = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.latest_positions = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.joint_history = deque(maxlen=self.joint_history_size)
        self.have_joint_state = False
        self._policy_busy = False
        self._policy_lock = threading.Lock()

        self.command_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("command_topic").value, 10
        )
        self.create_subscription(Image, self.get_parameter("camera_topic").value, self._image_cb, 5)
        depth_topic = self.get_parameter("depth_topic").value
        if depth_topic:
            self.create_subscription(Image, depth_topic, self._depth_cb, 5)
        camera_info_topic = self.get_parameter("camera_info_topic").value
        if camera_info_topic:
            self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 5)
        self.create_subscription(
            JointState, self.get_parameter("joint_state_topic").value, self._joint_cb, 10
        )
        period = 1.0 / float(self.get_parameter("command_rate_hz").value)
        self.timer = self.create_timer(period, self._tick)
        print("SO-101 GR00T bridge initialized", flush=True)

    def _load_scenario(self):
        scenario_file = self.get_parameter("scenario_file").value
        task_id = self.get_parameter("task_id").value
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

    def _image_cb(self, msg: Image):
        if msg.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
            self.get_logger().warn(f"Unsupported image encoding {msg.encoding}; expected rgb8/bgr8/rgba8/bgra8")
            return
        channels = 4 if msg.encoding in ("rgba8", "bgra8") else 3
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        arr = arr[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
        arr = arr[:, :, :3]
        if msg.encoding in ("bgr8", "bgra8"):
            arr = arr[:, :, ::-1]
        self.latest_image = self._resize_nearest(arr, self.image_height, self.image_width)

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
        self.latest_depth = self._resize_nearest(depth, self.image_height, self.image_width).astype(np.float32)

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
        by_name = {name: i for i, name in enumerate(msg.name)}
        if not all(name in by_name for name in JOINT_NAMES):
            return
        self.latest_positions = np.asarray([msg.position[by_name[name]] for name in JOINT_NAMES], dtype=np.float32)
        self.joint_history.append(self.latest_positions.copy())
        self.have_joint_state = True

    def _resize_nearest(self, image: np.ndarray, height: int, width: int) -> np.ndarray:
        y = np.linspace(0, image.shape[0] - 1, height).astype(np.int32)
        x = np.linspace(0, image.shape[1] - 1, width).astype(np.int32)
        return image[y][:, x]

    def _joint_history_array(self) -> np.ndarray:
        if not self.joint_history:
            self.joint_history.append(self.latest_positions.copy())
        while len(self.joint_history) < self.joint_history_size:
            self.joint_history.appendleft(self.joint_history[0].copy())
        return np.stack(tuple(self.joint_history), axis=0).reshape(1, self.joint_history_size, -1)

    def _observation(self) -> dict:
        image = self.latest_image
        if image is None:
            image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        observation = {
            "video": {self.video_key: image.reshape(1, 1, self.image_height, self.image_width, 3)},
            "state": {self.state_key: self._joint_history_array()},
            "language": {"task": [[self.instruction]]},
            "metadata": {
                "joint_names": JOINT_NAMES,
                "camera_info": self.latest_camera_info,
                "scenario": self._scenario_metadata(),
                "observation_time": time.time(),
            },
        }
        if self.latest_depth is not None and self.depth_key:
            observation["depth"] = {self.depth_key: self.latest_depth.reshape(1, 1, self.image_height, self.image_width, 1)}
        return observation

    def _scenario_metadata(self):
        if not self.scenario:
            return None
        return {
            "name": self.scenario.get("name"),
            "frame_id": self.scenario.get("frame_id"),
            "selected_task": self.scenario.get("selected_task"),
            "objects": self.scenario.get("objects", []),
        }

    def _extract_action(self, response, positions) -> np.ndarray:
        if isinstance(response, (list, tuple)) and response:
            action = response[0]
        else:
            action = response
        if self.action_key in action:
            raw = action[self.action_key]
        else:
            raw = next(iter(action.values()))
        target = np.asarray(raw, dtype=np.float32).reshape(-1, len(JOINT_NAMES))[0]
        delta = np.clip(target - positions, -self.max_joint_step, self.max_joint_step)
        return np.clip(positions + delta, LOWER_LIMITS, UPPER_LIMITS)

    def _tick(self):
        if not self.have_joint_state:
            return
        with self._policy_lock:
            if self._policy_busy:
                return
            self._policy_busy = True
        observation = self._observation()
        threading.Thread(
            target=self._policy_worker, args=(observation,), daemon=True
        ).start()

    def _policy_worker(self, observation):
        try:
            response = self.client.get_action(observation)
            # Clamp against where the joints are now, not a pre-request
            # snapshot, so a slow policy round-trip cannot command a step
            # larger than max_joint_step from the arm's actual position.
            target = self._extract_action(response, self.latest_positions.copy())
            msg = Float64MultiArray()
            msg.data = target.astype(float).tolist()
            self.command_pub.publish(msg)
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
