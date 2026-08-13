#!/usr/bin/env python3
"""Convert one ROS 2 bag per SO-101 demonstration into GR00T LeRobot v2."""

from __future__ import annotations

import argparse
from bisect import bisect_left
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
from typing import Any
import uuid

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from rosbags.highlevel import AnyReader


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
IMAGE_TOPIC = "/so101/camera/image_raw"
STATE_TOPIC = "/joint_states"
ACTION_TOPIC = "/so101/joint_commands"
MODALITY_TEMPLATE = Path("/workspace/groot_config/so101_isaac_modality.json")


def nearest_sample(samples: list[tuple[int, Any]], timestamp: int) -> tuple[int, Any]:
    stamps = [entry[0] for entry in samples]
    index = bisect_left(stamps, timestamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    if not candidates:
        raise ValueError("cannot align against an empty sample stream")
    return min(candidates, key=lambda entry: abs(entry[0] - timestamp))


def decode_image(message: Any) -> np.ndarray:
    encoding = message.encoding.lower()
    channels_by_encoding = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    if encoding not in channels_by_encoding:
        raise ValueError(f"unsupported ROS image encoding: {message.encoding}")
    channels = channels_by_encoding[encoding]
    raw = np.frombuffer(message.data, dtype=np.uint8)
    rows = raw.reshape(int(message.height), int(message.step))
    image = rows[:, : int(message.width) * channels].reshape(
        int(message.height), int(message.width), channels
    )
    image = image[:, :, :3]
    if encoding in {"bgr8", "bgra8"}:
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


def state_vector(message: Any) -> np.ndarray:
    indices = {name: index for index, name in enumerate(message.name)}
    missing = [name for name in JOINT_NAMES if name not in indices]
    if missing:
        raise ValueError(f"joint state is missing SO-101 joints: {missing}")
    return np.asarray(
        [message.position[indices[name]] for name in JOINT_NAMES],
        dtype=np.float32,
    )


def action_vector(message: Any) -> np.ndarray:
    values = np.asarray(message.data, dtype=np.float32)
    if values.shape != (6,):
        raise ValueError(
            f"expected a six-dimensional joint command, got {values.shape}"
        )
    return values


def load_bag(path: Path) -> dict[str, list[tuple[int, Any]]]:
    streams = {IMAGE_TOPIC: [], STATE_TOPIC: [], ACTION_TOPIC: []}
    with AnyReader([path]) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in streams
        ]
        discovered = {connection.topic for connection in connections}
        missing = set(streams) - discovered
        if missing:
            raise ValueError(f"{path} is missing required topics: {sorted(missing)}")
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            streams[connection.topic].append((timestamp, message))
    for topic, samples in streams.items():
        if not samples:
            raise ValueError(f"{path} has no messages on {topic}")
        samples.sort(key=lambda entry: entry[0])
    return streams


def sample_episode(
    streams: dict[str, list[tuple[int, Any]]],
    *,
    fps: int,
    max_skew_s: float,
) -> tuple[list[np.ndarray], np.ndarray, np.ndarray, np.ndarray]:
    images = streams[IMAGE_TOPIC]
    overlap_start = max(samples[0][0] for samples in streams.values())
    overlap_end = min(samples[-1][0] for samples in streams.values())
    if overlap_start >= overlap_end:
        raise ValueError(
            "image, state, and action streams have no overlapping interval"
        )
    minimum_interval_ns = int(1_000_000_000 / fps)
    selected = []
    last_timestamp = -minimum_interval_ns
    for entry in images:
        if not overlap_start <= entry[0] <= overlap_end:
            continue
        if entry[0] - last_timestamp >= minimum_interval_ns:
            selected.append(entry)
            last_timestamp = entry[0]
    if len(selected) < 2:
        raise ValueError("episode has fewer than two sampled image frames")

    frames = []
    states = []
    actions = []
    timestamps = []
    maximum_skew_ns = int(max_skew_s * 1_000_000_000)
    origin = selected[0][0]
    for image_timestamp, image_message in selected:
        state_timestamp, state_message = nearest_sample(
            streams[STATE_TOPIC], image_timestamp
        )
        action_timestamp, action_message = nearest_sample(
            streams[ACTION_TOPIC], image_timestamp
        )
        skew = max(
            abs(state_timestamp - image_timestamp),
            abs(action_timestamp - image_timestamp),
        )
        if skew > maximum_skew_ns:
            raise ValueError(
                f"state/action alignment skew {skew / 1e9:.4f}s exceeds "
                f"{max_skew_s:.4f}s"
            )
        frames.append(decode_image(image_message))
        states.append(state_vector(state_message))
        actions.append(action_vector(action_message))
        timestamps.append((image_timestamp - origin) / 1_000_000_000)

    state_array = np.stack(states)
    action_array = np.stack(actions)
    for name, values in (("state", state_array), ("action", action_array)):
        if not np.isfinite(values).all():
            raise ValueError(f"{name} contains NaN or infinity")
        if np.max(np.abs(values[:, :5])) > 3.5:
            raise ValueError(f"{name} arm values are not plausible radians")
        if values[:, 5].min() < -0.005 or values[:, 5].max() > 0.045:
            raise ValueError(f"{name} gripper values are not plausible metre openings")
    return frames, state_array, action_array, np.asarray(timestamps, dtype=np.float32)


def write_video(path: Path, frames: list[np.ndarray], fps: int) -> None:
    height, width, channels = frames[0].shape
    if channels != 3 or any(frame.shape != frames[0].shape for frame in frames):
        raise ValueError("all RGB frames in an episode must have the same HxWx3 shape")
    path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        str(fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        str(path),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    try:
        assert process.stdin is not None
        for frame in frames:
            process.stdin.write(frame.tobytes())
        process.stdin.close()
        return_code = process.wait()
    except Exception:
        if process.poll() is None:
            process.kill()
        process.wait()
        raise
    if return_code:
        raise RuntimeError(f"ffmpeg exited with status {return_code}")


def write_parquet(
    path: Path,
    *,
    states: np.ndarray,
    actions: np.ndarray,
    timestamps: np.ndarray,
    episode_index: int,
    global_start: int,
) -> None:
    length = len(states)
    schema = pa.schema(
        [
            pa.field("observation.state", pa.list_(pa.float32(), 6)),
            pa.field("action", pa.list_(pa.float32(), 6)),
            pa.field("timestamp", pa.float32()),
            pa.field("frame_index", pa.int64()),
            pa.field("episode_index", pa.int64()),
            pa.field("index", pa.int64()),
            pa.field("task_index", pa.int64()),
            pa.field("next.reward", pa.float32()),
            pa.field("next.done", pa.bool_()),
        ]
    )
    table = pa.Table.from_arrays(
        [
            pa.array(states.tolist(), type=pa.list_(pa.float32(), 6)),
            pa.array(actions.tolist(), type=pa.list_(pa.float32(), 6)),
            pa.array(timestamps, type=pa.float32()),
            pa.array(range(length), type=pa.int64()),
            pa.array([episode_index] * length, type=pa.int64()),
            pa.array(range(global_start, global_start + length), type=pa.int64()),
            pa.array([0] * length, type=pa.int64()),
            pa.array([0.0] * length, type=pa.float32()),
            pa.array([False] * (length - 1) + [True], type=pa.bool_()),
        ],
        schema=schema,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, path, compression="zstd")


def info_json(
    *,
    episodes: int,
    frames: int,
    fps: int,
    height: int,
    width: int,
) -> dict[str, Any]:
    vector_feature = {
        "dtype": "float32",
        "shape": [6],
        "names": [f"{name}.pos" for name in JOINT_NAMES],
    }
    return {
        "codebase_version": "v2.1",
        "robot_type": "so101_follower",
        "total_episodes": episodes,
        "total_frames": frames,
        "total_tasks": 1,
        "chunks_size": 1000,
        "fps": fps,
        "splits": {"train": f"0:{episodes}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {
            "action": vector_feature,
            "observation.state": vector_feature,
            "observation.images.front": {
                "dtype": "video",
                "shape": [height, width, 3],
                "names": ["height", "width", "channels"],
                "info": {
                    "video.height": height,
                    "video.width": width,
                    "video.codec": "h264",
                    "video.pix_fmt": "yuv420p",
                    "video.is_depth_map": False,
                    "video.fps": fps,
                    "video.channels": 3,
                    "has_audio": False,
                },
            },
            "timestamp": {"dtype": "float32", "shape": [1], "names": None},
            "frame_index": {"dtype": "int64", "shape": [1], "names": None},
            "episode_index": {"dtype": "int64", "shape": [1], "names": None},
            "index": {"dtype": "int64", "shape": [1], "names": None},
            "task_index": {"dtype": "int64", "shape": [1], "names": None},
        },
        "total_chunks": (episodes + 999) // 1000,
        "total_videos": episodes,
    }


def convert(args: argparse.Namespace) -> None:
    output = args.output_dataset.resolve()
    if output.exists():
        raise FileExistsError(f"output dataset already exists: {output}")
    bag_paths = sorted(path.resolve() for path in args.bag)
    if not bag_paths:
        raise ValueError("at least one --bag is required")
    for path in bag_paths:
        if not path.is_dir():
            raise FileNotFoundError(f"ROS bag directory does not exist: {path}")

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.parent / f".{output.name}.convert-{uuid.uuid4().hex}"
    temporary.mkdir()
    episodes = []
    total_frames = 0
    image_shape = None
    try:
        for episode_index, bag in enumerate(bag_paths):
            streams = load_bag(bag)
            frames, states, actions, timestamps = sample_episode(
                streams,
                fps=args.fps,
                max_skew_s=args.max_skew_s,
            )
            if image_shape is None:
                image_shape = frames[0].shape
            if frames[0].shape != image_shape:
                raise ValueError("camera resolution differs between ROS bags")
            chunk = episode_index // 1000
            write_parquet(
                temporary
                / "data"
                / f"chunk-{chunk:03d}"
                / f"episode_{episode_index:06d}.parquet",
                states=states,
                actions=actions,
                timestamps=timestamps,
                episode_index=episode_index,
                global_start=total_frames,
            )
            write_video(
                temporary
                / "videos"
                / f"chunk-{chunk:03d}"
                / "observation.images.front"
                / f"episode_{episode_index:06d}.mp4",
                frames,
                args.fps,
            )
            episodes.append(
                {
                    "episode_index": episode_index,
                    "tasks": [args.instruction],
                    "length": len(frames),
                }
            )
            total_frames += len(frames)
            print(f"Converted episode {episode_index}: {bag} ({len(frames)} frames)")

        assert image_shape is not None
        meta = temporary / "meta"
        meta.mkdir()
        (meta / "info.json").write_text(
            json.dumps(
                info_json(
                    episodes=len(episodes),
                    frames=total_frames,
                    fps=args.fps,
                    height=image_shape[0],
                    width=image_shape[1],
                ),
                indent=2,
            )
            + "\n"
        )
        (meta / "episodes.jsonl").write_text(
            "".join(json.dumps(episode) + "\n" for episode in episodes)
        )
        (meta / "tasks.jsonl").write_text(
            json.dumps({"task_index": 0, "task": args.instruction}) + "\n"
        )
        shutil.copy2(args.modality_template, meta / "modality.json")
        (meta / "so101_units.json").write_text(
            json.dumps(
                {
                    "contract": "cyberphysics_so101_isaac_v1",
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                    "source_bags": [str(path) for path in bag_paths],
                    "joint_order": JOINT_NAMES,
                    "arm_units": "radians",
                    "gripper_units": "metres_opening",
                    "gripper_range_m": [0.0, 0.04],
                    "action_semantics": "absolute_joint_position_targets",
                    "video_keys": ["front"],
                },
                indent=2,
            )
            + "\n"
        )
        os.replace(temporary, output)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    print(f"Created dataset: {output}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, action="append", required=True)
    parser.add_argument("--output-dataset", type=Path, required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--max-skew-s", type=float, default=0.1)
    parser.add_argument("--modality-template", type=Path, default=MODALITY_TEMPLATE)
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be positive")
    return args


if __name__ == "__main__":
    convert(parse_args())
