#!/usr/bin/env python3
"""Validate the GR00T LeRobot v2 and SO-101 Isaac unit contracts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import pyarrow.parquet as pq


JOINT_FEATURE_NAMES = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]
REQUIRED_META = ("info.json", "episodes.jsonl", "tasks.jsonl", "modality.json")
VECTOR_COLUMNS = ("observation.state", "action")
REQUIRED_COLUMNS = VECTOR_COLUMNS + ("task_index",)
EXPECTED_SPLITS = {
    "state": {"single_arm": {"start": 0, "end": 5}, "gripper": {"start": 5, "end": 6}},
    "action": {"single_arm": {"start": 0, "end": 5}, "gripper": {"start": 5, "end": 6}},
}
UNIT_CONTRACT = "cyberphysics_so101_isaac_v1"


def json_lines(path: Path) -> list[dict]:
    values = []
    for line_number, line in enumerate(path.read_text().splitlines(), start=1):
        if line.strip():
            try:
                values.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: {exc}") from exc
    return values


def validate_modality(meta: Path) -> dict:
    modality = json.loads((meta / "modality.json").read_text())
    for section, expected in EXPECTED_SPLITS.items():
        if modality.get(section) != expected:
            raise ValueError(
                f"modality.json {section!r} must be {expected}, got {modality.get(section)}"
            )
    front = modality.get("video", {}).get("front", {})
    if front.get("original_key") != "observation.images.front":
        raise ValueError(
            "modality.json must map video.front to observation.images.front"
        )
    annotation = modality.get("annotation", {}).get("human.task_description", {})
    if annotation.get("original_key") != "task_index":
        raise ValueError("modality.json must map human.task_description to task_index")
    return modality


def validate_marker(meta: Path) -> dict:
    path = meta / "so101_units.json"
    if not path.is_file():
        raise ValueError(
            f"missing {path}; normalize or explicitly mark the dataset before training"
        )
    marker = json.loads(path.read_text())
    if marker.get("contract") != UNIT_CONTRACT:
        raise ValueError(
            f"unsupported SO-101 unit contract: {marker.get('contract')!r}"
        )
    if marker.get("arm_units") != "radians":
        raise ValueError("SO-101 arm units must be radians")
    if marker.get("gripper_units") != "metres_opening":
        raise ValueError("SO-101 gripper units must be metres_opening")
    if marker.get("joint_order") != [
        name.removesuffix(".pos") for name in JOINT_FEATURE_NAMES
    ]:
        raise ValueError("SO-101 unit marker has the wrong joint order")
    return marker


def validate_vectors(
    path: Path, max_rows: int
) -> tuple[int, np.ndarray, np.ndarray, set[int]]:
    parquet = pq.ParquetFile(path)
    rows = parquet.metadata.num_rows
    batches = parquet.iter_batches(
        batch_size=max_rows,
        columns=list(REQUIRED_COLUMNS),
    )
    try:
        batch = next(batches)
    except StopIteration as exc:
        raise ValueError(f"empty parquet episode: {path}") from exc

    minimum = np.full(6, np.inf, dtype=np.float64)
    maximum = np.full(6, -np.inf, dtype=np.float64)
    for column in VECTOR_COLUMNS:
        values = np.asarray(
            batch.column(batch.schema.get_field_index(column)).to_pylist()
        )
        if values.ndim != 2 or values.shape[1] != 6:
            raise ValueError(
                f"{path}: {column} must contain Nx6 vectors, got {values.shape}"
            )
        if not np.isfinite(values).all():
            raise ValueError(f"{path}: {column} contains NaN or infinity")
        minimum = np.minimum(minimum, values.min(axis=0))
        maximum = np.maximum(maximum, values.max(axis=0))
    task_indices = np.asarray(
        batch.column(batch.schema.get_field_index("task_index")).to_pylist(),
        dtype=np.int64,
    )
    return rows, minimum, maximum, set(task_indices.tolist())


def validate_dataset(args: argparse.Namespace) -> dict:
    dataset = args.dataset_path.resolve()
    meta = dataset / "meta"
    if not dataset.is_dir():
        raise FileNotFoundError(f"dataset does not exist: {dataset}")
    missing = [name for name in REQUIRED_META if not (meta / name).is_file()]
    if missing:
        raise ValueError(f"dataset is missing metadata files: {missing}")

    info = json.loads((meta / "info.json").read_text())
    episodes = json_lines(meta / "episodes.jsonl")
    tasks = json_lines(meta / "tasks.jsonl")
    validate_modality(meta)
    marker = validate_marker(meta)

    if not episodes:
        raise ValueError("dataset contains no episodes")
    if not tasks:
        raise ValueError("dataset contains no task descriptions")
    if args.instruction and args.instruction not in {
        task.get("task") for task in tasks
    }:
        raise ValueError(f"instruction not found in tasks.jsonl: {args.instruction!r}")

    features = info.get("features", {})
    for feature in VECTOR_COLUMNS:
        definition = features.get(feature, {})
        if definition.get("shape") != [6]:
            raise ValueError(f"info.json {feature} shape must be [6]")
        names = definition.get("names")
        if names is not None and names != JOINT_FEATURE_NAMES:
            raise ValueError(f"info.json {feature} names have the wrong joint order")

    front = features.get("observation.images.front", {})
    if front.get("dtype") != "video":
        raise ValueError(
            "info.json is missing the observation.images.front video feature"
        )

    parquet_files = sorted(dataset.glob("data/chunk-*/*.parquet"))
    if len(parquet_files) != len(episodes):
        raise ValueError(
            f"episode metadata/parquet count mismatch: {len(episodes)} vs {len(parquet_files)}"
        )

    total_rows = 0
    minimum = np.full(6, np.inf, dtype=np.float64)
    maximum = np.full(6, -np.inf, dtype=np.float64)
    observed_task_indices: set[int] = set()
    for path in parquet_files:
        rows, file_minimum, file_maximum, file_tasks = validate_vectors(
            path, args.max_rows_per_file
        )
        total_rows += rows
        minimum = np.minimum(minimum, file_minimum)
        maximum = np.maximum(maximum, file_maximum)
        observed_task_indices.update(file_tasks)

    episode_rows = sum(int(episode["length"]) for episode in episodes)
    if total_rows != episode_rows:
        raise ValueError(
            f"parquet rows/episode lengths mismatch: {total_rows} vs {episode_rows}"
        )
    if int(info.get("total_frames", -1)) != total_rows:
        raise ValueError(
            f"info.json total_frames is {info.get('total_frames')}, expected {total_rows}"
        )
    if int(info.get("total_episodes", -1)) != len(episodes):
        raise ValueError("info.json total_episodes does not match episodes.jsonl")
    known_task_indices = {int(task["task_index"]) for task in tasks}
    unknown_task_indices = observed_task_indices - known_task_indices
    if unknown_task_indices:
        raise ValueError(
            "parquet data refers to unknown task indices: "
            f"{sorted(unknown_task_indices)}"
        )

    arm_absolute = float(np.max(np.abs(np.concatenate([minimum[:5], maximum[:5]]))))
    if arm_absolute > 3.5:
        raise ValueError(
            f"arm values exceed expected radians: max absolute {arm_absolute:.4f}"
        )
    if minimum[5] < -0.005 or maximum[5] > 0.045:
        raise ValueError(
            "gripper values exceed the expected metre-opening range: "
            f"[{minimum[5]:.5f}, {maximum[5]:.5f}]"
        )

    video_root = dataset / "videos"
    missing_videos = []
    for episode in episodes:
        index = int(episode["episode_index"])
        chunk = index // int(info.get("chunks_size", 1000))
        video = (
            video_root
            / f"chunk-{chunk:03d}"
            / "observation.images.front"
            / f"episode_{index:06d}.mp4"
        )
        if not video.is_file() or video.stat().st_size == 0:
            missing_videos.append(str(video))
    if missing_videos:
        raise ValueError(f"missing or empty front videos: {missing_videos[:3]}")

    return {
        "dataset": str(dataset),
        "contract": marker["contract"],
        "episodes": len(episodes),
        "frames": total_rows,
        "tasks": [task.get("task") for task in tasks],
        "fps": info.get("fps"),
        "sampled_minimum": minimum.tolist(),
        "sampled_maximum": maximum.tolist(),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-path", type=Path, required=True)
    parser.add_argument("--instruction")
    parser.add_argument(
        "--max-rows-per-file",
        type=int,
        default=4096,
        help="Rows sampled from each episode for numeric range checks.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    try:
        print(json.dumps(validate_dataset(parse_args()), indent=2) + "\n", end="")
    except Exception as exc:
        print(f"dataset validation failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
