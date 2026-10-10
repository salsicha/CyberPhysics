#!/usr/bin/env python3
"""Transactionally normalize an SO-101 LeRobot v2 dataset for the Isaac bridge."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import uuid

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from so101_joint_limits import JOINT_LIMITS


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
VECTOR_COLUMNS = ("observation.state", "action")
UNIT_MARKER = "so101_units.json"
# URDF arm joint [lower, upper] limits (radians) from their single source of
# truth, systems/so101/config/joint_limits.json. They are the physical range of
# a LeRobot-calibrated arm mapped with the nominal lerobot_joint_mapping.json,
# so they are asymmetric (e.g. elbow_flex -22.9..170.7 deg).
ARM_JOINT_LIMITS = JOINT_LIMITS[:5]
ARM_LIMIT_TOLERANCE_RAD = 0.1
# LeRobot use_degrees=True: deg = (raw - mid) * 360 / 4095 with an integer or
# half-integer mid, so every degree value is a multiple of 180/4095.
LEROBOT_DEGREE_STEP = 180.0 / 4095.0
LEROBOT_RANGE_LIMIT = 100.0
IDENTITY_JOINT_MAPPING = {
    name: {"sign": 1.0, "offset_deg": 0.0} for name in JOINT_NAMES[:5]
}


def load_joint_mapping(path: Path | None) -> dict[str, dict[str, float]]:
    """Per-joint ``urdf_deg = sign * lerobot_deg + offset_deg`` mapping.

    LeRobot's 0 deg is each joint's calibration mid-range pose, while this
    URDF's zero is the arm pointing straight up; see
    systems/so101/config/lerobot_joint_mapping.json.
    """
    if path is None:
        return IDENTITY_JOINT_MAPPING
    joints = json.loads(path.read_text()).get("joints", {})
    mapping = {}
    for name in JOINT_NAMES[:5]:
        entry = joints.get(name, {})
        sign = float(entry.get("sign", 1.0))
        offset = float(entry.get("offset_deg", 0.0))
        if sign not in (-1.0, 1.0) or not math.isfinite(offset):
            raise ValueError(f"{path}: {name} needs sign +1/-1 and a finite offset_deg")
        mapping[name] = {"sign": sign, "offset_deg": offset}
    return mapping


def is_identity_mapping(mapping: dict[str, dict[str, float]]) -> bool:
    return all(entry["sign"] == 1.0 and entry["offset_deg"] == 0.0 for entry in mapping.values())


def classify_lerobot_arm_units(arm: np.ndarray) -> str:
    """Classify LeRobot SO-101 arm values: "degrees", "range_m100_100" or "unknown".

    so101_follower defaults to use_degrees=False, which stores each arm joint
    as -100..100 of its calibrated travel (clipped exactly at +/-100). Those
    values look like plausible degrees, so they must be detected, not assumed.
    """
    arm = np.asarray(arm, dtype=np.float64)
    if arm.size == 0:
        return "unknown"
    if np.any(np.abs(arm) > LEROBOT_RANGE_LIMIT + 1e-3):
        return "degrees"
    steps = arm / LEROBOT_DEGREE_STEP
    if np.all(np.abs(steps - np.round(steps)) <= 0.02):
        return "degrees"
    if np.any(np.abs(np.abs(arm) - LEROBOT_RANGE_LIMIT) <= 1e-3):
        return "range_m100_100"
    return "unknown"


def read_arm_values(parquet_files: list[Path]) -> np.ndarray:
    """All arm-joint state/action values, for dataset-wide unit detection."""
    values = [np.empty((0, 5))]
    for parquet in parquet_files:
        table = pq.read_table(parquet)
        for column in VECTOR_COLUMNS:
            if column in table.column_names:
                matrix = np.asarray(table.column(column).to_pylist(), dtype=np.float64)
                if matrix.ndim == 2 and matrix.shape[1] == 6:
                    values.append(matrix[:, :5])
    return np.concatenate(values)


def atomic_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, delete=False
    ) as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)


def link_or_copy(source: str, destination: str) -> str:
    try:
        os.link(source, destination)
        return destination
    except OSError:
        return shutil.copy2(source, destination)


def transform_matrix(
    matrix: np.ndarray,
    *,
    input_units: str,
    gripper_closed_value: float | None,
    gripper_open_value: float | None,
    joint_mapping: dict[str, dict[str, float]] = IDENTITY_JOINT_MAPPING,
) -> np.ndarray:
    if matrix.ndim != 2 or matrix.shape[1] != 6:
        raise ValueError(f"expected an Nx6 SO-101 vector, got {matrix.shape}")
    if not np.isfinite(matrix).all():
        raise ValueError("state/action contains NaN or infinity")

    output = matrix.astype(np.float32, copy=True)
    if input_units == "degrees":
        if gripper_closed_value is None or gripper_open_value is None:
            raise ValueError(
                "degree input requires --gripper-closed-value and --gripper-open-value"
            )
        span = gripper_open_value - gripper_closed_value
        if math.isclose(span, 0.0):
            raise ValueError("gripper open and closed calibration values must differ")
        endpoint_tolerance = max(abs(span) * 0.05, 1e-6)
        low = min(gripper_closed_value, gripper_open_value) - endpoint_tolerance
        high = max(gripper_closed_value, gripper_open_value) + endpoint_tolerance
        observed_low = float(output[:, 5].min())
        observed_high = float(output[:, 5].max())
        if observed_low < low or observed_high > high:
            raise ValueError(
                "gripper values fall outside the calibrated range: "
                f"observed=[{observed_low:.3f}, {observed_high:.3f}], "
                f"calibrated=[{low:.3f}, {high:.3f}]"
            )
        signs = np.asarray([joint_mapping[name]["sign"] for name in JOINT_NAMES[:5]])
        offsets = np.asarray([joint_mapping[name]["offset_deg"] for name in JOINT_NAMES[:5]])
        arm = np.deg2rad(signs * output[:, :5].astype(np.float64) + offsets)
        beyond = (arm < ARM_JOINT_LIMITS[:, 0] - ARM_LIMIT_TOLERANCE_RAD) | (
            arm > ARM_JOINT_LIMITS[:, 1] + ARM_LIMIT_TOLERANCE_RAD
        )
        if np.any(beyond):
            joints = [JOINT_NAMES[index] for index in np.flatnonzero(beyond.any(axis=0))]
            raise ValueError(
                f"converted {joints} exceed the URDF joint limits; LeRobot's zero is the "
                "calibration mid-range pose, not the URDF zero, so set per-joint "
                "offset_deg/sign with --joint-mapping"
            )
        output[:, :5] = arm
        output[:, 5] = (output[:, 5] - gripper_closed_value) * (0.04 / span)
        output[:, 5] = np.clip(output[:, 5], 0.0, 0.04)
    else:
        if np.max(np.abs(output[:, :5])) > 3.5:
            raise ValueError("Isaac-unit arm values exceed the expected radian range")
        if output[:, 5].min() < -0.005 or output[:, 5].max() > 0.045:
            raise ValueError(
                "Isaac-unit gripper values must be approximately 0.0 to 0.04 m"
            )
    return output


def transform_parquet(
    path: Path,
    *,
    input_units: str,
    gripper_closed_value: float | None,
    gripper_open_value: float | None,
    joint_mapping: dict[str, dict[str, float]] = IDENTITY_JOINT_MAPPING,
) -> None:
    table = pq.read_table(path)
    for column in VECTOR_COLUMNS:
        if column not in table.column_names:
            raise ValueError(f"{path} is missing required column {column!r}")
        index = table.schema.get_field_index(column)
        field = table.schema.field(index)
        values = table.column(index).combine_chunks().to_pylist()
        matrix = np.asarray(values, dtype=np.float32)
        transformed = transform_matrix(
            matrix,
            input_units=input_units,
            gripper_closed_value=gripper_closed_value,
            gripper_open_value=gripper_open_value,
            joint_mapping=joint_mapping,
        )
        array = pa.array(transformed.tolist(), type=field.type)
        table = table.set_column(index, field, array)

    if "task_index" not in table.column_names:
        raise ValueError(f"{path} is missing required column 'task_index'")

    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
    try:
        pq.write_table(table, temporary, compression="zstd")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def prepare_dataset(args: argparse.Namespace) -> None:
    source = args.source_dataset.resolve()
    output = args.output_dataset.resolve()
    modality_template = args.modality_template.resolve()

    if not source.is_dir():
        raise FileNotFoundError(f"source dataset does not exist: {source}")
    if output.exists():
        raise FileExistsError(f"output dataset already exists: {output}")
    if source == output or source in output.parents:
        raise ValueError(
            "output dataset must not be the source or a child of the source"
        )
    if not modality_template.is_file():
        raise FileNotFoundError(
            f"modality template does not exist: {modality_template}"
        )

    existing_marker = source / "meta" / UNIT_MARKER
    if args.input_units == "degrees" and existing_marker.exists():
        marker = json.loads(existing_marker.read_text())
        if marker.get("contract") == "cyberphysics_so101_isaac_v1":
            raise ValueError(
                "source already carries the Isaac unit marker; refusing a second conversion"
            )

    parquet_files = sorted(source.glob("data/chunk-*/*.parquet"))
    if not parquet_files:
        raise FileNotFoundError(f"no LeRobot v2 parquet episodes found below {source}")

    joint_mapping = load_joint_mapping(args.joint_mapping)
    if args.input_units == "degrees":
        units = classify_lerobot_arm_units(read_arm_values(parquet_files))
        if units == "range_m100_100" and not args.skip_unit_detection:
            raise ValueError(
                "arm joints never exceed +/-100 and saturate at exactly +/-100: this looks "
                "like LeRobot so101_follower use_degrees=False (-100..100) data, not degrees. "
                "Converting it would need the arm's calibration; re-record with "
                "use_degrees=True (or pass --skip-unit-detection if it really is degrees)."
            )
        if units == "unknown":
            print(
                "WARNING: could not confirm LeRobot use_degrees=True arm values; check "
                "that the source dataset really stores degrees.",
                flush=True,
            )
        if is_identity_mapping(joint_mapping):
            print(
                "WARNING: identity joint mapping: assuming LeRobot's calibration zero "
                "equals the URDF zero for every arm joint (see --joint-mapping).",
                flush=True,
            )

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.parent / f".{output.name}.prepare-{uuid.uuid4().hex}"
    try:
        shutil.copytree(
            source,
            temporary,
            copy_function=link_or_copy,
            symlinks=True,
        )
        modality_destination = temporary / "meta" / "modality.json"
        modality_destination.unlink(missing_ok=True)
        shutil.copy2(modality_template, modality_destination)

        for source_parquet in parquet_files:
            relative = source_parquet.relative_to(source)
            transform_parquet(
                temporary / relative,
                input_units=args.input_units,
                gripper_closed_value=args.gripper_closed_value,
                gripper_open_value=args.gripper_open_value,
                joint_mapping=joint_mapping,
            )

        for derived_stats in ("stats.json", "relative_stats.json"):
            (temporary / "meta" / derived_stats).unlink(missing_ok=True)

        marker = {
            "contract": "cyberphysics_so101_isaac_v1",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "source_dataset": str(source),
            "input_units": args.input_units,
            "joint_order": JOINT_NAMES,
            "arm_units": "radians",
            "gripper_units": "metres_opening",
            "gripper_range_m": [0.0, 0.04],
            "action_semantics": "absolute_joint_position_targets",
            "video_keys": ["front"],
        }
        if args.input_units == "degrees":
            marker["gripper_input_closed"] = args.gripper_closed_value
            marker["gripper_input_open"] = args.gripper_open_value
            marker["lerobot_arm_units"] = "degrees (so101_follower use_degrees=True)"
            marker["joint_mapping"] = joint_mapping
        atomic_json(temporary / "meta" / UNIT_MARKER, marker)
        os.replace(temporary, output)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    print(f"Prepared Isaac-compatible dataset: {output}")
    print(f"Episodes: {len(parquet_files)}")
    print(f"Unit marker: {output / 'meta' / UNIT_MARKER}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dataset", type=Path, required=True)
    parser.add_argument("--output-dataset", type=Path, required=True)
    parser.add_argument(
        "--input-units",
        choices=("degrees", "isaac"),
        required=True,
        help=(
            "degrees: LeRobot so101_follower data recorded with use_degrees=True "
            "(LeRobot's default use_degrees=False -100..100 data is detected and rejected); "
            "isaac: radians/metres."
        ),
    )
    parser.add_argument(
        "--joint-mapping",
        type=Path,
        help=(
            "JSON per-joint sign/offset_deg mapping from LeRobot degrees to the URDF "
            "(e.g. /workspace/systems/so101/config/lerobot_joint_mapping.json); "
            "default: identity"
        ),
    )
    parser.add_argument(
        "--skip-unit-detection",
        action="store_true",
        help="Convert degree input even if it looks like LeRobot -100..100 data.",
    )
    parser.add_argument(
        "--gripper-closed-value",
        type=float,
        help="Raw LeRobot gripper value at fully closed; required for degree input.",
    )
    parser.add_argument(
        "--gripper-open-value",
        type=float,
        help="Raw LeRobot gripper value at 0.04 m opening; required for degree input.",
    )
    parser.add_argument(
        "--modality-template",
        type=Path,
        default=Path("/workspace/groot_config/so101_isaac_modality.json"),
    )
    args = parser.parse_args()
    if args.input_units == "degrees" and (
        args.gripper_closed_value is None or args.gripper_open_value is None
    ):
        parser.error(
            "--input-units degrees requires --gripper-closed-value and --gripper-open-value"
        )
    return args


if __name__ == "__main__":
    prepare_dataset(parse_args())
