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
        output[:, :5] = np.deg2rad(output[:, :5])
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
        help="Use degrees for standard LeRobot SO-101 data or isaac for radians/metres.",
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
