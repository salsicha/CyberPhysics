#!/usr/bin/env bash

set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/workflow_common.sh"

REPO_ID=""
SOURCE_DATASET=""
OUTPUT_DATASET=""
INPUT_UNITS=""
GRIPPER_CLOSED=""
GRIPPER_OPEN=""

usage() {
    cat <<'EOF'
Usage:
  applications/groot/scripts/prepare_dataset.sh --repo-id USER/DATASET [options]
  applications/groot/scripts/prepare_dataset.sh --source-dataset PATH [options]

Convert LeRobot v3 Hub data when needed, normalize it into the SO-101 Isaac
radians/metres contract, install the one-camera modality config, and validate it.

Required:
  exactly one of --repo-id or --source-dataset
  --input-units degrees|isaac
  --output-dataset PATH

Degree input additionally requires:
  --gripper-closed-value VALUE
  --gripper-open-value VALUE

All paths must be below data/groot (or their /workspace/data equivalents).
EOF
}

while (($#)); do
    case "$1" in
        --repo-id) REPO_ID="$2"; shift 2 ;;
        --source-dataset) SOURCE_DATASET="$2"; shift 2 ;;
        --output-dataset) OUTPUT_DATASET="$(container_data_path "$2")"; shift 2 ;;
        --input-units) INPUT_UNITS="$2"; shift 2 ;;
        --gripper-closed-value) GRIPPER_CLOSED="$2"; shift 2 ;;
        --gripper-open-value) GRIPPER_OPEN="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

if [[ -n "${REPO_ID}" && -n "${SOURCE_DATASET}" ]] || [[ -z "${REPO_ID}" && -z "${SOURCE_DATASET}" ]]; then
    die "Specify exactly one of --repo-id or --source-dataset"
fi
[[ "${INPUT_UNITS}" == "degrees" || "${INPUT_UNITS}" == "isaac" ]] ||
    die "--input-units must be degrees or isaac"
[[ -n "${OUTPUT_DATASET}" ]] || die "--output-dataset is required"
if [[ "${INPUT_UNITS}" == "degrees" ]]; then
    [[ -n "${GRIPPER_CLOSED}" && -n "${GRIPPER_OPEN}" ]] ||
        die "Degree input requires both gripper calibration endpoint values"
fi

ensure_workflow_directories
[[ ! -e "$(host_data_path "${OUTPUT_DATASET}")" ]] ||
    die "Output dataset already exists: $(host_data_path "${OUTPUT_DATASET}")"

if [[ -n "${REPO_ID}" ]]; then
    [[ "${REPO_ID}" =~ ^[^/]+/[^/]+$ ]] || die "--repo-id must have USER/DATASET form"
    SLUG="${REPO_ID//\//__}"
    CONVERSION_ROOT="${GROOT_CONTAINER_DATA_ROOT}/datasets/.converted/${SLUG}"
    SOURCE_DATASET="${CONVERSION_ROOT}/${REPO_ID}"
    [[ ! -e "$(host_data_path "${CONVERSION_ROOT}")" ]] ||
        die "Conversion workspace already exists: $(host_data_path "${CONVERSION_ROOT}")"
    mkdir -p "$(host_data_path "${CONVERSION_ROOT}")"
    groot_run uv run --project scripts/lerobot_conversion \
        python scripts/lerobot_conversion/convert_v3_to_v2.py \
        --repo-id "${REPO_ID}" \
        --root "${CONVERSION_ROOT}"
else
    SOURCE_DATASET="$(container_data_path "${SOURCE_DATASET}")"
    require_host_directory "${SOURCE_DATASET}"
fi

ARGS=(
    python /workspace/groot_scripts/normalize_so101_dataset.py
    --source-dataset "${SOURCE_DATASET}"
    --output-dataset "${OUTPUT_DATASET}"
    --input-units "${INPUT_UNITS}"
)
if [[ "${INPUT_UNITS}" == "degrees" ]]; then
    ARGS+=(
        --gripper-closed-value "${GRIPPER_CLOSED}"
        --gripper-open-value "${GRIPPER_OPEN}"
    )
fi

groot_run "${ARGS[@]}"
groot_run python /workspace/groot_scripts/validate_so101_dataset.py \
    --dataset-path "${OUTPUT_DATASET}"
echo "Prepared dataset: $(host_data_path "${OUTPUT_DATASET}")"
