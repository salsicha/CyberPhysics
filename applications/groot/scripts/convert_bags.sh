#!/usr/bin/env bash

set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/workflow_common.sh"

OUTPUT_DATASET=""
INSTRUCTION=""
FPS=5
MAX_SKEW=0.1
BAGS=()

usage() {
    cat <<'EOF'
Usage: applications/groot/scripts/convert_bags.sh [options] --bag PATH [--bag PATH ...]

Convert one successful Isaac ROS 2 bag per episode into a validated GR00T
LeRobot v2 dataset. Bag and output paths must be below data/groot.

Required:
  --bag PATH               Repeat for each episode
  --output-dataset PATH
  --instruction TEXT

Options:
  --fps N                  Dataset sampling rate (default: 5)
  --max-skew-s SECONDS     Maximum image/state/action skew (default: 0.1)
EOF
}

while (($#)); do
    case "$1" in
        --bag) BAGS+=("$(container_data_path "$2")"); shift 2 ;;
        --output-dataset) OUTPUT_DATASET="$(container_data_path "$2")"; shift 2 ;;
        --instruction) INSTRUCTION="$2"; shift 2 ;;
        --fps) FPS="$2"; shift 2 ;;
        --max-skew-s) MAX_SKEW="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

((${#BAGS[@]} > 0)) || die "At least one --bag is required"
[[ -n "${OUTPUT_DATASET}" ]] || die "--output-dataset is required"
[[ -n "${INSTRUCTION}" ]] || die "--instruction is required"
[[ ! -e "$(host_data_path "${OUTPUT_DATASET}")" ]] ||
    die "Output dataset already exists: $(host_data_path "${OUTPUT_DATASET}")"
for bag in "${BAGS[@]}"; do
    require_host_directory "${bag}"
done

ARGS=(
    python /workspace/groot_scripts/convert_rosbags_to_lerobot.py
    --output-dataset "${OUTPUT_DATASET}"
    --instruction "${INSTRUCTION}"
    --fps "${FPS}"
    --max-skew-s "${MAX_SKEW}"
)
for bag in "${BAGS[@]}"; do
    ARGS+=(--bag "${bag}")
done

groot_run "${ARGS[@]}"
groot_run python /workspace/groot_scripts/validate_so101_dataset.py \
    --dataset-path "${OUTPUT_DATASET}" \
    --instruction "${INSTRUCTION}"
