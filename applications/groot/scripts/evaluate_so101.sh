#!/usr/bin/env bash

set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/workflow_common.sh"

DATASET=""
CHECKPOINT=""
TRAJ_IDS=(0)
EXECUTION_HORIZON=16
STEPS=400
OUTPUT_DIR=""

usage() {
    cat <<'EOF'
Usage: applications/groot/scripts/evaluate_so101.sh [options]

Required:
  --dataset PATH
  --checkpoint PATH

Options:
  --traj-ids ID [ID ...]      Default: 0; terminate the list with another option
  --execution-horizon N       Default: 16
  --steps N                   Default: 400
  --output-dir PATH           Default: CHECKPOINT/open_loop
EOF
}

while (($#)); do
    case "$1" in
        --dataset) DATASET="$(container_data_path "$2")"; shift 2 ;;
        --checkpoint) CHECKPOINT="$(container_data_path "$2")"; shift 2 ;;
        --traj-ids)
            TRAJ_IDS=()
            shift
            while (($#)) && [[ "$1" != --* ]]; do
                TRAJ_IDS+=("$1")
                shift
            done
            ;;
        --execution-horizon) EXECUTION_HORIZON="$2"; shift 2 ;;
        --steps) STEPS="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$(container_data_path "$2")"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

[[ -n "${DATASET}" ]] || die "--dataset is required"
[[ -n "${CHECKPOINT}" ]] || die "--checkpoint is required"
((${#TRAJ_IDS[@]} > 0)) || die "--traj-ids requires at least one ID"
require_host_directory "${DATASET}"
require_checkpoint_directory "${CHECKPOINT}"
if [[ -z "${OUTPUT_DIR}" ]]; then
    OUTPUT_DIR="${CHECKPOINT}/open_loop"
fi
mkdir -p "$(host_data_path "${OUTPUT_DIR}")"

groot_run python /workspace/groot_scripts/validate_so101_dataset.py \
    --dataset-path "${DATASET}"
groot_run python /workspace/groot_scripts/open_loop_eval_so101.py \
    --dataset-path "${DATASET}" \
    --model-path "${CHECKPOINT}" \
    --output-dir "${OUTPUT_DIR}" \
    --traj-ids "${TRAJ_IDS[@]}" \
    --execution-horizon "${EXECUTION_HORIZON}" \
    --steps "${STEPS}"
