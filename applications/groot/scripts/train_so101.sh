#!/usr/bin/env bash

set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/workflow_common.sh"

DATASET=""
OUTPUT=""
BASE_MODEL="${GROOT_CONTAINER_DATA_ROOT}/models/GR00T-N1.7-3B"
NUM_GPUS=1
MAX_STEPS=10000
SAVE_STEPS=1000
GLOBAL_BATCH_SIZE=32
DATALOADER_WORKERS=4
STATE_DROPOUT=0.2
USE_WANDB=0
RESUME=false
EXTRA_ARGS=()

usage() {
    cat <<'EOF'
Usage: applications/groot/scripts/train_so101.sh [options] [-- extra args]

Required:
  --dataset PATH              Validated dataset below data/groot
  --output-dir PATH           New training output below data/groot

Options:
  --base-model PATH_OR_ID     Default: data/groot/models/GR00T-N1.7-3B
  --num-gpus N                Default: 1
  --max-steps N               Default: 10000
  --save-steps N              Default: 1000
  --global-batch-size N       Default: 32
  --dataloader-workers N      Default: 4
  --state-dropout-prob P      Default: 0.2
  --use-wandb                 Enable Weights & Biases
  --resume                    Resume the latest checkpoint in output-dir
EOF
}

while (($#)); do
    case "$1" in
        --dataset) DATASET="$(container_data_path "$2")"; shift 2 ;;
        --output-dir) OUTPUT="$(container_data_path "$2")"; shift 2 ;;
        --base-model)
            CANDIDATE_HOST_PATH="$(host_data_path "$2")"
            if [[ "$2" == /workspace/data/* || "$2" == data/groot/* || -d "${CANDIDATE_HOST_PATH}" ]]; then
                BASE_MODEL="$(container_data_path "$2")"
            elif [[ "$2" =~ ^[^/]+/[^/]+$ ]]; then
                BASE_MODEL="$2"
            else
                die "--base-model must be a path below data/groot or a Hugging Face USER/MODEL ID"
            fi
            shift 2
            ;;
        --num-gpus) NUM_GPUS="$2"; shift 2 ;;
        --max-steps) MAX_STEPS="$2"; shift 2 ;;
        --save-steps) SAVE_STEPS="$2"; shift 2 ;;
        --global-batch-size) GLOBAL_BATCH_SIZE="$2"; shift 2 ;;
        --dataloader-workers) DATALOADER_WORKERS="$2"; shift 2 ;;
        --state-dropout-prob) STATE_DROPOUT="$2"; shift 2 ;;
        --use-wandb) USE_WANDB=1; shift ;;
        --resume) RESUME=true; shift ;;
        --) shift; EXTRA_ARGS=("$@"); break ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

[[ -n "${DATASET}" ]] || die "--dataset is required"
[[ -n "${OUTPUT}" ]] || die "--output-dir is required"
require_host_directory "${DATASET}"
if [[ "${BASE_MODEL}" == /workspace/data/* ]]; then
    require_host_directory "${BASE_MODEL}"
fi
if [[ "${RESUME}" == false && -e "$(host_data_path "${OUTPUT}")" ]]; then
    die "Output already exists; choose a new path or use --resume"
fi
if [[ "${RESUME}" == true && ! -d "$(host_data_path "${OUTPUT}")" ]]; then
    die "Cannot resume because the output directory does not exist"
fi
mkdir -p "$(host_data_path "${OUTPUT}")"

groot_run python /workspace/groot_scripts/validate_so101_dataset.py \
    --dataset-path "${DATASET}"

TRAIN_ARGS=(
    bash examples/finetune.sh
    --base-model-path "${BASE_MODEL}"
    --dataset-path "${DATASET}"
    --modality-config-path /workspace/groot_config/so101_isaac_config.py
    --embodiment-tag NEW_EMBODIMENT
    --output-dir "${OUTPUT}"
    --state-dropout-prob "${STATE_DROPOUT}"
)
if [[ "${RESUME}" == true ]]; then
    TRAIN_ARGS+=(--resume-from-checkpoint)
fi
if ((${#EXTRA_ARGS[@]} > 0)); then
    TRAIN_ARGS+=(-- "${EXTRA_ARGS[@]}")
fi

groot_run env \
    "NUM_GPUS=${NUM_GPUS}" \
    "MAX_STEPS=${MAX_STEPS}" \
    "SAVE_STEPS=${SAVE_STEPS}" \
    "GLOBAL_BATCH_SIZE=${GLOBAL_BATCH_SIZE}" \
    "DATALOADER_NUM_WORKERS=${DATALOADER_WORKERS}" \
    "USE_WANDB=${USE_WANDB}" \
    "${TRAIN_ARGS[@]}"
