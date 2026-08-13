#!/usr/bin/env bash

set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/workflow_common.sh"

CHECKPOINT=""
HEADLESS=false
OBSERVE=false
TASK_ID="pick_red_block_left_bin"
DETACH=false

usage() {
    cat <<'EOF'
Usage: applications/groot/scripts/run_so101_isaac.sh --checkpoint PATH [options]

Run a validated NEW_EMBODIMENT checkpoint with so101_groot_isaac.yaml.

Options:
  --headless                Disable the Isaac window
  --observe                 Start the rosbridge observation profile
  --task-id ID              Default: pick_red_block_left_bin
  --detach                  Start Compose in the background
EOF
}

while (($#)); do
    case "$1" in
        --checkpoint) CHECKPOINT="$(container_data_path "$2")"; shift 2 ;;
        --headless) HEADLESS=true; shift ;;
        --observe) OBSERVE=true; shift ;;
        --task-id) TASK_ID="$2"; shift 2 ;;
        --detach) DETACH=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

[[ -n "${CHECKPOINT}" ]] || die "--checkpoint is required"
require_checkpoint_directory "${CHECKPOINT}"

COMPOSE_ARGS=(-f "${GROOT_COMPOSE_FILE}" --profile sim)
if [[ "${OBSERVE}" == true ]]; then
    COMPOSE_ARGS+=(--profile observe)
fi
COMPOSE_ARGS+=(up)
if [[ "${DETACH}" == true ]]; then
    COMPOSE_ARGS+=(-d)
fi

echo "Starting SO-101 Isaac with checkpoint $(host_data_path "${CHECKPOINT}")"
env \
    SO101_GROOT_MODE=real \
    SO101_HEADLESS="${HEADLESS}" \
    SO101_TASK_ID="${TASK_ID}" \
    GR00T_MODEL_PATH="${CHECKPOINT}" \
    GR00T_EMBODIMENT_TAG=NEW_EMBODIMENT \
    docker compose "${COMPOSE_ARGS[@]}"
