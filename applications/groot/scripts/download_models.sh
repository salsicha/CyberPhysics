#!/usr/bin/env bash

set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/workflow_common.sh"

BASE_MODEL="nvidia/GR00T-N1.7-3B"
BACKBONE_MODEL="nvidia/Cosmos-Reason2-2B"
LOCAL_DIR="${GROOT_CONTAINER_DATA_ROOT}/models/GR00T-N1.7-3B"
SKIP_BACKBONE=false

usage() {
    cat <<'EOF'
Usage: applications/groot/scripts/download_models.sh [options]

Download GR00T N1.7 into the persistent data/groot and cache/groot mounts.

Options:
  --base-model ID       Hugging Face base model ID
  --backbone-model ID   Hugging Face backbone model ID
  --local-dir PATH      Destination below data/groot or /workspace/data
  --skip-backbone       Do not pre-cache the gated Cosmos backbone
  -h, --help            Show this help
EOF
}

while (($#)); do
    case "$1" in
        --base-model) BASE_MODEL="$2"; shift 2 ;;
        --backbone-model) BACKBONE_MODEL="$2"; shift 2 ;;
        --local-dir) LOCAL_DIR="$(container_data_path "$2")"; shift 2 ;;
        --skip-backbone) SKIP_BACKBONE=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

ensure_workflow_directories
mkdir -p "$(host_data_path "${LOCAL_DIR}")"

if ! groot_run hf auth whoami >/dev/null 2>&1; then
    die "Hugging Face authentication failed. Authenticate through the mounted cache before downloading. See applications/groot/README.md."
fi

echo "Downloading ${BASE_MODEL} to ${LOCAL_DIR}"
groot_run hf download "${BASE_MODEL}" --local-dir "${LOCAL_DIR}"

if [[ "${SKIP_BACKBONE}" == false ]]; then
    echo "Caching gated backbone ${BACKBONE_MODEL}"
    groot_run hf download "${BACKBONE_MODEL}"
fi

echo "Model download complete: $(host_data_path "${LOCAL_DIR}")"
