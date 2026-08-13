#!/usr/bin/env bash

set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/workflow_common.sh"

MIN_VRAM_GIB=40
SKIP_IMAGE=false

usage() {
    cat <<'EOF'
Usage: applications/groot/scripts/check_prerequisites.sh [options]

Check host GPU, Docker/Compose, disk, repository mounts, and the GR00T image.

Options:
  --min-vram-gib N   Required GPU memory for this check (default: 40)
  --skip-image       Do not require or smoke-test cyberphysics/groot:latest
EOF
}

while (($#)); do
    case "$1" in
        --min-vram-gib) MIN_VRAM_GIB="$2"; shift 2 ;;
        --skip-image) SKIP_IMAGE=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

[[ "${MIN_VRAM_GIB}" =~ ^[0-9]+$ ]] || die "--min-vram-gib must be a non-negative integer"

require_command docker
require_command nvidia-smi
ensure_workflow_directories

docker compose version
docker compose -f "${GROOT_COMPOSE_FILE}" --profile sim config --quiet

GPU_LINE="$(nvidia-smi --query-gpu=index,name,memory.total,driver_version --format=csv,noheader,nounits | head -n 1)"
[[ -n "${GPU_LINE}" ]] || die "nvidia-smi did not report a GPU"
IFS=',' read -r GPU_INDEX GPU_NAME GPU_MEMORY_MIB GPU_DRIVER <<<"${GPU_LINE}"
GPU_MEMORY_MIB="${GPU_MEMORY_MIB//[[:space:]]/}"
MIN_MEMORY_MIB=$((MIN_VRAM_GIB * 1024))
((GPU_MEMORY_MIB >= MIN_MEMORY_MIB)) ||
    die "GPU${GPU_NAME} has ${GPU_MEMORY_MIB} MiB; ${MIN_MEMORY_MIB} MiB is required"
echo "GPU:${GPU_NAME}; ${GPU_MEMORY_MIB} MiB; driver${GPU_DRIVER}"

docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi >/dev/null
df -h "${CYBERPHYSICS_ROOT}"

if [[ "${SKIP_IMAGE}" == false ]]; then
    docker image inspect cyberphysics/groot:latest >/dev/null ||
        die "cyberphysics/groot:latest is not built"
    groot_run python -c \
        'import sys, torch, gr00t; print(f"python={sys.version.split()[0]} cuda={torch.cuda.is_available()}")'
fi

echo "GR00T prerequisites passed."
