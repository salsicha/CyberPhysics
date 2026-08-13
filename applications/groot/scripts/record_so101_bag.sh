#!/usr/bin/env bash

set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/workflow_common.sh"

NAME=""

usage() {
    cat <<'EOF'
Usage: applications/groot/scripts/record_so101_bag.sh --name EPISODE_NAME

Record one running Isaac demonstration. Start the sim profile first, begin this
script immediately before the demonstration, and stop it with Ctrl-C after a
successful episode. Failed episodes should be discarded rather than trained.
EOF
}

while (($#)); do
    case "$1" in
        --name) NAME="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

[[ -n "${NAME}" ]] || die "--name is required"
[[ "${NAME}" =~ ^[A-Za-z0-9._-]+$ ]] || die "Episode name may contain only letters, numbers, dot, underscore, and dash"
ensure_workflow_directories

OUTPUT="${GROOT_HOST_DATA_ROOT}/bags/${NAME}"
[[ ! -e "${OUTPUT}" ]] || die "Bag already exists: ${OUTPUT}"

echo "Recording ${OUTPUT}; press Ctrl-C after the successful episode."
docker compose \
    -f "${GROOT_COMPOSE_FILE}" \
    --profile sim \
    run --rm --no-deps so101_groot_bridge \
    bash -lc \
    'source "/opt/ros/${ROS_DISTRO}/setup.bash"; source /workspace/install/setup.bash; bash /workspace/so101_scripts/record_groot_demo_bag.sh "$1"' \
    _ \
    "/workspace/data/bags/${NAME}"
