#!/usr/bin/env bash

# Shared host-side helpers for the SO-101 GR00T workflow.

set -euo pipefail

GROOT_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GROOT_APP_DIR="$(cd -- "${GROOT_SCRIPT_DIR}/.." && pwd)"
CYBERPHYSICS_ROOT="$(cd -- "${GROOT_APP_DIR}/../.." && pwd)"
GROOT_COMPOSE_FILE="${CYBERPHYSICS_ROOT}/compositions/so101_groot_isaac.yaml"
GROOT_HOST_DATA_ROOT="${CYBERPHYSICS_ROOT}/data/groot"
GROOT_CONTAINER_DATA_ROOT="/workspace/data"

die() {
    echo "ERROR: $*" >&2
    exit 1
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

ensure_workflow_directories() {
    mkdir -p \
        "${GROOT_HOST_DATA_ROOT}/bags" \
        "${GROOT_HOST_DATA_ROOT}/datasets" \
        "${GROOT_HOST_DATA_ROOT}/models" \
        "${GROOT_HOST_DATA_ROOT}/training" \
        "${CYBERPHYSICS_ROOT}/cache/groot"
}

groot_run() {
    require_command docker
    docker compose \
        -f "${GROOT_COMPOSE_FILE}" \
        --profile sim \
        run --rm --no-deps groot_policy "$@"
}

host_data_path() {
    local value="$1"
    if [[ "${value}" == "${GROOT_CONTAINER_DATA_ROOT}"/* ]]; then
        printf '%s/%s\n' \
            "${GROOT_HOST_DATA_ROOT}" \
            "${value#${GROOT_CONTAINER_DATA_ROOT}/}"
        return
    fi
    if [[ "${value}" != /* ]]; then
        value="${CYBERPHYSICS_ROOT}/${value}"
    fi
    realpath -m -- "${value}"
}

container_data_path() {
    local host_path
    host_path="$(host_data_path "$1")"
    case "${host_path}" in
        "${GROOT_HOST_DATA_ROOT}"/*)
            printf '%s/%s\n' \
                "${GROOT_CONTAINER_DATA_ROOT}" \
                "${host_path#${GROOT_HOST_DATA_ROOT}/}"
            ;;
        *)
            die "Path must be inside ${GROOT_HOST_DATA_ROOT}: ${host_path}"
            ;;
    esac
}

require_host_directory() {
    local path
    path="$(host_data_path "$1")"
    [[ -d "${path}" ]] || die "Directory does not exist: ${path}"
}

require_checkpoint_directory() {
    local path
    path="$(host_data_path "$1")"
    [[ -d "${path}" ]] || die "Checkpoint directory does not exist: ${path}"
    [[ -f "${path}/config.json" ]] || die "Checkpoint is missing config.json: ${path}"
    [[ -f "${path}/processor_config.json" ]] || die "Checkpoint is missing processor_config.json: ${path}"
    [[ -f "${path}/statistics.json" ]] || die "Checkpoint is missing statistics.json: ${path}"
}
