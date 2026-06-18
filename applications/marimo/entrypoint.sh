#!/usr/bin/env bash
set -euo pipefail

mkdir -p "${MARIMO_WORKDIR:-/notebooks}"
cd "${MARIMO_WORKDIR:-/notebooks}"

exec "$@"
