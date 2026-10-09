#!/bin/bash
set -euo pipefail

RACECAR_SIM_DIR="${RACECAR_SIM_DIR:-/opt/racecarneo/RacecarNeo-Simulator/RacecarSim_Linux_v2.8.3}"
cd "${RACECAR_SIM_DIR}"

# The Unity player is named after its build directory (RacecarSim_Linux_<version>).
exec xvfb-run -a \
  -s "${RACECAR_XVFB_SERVER_ARGS:--screen 0 1280x720x24}" \
  "./$(basename "${RACECAR_SIM_DIR}").x86_64" \
  -batchmode \
  -logFile "${RACECAR_SIM_LOG_FILE:-/dev/stdout}" \
  "$@"
