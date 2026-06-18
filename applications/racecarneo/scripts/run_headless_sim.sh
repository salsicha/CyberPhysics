#!/bin/bash
set -euo pipefail

cd "${RACECAR_SIM_DIR:-/opt/racecarneo/RacecarNeo-Simulator/RacecarSim_Linux_v2.8.3}"

exec xvfb-run -a \
  -s "${RACECAR_XVFB_SERVER_ARGS:--screen 0 1280x720x24}" \
  ./RacecarSim_Linux_v2.8.3.x86_64 \
  -batchmode \
  -logFile "${RACECAR_SIM_LOG_FILE:-/dev/stdout}" \
  "$@"
