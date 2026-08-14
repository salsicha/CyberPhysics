#!/bin/bash
set -euo pipefail

vehicle="${ARDUPILOT_VEHICLE:?Set ARDUPILOT_VEHICLE (for example Rover or ArduSub)}"
frame="${ARDUPILOT_FRAME:?Set ARDUPILOT_FRAME (for example rover or vectored)}"
initial_lat="${INITIAL_LAT:-37.9234}"
initial_lon="${INITIAL_LON:--122.5967}"
origin_alt="${ORIGIN_ALT:-0.0}"

args=(
  -v "${vehicle}"
  -f "${frame}"
  --no-rebuild
  --no-mavproxy
  --custom-location="${initial_lat},${initial_lon},${origin_alt},0"
)
if [[ -n "${ARDUPILOT_PARAM_FILE:-}" ]]; then
  args+=(--add-param-file="${ARDUPILOT_PARAM_FILE}")
fi

cd /ardupilot
exec python3 Tools/autotest/sim_vehicle.py "${args[@]}"

