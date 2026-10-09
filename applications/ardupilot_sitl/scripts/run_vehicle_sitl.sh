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
# Hardware baseline first, then SITL-only relaxations.
for params in "${ARDUPILOT_PARAM_FILE:-}" "${ARDUPILOT_SITL_PARAM_FILE:-}"; do
  if [[ -n "$params" ]]; then
    args+=(--add-param-file="${params}")
  fi
done

cd /ardupilot
exec python3 Tools/autotest/sim_vehicle.py "${args[@]}"
