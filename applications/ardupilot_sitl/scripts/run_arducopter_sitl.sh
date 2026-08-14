#!/bin/bash
set -euo pipefail

sim_host="${SIM_JSON_HOST:-127.0.0.1}"
initial_lat="${INITIAL_LAT:-37.9234}"
initial_lon="${INITIAL_LON:--122.5967}"
origin_alt="${ORIGIN_ALT:-781.0}"
terrain_manifest="${TERRAIN_MANIFEST:-/data/navsim_assets/manifest.json}"
base_params="${ARDUCOPTER_BASE_PARAM_FILE:-/workspace/systems/aerodrone/config/arducopter.params}"
backend_params="${ARDUCOPTER_PARAM_FILE:-}"

if [[ -r "$terrain_manifest" ]]; then
  home_csv="$(python3 /workspace/applications/ardupilot_sitl/scripts/terrain_home.py "$terrain_manifest")"
  IFS=, read -r initial_lat initial_lon origin_alt <<< "$home_csv"
fi

args=(
  -v ArduCopter
  -f quad
  --model "JSON:${sim_host}"
  --no-rebuild
  --no-mavproxy
  --custom-location="${initial_lat},${initial_lon},${origin_alt},0"
  --add-param-file="${base_params}"
)
if [[ -n "$backend_params" && "$backend_params" != "$base_params" ]]; then
  args+=(--add-param-file="${backend_params}")
fi

cd /ardupilot
exec python3 Tools/autotest/sim_vehicle.py "${args[@]}"
