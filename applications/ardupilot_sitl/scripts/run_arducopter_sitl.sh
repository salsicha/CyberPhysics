#!/bin/bash
set -euo pipefail

sim_host="${SIM_JSON_HOST:-127.0.0.1}"
# JSON:<host> drives an external physics backend (Gazebo, Isaac);
# "builtin" uses ArduPilot's own multicopter model (tools/gps_denied_sitl).
sitl_model="${ARDUCOPTER_SITL_MODEL:-JSON:${sim_host}}"
# Instance N serves MAVLink on TCP 5760 + 10 * N.
sitl_instance="${SITL_INSTANCE:-0}"
initial_lat="${INITIAL_LAT:-37.9234}"
initial_lon="${INITIAL_LON:--122.5967}"
origin_alt="${ORIGIN_ALT:-781.0}"
terrain_manifest="${TERRAIN_MANIFEST:-/data/navsim_assets/manifest.json}"
config_dir=/workspace/systems/aerodrone/config
# Layering order: hardware baseline -> SITL-only relaxations -> GPS-denied
# corrections (GPS_INPUT_CORRECTIONS, default true like every stack's
# navigation bridge; false skips the file) -> simulator backend compensation.
base_params="${ARDUCOPTER_BASE_PARAM_FILE:-${config_dir}/arducopter.params}"
sitl_params="${ARDUCOPTER_SITL_PARAM_FILE:-${config_dir}/arducopter_sitl.params}"
gps_input_params=""
if [[ "${GPS_INPUT_CORRECTIONS:-true}" == "true" ]]; then
  gps_input_params="${ARDUCOPTER_GPS_INPUT_PARAM_FILE:-${config_dir}/arducopter_gps_input.params}"
fi
backend_params="${ARDUCOPTER_PARAM_FILE:-}"

if [[ -r "$terrain_manifest" ]]; then
  home_csv="$(python3 /workspace/applications/ardupilot_sitl/scripts/terrain_home.py "$terrain_manifest")"
  IFS=, read -r initial_lat initial_lon origin_alt <<< "$home_csv"
fi

args=(
  -v ArduCopter
  -f quad
  --no-rebuild
  --no-mavproxy
  -I "${sitl_instance}"
  --custom-location="${initial_lat},${initial_lon},${origin_alt},0"
)
if [[ "$sitl_model" != "builtin" ]]; then
  args+=(--model "$sitl_model")
fi
for params in "$base_params" "$sitl_params" "$gps_input_params" "$backend_params"; do
  [[ -n "$params" ]] || continue
  for added in "${args[@]}"; do
    [[ "$added" == "--add-param-file=${params}" ]] && continue 2
  done
  args+=(--add-param-file="${params}")
done

cd /ardupilot
exec python3 Tools/autotest/sim_vehicle.py "${args[@]}"
