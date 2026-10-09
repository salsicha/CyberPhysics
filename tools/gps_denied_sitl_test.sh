#!/usr/bin/env bash
# Aerodrone GPS-denied SITL test: ArduCopter flies a GUIDED square on GPS,
# loses its GPS mid-flight, and must keep flying GUIDED on DemNav/WildNav
# corrections fed back as GPS_INPUT. Scores the fused navigation fix and
# ArduPilot's own position against simulator ground truth.
#
# Intended for nightly/local runs on a host with the cyberphysics navigator,
# aerostack2, demnav and wildnav images (the working tree's Python sources
# are mounted into them). See tools/gps_denied_sitl_test.md.
#
# Usage: tools/gps_denied_sitl_test.sh [scorer options...]
# Environment:
#   GPS_INPUT_CORRECTIONS  true (default) | false (control run: no feedback)
#   ROS_DOMAIN_ID          ROS 2 domain for the run (default 0)
#   SITL_INSTANCE          ArduPilot SITL instance; MAVLink on 5760+10*N (default 0)
#   GPS_DENIED_OUTPUT      telemetry directory (default generated/gps_denied_sitl)
#   GPS_DENIED_DENY_AFTER_S / GPS_DENIED_DENIED_S / GPS_DENIED_RESTORE_S /
#   GPS_DENIED_TAKEOFF_ALT_M / GPS_DENIED_LEG_M   mission timing and shape
#   GPS_DENIED_TIMEOUT_S   overall wall-clock limit (default 1500)
set -euo pipefail
cd "$(dirname "$0")/.."

export GPS_INPUT_CORRECTIONS="${GPS_INPUT_CORRECTIONS:-true}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export SITL_INSTANCE="${SITL_INSTANCE:-0}"
export SITL_PORT=$((5760 + 10 * SITL_INSTANCE))
export INITIAL_LAT="${INITIAL_LAT:-37.9234}"
export INITIAL_LON="${INITIAL_LON:--122.5967}"
export GPS_DENIED_OUTPUT="${GPS_DENIED_OUTPUT:-$PWD/generated/gps_denied_sitl}"
timeout_s="${GPS_DENIED_TIMEOUT_S:-1500}"
project="${GPS_DENIED_PROJECT:-gps_denied_sitl_${SITL_INSTANCE}}"
mkdir -p "$GPS_DENIED_OUTPUT"
rm -f "$GPS_DENIED_OUTPUT/telemetry.json"

COMPOSE=(docker compose -p "$project" -f tools/gps_denied_sitl.yaml)
cleanup() {
    "${COMPOSE[@]}" logs --no-color > "$GPS_DENIED_OUTPUT/stack.log" 2>&1 || true
    "${COMPOSE[@]}" down --remove-orphans > /dev/null 2>&1 || true
}
trap cleanup EXIT

# SITL's flat ground must sit at the synthetic terrain height under home so
# the simulated camera sees the altitude ArduPilot flies.
export ORIGIN_ALT="${ORIGIN_ALT:-$(docker run --rm --entrypoint python3 \
    -v "$PWD/applications/aerostack2/synthetic_world:/opt/sw:ro" -e PYTHONPATH=/opt/sw \
    "salsicha/cyberphysics:aerostack2-${CYBERPHYSICS_TAG:-latest}" \
    -c 'from synthetic_world import terrain_height; print(round(float(terrain_height(0.0, 0.0)), 2))')}"

echo "GPS-denied SITL: corrections=${GPS_INPUT_CORRECTIONS} domain=${ROS_DOMAIN_ID}" \
     "instance=${SITL_INSTANCE} home_alt=${ORIGIN_ALT} m output=${GPS_DENIED_OUTPUT}"
"${COMPOSE[@]}" up -d

deadline=$(( $(date +%s) + timeout_s ))
mission="$("${COMPOSE[@]}" ps -a -q mission)"
while [ "$(date +%s)" -lt "$deadline" ]; do
    state="$(docker inspect -f '{{.State.Status}}' "$mission" 2>/dev/null || echo missing)"
    [ "$state" = exited ] && break
    sleep 10
done
docker logs "$mission" 2>&1 | grep -E 'EVENT|FCU: |Wrote|Error|error' | tail -40 || true

if [ ! -s "$GPS_DENIED_OUTPUT/telemetry.json" ]; then
    echo "GPS-DENIED SITL ERROR: no telemetry (mission state: ${state:-unknown})"
    exit 2
fi
python3 tools/gps_denied_sitl_scorer.py "$GPS_DENIED_OUTPUT/telemetry.json" "$@" \
    | tee "$GPS_DENIED_OUTPUT/score.json"
exit "${PIPESTATUS[0]}"
