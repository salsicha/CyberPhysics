#!/usr/bin/env bash
# Aerodrone GPS-denied simulation test: flies the wilderness mission of the
# Gazebo or Isaac composition, denies GPS 1 for a window of
# simulation time during the cruise (the `denial` profile's gps_denier) and
# scores the recording (gps_denial_recorder) with
# tools/gps_denied_sim_scorer.py against the stack's thresholds. The
# navigation stack validator (`validation` profile) runs alongside; a failed
# or unfinished validation fails the test (exit 3) even when the score
# passes. Intended for nightly/local runs on a host with the stack's images
# and GPU. See tools/gps_denied_sim_test.md.
#
# Usage: tools/gps_denied_sim_test.sh <gazebo|isaac> [scorer options...]
# Environment:
#   ROS_DOMAIN_ID              ROS 2 domain for the run (default 0)
#   GPS_DENIAL_START_S / GPS_DENIAL_DURATION_S / GPS_DENIAL_RECOVERY_S
#                              the denial window in simulation time (defaults
#                              in applications/aerostack2/config/mount_tamalpais.env)
#   GPS_DENIED_OUTPUT          run directory (default generated/gps_denied_sim/<stack>)
#   GPS_DENIED_PROJECT         compose project name (default gps_denied_<stack>)
#   GPS_DENIED_COMPOSE_OVERRIDE  extra compose file layered on the stack, e.g.
#                              tools/gps_denied_sim_control.yaml (control run)
#   GPS_DENIED_TIMEOUT_S       overall wall-clock limit (default gazebo 1800,
#                              isaac 3300)
#   GPS_DENIED_KEEP_VOLUMES    true keeps the stack's DEM/tile caches and
#                              terrain assets for the next run (default false)
#   NAVSIM_REQUIRE_VALID / NAVSIM_REQUIRE_CORRECTION  validator requirements
#                              (default "demnav wildnav" / true)
# Exit status: 0 pass, 1 score failed, 2 no usable run, 3 validation failed.
set -euo pipefail
cd "$(dirname "$0")/.."

stack="${1:-}"
case "$stack" in
    gazebo) composition=compositions/aerostack2_sim.yaml; default_timeout=1800 ;;
    isaac) composition=compositions/aerostack2_isaac.yaml; default_timeout=3300 ;;
    *) echo "usage: $0 <gazebo|isaac> [scorer options...]" >&2; exit 2 ;;
esac
shift

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export NAVSIM_REQUIRE_VALID="${NAVSIM_REQUIRE_VALID-demnav wildnav}"
export NAVSIM_REQUIRE_CORRECTION="${NAVSIM_REQUIRE_CORRECTION:-true}"
GPS_DENIED_OUTPUT="${GPS_DENIED_OUTPUT:-generated/gps_denied_sim/$stack}"
mkdir -p "$GPS_DENIED_OUTPUT"
# Absolute: compose resolves relative bind mounts against compositions/.
GPS_DENIED_OUTPUT="$(cd "$GPS_DENIED_OUTPUT" && pwd)"
export GPS_DENIED_OUTPUT
timeout_s="${GPS_DENIED_TIMEOUT_S:-$default_timeout}"
project="${GPS_DENIED_PROJECT:-gps_denied_$stack}"
events="$GPS_DENIED_OUTPUT/denial_events.jsonl"
for name in recording.jsonl recording.jsonl.gz denial_events.jsonl score.json \
            validation.log stack.log stack.log.gz up.log; do
    rm -f "${GPS_DENIED_OUTPUT:?}/$name"
done

COMPOSE=(docker compose -p "$project" -f "$composition"
         ${GPS_DENIED_COMPOSE_OVERRIDE:+-f "$GPS_DENIED_COMPOSE_OVERRIDE"}
         --profile validation --profile denial)
down_args=(--remove-orphans)
[ "${GPS_DENIED_KEEP_VOLUMES:-false}" = true ] || down_args+=(-v)
torn_down=false
teardown() {
    "$torn_down" && return 0
    torn_down=true
    "${COMPOSE[@]}" logs --no-color --timestamps > "$GPS_DENIED_OUTPUT/stack.log" 2>&1 || true
    "${COMPOSE[@]}" down "${down_args[@]}" > /dev/null 2>&1 || true
}
trap teardown EXIT
id_of() { "${COMPOSE[@]}" ps -a -q "$1" 2>/dev/null || true; }
state_of() { docker inspect -f '{{.State.Status}}' "$1" 2>/dev/null || echo missing; }
exit_code_of() { docker inspect -f '{{.State.ExitCode}}' "$1" 2>/dev/null || echo none; }

echo "GPS-denied $stack: project $project, ROS domain $ROS_DOMAIN_ID," \
     "GPS 1 denied ${GPS_DENIAL_START_S:-60.0} s after cruise for" \
     "${GPS_DENIAL_DURATION_S:-150.0} s of simulation time, output $GPS_DENIED_OUTPUT"
deadline=$(( $(date +%s) + timeout_s ))
# Returns once flight_ready has completed (the vehicle can fly the mission).
if ! timeout "$timeout_s" "${COMPOSE[@]}" up -d > "$GPS_DENIED_OUTPUT/up.log" 2>&1; then
    tail -20 "$GPS_DENIED_OUTPUT/up.log"
    echo "GPS-DENIED SIM ERROR: the $stack stack did not come up (see up.log, stack.log)"
    exit 2
fi
echo "Stack up after $(( timeout_s - (deadline - $(date +%s)) )) s; waiting for the denial window"

denier="$(id_of gps_denier)"
mission="$(id_of wilderness_mission)"
if [ -z "$denier" ]; then
    echo "GPS-DENIED SIM ERROR: no gps_denier service (the denial profile) in $composition"
    exit 2
fi
shown=0
problem="timed out after ${timeout_s} s"
while [ "$(date +%s)" -lt "$deadline" ]; do
    # The denier's steps, as they happen (one JSON line each).
    if [ -f "$events" ]; then
        total="$(wc -l < "$events")"
        if [ "$total" -gt "$shown" ]; then
            sed -n "$((shown + 1)),${total}p" "$events"
            shown="$total"
        fi
    fi
    if [ "$(state_of "$denier")" = exited ]; then
        problem=""
        break
    fi
    if [ "$(state_of "$mission")" = exited ] &&
           ! grep -q '"event": "cruise"' "$events" 2>/dev/null; then
        problem="the mission ended before reaching cruise"
        break
    fi
    sleep 15
done
if [ -n "$problem" ]; then
    echo "GPS-DENIED SIM ERROR: $problem"
elif [ "$(exit_code_of "$denier")" != 0 ]; then
    echo "GPS-DENIED SIM ERROR: gps_denier exited $(exit_code_of "$denier")"
fi

validator="$(id_of navsim_validator)"
validation="unfinished ($(state_of "$validator"))"
if [ "$(state_of "$validator")" = exited ]; then
    if [ "$(exit_code_of "$validator")" = 0 ]; then validation=passed; else validation=failed; fi
fi
"${COMPOSE[@]}" logs --no-color navsim_validator > "$GPS_DENIED_OUTPUT/validation.log" 2>&1 || true
# The denier's events carry wall time too: was the stack validated before the denial?
echo "Navigation stack validation $validation" \
     "(finished $(docker inspect -f '{{.State.FinishedAt}}' "$validator" 2>/dev/null || echo never))"
teardown

status=0
python3 tools/gps_denied_sim_scorer.py "$GPS_DENIED_OUTPUT" --stack "$stack" "$@" || status=$?
# Compact: the scorer reads the compressed recording too.
gzip -f "$GPS_DENIED_OUTPUT/recording.jsonl" "$GPS_DENIED_OUTPUT/stack.log" 2> /dev/null || true
if [ "$validation" != passed ]; then
    echo "GPS-DENIED SIM ERROR: navigation stack validation $validation" \
         "(see $GPS_DENIED_OUTPUT/validation.log)"
    [ "$status" -eq 0 ] && status=3
fi
exit "$status"
