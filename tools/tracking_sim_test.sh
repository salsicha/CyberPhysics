#!/usr/bin/env bash
# Aerodrone object-tracking simulation test (systems/aerodrone/tracking.md):
# runs a composition with the `tracking` profile (the YOLO detector on the
# simulated camera, end to end; TRACKING_TEST_DETECTOR=truth: the
# `tracking_truth` profile, the ground-truth detector's boxes) and
# tools/tracking_sim_test.yaml, whose tracking_sim_test service
# (tools/tracking_sim_test.py fly) takes off, picks the targets by the ids
# of their boxes as the web UI's operator would, tests that a track
# pre-empts a GoTo, follows the car centered and then slant, hides a person
# under the terrain (lost, then tracked again under its new id), stops,
# tests the pre-emption of tracking by a map goal and by a GoTo, and lands;
# and the scorer's thresholds then judge the recording. With
# TRACKING_TEST_SCENARIO=fire it flies the foam dart launcher's fire control
# instead (systems/aerodrone/tracking.md, "Foam dart launcher"; the
# simulators' stand-in launcher): a shot at the car, halted (allowed), a fire
# command at a standing person and one at the car with a person in view
# (both refused), then lands. Either way the recording is scored
# (tools/tracking_sim_test.py score), with the host's and the containers'
# CPU sampled while the test flies.
#
# Usage: tools/tracking_sim_test.sh <gazebo|isaac>
# Environment:
#   ROS_DOMAIN_ID               ROS 2 domain for the run (default 0)
#   TRACKING_TEST_DETECTOR      real (default) or truth
#   TRACKING_TRUTH_SYNTHETIC    with truth: true makes the truth detector make
#                               up its own targets (tracking_truth_detector.py)
#                               instead of the simulator's /simulation/targets
#   TRACKING_TEST_OUTPUT        run directory (default generated/tracking_sim/<stack>)
#   TRACKING_TEST_PROJECT       compose project name (default tracking_<stack>)
#   TRACKING_TEST_TIMEOUT_S     overall wall-clock limit (default gazebo 2400,
#                               isaac 7200)
#   TRACKING_TEST_KEEP_VOLUMES  true keeps the DEM/tile caches and terrain
#                               assets for the next run (default false)
#   TRACKING_TEST_COMPOSE_OVERRIDE  extra compose file layered last
#   TRACKING_TEST_SCENARIO      tracking (default) or fire
#   TRACKING_TEST_TARGET, TRACKING_TEST_LOST_TARGET, TRACKING_TEST_ALTITUDE_M,
#   TRACKING_TEST_LOST_ALTITUDE_M, TRACKING_TEST_SLANT_RANGE_M, TRACKING_TEST_DEPRESSION_DEG,
#   TRACKING_TEST_CENTERED_S, TRACKING_TEST_SLANT_S
#                               see tools/tracking_sim_test.yaml
# Writes recording.jsonl, score.json, cpu.jsonl, stack.log and up.log.
# Exit status: 0 pass, 1 a check failed, 2 no usable run.
set -euo pipefail
cd "$(dirname "$0")/.."

stack="${1:-}"
layers=(-f tools/tracking_sim_test.yaml)
case "$stack" in
    gazebo) composition=compositions/aerostack2_sim.yaml; default_timeout=2400
            layers+=(-f tools/tracking_sim_test_gazebo.yaml) ;;
    isaac) composition=compositions/aerostack2_isaac.yaml; default_timeout=7200 ;;
    *) echo "usage: $0 <gazebo|isaac>" >&2; exit 2 ;;
esac
[ -z "${TRACKING_TEST_COMPOSE_OVERRIDE:-}" ] || layers+=(-f "$TRACKING_TEST_COMPOSE_OVERRIDE")

env_value() {  # the value of $1 in the stack's tracking env file
    sed -n "s/^$1=//p" "applications/aerostack2/config/tracking_$stack.env"
}

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export TRACKING_TEST_STACK="$stack"
export TRACKING_TEST_DETECTOR="${TRACKING_TEST_DETECTOR:-real}"
case "$TRACKING_TEST_DETECTOR" in
    real) profile=tracking ;;
    truth) profile=tracking_truth ;;
    *) echo "TRACKING_TEST_DETECTOR must be real or truth" >&2; exit 2 ;;
esac
export TRACKING_TEST_MOUNT_YAW_DEG TRACKING_TEST_CAMERA_STABILIZED
TRACKING_TEST_MOUNT_YAW_DEG="$(env_value TRACKING_CAMERA_MOUNT_YAW_DEG)"
TRACKING_TEST_CAMERA_STABILIZED="$(env_value TRACKING_CAMERA_STABILIZED)"
export TRACKING_TRUTH_SYNTHETIC="${TRACKING_TRUTH_SYNTHETIC:-false}"
if [ "$TRACKING_TRUTH_SYNTHETIC" = true ]; then
    export TRACKING_TEST_TARGETS_TOPIC=/tracking/truth/targets
fi
TRACKING_TEST_OUTPUT="${TRACKING_TEST_OUTPUT:-generated/tracking_sim/$stack}"
mkdir -p "$TRACKING_TEST_OUTPUT"
# Absolute: compose resolves relative bind mounts against compositions/.
TRACKING_TEST_OUTPUT="$(cd "$TRACKING_TEST_OUTPUT" && pwd)"
export TRACKING_TEST_OUTPUT
rm -f "$TRACKING_TEST_OUTPUT"/{recording.jsonl,score.json,cpu.jsonl,stack.log,up.log}
timeout_s="${TRACKING_TEST_TIMEOUT_S:-$default_timeout}"
project="${TRACKING_TEST_PROJECT:-tracking_$stack}"

COMPOSE=(docker compose -p "$project" -f "$composition" "${layers[@]}"
         --profile "$profile")
down_args=(--remove-orphans)
[ "${TRACKING_TEST_KEEP_VOLUMES:-false}" = true ] || down_args+=(-v)
torn_down=false
sampler=""
teardown() {
    "$torn_down" && return 0
    torn_down=true
    [ -z "$sampler" ] || kill "$sampler" 2> /dev/null || true
    "${COMPOSE[@]}" logs --no-color --timestamps > "$TRACKING_TEST_OUTPUT/stack.log" 2>&1 || true
    "${COMPOSE[@]}" down "${down_args[@]}" > /dev/null 2>&1 || true
}
trap teardown EXIT

echo "Tracking test $stack: project $project, ROS domain $ROS_DOMAIN_ID," \
     "detector $TRACKING_TEST_DETECTOR, synthetic targets $TRACKING_TRUTH_SYNTHETIC," \
     "output $TRACKING_TEST_OUTPUT"
deadline=$(( $(date +%s) + timeout_s ))
if ! timeout "$timeout_s" "${COMPOSE[@]}" up -d > "$TRACKING_TEST_OUTPUT/up.log" 2>&1; then
    tail -20 "$TRACKING_TEST_OUTPUT/up.log"
    echo "TRACKING SIM ERROR: the $stack stack did not come up (see up.log, stack.log)"
    exit 2
fi
driver="$("${COMPOSE[@]}" ps -a -q tracking_sim_test)"
python3 tools/tracking_sim_test.py cpu --project "$project" \
    --output "$TRACKING_TEST_OUTPUT/cpu.jsonl" &
sampler=$!
echo "Stack up; the test runs (docker logs -f $driver)"
while [ "$(date +%s)" -lt "$deadline" ]; do
    [ "$(docker inspect -f '{{.State.Status}}' "$driver" 2>/dev/null)" = exited ] && break
    sleep 10
done
status="$(docker inspect -f '{{.State.Status}} {{.State.ExitCode}}' "$driver" 2>/dev/null || echo missing)"
echo "Test driver: $status"
teardown
if [ ! -s "$TRACKING_TEST_OUTPUT/recording.jsonl" ]; then
    echo "TRACKING SIM ERROR: no recording"
    exit 2
fi
result=0
python3 tools/tracking_sim_test.py score "$TRACKING_TEST_OUTPUT/recording.jsonl" \
    --cpu "$TRACKING_TEST_OUTPUT/cpu.jsonl" --stack-log "$TRACKING_TEST_OUTPUT/stack.log" \
    > "$TRACKING_TEST_OUTPUT/score.json" || result=1
cat "$TRACKING_TEST_OUTPUT/score.json"
[ "$status" = "exited 0" ] || result=1
exit "$result"
