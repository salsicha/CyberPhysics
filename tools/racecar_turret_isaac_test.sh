#!/usr/bin/env bash
# RACECAR Turret Isaac evaluation (systems/racecar_turret/interfaces.md,
# applications/isaac/RACECAR_TURRET.md): runs compositions/racecar_turret_isaac.yaml
# with its `evaluation` profile once per run label, in which
# tools/racecar_turret_evaluator.py selects the target through
# /turret/command as the UI does, drives the Nav2 course and records the
# run; then scores every run with tools/racecar_turret_scorer.py against
# systems/racecar_turret/validation/acceptance_thresholds.json.
#
#   compensated     the tracker as configured (base-rotation compensation
#                   and target lead on)
#   uncompensated   both off (RACECAR_TURRET_COMPENSATION=false), for
#                   comparison
#
# Each run is a fresh stack (compose project RACECAR_TURRET_TEST_PROJECT),
# torn down with its volumes afterwards, also on failure or interruption.
# GPU memory, GPU load and the containers' CPU and memory are sampled
# every 20 s into resources.log. Each run also saves nvblox's map slice and
# Nav2's global costmap every 5 s (maps/), and
# tools/racecar_turret_map_coverage.py scores them against the scenario's
# obstacles (map_coverage.json; informational, not a threshold).
#
# Usage: tools/racecar_turret_isaac_test.sh [compensated] [uncompensated]
# Environment:
#   ROS_DOMAIN_ID                     ROS 2 domain for the runs (default 0)
#   RACECAR_TURRET_TEST_OUTPUT        output directory (default
#                                     generated/racecar_turret_isaac); a
#                                     subdirectory per run, summary.json
#   RACECAR_TURRET_TEST_PROJECT       compose project name (default rt_isaac)
#   RACECAR_TURRET_TEST_TIMEOUT_S     wall-clock limit of each run (default 5400)
#   RACECAR_TURRET_TEST_KEEP_VOLUMES  true keeps the downloaded people models
#                                     and baked assets (default false)
# Exit status: 0 every threshold met, 1 one missed, 2 no usable run.
set -euo pipefail
cd "$(dirname "$0")/.."

runs=("$@")
[ "${#runs[@]}" -gt 0 ] || runs=(compensated uncompensated)
for label in "${runs[@]}"; do
    case "$label" in
        compensated|uncompensated) ;;
        *) echo "usage: $0 [compensated] [uncompensated]" >&2; exit 2 ;;
    esac
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
output="${RACECAR_TURRET_TEST_OUTPUT:-generated/racecar_turret_isaac}"
mkdir -p "$output"
# Absolute: compose resolves relative bind mounts against compositions/.
output="$(cd "$output" && pwd)"
project="${RACECAR_TURRET_TEST_PROJECT:-rt_isaac}"
timeout_s="${RACECAR_TURRET_TEST_TIMEOUT_S:-5400}"
COMPOSE=(docker compose -p "$project" -f compositions/racecar_turret_isaac.yaml
         --profile evaluation)
down_args=(--remove-orphans)
[ "${RACECAR_TURRET_TEST_KEEP_VOLUMES:-false}" = true ] || down_args+=(-v)

run_dir=""
sampler=""
teardown() {
    [ -n "$sampler" ] && kill "$sampler" 2>/dev/null || true
    sampler=""
    [ -n "$run_dir" ] || return 0
    "${COMPOSE[@]}" logs --no-color --timestamps > "$run_dir/stack.log" 2>&1 || true
    "${COMPOSE[@]}" down "${down_args[@]}" > /dev/null 2>&1 || true
    gzip -f "$run_dir/stack.log" 2> /dev/null || true
    run_dir=""
}
trap teardown EXIT
trap 'exit 130' INT TERM
state_of() { docker inspect -f '{{.State.Status}}' "$1" 2>/dev/null || echo missing; }
exit_code_of() { docker inspect -f '{{.State.ExitCode}}' "$1" 2>/dev/null || echo none; }

sample_resources() {  # $1: log file
    while true; do
        {
            echo "== $(date -Iseconds)"
            nvidia-smi --query-gpu=memory.used,memory.total,utilization.gpu \
                --format=csv,noheader 2>/dev/null | sed 's/^/gpu: /'
            nvidia-smi --query-compute-apps=pid,process_name,used_memory \
                --format=csv,noheader 2>/dev/null | sed 's/^/gpu_app: /'
            free -m | awk '/^Mem:/ {print "host_mem_used_mib: " $3}'
            ids="$(docker ps -q --filter "label=com.docker.compose.project=$project")"
            # shellcheck disable=SC2086  # one argument per container
            [ -z "$ids" ] || docker stats --no-stream \
                --format '{{.Name}} {{.CPUPerc}} {{.MemUsage}}' $ids 2>/dev/null \
                | sed 's/^/container: /'
        } >> "$1"
        sleep 20
    done
}

for label in "${runs[@]}"; do
    run_dir="$output/$label"
    mkdir -p "$run_dir"
    rm -f "$run_dir"/{recording.jsonl,recording.jsonl.gz,score.json,stack.log,stack.log.gz,up.log,resources.log,map_coverage.json}
    # Created here so the map recorder's files (root's) stay removable.
    mkdir -p "$run_dir/maps"
    rm -f "$run_dir"/maps/*.npz "$run_dir"/maps/.*.npz
    export RACECAR_TURRET_OUTPUT="$run_dir" RACECAR_TURRET_LABEL="$label"
    if [ "$label" = compensated ]; then
        export RACECAR_TURRET_COMPENSATION=true
    else
        export RACECAR_TURRET_COMPENSATION=false
    fi
    echo "RACECAR Turret Isaac $label: project $project, ROS domain $ROS_DOMAIN_ID," \
         "base compensation and target lead $RACECAR_TURRET_COMPENSATION, output $run_dir"
    sample_resources "$run_dir/resources.log" &
    sampler=$!
    started=$(date +%s)
    deadline=$(( started + timeout_s ))
    # Returns once the simulator and the tracker are healthy and the
    # evaluator has started.
    if ! timeout "$timeout_s" "${COMPOSE[@]}" up -d > "$run_dir/up.log" 2>&1; then
        tail -20 "$run_dir/up.log"
        echo "RACECAR TURRET ISAAC ERROR: the stack did not come up (see up.log, stack.log.gz)"
        teardown
        continue
    fi
    echo "Stack up after $(( $(date +%s) - started )) s; driving the course"
    evaluator="$("${COMPOSE[@]}" ps -a -q racecar_turret_evaluation 2>/dev/null || true)"
    shown=""
    while [ "$(date +%s)" -lt "$deadline" ] && [ "$(state_of "$evaluator")" != exited ]; do
        # The evaluator's steps as they happen (its event lines).
        events="$(grep -a '"event"' "$run_dir/recording.jsonl" 2>/dev/null | cut -c1-220 || true)"
        if [ "$events" != "$shown" ]; then
            { diff <(printf '%s\n' "$shown") <(printf '%s\n' "$events") || true; } \
                | sed -n 's/^> //p'
            shown="$events"
        fi
        sleep 15
    done
    if [ "$(state_of "$evaluator")" != exited ]; then
        echo "RACECAR TURRET ISAAC ERROR: $label timed out after $timeout_s s"
    elif [ "$(exit_code_of "$evaluator")" != 0 ]; then
        echo "RACECAR TURRET ISAAC ERROR: the evaluator exited $(exit_code_of "$evaluator")" \
             "(2: stack not ready, 3: target not selected)"
    fi
    grep -a '"event"' "$run_dir/recording.jsonl" 2>/dev/null | tail -1 | cut -c1-300 || true
    echo "$label took $(( $(date +%s) - started )) s of wall clock; peak GPU memory" \
         "$(sed -n 's/^gpu: \([0-9]*\) MiB.*/\1/p' "$run_dir/resources.log" | sort -n | tail -1) MiB"
    teardown
done

status=0
args=()
for label in "${runs[@]}"; do args+=(--run "$label=$output/$label"); done
python3 tools/racecar_turret_scorer.py "${args[@]}" \
    --thresholds systems/racecar_turret/validation/acceptance_thresholds.json \
    --output "$output/summary.json" || status=$?
for label in "${runs[@]}"; do
    python3 tools/racecar_turret_map_coverage.py score --run-dir "$output/$label" \
        --scenario systems/racecar_turret/scenarios/isaac_arena.json --label "$label" \
        --output "$output/$label/map_coverage.json" \
        || echo "RACECAR TURRET ISAAC: no map coverage for $label"
    gzip -f "$output/$label/recording.jsonl" 2> /dev/null || true
done
exit "$status"
