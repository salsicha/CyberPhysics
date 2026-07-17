#!/usr/bin/env bash
# Boot the airplane SITL stack, fly for a while, and assert navigation
# accuracy via the nav_evaluator node. Intended for nightly runs on a host
# with the cyberphysics images already built (make -C applications
# build_ardupilot_sitl build_demnav build_wildnav).
#
# Usage: tools/sitl_smoke_test.sh [duration_seconds]     (default 600)
# Thresholds: SMOKE_MAX_RMSE_M (default 25.0), SMOKE_MIN_FIXES (default 3).
set -euo pipefail
cd "$(dirname "$0")/.."

DURATION="${1:-600}"
COMPOSE=(docker compose --env-file systems/airplane/config/sitl.env
         -f compositions/airplane_sim.yaml)

# Exported host env overrides the --env-file for compose interpolation.
export NAV_EVAL_ASSERT_AFTER_S="${DURATION}"
export NAV_EVAL_MAX_RMSE_M="${SMOKE_MAX_RMSE_M:-25.0}"
export NAV_EVAL_MIN_FIXES="${SMOKE_MIN_FIXES:-3}"
export NAV_EVAL_REPORT_PERIOD_S="30.0"

cleanup() { "${COMPOSE[@]}" down --remove-orphans >/dev/null 2>&1 || true; }
trap cleanup EXIT

echo "Starting airplane SITL stack (asserting after ${DURATION}s)..."
"${COMPOSE[@]}" up -d

deadline=$(( $(date +%s) + DURATION + 180 ))
result=""
while [ "$(date +%s)" -lt "$deadline" ]; do
    result="$(docker logs airplane_sim_evaluator 2>&1 |
              grep 'NAV-EVAL RESULT:' | tail -1 || true)"
    [ -n "$result" ] && break
    sleep 15
done

echo "--- evaluator summary ---"
docker logs airplane_sim_evaluator 2>&1 | grep 'NAV-EVAL' | tail -5 || true

case "$result" in
    *PASS*) echo "SMOKE TEST PASS"; exit 0 ;;
    *FAIL*) echo "SMOKE TEST FAIL"; exit 1 ;;
    *) echo "SMOKE TEST ERROR: no evaluator result before timeout"; exit 2 ;;
esac
