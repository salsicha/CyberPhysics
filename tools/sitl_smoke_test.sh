#!/usr/bin/env bash
# Boot the airplane SITL stack, fly, optionally deny GPS, and assert
# navigation accuracy via the nav_evaluator node. Intended for nightly runs
# on a host with the cyberphysics images already built (make -C applications
# build_ardupilot_sitl build_demnav build_wildnav).
#
# Accuracy is scored against simulator ground truth (SIM_STATE via
# applications/ardupilot_sitl/scripts/sitl_ground_truth.py), not against the
# simulated GPS the navigation stack itself uses, and the synthetic camera
# renders at the true pose, so the GPS can be degraded or switched off.
#
# Usage: tools/sitl_smoke_test.sh [duration_seconds]     (default 600)
# Thresholds: SMOKE_MAX_RMSE_M (default 25.0), SMOKE_MIN_FIXES (default 3).
# SMOKE_FLY=1 (default) arms ArduPlane in TAKEOFF mode so it climbs to
# TKOFF_ALT and circles; SMOKE_FLY=0 keeps the old stationary check.
# SMOKE_GPS_DENY_AFTER_S=N disables the simulated GPS (SIM_GPS_DISABLE=1)
# N seconds after the stack is up (default: never).
set -euo pipefail
cd "$(dirname "$0")/.."

DURATION="${1:-600}"
# Own project name: "down --remove-orphans" in the default project (named
# after compositions/) would also remove other stacks' containers.
COMPOSE=(docker compose -p "${SMOKE_PROJECT:-airplane_smoke}"
         --env-file systems/airplane/config/sitl.env
         -f compositions/airplane_sim.yaml ${SMOKE_COMPOSE_OVERRIDE:+-f "$SMOKE_COMPOSE_OVERRIDE"})

# Exported host env overrides the --env-file for compose interpolation.
# The evaluator declares these as doubles: "600" would be rejected as an
# integer parameter, so always pass a decimal point.
export NAV_EVAL_ASSERT_AFTER_S="$(printf '%.1f' "${DURATION}")"
export NAV_EVAL_MAX_RMSE_M="$(printf '%.1f' "${SMOKE_MAX_RMSE_M:-25.0}")"
export NAV_EVAL_MIN_FIXES="${SMOKE_MIN_FIXES:-3}"
export NAV_EVAL_REPORT_PERIOD_S="30.0"

cleanup() { "${COMPOSE[@]}" down --remove-orphans >/dev/null 2>&1 || true; }
trap cleanup EXIT

container() { "${COMPOSE[@]}" ps -a -q "$1"; }
ros() {
    docker exec "$(container airplane_mavros)" bash -c \
        "source /opt/ros/humble/setup.bash && $*" 2>/dev/null
}

echo "Starting airplane SITL stack (asserting after ${DURATION}s)..."
"${COMPOSE[@]}" up -d
started=$(date +%s)

if [ "${SMOKE_FLY:-1}" = "1" ]; then
    # Arm in TAKEOFF once ArduPlane accepts it (EKF/GPS ready). MAVROS has
    # no name for Plane's TAKEOFF mode (13): use MAV_CMD_DO_SET_MODE.
    for _ in $(seq 1 60); do
        ros "ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
            \"{command: 176, param1: 1.0, param2: 13.0}\"" \
            | grep -q 'success=True' || { sleep 5; continue; }
        if ros "ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\"" \
                | grep -q 'success=True'; then
            echo "ArduPlane armed in TAKEOFF"
            break
        fi
        sleep 5
    done
fi

deny_after="${SMOKE_GPS_DENY_AFTER_S:-}"
denied=0
deadline=$(( started + DURATION + 180 ))
result=""
while [ "$(date +%s)" -lt "$deadline" ]; do
    if [ -n "$deny_after" ] && [ "$denied" = 0 ] && \
            [ "$(date +%s)" -ge $(( started + deny_after )) ]; then
        ros "ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
            \"{force_set: true, param_id: SIM_GPS_DISABLE, value: {type: 2, integer_value: 1}}\"" \
            | grep -q 'success=True' && { denied=1; echo "Simulated GPS disabled"; }
    fi
    result="$(docker logs "$(container airplane_evaluator)" 2>&1 |
              grep 'NAV-EVAL RESULT:' | tail -1 || true)"
    [ -n "$result" ] && break
    sleep 15
done

echo "--- evaluator summary ---"
docker logs "$(container airplane_evaluator)" 2>&1 | grep 'NAV-EVAL' | tail -5 || true

case "$result" in
    *PASS*) echo "SMOKE TEST PASS"; exit 0 ;;
    *FAIL*) echo "SMOKE TEST FAIL"; exit 1 ;;
    *) echo "SMOKE TEST ERROR: no evaluator result before timeout"; exit 2 ;;
esac
