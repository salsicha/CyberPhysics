#!/usr/bin/env bash
# Health check for a running airplane SITL stack: container state, FCU
# link, and message flow on the topics every downstream consumer needs.
# The stack fails silently by design (matchers just publish nothing), so
# run this whenever the sim is up but no fixes appear.
#
# Usage: tools/doctor.sh          (after docker compose ... up)
set -uo pipefail
cd "$(dirname "$0")/.."

NS="${AIRPLANE_NAMESPACE:-plane0}"
pass=0
fail=0

check() {
    local name="$1"
    shift
    if bash -c "$*" >/dev/null 2>&1; then
        printf 'PASS  %s\n' "$name"
        pass=$((pass + 1))
    else
        printf 'FAIL  %s\n' "$name"
        fail=$((fail + 1))
    fi
}

# docker exec through /entrypoint.sh so ROS + the venv are sourced.
topic() {  # topic name, container, timeout
    echo "docker exec $2 /entrypoint.sh timeout ${3:-10} \
          ros2 topic echo --once --no-arr $1"
}

for c in airplane_sitl airplane_sim_mavros airplane_sim_bridge \
         airplane_satellite_camera airplane_sim_demnav \
         airplane_sim_wildnav airplane_sim_evaluator; do
    check "container $c running" \
        "docker inspect -f '{{.State.Running}}' $c 2>/dev/null | grep -q true"
done

check "MAVROS FCU connected" \
    "docker exec airplane_sim_mavros /entrypoint.sh timeout 10 \
     ros2 topic echo --once /mavros/state | grep -q 'connected: true'"
check "GPS flowing"            "$(topic /${NS}/sensor_measurements/gps airplane_sim_bridge)"
check "odometry flowing"       "$(topic /${NS}/sensor_measurements/odom airplane_sim_bridge)"
check "camera image flowing"   "$(topic /airplane/downward/image_highres airplane_satellite_camera 15)"
check "depth flowing"          "$(topic /airplane/downward/relative_depth airplane_satellite_camera 15)"
check "demnav publishing"      "$(topic /demnav/valid airplane_sim_demnav 15)"
check "wildnav publishing"     "$(topic /wildnav/valid airplane_sim_wildnav 20)"
check "fused odometry flowing" "$(topic /airplane/navigation/odometry airplane_sim_wildnav 15)"
check "evaluator reporting"    "$(topic /nav_eval/summary airplane_sim_evaluator 60)"

echo
echo "${pass} passed, ${fail} failed"
[ "$fail" -eq 0 ]
