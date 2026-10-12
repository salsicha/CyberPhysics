#!/bin/bash
# Start racecar_turret_isaac.py through Isaac Sim's python.sh and forward
# SIGTERM and SIGINT to the simulator. python.sh runs python as a child
# instead of exec'ing it, so `docker compose down` would otherwise never
# reach the simulator: the container is SIGKILLed after the stop timeout and
# Kit does not shut down cleanly.
set -u
script="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/racecar_turret_isaac.py"
cd /isaac-sim
./python.sh "${script}" "$@" &
launcher=$!

stop() {
    pkill -TERM -f -- "kit/python/bin/python3 .*racecar_turret_isaac.py" || kill -TERM "${launcher}" 2>/dev/null
}
trap stop TERM INT

status=0
wait "${launcher}" || status=$?
# A trapped signal interrupts `wait`; keep waiting until the simulator exits.
while kill -0 "${launcher}" 2>/dev/null; do
    status=0
    wait "${launcher}" || status=$?
done
exit "${status}"
