#!/usr/bin/env bash
# Browser test of the aerodrone web UI (applications/aerostack2/src/
# as2_platform_blueos/web) against the mock tracker, with no simulator:
#
#   tools/tracking_ui_test.sh [OUTPUT_DIR]
#
# On a private Docker network, one aerostack2 container runs rosbridge, the
# page's HTTP server (the repository's web directory, mounted) and
# applications/aerostack2/scripts/mock_tracking_publisher.py --static
# --navigation, and a second mock with the foam dart launcher (--launcher)
# under the prefix /launcher_test/tracking; the Playwright image then runs
# tools/tracking_ui_browser_check.py against http://<container>:8080/ (and
# its ?prefix=/launcher_test/tracking for the launcher's card). The page
# loads Leaflet, roslib and the map tiles from the internet. Screenshots, the mock's log of the commands it
# received and the container logs go to OUTPUT_DIR (default
# generated/tracking_ui). Removes its containers and network on exit.
#
# Environment: CYBERPHYSICS_TAG (aerostack2 image tag, default latest),
# PLAYWRIGHT_VERSION (default 1.63.0: the Playwright image, whose browsers
# need the same version of the Python package, installed from PyPI at each
# run), ROS_DOMAIN_ID (default 97), TRACKING_UI_TEST_NAME (container and
# network name, default tracking_ui_test), TRACKING_UI_CHECK_ARGS (extra
# arguments of the browser check, e.g. --only-launcher).
set -euo pipefail

repo=$(cd "$(dirname "$0")/.." && pwd)
out=$(mkdir -p "${1:-$repo/generated/tracking_ui}" && cd "${1:-$repo/generated/tracking_ui}" && pwd)
name=${TRACKING_UI_TEST_NAME:-tracking_ui_test}
aerostack2=salsicha/cyberphysics:aerostack2-${CYBERPHYSICS_TAG:-latest}
playwright_version=${PLAYWRIGHT_VERSION:-1.63.0}
user="$(id -u):$(id -g)"

cleanup() {
  docker rm -f "$name" > /dev/null 2>&1 || true
  docker network rm "$name" > /dev/null 2>&1 || true
}
trap cleanup EXIT
cleanup

: > "$out/commands.jsonl"
: > "$out/launcher_commands.jsonl"
docker network create "$name" > /dev/null
docker run -d --name "$name" --network "$name" --user "$user" \
  -e HOME=/tmp -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-97}" \
  -v "$repo/applications/aerostack2/src/as2_platform_blueos/web:/web:ro" \
  -v "$repo/applications/aerostack2/scripts:/scripts:ro" \
  -v "$out:/out" \
  "$aerostack2" bash -c '
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9092 > /out/rosbridge.log 2>&1 &
    python3 /scripts/mock_tracking_publisher.py --static --navigation \
      --command-log /out/commands.jsonl > /out/mock.log 2>&1 &
    python3 /scripts/mock_tracking_publisher.py --static --launcher \
      --prefix /launcher_test/tracking \
      --command-log /out/launcher_commands.jsonl > /out/launcher_mock.log 2>&1 &
    exec python3 -m http.server 8080 --directory /web > /out/http.log 2>&1' > /dev/null

# rosbridge and the mock are up once the mock has printed its banner and
# rosbridge listens.
for _ in $(seq 60); do
  if command grep -aq 'mock tracking publisher' "$out/mock.log" 2> /dev/null &&
     command grep -aq 'mock tracking publisher' "$out/launcher_mock.log" 2> /dev/null &&
     command grep -aq 'Rosbridge WebSocket server started' "$out/rosbridge.log" 2> /dev/null; then
    break
  fi
  sleep 1
done

docker run --rm --network "$name" --user "$user" -e HOME=/tmp --shm-size 1g \
  -e PLAYWRIGHT_VERSION="$playwright_version" -e URL="http://$name:8080/" \
  -e TRACKING_UI_CHECK_ARGS="${TRACKING_UI_CHECK_ARGS:-}" \
  -v "$repo/tools:/tools:ro" -v "$out:/out" \
  "mcr.microsoft.com/playwright/python:v$playwright_version-noble" bash -c '
    pip install --quiet --disable-pip-version-check --target /tmp/playwright \
      "playwright==$PLAYWRIGHT_VERSION" &&
    PYTHONPATH=/tmp/playwright python3 /tools/tracking_ui_browser_check.py \
      --url "$URL" --commands /out/commands.jsonl --out /out \
      --launcher-prefix /launcher_test/tracking \
      --launcher-commands /out/launcher_commands.jsonl $TRACKING_UI_CHECK_ARGS'
