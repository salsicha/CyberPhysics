#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /venv/bin/activate
export PYTHONPATH="${RACECAR_LIBRARY_DIR:-/opt/racecarneo/racecar-neo-library/library}:${RACECAR_LIBRARY_DIR:-/opt/racecarneo/racecar-neo-library/library}/simulation:${PYTHONPATH}"
exec "$@"
