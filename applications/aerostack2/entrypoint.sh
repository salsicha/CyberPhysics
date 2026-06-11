#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /venv/bin/activate
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi
exec "$@"
