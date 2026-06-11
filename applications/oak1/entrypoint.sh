#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /venv/bin/activate
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi
# ptp4l -i enxf8e43be7e67b -m -S > /dev/null 2>&1 &
exec "$@"
