#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /venv/bin/activate
# ptp4l -i enxf8e43be7e67b -m -S > /dev/null 2>&1 &
exec "$@"
