#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /venv/bin/activate
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi
exec "$@"
