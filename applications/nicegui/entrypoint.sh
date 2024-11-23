#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /venv/bin/activate
source /ros2_ws/install/setup.bash
exec "$@"
