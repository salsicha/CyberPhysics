#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO:-${ROS2_DISTRO:-jazzy}}/setup.bash
source /venv/bin/activate
source /workspace/install/setup.bash
exec "$@"
