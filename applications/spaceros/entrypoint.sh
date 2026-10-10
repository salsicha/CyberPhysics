#!/bin/bash
# Container entrypoint, also sourced by ~/.bashrc (the Space ROS base image
# appends `source /entrypoint.sh`), so interactive shells get the same
# environment. Hence no `set -e`, and exec only when run as a program.

# Underlays in the order the demo workspace was built against: ROS 2 Jazzy
# (apt: Gazebo, ros2_control, ...), then Space ROS on top
# shellcheck source=/dev/null
if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi

# shellcheck source=/dev/null
if [ -f "${SPACEROS_DIR:-/opt/ros/spaceros}/setup.bash" ]; then
    source "${SPACEROS_DIR:-/opt/ros/spaceros}/setup.bash"
fi

# Space ROS Curiosity rover demo workspace
# shellcheck source=/dev/null
if [ -f "/demos/curiosity_rover/install/setup.bash" ]; then
    source /demos/curiosity_rover/install/setup.bash
fi

# shellcheck source=/dev/null
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

# shellcheck source=/dev/null
if [ -f /venv/bin/activate ]; then
    source /venv/bin/activate
fi

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    exec "$@"
fi
