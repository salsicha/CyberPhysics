#!/bin/bash
set -e

if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi

if [ -f "/demos/curiosity_rover/install/setup.bash" ]; then
    source /demos/curiosity_rover/install/setup.bash
fi

if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

if [ -f /venv/bin/activate ]; then
    source /venv/bin/activate
fi

exec "$@"
