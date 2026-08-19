#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /workspace/install/setup.bash

exec "$@"
