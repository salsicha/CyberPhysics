#!/bin/bash
# Load the external Python 3.12 ROS installation before Kit/its ROS bridge.
set -e
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
exec "$@"
