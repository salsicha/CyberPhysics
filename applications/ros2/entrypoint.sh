#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /venv/bin/activate
# export ROS_IP=0.0.0.0
exec "$@"
