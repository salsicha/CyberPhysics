#!/bin/bash
set -e
# ROS 2 for the client and rosbridge; the server launcher strips it again
# before starting Unreal Engine (rclUE ships its own ROS 2 libraries)
source /opt/ros/jazzy/setup.bash
source /venv/bin/activate
exec "$@"
