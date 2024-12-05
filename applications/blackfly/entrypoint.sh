#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
ptp4l -i enxf8e43be7e67b -m -S > /dev/null 2>&1 &
export ROS_IP=0.0.0.0
exec "$@"
