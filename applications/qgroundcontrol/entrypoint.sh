#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /venv/bin/activate

# MAVLink <-> ROS 2 bridge for a GCS link, e.g.:
# ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@ -p gcs_url:=udp://@localhost:14550 &
exec "$@"
