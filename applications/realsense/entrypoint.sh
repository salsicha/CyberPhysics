#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
# ros-jazzy-librealsense2 installs librealsense2.so under lib/<arch>-linux-gnu,
# which the ros2 base image's setup.bash does not add to the library path.
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib/$(uname -m)-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec "$@"
