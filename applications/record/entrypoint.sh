#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /env.list
export LD_LIBRARY_PATH="/opt/ros/humble/lib"

ptp4l -i enxf8e43be7e67b -m -S > /dev/null 2>&1 &

exec "$@"
