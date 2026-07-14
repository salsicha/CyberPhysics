#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
# Default preserves the original hardcoded rig NIC; override per host.
PTP_INTERFACE="${PTP_INTERFACE:-enxf8e43be7e67b}"
if [ -n "${PTP_INTERFACE}" ]; then
    echo "Starting ptp4l on interface ${PTP_INTERFACE}"
    ptp4l -i "${PTP_INTERFACE}" -m -S &
else
    echo "PTP_INTERFACE not set, skipping ptp4l"
fi
export ROS_IP=0.0.0.0
exec "$@"
