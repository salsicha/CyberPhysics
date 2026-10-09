#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
# System env list (KEY=VALUE lines), mounted by the compose file at /env.
ENV_LIST="${ENV_LIST:-/env/env.list}"
if [ -f "${ENV_LIST}" ]; then
    set -a
    source "${ENV_LIST}"
    set +a
fi
export ENV_LIST
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib"

# Default preserves the original hardcoded rig NIC; override per host, or set
# PTP_INTERFACE= to disable. Started once per container: the marker keeps any
# shell that sources this file again from spawning another ptp4l.
PTP_INTERFACE="${PTP_INTERFACE-enxf8e43be7e67b}"
if [ -n "${PTP_INTERFACE}" ] && [ -z "${RECORD_PTP4L_STARTED:-}" ]; then
    echo "Starting ptp4l on interface ${PTP_INTERFACE}"
    ptp4l -i "${PTP_INTERFACE}" -m -S < /dev/null &
    export RECORD_PTP4L_STARTED=1
elif [ -z "${PTP_INTERFACE}" ]; then
    echo "PTP_INTERFACE not set, skipping ptp4l"
fi

exec "$@"
