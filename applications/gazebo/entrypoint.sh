#!/bin/bash
set -e
source /opt/ros/iron/setup.bash
source /workspace/install/setup.bash
source /venv/bin/activate
exec "$@"
