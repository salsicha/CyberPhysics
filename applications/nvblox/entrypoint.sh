#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /venv/bin/activate
# source /workspace/install/setup.bash
exec "$@"
