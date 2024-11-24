#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /venv/bin/activate
source /workspace/install/setup.bash
exec "$@"
