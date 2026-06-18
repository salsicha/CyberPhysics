#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /venv/bin/activate
export PYTHONPATH="/venv/lib/python3.10/site-packages:${PYTHONPATH}"
source /workspace/install/setup.bash
exec "$@"
