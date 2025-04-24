#!/bin/bash
set -e
source /venv/bin/activate
source /opt/ros/humble/setup.bash
exec "$@"
