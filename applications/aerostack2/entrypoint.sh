#!/bin/bash
set -e
source /venv/bin/activate
source /opt/ros/jazzy/setup.bash
exec "$@"
