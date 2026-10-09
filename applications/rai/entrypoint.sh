#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
# RAI's own environment script (activates /rai/.venv, sources the colcon
# workspace and extends PYTHONPATH); it uses paths relative to /rai
pushd /rai > /dev/null
SHELL=/bin/bash source ./setup_shell.sh > /dev/null
popd > /dev/null
exec "$@"
