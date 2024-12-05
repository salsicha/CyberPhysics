#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
export ROS_IP=0.0.0.0

# _fcu_url:='tcp://:14650'
# rosrun mavros mavros_node _gcs_url:='udp://:14551@localhost:14550' &
# roslaunch mavros px4.launch gcs_url:="udp://:14551@localhost:14550" fcs_url:="tcp://4560"
# roslaunch px4 px4.launch
exec "$@"
