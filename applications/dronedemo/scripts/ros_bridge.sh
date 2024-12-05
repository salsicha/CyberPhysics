#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.sh

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=0.0.0.0

ros2 run ros1_bridge dynamic_bridge
