

# Gazebo


## Drone example

cd .. && rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && colcon build --packages-select-regex sjtu*


Drone Topics
Sensors

The folowing sensors are currently implemented:

    ~/front/image_raw [sensor_msgs/msg/Image]
    ~/bottom/image_raw [sensor_msgs/msg/Image]
    ~/sonar/out [sensor_msgs/msg/Range]
    ~/imu/out [sensor_msgs/msg/Imu]
    ~/gps/nav [sensor_msgs/msg/NavSatFix]
    ~/gps/vel [geometry_msgs/msg/TwistStamped]
    ~/joint_states [sensor_msgs/msg/JointState]

Control

The following control topics are currently subscribed to:

    ~/cmd_vel [geometry_msgs/msg/Twist]: Steers the drone
    ~/land [std_msgs/msg/Empty]: Lands the drone
    ~/takeoff [std_msgs/msg/Empty]: Starts the drone
    ~/posctrl [std_msgs/msg/Bool]: Toggling between position control (give drone a pose via cmd_vel) and normal control (only use cmd_vel)
    ~/dronevel_mode [std_msgs/msg/Bool]: Toggeling between velocity and tilt control in normal control mode.
    ~/cmd_mode [std_msgs/msg/Bool]: Publishes the current control mode (position or normal control)
    ~/state [std_msgs/msg/Int8]: Publishes the current state of the drone (0: landed, 1: flying, 2: hovering)
    ~/reset [std_msgs/msg/Empty]: Resets the drone

Ground Truth

The following ground truth topics are currently published:

    ~/gt_acc [geometry_msgs/msg/Twist]: ground truth acceleration
    ~/gt_pose [geometry_msgs/msg/Pose]: ground truth pose
    ~/gt_vel [geometry_msgs/msg/Twist]: ground truth velocity



