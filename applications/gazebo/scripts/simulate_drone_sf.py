#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Define the world and model URLs from OpenRobotics Fuel
    # San Francisco world
    world_url = 'https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/San Francisco'
    # X3 UAV Drone model
    drone_url = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV'

    return LaunchDescription([
        # 1. Launch Gazebo Sim with the San Francisco world
        # -r runs the simulation immediately on start
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_url],
            output='screen'
        ),

        # 2. Spawn the Drone
        # Spawn at z=10.0 to ensure it starts above the ground/buildings in SF
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'drone',
                '-url', drone_url,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '10.0'
            ],
            output='screen'
        ),

        # 3. ROS 2 <-> Gazebo Bridge
        # Bridge essential topics for drone communication
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Clock (Gazebo -> ROS)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Command Velocity (ROS -> Gazebo)
                '/X3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                # Odometry (Gazebo -> ROS)
                '/X3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # IMU (Gazebo -> ROS)
                '/X3/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                # Magnetometer (Gazebo -> ROS)
                '/X3/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',
                # Motor Speed Control (ROS -> Gazebo) - For direct actuator control
                '/X3/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators'
            ],
            output='screen'
        )
    ])