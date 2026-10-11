"""Publish the RACECAR Neo sensor frames from the URDF.

Both the simulator and the hardware compositions run this, so
urdf/racecarneo.urdf.xacro is the only place sensor extrinsics are defined.
Set camera_optical_frames:=false when a camera driver (the RealSense node)
publishes racecarneo/camera_link -> optical frames itself, and
sim_geometry:=true for the (smaller) RacecarSim car's sensor mounts. The
RACECAR Turret passes its own URDF (urdf:=) and reads the turret joints
from joint_states_topic:=/turret/joint_states.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_urdf = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '..', 'urdf', 'racecarneo.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf'),
                 ' camera_optical_frames:=', LaunchConfiguration('camera_optical_frames'),
                 ' sim_geometry:=', LaunchConfiguration('sim_geometry')]),
        value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value=default_urdf),
        DeclareLaunchArgument(
            'camera_optical_frames', default_value='true',
            description='Publish camera_link -> optical frames (false when the camera driver does)'),
        DeclareLaunchArgument(
            'sim_geometry', default_value='false',
            description='Use the RacecarSim car sensor mounts instead of the hardware ones'),
        DeclareLaunchArgument(
            'joint_states_topic', default_value='joint_states',
            description='sensor_msgs/JointState of the movable joints, if any'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))],
            output='screen',
        ),
    ])
