from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = Path(get_package_share_directory("so101_description"))
    xacro_file = package_dir / "urdf" / "so101.urdf.xacro"
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = Command(["xacro ", str(xacro_file), " use_gazebo:=false"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
    ])
