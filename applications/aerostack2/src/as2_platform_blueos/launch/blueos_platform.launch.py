import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('as2_platform_blueos')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        DeclareLaunchArgument('mavros_namespace', default_value='/mavros'),
        DeclareLaunchArgument('takeoff_altitude', default_value='2.0'),
        Node(
            package='as2_platform_blueos',
            executable='as2_platform_blueos_node',
            name='platform',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                os.path.join(package_dir, 'config', 'platform.yaml'),
                {
                    'control_modes_file': os.path.join(
                        package_dir, 'config', 'control_modes.yaml'),
                    'mavros_namespace': LaunchConfiguration('mavros_namespace'),
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                },
            ],
        ),
    ])
