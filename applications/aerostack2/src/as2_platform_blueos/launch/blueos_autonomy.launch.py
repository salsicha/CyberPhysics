import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def package_launch(package, filename):
    return PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', filename))


def generate_launch_description():
    platform_dir = get_package_share_directory('as2_platform_blueos')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        DeclareLaunchArgument('mavros_namespace', default_value='/mavros'),
        DeclareLaunchArgument('mapnav_odom_topic', default_value='/mapnav/odometry'),
        DeclareLaunchArgument('takeoff_altitude', default_value='2.0'),
        Node(
            package='as2_platform_blueos',
            executable='as2_platform_blueos_node',
            name='platform',
            namespace=namespace,
            output='screen',
            parameters=[
                os.path.join(platform_dir, 'config', 'platform.yaml'),
                {
                    'control_modes_file': os.path.join(
                        platform_dir, 'config', 'control_modes.yaml'),
                    'mavros_namespace': LaunchConfiguration('mavros_namespace'),
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                },
            ],
        ),
        Node(
            package='as2_state_estimator',
            executable='as2_state_estimator_node',
            name='state_estimator',
            namespace=namespace,
            output='screen',
            parameters=[{
                'plugin_name': 'raw_odometry',
                'base_frame': 'base_link',
                'global_ref_frame': 'earth',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'odom_topic': LaunchConfiguration('mapnav_odom_topic'),
                'use_gps': False,
                'set_map_to_odom': True,
                'earth_to_map.x': 0.0,
                'earth_to_map.y': 0.0,
                'earth_to_map.z': 0.0,
            }],
        ),
        IncludeLaunchDescription(
            package_launch(
                'as2_motion_controller', 'pid_speed_controller.launch.py'),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': 'false',
                'plugin_name': 'pid_speed_controller',
            }.items(),
        ),
        IncludeLaunchDescription(
            package_launch(
                'as2_behaviors_motion', 'motion_behaviors_launch.py'),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': 'false',
                'follow_path_plugin_name':
                    'follow_path_plugin_position',
                'go_to_plugin_name': 'go_to_plugin_position',
                'land_plugin_name': 'land_plugin_speed',
                'takeoff_plugin_name': 'takeoff_plugin_speed',
            }.items(),
        ),
    ])
