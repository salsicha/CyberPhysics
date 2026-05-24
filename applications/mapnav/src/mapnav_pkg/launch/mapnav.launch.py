from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        ('gps_topic',        '/fix',                   'Input GPS topic (sensor_msgs/NavSatFix)'),
        ('depth_topic',      '/depth_anything/depth',  'Input depth image topic (sensor_msgs/Image)'),
        ('odom_topic',       '/odom',                  'Input odometry topic (nav_msgs/Odometry)'),
        ('output_topic',     '/mapnav/fix',             'Output corrected position topic'),
        ('initial_lat',      '0.0',                    'Initial latitude for height map download'),
        ('initial_lon',      '0.0',                    'Initial longitude for height map download'),
        ('area_km',          '10.0',                   'Area to cache in km (square)'),
        ('search_radius_m',  '200.0',                  'Search radius for terrain matching in metres'),
        ('fov_deg',          '90.0',                   'Camera downward field of view in degrees'),
        ('dem_resolution_m', '30.0',                   'DEM grid resolution in metres'),
        ('min_correlation',  '0.3',                    'Minimum NCC score to publish a fix'),
        ('cache_dir',        '/data/mapnav_cache',     'Directory for cached DEM tiles'),
    ]

    declare_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in args
    ]

    node = Node(
        package='mapnav_pkg',
        executable='mapnav_node',
        name='mapnav_node',
        output='screen',
        parameters=[{name: LaunchConfiguration(name) for name, _, _ in args}],
    )

    return LaunchDescription(declare_args + [node])
