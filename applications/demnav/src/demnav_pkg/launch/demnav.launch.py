from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        ('gps_topic',        '/fix',                   'Input GPS topic (sensor_msgs/NavSatFix)'),
        ('depth_topic',      '/oak1/relative_depth',   'Input relative depth image topic'),
        ('camera_info_topic', '/oak1/camera_info',     'Input camera calibration topic'),
        ('odom_topic',       '/odom',                  'Input odometry topic (nav_msgs/Odometry)'),
        ('output_topic',     '/demnav/fix',             'Output corrected position topic'),
        ('odometry_output_topic', '/demnav/odometry',  'Output metric odometry topic'),
        ('pose_output_topic', '/demnav/pose',           'Output metric pose topic'),
        ('metric_depth_topic', '/demnav/metric_depth',  'Output metric depth image'),
        ('pointcloud_topic', '/demnav/points',          'Output metric point cloud'),
        ('confidence_topic', '/demnav/confidence',      'Terrain match confidence'),
        ('scale_topic', '/demnav/depth_scale',          'Relative-to-metric depth scale'),
        ('valid_topic', '/demnav/valid',                'Whether the latest match is valid'),
        ('initial_lat',      '0.0',                    'Initial latitude for height map download'),
        ('initial_lon',      '0.0',                    'Initial longitude for height map download'),
        ('origin_alt',       '0.0',                    'Altitude used as local map zero'),
        ('area_km',          '10.0',                   'Area to cache in km (square)'),
        ('search_radius_m',  '200.0',                  'Search radius for terrain matching in metres'),
        ('fov_deg',          '90.0',                   'Camera downward field of view in degrees'),
        ('dem_resolution_m', '30.0',                   'DEM grid resolution in metres'),
        ('min_correlation',  '0.3',                    'Minimum NCC score to publish a fix'),
        ('cache_dir',        '/data/demnav_cache',     'Directory for cached DEM tiles'),
        ('synthetic_terrain', 'false',                  'Use simulator terrain instead of SRTM'),
        ('pointcloud_stride', '4',                     'Depth pixels skipped per point'),
        ('min_dem_footprint_pixels', '5',              'Minimum DEM cells across a match'),
        ('min_metric_depth_m', '0.2',                  'Minimum valid metric depth'),
        ('max_metric_depth_m', '500.0',                'Maximum valid metric depth'),
    ]

    declare_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in args
    ]

    node = Node(
        package='demnav_pkg',
        executable='demnav_node',
        name='demnav_node',
        output='screen',
        parameters=[{name: LaunchConfiguration(name) for name, _, _ in args}],
    )

    return LaunchDescription(declare_args + [node])
