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
        ('origin_topic',     '/navigation/origin',     'EKF origin (geographic_msgs/GeoPointStamped); local outputs wait for it'),
        ('prior_fix_topic',  '/navigation/fix',        'Fused position used as the search prior (sensor_msgs/NavSatFix)'),
        ('prior_timeout_s',  '2.0',                    'Maximum age of the prior or GPS fix used as search centre'),
        ('rangefinder_topic', '/aerodrone/rangefinder', 'Downward rangefinder (sensor_msgs/Range); empty disables'),
        ('rangefinder_timeout_s', '0.5',               'Maximum age of a rangefinder reading'),
        ('initial_lat',      '0.0',                    'Height map pre-load latitude (and synthetic-world anchor)'),
        ('initial_lon',      '0.0',                    'Height map pre-load longitude (and synthetic-world anchor)'),
        ('area_km',          '10.0',                   'Area to cache in km (square)'),
        ('search_radius_m',  '200.0',                  'Search radius for terrain matching in metres'),
        ('search_radius_sigma_scale', '3.0',           'Search radius growth per prior horizontal sigma'),
        ('max_search_radius_m', '600.0',               'Upper bound of the widened search radius'),
        ('fov_deg',          '90.0',                   'Horizontal field of view when no camera_info is received'),
        ('camera_mount_yaw_deg', '0.0',                'Image-top heading relative to the nose, counter-clockwise from above'),
        ('camera_stabilized', 'false',                 'Camera is roll/pitch-stabilised (gimbal), not body-fixed'),
        ('depth_is_metric', 'false',                   'depth_topic is metric depth along the optical axis (simulators), not relative depth'),
        ('attitude_sigma_deg', '1.0',                  'Attitude (EKF + camera mount) 1-sigma for the fix covariance'),
        ('heading_sigma_deg', '1.0',                   'Heading (EKF + camera mount) 1-sigma for the fix covariance'),
        ('match_sigma_scale', '2.0',                   'Scale on the correlation peak spread in the fix covariance'),
        ('match_sigma_floor_cells', '0.1',             'Floor of the matching error in the fix covariance, DEM cells'),
        ('stamp_sigma_s', '0.05',                      'Camera stamp 1-sigma for the fix covariance'),
        ('gps_altitude_datum', 'ellipsoid',            'Datum of prior/GPS altitudes: ellipsoid (MAVROS) or msl'),
        ('geoid_file', '/usr/share/GeographicLib/geoids/egm96-5.pgm', 'GeographicLib geoid grid for ellipsoid altitudes'),
        ('dem_resolution_m', '30.0',                   'DEM grid resolution in metres'),
        ('min_correlation',  '0.3',                    'Minimum NCC score to publish a fix'),
        ('min_peak_uniqueness', '0.5',                 'Minimum margin of the match over the best terrain look-alike (0-1)'),
        ('cache_dir',        '/data/demnav_cache',     'Directory for cached DEM tiles'),
        ('synthetic_terrain', 'false',                  'Use simulator terrain instead of SRTM'),
        ('pointcloud_stride', '4',                     'Depth pixels skipped per point'),
        ('min_dem_footprint_pixels', '5',              'Minimum DEM cells across a match'),
        ('min_metric_depth_m', '0.2',                  'Minimum valid metric depth'),
        ('max_metric_depth_m', '500.0',                'Maximum valid metric depth'),
        ('max_attitude_deg', '20.0',                   'Skip matching when |roll| or |pitch| exceeds this (0 disables)'),
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
