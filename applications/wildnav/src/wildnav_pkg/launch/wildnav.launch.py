from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        ('image_topic', '/oak1/image_highres'),
        ('camera_info_topic', '/oak1/image_highres/camera_info'),
        ('prediction_odom_topic', '/navigation/odometry'),
        ('raw_odom_topic', '/drone0/sensor_measurements/odom'),
        ('gps_topic', '/drone0/sensor_measurements/gps'),
        ('prior_fix_topic', '/navigation/fix'),
        ('origin_topic', '/navigation/origin'),
        ('prior_timeout_s', '2.0'),
        ('initial_lat', '0.0'),
        ('initial_lon', '0.0'),
        ('cache_dir', '/data/wildnav_cache'),
        ('zoom', '18'),
        ('search_radius_m', '250.0'),
        ('search_radius_sigma_scale', '3.0'),
        ('max_search_radius_m', '750.0'),
        ('max_tiles', '49'),
        ('max_tile_downloads_per_cycle', '8'),
        ('match_rate_hz', '0.2'),
        ('feature_backend', 'SIFT'),
        ('max_features', '1500'),
        ('ratio_threshold', '0.72'),
        ('min_matches', '16'),
        ('min_inliers', '10'),
        ('min_inlier_ratio', '0.25'),
        ('max_reprojection_error_px', '8.0'),
        ('max_position_jump_m', '250.0'),
        ('max_attitude_deg', '20.0'),
        ('camera_mount_yaw_deg', '0.0'),
        ('camera_stabilized', 'false'),
        ('fov_deg', '69.0'),
        ('attitude_sigma_deg', '1.0'),
        ('stamp_sigma_s', '0.05'),
        ('match_sigma_floor_m', '3.0'),
        ('navigation_output_topic', '/navigation/odometry'),
        ('minimum_demnav_confidence', '0.35'),
        ('minimum_wildnav_confidence', '0.20'),
        ('correction_timeout_s', '30.0'),
    ]
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in args
    ]
    declarations.append(DeclareLaunchArgument(
        'tile_url_template',
        default_value=EnvironmentVariable(
            'WILDNAV_TILE_URL',
            default_value=(
                'https://server.arcgisonline.com/ArcGIS/rest/services/'
                'World_Imagery/MapServer/tile/{z}/{y}/{x}'))))
    values = {name: LaunchConfiguration(name) for name, _ in args}
    values['tile_url_template'] = LaunchConfiguration('tile_url_template')

    wildnav = Node(
        package='wildnav_pkg',
        executable='wildnav_node',
        name='wildnav_node',
        output='screen',
        parameters=[{
            'image_topic': values['image_topic'],
            'camera_info_topic': values['camera_info_topic'],
            'odom_topic': values['prediction_odom_topic'],
            'gps_topic': values['gps_topic'],
            'prior_fix_topic': values['prior_fix_topic'],
            'origin_topic': values['origin_topic'],
            'prior_timeout_s': values['prior_timeout_s'],
            'initial_lat': values['initial_lat'],
            'initial_lon': values['initial_lon'],
            'cache_dir': values['cache_dir'],
            'tile_url_template': values['tile_url_template'],
            'zoom': values['zoom'],
            'search_radius_m': values['search_radius_m'],
            'search_radius_sigma_scale':
                values['search_radius_sigma_scale'],
            'max_search_radius_m': values['max_search_radius_m'],
            'max_tiles': values['max_tiles'],
            'max_tile_downloads_per_cycle':
                values['max_tile_downloads_per_cycle'],
            'match_rate_hz': values['match_rate_hz'],
            'feature_backend': values['feature_backend'],
            'max_features': values['max_features'],
            'ratio_threshold': values['ratio_threshold'],
            'min_matches': values['min_matches'],
            'min_inliers': values['min_inliers'],
            'min_inlier_ratio': values['min_inlier_ratio'],
            'max_reprojection_error_px':
                values['max_reprojection_error_px'],
            'max_position_jump_m': values['max_position_jump_m'],
            'max_attitude_deg': values['max_attitude_deg'],
            'camera_mount_yaw_deg': values['camera_mount_yaw_deg'],
            'camera_stabilized': values['camera_stabilized'],
            'fov_deg': values['fov_deg'],
            'attitude_sigma_deg': values['attitude_sigma_deg'],
            'stamp_sigma_s': values['stamp_sigma_s'],
            'match_sigma_floor_m': values['match_sigma_floor_m'],
        }],
    )
    fusion = Node(
        package='wildnav_pkg',
        executable='navigation_fusion_node',
        name='navigation_fusion_node',
        output='screen',
        parameters=[{
            'raw_odom_topic': values['raw_odom_topic'],
            'origin_topic': values['origin_topic'],
            'output_topic': values['navigation_output_topic'],
            'minimum_demnav_confidence':
                values['minimum_demnav_confidence'],
            'minimum_wildnav_confidence':
                values['minimum_wildnav_confidence'],
            'correction_timeout_s': values['correction_timeout_s'],
        }],
    )
    return LaunchDescription(declarations + [wildnav, fusion])
