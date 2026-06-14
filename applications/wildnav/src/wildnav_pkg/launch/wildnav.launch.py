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
        ('initial_lat', '0.0'),
        ('initial_lon', '0.0'),
        ('origin_alt', '0.0'),
        ('cache_dir', '/data/wildnav_cache'),
        ('zoom', '18'),
        ('search_radius_m', '250.0'),
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
        ('demnav_odom_topic', '/demnav/odometry'),
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
            'initial_lat': values['initial_lat'],
            'initial_lon': values['initial_lon'],
            'origin_alt': values['origin_alt'],
            'cache_dir': values['cache_dir'],
            'tile_url_template': values['tile_url_template'],
            'zoom': values['zoom'],
            'search_radius_m': values['search_radius_m'],
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
        }],
    )
    fusion = Node(
        package='wildnav_pkg',
        executable='navigation_fusion_node',
        name='navigation_fusion_node',
        output='screen',
        parameters=[{
            'raw_odom_topic': values['raw_odom_topic'],
            'demnav_odom_topic': values['demnav_odom_topic'],
            'output_topic': values['navigation_output_topic'],
            'minimum_demnav_confidence':
                values['minimum_demnav_confidence'],
            'minimum_wildnav_confidence':
                values['minimum_wildnav_confidence'],
            'correction_timeout_s': values['correction_timeout_s'],
        }],
    )
    return LaunchDescription(declarations + [wildnav, fusion])
