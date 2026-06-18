from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    base_config = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'),
        'config/nvblox/nvblox_base.yaml',
    )
    sim_config = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'),
        'config/nvblox/specializations/nvblox_sim.yaml',
    )

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=[
            ('camera_0/depth/image', '/depth_anything/depth/image'),
            ('camera_0/depth/camera_info', '/camera/color/camera_info'),
            ('camera_0/color/image', '/camera/color/image_raw'),
            ('camera_0/color/camera_info', '/camera/color/camera_info'),
            ('pointcloud', '/depth_anything/points'),
        ],
        parameters=[
            base_config,
            sim_config,
            config_file,
        ],
    )

    container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[nvblox_node],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='/workspace/config/nvblox.yaml',
            description='RACECAR Neo nvblox parameter override file',
        ),
        container,
    ])
