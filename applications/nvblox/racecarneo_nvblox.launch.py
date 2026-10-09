from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


# (launch argument, default topic, description). Defaults are the RACECAR Neo
# topics in systems/racecarneo/config/topics.yaml; the hardware composition
# overrides them through RACECAR_*_TOPIC variables.
TOPIC_ARGUMENTS = [
    ('depth_image_topic', '/depth_anything/depth/image', 'Metric depth image'),
    ('depth_camera_info_topic', '/camera/color/camera_info',
     'Camera info for the depth image (Depth Anything output is in the color camera)'),
    ('color_image_topic', '/camera/color/image_raw', 'Color image'),
    ('color_camera_info_topic', '/camera/color/camera_info', 'Color camera info'),
    ('pointcloud_topic', '/depth_anything/points', 'Point cloud (used only with use_lidar)'),
]


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
            ('camera_0/depth/image', LaunchConfiguration('depth_image_topic')),
            ('camera_0/depth/camera_info', LaunchConfiguration('depth_camera_info_topic')),
            ('camera_0/color/image', LaunchConfiguration('color_image_topic')),
            ('camera_0/color/camera_info', LaunchConfiguration('color_camera_info_topic')),
            ('pointcloud', LaunchConfiguration('pointcloud_topic')),
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
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in TOPIC_ARGUMENTS],
        container,
    ])
