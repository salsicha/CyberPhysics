from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        'image_topic': '/camera/color/image_raw',
        'camera_info_topic': '/camera/color/camera_info',
        'depth_topic': '/depth_anything/depth/image',
        'pointcloud_topic': '/depth_anything/points',
        'encoder': 'vits',
        'metric_dataset': 'vkitti',
        'max_depth': '80.0',
        'min_depth': '0.05',
        'publish_pointcloud': 'true',
        'pointcloud_stride': '4',
    }

    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in args.items()
    ]

    depth_node = Node(
        package='depthanything_pkg',
        executable='depth_node',
        name='depth_anything_v2_node',
        output='screen',
        parameters=[{name: LaunchConfiguration(name) for name in args}],
    )

    return LaunchDescription([*declarations, depth_node])
