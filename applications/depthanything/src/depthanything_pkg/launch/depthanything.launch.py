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
        # Optional per-frame scale alignment (README.md); empty: off.
        'scale_reference_topic': '',
        'scale_reference_min_m': '0.3',
        'scale_reference_max_m': '4.0',
        'scale_reference_stride': '8',
        'scale_reference_min_samples': '100',
        'scale_reference_max_age_s': '0.05',
        'scale_hold_s': '1.0',
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
