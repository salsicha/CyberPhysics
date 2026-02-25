from launch import LaunchDescription
from launch.actions import DeclareLaunchParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    return LaunchDescription([
        DeclareLaunchParameter(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchParameter(
            'input_topic',
            default_value='/camera/depth/color/points',
            description='Input point cloud topic to filter'),

        DeclareLaunchParameter(
            'output_topic',
            default_value='/camera/depth/color/points_filtered',
            description='Output filtered point cloud topic'),

        Node(
            package='robot_self_filter',
            executable='self_filter_node',
            name='so101_self_filter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'cloud_topic': input_topic,
                'aligned_cloud_topic': output_topic,
                'robot_frame': 'base_link',
                'min_distance': 0.02, # Padding around the robot links
                # List of links to filter out (SO-101 Arm)
                'links': ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'gripper_base', 'gripper_jaw']
            }]
        )
    ])