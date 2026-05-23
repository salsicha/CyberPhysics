from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    encoder = LaunchConfiguration('encoder')

    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Topic for the input RGB image')

    declare_depth_topic_cmd = DeclareLaunchArgument(
        'depth_topic',
        default_value='/depth_anything/depth',
        description='Topic for the output depth map')

    declare_encoder_cmd = DeclareLaunchArgument(
        'encoder',
        default_value='vits',
        description='Encoder model to use: vits, vitb, vitl')

    depth_node = Node(
        package='depthanything_pkg',
        executable='depth_node',
        name='depth_anything_v2_node',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'depth_topic': depth_topic,
            'encoder': encoder
        }]
    )

    return LaunchDescription([
        declare_image_topic_cmd,
        declare_depth_topic_cmd,
        declare_encoder_cmd,
        depth_node
    ])
