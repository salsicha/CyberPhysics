"""DJI Tello driver without the GUIs of tello-ros2's own launch.py.

Starts the ``tello`` driver node (through tello_driver.py) and the static
``map -> drone`` transform that upstream's launch.py publishes. When the
drone does not answer the SDK ``command`` handshake (3 tries of 7 s, or at
once when there is no route to it) the driver logs that and exits; it is
respawned so the stack keeps retrying until the host joins the Tello's WiFi.

The driver always talks to 192.168.10.1, the Tello's address on its own
WiFi: its ``tello_ip`` parameter has no effect with the pinned fork, which
sets ``Tello.TELLO_IP`` after djitellopy has bound it as a default argument.

Keyboard teleop (``tello_control``) needs a display and runs as a separate
compose service (profile ``teleop``).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tf_base = LaunchConfiguration('tf_base')
    tf_drone = LaunchConfiguration('tf_drone')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tf_base', default_value='map',
            description='Frame of the odometry messages'),
        DeclareLaunchArgument(
            'tf_drone', default_value='drone',
            description='Frame of the camera image and IMU messages'),

        Node(
            executable='/tello-ros2/tello_driver.py',
            name='tello',
            namespace='/',
            output='screen',
            parameters=[{
                'tf_base': tf_base,
                'tf_drone': tf_drone,
            }],
            respawn=True,
            respawn_delay=5.0,
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf',
            namespace='/',
            arguments=['--frame-id', tf_base, '--child-frame-id', tf_drone],
        ),
    ])
