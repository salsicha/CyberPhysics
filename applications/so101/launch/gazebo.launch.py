from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros_gz_dir = Path(get_package_share_directory("ros_gz_sim"))
    xacro_file = LaunchConfiguration("xacro_file")
    controllers = LaunchConfiguration("controllers_file")
    world = LaunchConfiguration("world_file")

    robot_description = Command([
        "xacro ", xacro_file,
        " use_gazebo:=true",
        " controllers_file:=", controllers,
    ])
    gz_args = IfElseSubstitution(
        LaunchConfiguration("headless"),
        if_value=["-s -r ", world],
        else_value=["-r ", world],
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_dir / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "so101", "-topic", "robot_description", "-z", "0.001"],
        output="screen",
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["so101_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    relay = Node(
        package="so101_description",
        executable="gazebo_command_relay.py",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument(
            "xacro_file",
            default_value="/workspace/systems/so101/urdf/so101.urdf.xacro",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="/workspace/systems/so101/config/controllers.yaml",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="/workspace/systems/so101/worlds/picking_table.sdf",
        ),
        gazebo,
        state_publisher,
        clock_bridge,
        spawn,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=[TimerAction(period=2.0, actions=[joint_state_broadcaster])],
        )),
        TimerAction(period=5.0, actions=[position_controller, relay]),
    ])
