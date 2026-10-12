import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node

# The world's RGB-D sensor (scenario_world.py) publishes on these gz topics;
# they are bridged to the SO-101 camera topics every simulator publishes.
GZ_CAMERA = "/so101/camera"
CAMERA_BRIDGES = (
    (f"{GZ_CAMERA}/image", "sensor_msgs/msg/Image", "gz.msgs.Image", "/so101/camera/image_raw"),
    (f"{GZ_CAMERA}/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo",
     "/so101/camera/camera_info"),
    (f"{GZ_CAMERA}/depth_image", "sensor_msgs/msg/Image", "gz.msgs.Image",
     "/so101/camera/depth/image_rect_raw"),
)
# The depth image is registered to the colour image (one rgbd sensor), so it
# shares the colour camera's intrinsics.
DEPTH_INFO_BRIDGE = (f"{GZ_CAMERA}/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo",
                     "/so101/camera/depth/camera_info")


def _bridge(name, entries):
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=name,
        arguments=[f"{gz}@{ros_type}[{gz_type}" for gz, ros_type, gz_type, _ in entries],
        remappings=[(gz, ros) for gz, _, _, ros in entries],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )


def _spawn_robot(context):
    """Spawn the arm at the scenario's robot_spawn (the scene's one source of
    its height: the underside of the arm's base on its riser)."""
    scenario = json.loads(Path(LaunchConfiguration("scenario_file").perform(context)).read_text())
    spawn = scenario.get("robot_spawn", {})
    x, y, z = (float(value) for value in spawn.get("xyz", [0.0, 0.0, 0.0]))
    roll, pitch, yaw = (float(value) for value in spawn.get("rpy", [0.0, 0.0, 0.0]))
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "so101", "-topic", "robot_description",
                   "-x", str(x), "-y", str(y), "-z", str(z),
                   "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)],
        output="screen",
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    return [
        spawn_node,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_node,
            on_exit=[TimerAction(period=2.0, actions=[joint_state_broadcaster])],
        )),
    ]


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
        if_value=["-s -r --headless-rendering ", world],
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
        # Generated from scenario_file by scenario_world.py.
        DeclareLaunchArgument(
            "world_file",
            default_value="/workspace/systems/so101/worlds/picking_table.sdf",
        ),
        DeclareLaunchArgument(
            "scenario_file",
            default_value="/workspace/systems/so101/scenarios/picking_table.json",
        ),
        gazebo,
        state_publisher,
        clock_bridge,
        _bridge("so101_camera_bridge", CAMERA_BRIDGES),
        _bridge("so101_depth_info_bridge", (DEPTH_INFO_BRIDGE,)),
        OpaqueFunction(function=_spawn_robot),
        TimerAction(period=5.0, actions=[position_controller, relay]),
    ])
