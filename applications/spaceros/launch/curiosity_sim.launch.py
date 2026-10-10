"""Space ROS Curiosity rover demo: Gazebo, ros2_control and the demo nodes.

Reproduces curiosity_gazebo/launch/curiosity_gazebo.launch.py from
space-ros/demos (the tag pinned in applications/spaceros/Dockerfile), which
upstream runs in its curiosity_gui container, together with
curiosity_rover_demo/launch/mars_rover.launch.py (its curiosity_demo
container). Added arguments:

  gui:=false        Gazebo server only, rendering sensors through EGL
                    (no X server needed)
  demo:=false       do not start the curiosity_rover_demo nodes
  gz_verbosity:=N   gz sim -v level (upstream uses 4)
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

CONTROLLERS = (
    "arm_joint_trajectory_controller",
    "mast_joint_trajectory_controller",
    "wheel_velocity_controller",
    "steer_position_controller",
    "wheel_tree_position_controller",
)


def _truthy(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _gazebo(context):
    gazebo_share = get_package_share_directory("curiosity_gazebo")
    world = os.path.join(gazebo_share, "worlds", "mars_curiosity.world")
    gz_args = f"{world} -r -v {LaunchConfiguration('gz_verbosity').perform(context)}"
    if not _truthy(LaunchConfiguration("gui").perform(context)):
        gz_args += " -s --headless-rendering"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ),
            launch_arguments=[
                ("gz_args", gz_args),
                # A crashed simulator stops the launch (and the container)
                ("on_exit_shutdown", "true"),
            ],
        )
    ]


def generate_launch_description():
    gazebo_share = get_package_share_directory("curiosity_gazebo")
    description_share = get_package_share_directory("curiosity_description")
    demo_share = get_package_share_directory("curiosity_rover_demo")

    env_gz_plugin = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        os.pathsep.join(
            [
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                os.environ.get("LD_LIBRARY_PATH", ""),
            ]
        ),
    )
    env_gz_resource = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join(
            [
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                os.path.join(gazebo_share, "models"),
                description_share,
            ]
        ),
    )

    urdf = os.path.join(
        description_share, "models", "urdf", "curiosity_mars_rover.xacro"
    )
    robot_description = {"robot_description": xacro.process_file(urdf).toxml()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "curiosity_mars_rover",
            "-topic", "robot_description",
            "-z", "-7.5",
        ],
        output="screen",
    )
    odom_tf = Node(
        package="curiosity_gazebo",
        executable="odom_tf_publisher",
        name="odom_tf_publisher",
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/curiosity_mars_rover/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
        output="screen",
    )
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/image_raw", "/image_raw"],
        output="screen",
    )

    # Upstream chains `ros2 control load_controller --set-state active` calls
    # behind a set_hardware_component_state workaround. ros2_control in Jazzy
    # activates the GazeboSystem hardware itself, and the spawner's switch
    # timeout tolerates a simulator that runs well below real time (the
    # load_controller default of 5 s does not)
    def spawner(*controllers):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                *controllers,
                "--controller-manager-timeout", "120",
                "--switch-timeout", "60",
                "--service-call-timeout", "90",
            ],
            output="screen",
        )

    joint_state_broadcaster = spawner("joint_state_broadcaster")

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(demo_share, "launch", "mars_rover.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("demo")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true",
                description="Start the Gazebo GUI (false: server only)",
            ),
            DeclareLaunchArgument(
                "demo", default_value="true",
                description="Start the curiosity_rover_demo nodes",
            ),
            DeclareLaunchArgument(
                "gz_verbosity", default_value="3",
                description="gz sim verbosity (0-4)",
            ),
            env_gz_plugin,
            env_gz_resource,
            SetParameter(name="use_sim_time", value=True),
            OpaqueFunction(function=_gazebo),
            robot_state_publisher,
            spawn,
            odom_tf,
            bridge,
            image_bridge,
            RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[spawner(*CONTROLLERS)],
                )
            ),
            demo,
        ]
    )
