"""Launch the VINS-Mono feature tracker and visual-inertial estimator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    enable_estimator = LaunchConfiguration("enable_estimator")
    enable_pose_graph = LaunchConfiguration("enable_pose_graph")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )

    config_share = Path(get_package_share_directory("config_pkg"))

    common_parameters = {
        "config_file": config_file,
        "use_sim_time": use_sim_time,
    }

    feature_tracker = Node(
        package="feature_tracker",
        executable="feature_tracker",
        namespace="feature_tracker",
        name="feature_tracker",
        output="screen",
        parameters=[
            common_parameters,
            {"vins_folder": f"{config_share}/"},
        ],
    )

    estimator = Node(
        package="vins_estimator",
        executable="vins_estimator",
        namespace="vins_estimator",
        name="vins_estimator",
        output="screen",
        condition=IfCondition(enable_estimator),
        parameters=[common_parameters],
    )

    pose_graph = Node(
        package="pose_graph",
        executable="pose_graph",
        namespace="pose_graph",
        name="pose_graph",
        output="screen",
        condition=IfCondition(enable_pose_graph),
        parameters=[
            common_parameters,
            {
                "support_file": str(config_share / "support_files"),
                "visualization_shift_x": 0,
                "visualization_shift_y": 0,
                "skip_cnt": 0,
                "skip_dis": 0.0,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="/etc/vinsmono/calibration.yaml",
                description="OpenCV YAML containing topics, calibration, and VINS tuning",
            ),
            DeclareLaunchArgument(
                "enable_estimator",
                default_value="true",
                description="Start the visual-inertial estimator",
            ),
            DeclareLaunchArgument(
                "enable_pose_graph",
                default_value="false",
                description="Start optional loop closure and pose graph processing",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock",
            ),
            feature_tracker,
            estimator,
            pose_graph,
        ]
    )
