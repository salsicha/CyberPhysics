"""AS2 Gazebo bridges with simulator truth isolated from autonomy TF."""

from __future__ import annotations

import json

from as2_gazebo_assets.utils.launch_exception import InvalidSimulationConfigFile
from as2_gazebo_assets.world import World
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def drone_bridges(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    config_file = LaunchConfiguration("simulation_config_file").perform(context)
    with open(config_file, "r", encoding="utf-8") as stream:
        if config_file.endswith(".json"):
            config = json.load(stream)
        elif config_file.endswith((".yaml", ".yml")):
            config = yaml.safe_load(stream)
        else:
            raise InvalidSimulationConfigFile("Invalid configuration file extension.")
    world = World(**config)

    for drone in world.drones:
        if drone.model_name != namespace:
            continue
        bridges, custom_bridges = drone.bridges(world.world_name)
        remappings = []
        for bridge in bridges:
            source, destination = bridge.remapping()
            if destination == "/tf":
                destination = "/simulation/tf"
            remappings.append((source, destination))

        nodes = [Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=drone.model_name,
            output="screen",
            arguments=[bridge.argument() for bridge in bridges],
            remappings=remappings,
        )]
        nodes.append(Node(
            package="as2_gazebo_assets",
            executable="ground_truth_bridge",
            namespace=drone.model_name,
            output="screen",
            parameters=[{
                "name_space": drone.model_name,
                "pose_frame_id": "earth",
                "twist_frame_id": drone.model_name + "/base_link",
            }],
            remappings=[("/tf", "/simulation/tf")],
        ))
        # Payload custom bridges are static optical transforms.  The first
        # generated custom node is the ground-truth bridge replaced above.
        nodes.extend(custom_bridges[1:])
        return nodes

    return [
        LogInfo(msg="Gazebo bridge creation failed."),
        LogInfo(msg=f"Drone ID: {namespace} not found in {config_file}."),
        Shutdown(reason="Aborting"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("simulation_config_file"),
        DeclareLaunchArgument("namespace"),
        OpaqueFunction(function=drone_bridges),
    ])
