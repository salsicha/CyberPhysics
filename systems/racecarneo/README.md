# RACECAR Neo System

This folder contains configuration for the MIT RACECAR Neo hardware/simulation setup. The reusable application code lives in `applications/`; this directory holds the files that are specific to this vehicle.

## Contents

- `config/bridge.yaml`: ROS parameters for the official Unity simulator bridge.
- `config/cmd_vel_to_ackermann.yaml`: conversion from Nav2 velocity commands to normalized Ackermann commands.
- `config/depthanything.yaml`: metric Depth Anything topic/model settings.
- `config/nav2.yaml`: Nav2 planner, controller, behavior, and nvblox costmap-layer settings.
- `config/nvblox.yaml`: nvblox TSDF/ESDF mapper overrides for the RACECAR camera setup.
- `config/slam_toolbox.yaml`: online SLAM Toolbox settings for `map -> odom`.
- `config/topics.yaml`: canonical topics and frames for this system.
- `calibration/camera.yaml`: simulator camera calibration placeholder.
- `urdf/racecarneo.urdf.xacro`: lightweight frame model for ROS visualization.
- `scenarios/campus_closed_course.json`: tracked source-of-truth contract for the realistic outdoor/indoor closed-course environment, semantic labels, collision geometry, dynamic actors, spawn poses, and scenario variants.
- `missions/nav2_waypoints.json`: Nav2 waypoint missions for loop, slalom, lane-change, obstacle-avoidance, recovery, and multi-lap validation.

## Runtime Pipeline

The full composition is `compositions/racecarneo.yaml`:

```bash
docker compose -f compositions/racecarneo.yaml up
```

Data flow:

```text
Unity simulator -> racecarneo_bridge -> /camera/color/image_raw
/camera/color/image_raw -> Depth Anything -> /depth_anything/depth/image
/depth_anything/depth/image + /camera/color/camera_info -> nvblox -> map slices
nvblox map slices + SLAM map frame -> Nav2 -> /cmd_vel
/cmd_vel -> cmd_vel_to_ackermann -> /ackermann_cmd -> simulator
```

The NiceGUI frontend observes odometry, camera, and depth status and sends Nav2 waypoints through `NavigateToPose`. The closed-course scenario contract in `systems/racecarneo/scenarios/campus_closed_course.json` defines the environment geometry that Unity/Gazebo/Isaac assets should instantiate for validation.

## Asset Policy

The official MIT Unity simulator and Python library are downloaded into the `cyberphysics/racecarneo` image during build. Generated maps, bags, logs, and simulator assets should not be committed here.

A more detailed Gazebo or Isaac model can be added here later by extending the URDF with wheel joints, Ackermann steering controllers, sensors, and simulator-specific tags.
