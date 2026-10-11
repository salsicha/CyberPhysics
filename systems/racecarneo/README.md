# RACECAR Neo System

This folder contains configuration for the MIT RACECAR Neo hardware/simulation setup. The reusable application code lives in `applications/`; this directory holds the files that are specific to this vehicle.

## Contents

- `config/bridge.yaml`: ROS parameters for the official Unity simulator bridge.
- `config/cmd_vel_to_ackermann.yaml`: conversion from Nav2 velocity commands to Ackermann commands in SI units (m/s, rad). Manual driving on `/cmd_vel_teleop` (the web UI's joystick) takes priority while it keeps arriving (0.5 s timeout, then zero speed), a true on `/racecarneo/stop_request` (transient-local) latches zero speed until a false, and `/racecarneo/control_status` reports the source, the stop and the command as JSON (`systems/racecar_turret/interfaces.md`).
- `config/depthanything.yaml`: metric Depth Anything topic/model settings.
- `config/nav2.yaml`: Nav2 planner, controller, behavior, and nvblox costmap-layer settings.
- `config/nvblox.yaml`: nvblox TSDF/ESDF mapper overrides for the RACECAR camera setup.
- `config/slam_toolbox.yaml`: online SLAM Toolbox settings for `map -> odom`.
- `config/topics.yaml`: canonical topics and frames for this system.
- `config/sensor_models.yaml`: hardware-to-simulation sensor, actuator, safety, and noise-model contract.
- `calibration/camera.yaml`: simulator camera intrinsics (replace for hardware).
- `urdf/racecarneo.urdf.xacro`: frame model and the single source of sensor extrinsics for simulator and hardware. Frame names match `config/topics.yaml`.
- `launch/racecarneo_description.launch.py`: runs `robot_state_publisher` on the URDF; pass `camera_optical_frames:=false` when the RealSense driver publishes the optical frames. The RACECAR Turret passes its own URDF (`urdf:=`) and `joint_states_topic:=/turret/joint_states`.
- `scenarios/campus_closed_course.json`: tracked source-of-truth contract for the realistic outdoor/indoor closed-course environment, semantic labels, collision geometry, dynamic actors, spawn poses, and scenario variants.
- `missions/nav2_waypoints.json`: Nav2 waypoint missions for loop, slalom, lane-change, obstacle-avoidance, recovery, and multi-lap validation.
- `validation/sensor_behavior_tests.json`: sensor-driven behavior test manifest for nominal, degraded-camera, lidar-dropout, IMU-bias, and odometry-slip runs.
- `validation/acceptance_thresholds.json`: default Racecar Neo mission acceptance thresholds.

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
/cmd_vel (or /cmd_vel_teleop) -> cmd_vel_to_ackermann -> /ackermann_cmd (m/s, rad) -> simulator bridge
URDF -> robot_state_publisher -> base_link -> racecarneo/* sensor frames
```

The NiceGUI frontend (`applications/nicegui`) shows the SLAM map with the `map -> base_link` pose and the Nav2 path, the camera, the drive status and pipeline freshness; it sends Nav2 waypoints in the `map` frame through `NavigateToPose` (tap the map or type one), cancels them, drives manually with an on-screen joystick on `/cmd_vel_teleop`, and stops the car with `/racecarneo/stop_request`. The closed-course scenario contract in `systems/racecarneo/scenarios/campus_closed_course.json` defines the environment geometry that Unity/Gazebo/Isaac assets should instantiate for validation.

## Asset Policy

The official MIT Unity simulator and Python library are downloaded into the `salsicha/cyberphysics:racecarneo-latest` image during build. Generated maps, bags, logs, and simulator assets should not be committed here.

A more detailed Gazebo or Isaac model can be added here later by extending the URDF with wheel joints, Ackermann steering controllers, sensors, and simulator-specific tags.
