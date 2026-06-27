# MIT RACECAR Neo Application

This application packages the official MIT RACECAR Neo Linux simulator and Python library, then exposes the simulator through ROS 2.

The image downloads upstream assets at build time:

- `MITRacecarNeo/RacecarNeo-Simulator`, `linux` branch
- `MITRacecarNeo/racecar-neo-library`, `main` branch

Simulator assets are not stored in this repository.

## Build

```bash
make -C applications build_racecarneo
```

## Headless Simulator Bridge

Run only the simulator and ROS bridge:

```bash
docker compose -f compositions/racecarneo.yaml up racecarneo_sim racecarneo_bridge
```

The Unity simulator runs under `xvfb-run` in `racecarneo_sim`, so it does not need a host X11 display. The bridge connects over UDP ports `5064` and `5065`, publishes ROS 2 sensor/state topics, and subscribes to Ackermann drive commands.

Published topics:

- `/camera/color/image_raw` (`sensor_msgs/Image`, `bgr8`)
- `/camera/depth/image_rect_raw` (`sensor_msgs/Image`, `32FC1`, meters)
- `/camera/color/camera_info` (`sensor_msgs/CameraInfo`)
- `/scan` (`sensor_msgs/LaserScan`)
- `/imu/data_raw` (`sensor_msgs/Imu`)
- `/odom` (`nav_msgs/Odometry`)
- `/ackermann_feedback` (`ackermann_msgs/AckermannDriveStamped`)
- `/racecarneo/manual_override` (`std_msgs/Bool`)
- `/racecarneo/estop` (`std_msgs/Bool`)
- `/racecarneo/battery_voltage` (`std_msgs/Float32`)
- TF for `odom -> base_link` and simulated sensor frames

Subscribed topics:

- `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)

The bridge sends the MIT `racecar_go` autostart packet during startup. If a simulator build still waits for an in-window click, the bridge will publish as soon as the simulator emits update frames.

## Full Navigation Stack

Run the full RACECAR Neo stack:

```bash
docker compose -f compositions/racecarneo.yaml up
```

The composition runs this pipeline:

1. `racecarneo_bridge` publishes camera, lidar, IMU, odometry, and TF.
2. `racecarneo_depthanything` estimates metric monocular depth from `/camera/color/image_raw`.
3. `racecarneo_nvblox` consumes the estimated depth image and camera info, then publishes TSDF/ESDF map slices.
4. `racecarneo_slam` runs SLAM Toolbox to provide `map -> odom`.
5. `racecarneo_nav2` plans and controls with Nav2 using nvblox costmap layers plus lidar fallback.
6. `racecarneo_cmd_vel_to_ackermann` converts Nav2 `/cmd_vel` into `/ackermann_cmd`.
7. `racecarneo_nicegui` serves a browser UI for observation and waypoint commands.

NiceGUI is available at `http://localhost:8080`.

## Stack Validation

Run the full stack and validate sensor topic freshness, nvblox map-slice
presence, and TF connectivity from `map` through `odom`, `base_link`, camera,
lidar, and IMU frames:

```bash
docker compose --profile validation -f compositions/racecarneo.yaml up racecarneo_stack_validator
```

This validator exercises the RealSense-equivalent camera/depth topics, Depth
Anything metric depth, nvblox output topics, SLAM/Nav2 frames, and the simulated
Ackermann feedback path used by hardware.

## Waypoint Missions

Run the full stack and send a tracked mission through Nav2's
`NavigateThroughPoses` action:

```bash
RACECAR_MISSION_ID=simple_loop \
  docker compose --profile missions -f compositions/racecarneo.yaml up racecarneo_mission_runner
```

Mission definitions live in
`systems/racecarneo/missions/nav2_waypoints.json` and include route frame, pose,
tolerance, speed limit, expected route length, and success criteria. Supported
missions are `simple_loop`, `cone_slalom`, `lane_change_obstacle_avoidance`,
`reverse_recovery`, and `multi_lap_endurance`.

## Sensor-Driven Behavior Tests

Score mission telemetry against the sensor-behavior test manifest:

```bash
python3 applications/racecarneo/scripts/score_sensor_behavior.py \
  --manifest systems/racecarneo/validation/sensor_behavior_tests.json \
  --telemetry /tmp/racecarneo_behavior_run.json \
  --test-id nominal_sensor_navigation
```

Telemetry should report planner input topics, localization/map-consumption flags,
collision and recovery counts, and per-sample lateral error, localization drift,
costmap age, obstacle clearance, depth age, and speed. The scorer fails if
runtime planner inputs include simulator ground truth, if localization or map
consumption is missing, if recovery does not complete, or if stale/degraded depth
permits unsafe throttle.

## Acceptance Report

Aggregate scored mission runs against Racecar Neo acceptance thresholds:

```bash
python3 applications/racecarneo/scripts/racecarneo_acceptance_report.py \
  --metrics /tmp/racecarneo_metrics/*.json \
  --thresholds systems/racecarneo/validation/acceptance_thresholds.json
```

The report enforces at least 95 percent waypoint completion, zero nominal
collisions, bounded recovery contact speed, lateral tracking error, stop-line
error, localization drift, costmap freshness, command saturation, and actuator
lag thresholds.

## Configuration

Robot-specific config lives in `systems/racecarneo/`:

- `config/bridge.yaml`: simulator bridge parameters
- `config/depthanything.yaml`: metric depth parameters
- `config/nvblox.yaml`: nvblox mapper overrides
- `config/nav2.yaml`: Nav2 parameters and nvblox costmap layers
- `config/slam_toolbox.yaml`: online SLAM parameters
- `config/topics.yaml`: topic and frame conventions
- `config/sensor_models.yaml`: hardware-equivalent sensor, actuator, safety, and noise model settings
- `urdf/racecarneo.urdf.xacro`: lightweight visualization model

No maps or simulator assets are committed to the repo.
