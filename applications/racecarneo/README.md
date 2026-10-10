# MIT RACECAR Neo Application

This application packages the official MIT RACECAR Neo Linux simulator and Python library, then exposes the simulator through ROS 2.

The image downloads upstream assets at build time, pinned to commits in the
Dockerfile:

- `MITRacecarNeo/RacecarNeo-Simulator`: RacecarSim v2.8.3 (`250f72d`)
- `MITRacecarNeo/racecar-neo-library`: `494d16e`, which speaks the v2.8 UDP protocol

The two move together. RacecarSim v2.9+ uses protocol 2, REP-103 IMU axes and
1080 lidar samples, so upgrading also means updating
`scripts/racecarneo_ros_bridge.py`.

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

This also starts `racecarneo_robot_state_publisher` for the sensor frames.

The Unity simulator runs on its own Xvfb display in `racecarneo_sim`, so it does
not need a host X11 display, and starts a level by itself (see
[Unattended simulator start](#unattended-simulator-start)). The bridge connects
over UDP ports `5064` and `5065`, publishes ROS 2 sensor/state topics, and
subscribes to Ackermann drive commands. Its log shows
`RACECAR Neo simulator connected; publishing ROS topics` about 5 s after
`up`. With GPU rendering every bridge topic (camera, depth, scan, IMU, odom)
publishes at the simulator's frame rate, about 18-22 Hz with the full stack
running.

Published topics:

- `/camera/color/image_raw` (`sensor_msgs/Image`, `bgr8`)
- `/camera/depth/image_rect_raw` (`sensor_msgs/Image`, `32FC1`, meters)
- `/camera/color/camera_info` (`sensor_msgs/CameraInfo`)
- `/scan` (`sensor_msgs/LaserScan`)
- `/imu/data_raw` (`sensor_msgs/Imu`)
- `/odom` (`nav_msgs/Odometry`): the estimated speed (see
  [Speed control](#speed-control)) along the heading integrated from the
  simulator's yaw rate; the twist is that speed and yaw rate
- `/ackermann_feedback` (`ackermann_msgs/AckermannDriveStamped`, m/s and rad):
  estimated speed and the steering angle sent to the simulator
- `/racecarneo/manual_override` (`std_msgs/Bool`)
- `/racecarneo/estop` (`std_msgs/Bool`)
- `/racecarneo/battery_voltage` (`std_msgs/Float32`)
- TF for `odom -> base_link`; the sensor frames (`base_link -> racecarneo/*`)
  come from `racecarneo_robot_state_publisher` and the shared URDF

Subscribed topics:

- `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`): SI units, speed in
  m/s and steering angle in rad, positive left. The speed is the setpoint of
  the bridge's speed controller, clamped to +-`max_speed` (1.0 m/s); the
  steering angle is divided by `max_steering_angle` (the simulator's 20 deg
  lock) and clamped to [-1, 1]. Without commands for `command_timeout_s`
  (0.5 s) the car brakes to a stop.

### Unattended simulator start

RacecarSim v2.8.3 has no command-line option, PlayerPrefs key or network
message that starts a level. It opens on its main menu; a level's Python
ports (UDP `5064`/`5065`) only exist after **Begin Simulation**, and a
connected program then waits until the user presses START (Enter) to enter
user-program mode. The async port accepts only connect, exit, camera and lidar
requests, so the library's `racecar_go` packet (header 8) is ignored. A
`-batchmode` player has no window and reads no input, so it can never get
past the menu.

`scripts/run_headless_sim.sh` therefore plays the user, without patching the
simulator:

1. It starts Xvfb on a free display (99 and up; under `network_mode: host`
   the host's own X displays are skipped) and runs the player windowed at the
   Xvfb screen size.
2. It clicks **Begin Simulation** with `xdotool` once a second, at the
   button's position relative to the window, until the simulator process has
   bound its Python ports. This loads the menu defaults: *Sandbox
   Environments / Demo World* in *Exploration* mode (typically 4-5 s).
3. It then presses Enter every `RACECAR_SIM_START_PERIOD_S` seconds. Before the
   bridge connects this only shows a "connect a Python program" hint; once
   the bridge has connected it enters user-program mode; afterwards it is a
   no-op. If the bridge exits or stalls for 5 s the simulator drops back to
   default drive, and the next press resumes user-program mode when the bridge
   is back.

The script refuses to start when UDP `5064`/`5065` are already bound (one
simulator per host under host networking), stops the simulator if no level
loads within `RACECAR_SIM_LEVEL_TIMEOUT_S` (120 s), and forwards SIGTERM to the
player. When the simulator exits it sends its exit command and the bridge
shuts down too.

Rendering uses Vulkan on the NVIDIA GPU when the container has the NVIDIA
Vulkan driver, which the composition requests (a GPU reservation and
`NVIDIA_DRIVER_CAPABILITIES=graphics,utility`; the simulator needs ~150 MiB of
GPU memory). Without it the script falls back to Mesa's software OpenGL,
which works but runs the simulator, and so every bridge topic, at about 3 Hz.

| Variable | Default | |
| --- | --- | --- |
| `RACECAR_XVFB_SERVER_ARGS` | `-screen 0 1280x720x24` | Xvfb screen; keep it 16:9, the menu click is relative |
| `RACECAR_SIM_GRAPHICS_API` | `auto` | `vulkan`, `opengl` or `auto` (Vulkan if the NVIDIA driver is present) |
| `RACECAR_SIM_LOG_FILE` | `/dev/stdout` | Unity player log |
| `RACECAR_SIM_START_PERIOD_S` | `1` | seconds between START presses |
| `RACECAR_SIM_LEVEL_TIMEOUT_S` | `120` | give up when no level loads |

Only the menu's default level is started. To look at the simulator's window:

```bash
docker exec racecarneo_sim bash -c '. /tmp/racecarsim.env && xwd -root -silent' > sim.xwd
convert sim.xwd sim.png   # ImageMagick on the host
```

Simulator model limits (RacecarSim v2.8.3):

- Steering full lock is 20 degrees (`max_steering_angle: 0.349`).
- The simulated car is smaller than the hardware description: wheelbase
  0.225 m (wheel colliders at +-1.125 Unity units, 1 unit = 0.1 m), lidar
  0.125 m above `base_link`, camera 0.10 m forward and 0.145 m up. The bridge
  (`bridge.yaml`), the simulator's `cmd_vel_to_ackermann` overrides and the
  URDF's `sim_geometry:=true` use these values; its tightest turn is ~0.62 m,
  inside Nav2's 0.75 m minimum turning radius.
- The throttle is motor torque (`Drive.FixedUpdate`), not a speed, and the
  simulator reports no wheel or body speed; the bridge closes the speed loop
  itself (next section). Exactly zero throttle engages the simulator's wheel
  brake; any other value, however small, lets the car coast.
- The camera renders 640x480 with a 42.5 degree vertical field of view;
  `/camera/color/camera_info` publishes the matching intrinsics.
- `/imu/data_raw` is converted from the simulator's Unity axes to ROS FLU,
  with gravity as +9.81 m/s^2 on z at rest (level-ground assumption).

### Speed control

`racecarneo_ros_bridge.py` runs a speed loop every simulator frame
(parameters in `systems/racecarneo/config/bridge.yaml`):

- **Estimate** (`SpeedEstimator`): the simulator's forward acceleration is
  integrated over its frame time. Measured against lidar ranges to the end
  wall of the *Long Hallway Sandbox* level, that IMU reads 0.01-0.05 m/s^2
  high whenever the motor pushes, under-reads hard braking and lags it by
  about two frames, so pure integration drifts by 0.2-0.5 m/s within a few
  maneuvers. The estimate is therefore pulled toward a drive model identified
  from the same lidar data (dv/dt = 1.18 u - (0.85 + 0.15 s^2) v for throttle
  u and steering ratio s; at exactly zero throttle the brake decelerates at
  ~1 m/s^2) with a 1 s time constant while driving and 0.25 s while
  braking, and reset to zero after 0.3 s of zero throttle with a still IMU
  (zero-velocity update). In steady state the speed therefore comes from the
  model: the car runs ~20% slower than estimated at full steering lock and
  the model knows nothing about slopes.
- **Controller** (`SpeedController`): feedforward from the model (0.72
  throttle per m/s) plus PI on the estimate; the integral only runs within
  0.05 m/s of the setpoint and freezes while the throttle saturates. A stop
  command (speed 0, the command watchdog, manual override) brakes with
  reverse throttle 3 x speed while the estimate is above 0.15 m/s, then sends
  exactly zero throttle, the simulator's own brake, which cannot reverse the
  car. E-stop cuts the throttle to zero at once.
- **Odometry**: the estimated speed (with the configured wheel slip noise and
  encoder quantization) along the heading integrated from the simulator's yaw
  rate. The steering model (0.225 m wheelbase, 20 deg lock, Nav2's 0.75 m
  minimum turning radius) is unchanged.

Measured with the real simulator (true speed from the lidar range to a wall
behind the car, least-squares over the second half of the hold; 6 s holds
in the *Long Hallway Sandbox*, selected from the menu by hand):

| Command | True speed | Distance after the stop command |
| --- | --- | --- |
| 0.2 m/s (Long Hallway, 4 runs) | 0.176-0.186 m/s (-12 to -7%) | 0.09-0.10 m |
| 0.5 m/s (Long Hallway, 4 runs) | 0.478-0.501 m/s (-4 to 0%) | 0.22-0.41 m (wall 7-8 m away, +-0.07 m lidar noise) |
| -0.2 m/s (Demo World, reverse, 4 s hold) | -0.180 m/s (-10%) | 0.09 m |
| -0.5 m/s (Demo World, reverse, 2 runs, 2.5 s hold) | -0.470 to -0.474 m/s (-6 to -5%) | 0.23-0.24 m |

The estimate runs 0.01-0.02 m/s above the true speed at 0.2 m/s (the IMU's
bias through the 1 s model time constant), so slow commands come out up to
~12% slow.

Before this loop the same 0.2 m/s command reached ~0.6 m/s after 2 s and
kept accelerating, a zero command coasted ~1 m, and a 3 s left turn showed
69 deg in `/odom` against ~130 deg from the IMU.

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
8. `foxglove_bridge` serves the ROS graph to Foxglove at `ws://localhost:8765`.

NiceGUI is available at `http://localhost:8080`.

The ROS containers do not share `/dev/shm`, so FastDDS runs over UDP only, with 4 MiB socket buffers (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4?sockets_size=4194304`). With the 208 KiB default, best-effort subscribers lost the 640x480 frames: with the simulator, bridge and Depth Anything running they received 74 % of the colour and 23 % of the depth frames (14.7 and 4.5 of 19.8 Hz), and Depth Anything published 3.8 Hz. With the larger buffers they receive every frame and Depth Anything publishes 17.9 Hz; it now keeps its five-frame input queue full, so its depth is about 0.3 s old (median 0.13 s before). The kernel caps the buffers at `net.core.rmem_max` and `net.core.wmem_max` (see "DDS transport" in `applications/aerostack2/README.md`).

Closed-loop status: the stack comes up unattended and Nav2 reaches goals in
Demo World. `NavigateToPose` goals relative to the start pose, each on a
freshly started stack (SLAM, Depth Anything and nvblox running):

| Goal (x, y, yaw) | Runs | Succeeded | Recoveries | Final error (SLAM map) |
| --- | --- | --- | --- | --- |
| 3.0, -1.5, -30 deg | 6 | 6 | 0 | 0.23-0.24 m, 0.7-4.1 deg |
| 1.0, -3.0, -90 deg (right turn) | 3 | 3 | 0 | 0.23 m, 2-10 deg |
| 5.0, -1.5, 0 deg | 3 | 3 | 0 | 0.22-0.23 m, 3-6 deg |

The goal checker's tolerance is 0.25 m / 0.35 rad. Matching the final scan
against the SLAM map put the car within 0.07 m of the pose the stack
reported. `/odom` still over-reports distance by 7-10% against the SLAM map
on these routes, which SLAM corrects. Before the speed loop, the first goal
succeeded once in two runs after five recoveries; the other run stopped
against a wall ~1 m from where the stack believed it was. SLAM only maps
where the lidar hits something, so in the open Demo World a goal outside
the mapped area is rejected as `GOAL_OUTSIDE_MAP` (204).

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
- `urdf/racecarneo.urdf.xacro`: frame model and the only definition of sensor
  extrinsics, published by `launch/racecarneo_description.launch.py` in both
  the simulator and hardware compositions

No maps or simulator assets are committed to the repo.
