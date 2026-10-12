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
  0.125 m above `base_link`, camera 0.10 m forward and 0.145 m up in the
  prefab; the rendered depth puts the camera 0.120 m above the floor,
  pitched 0.81 deg up, which the URDF uses ("Depth for nvblox"). The bridge
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
2. `racecarneo_depthanything` estimates metric monocular depth from `/camera/color/image_raw` and publishes it as on the car; nothing in this composition maps it (below).
3. `racecarneo_nvblox` integrates the bridge's depth image (`/camera/depth/image_rect_raw`, registered to the colour camera) with `/camera/color/camera_info`, then publishes TSDF/ESDF map slices. `RACECAR_NVBLOX_DEPTH_TOPIC=/depth_anything/depth/image` maps Depth Anything's depth instead.
4. `racecarneo_slam` runs SLAM Toolbox to provide `map -> odom`.
5. `racecarneo_nav2` plans and controls with Nav2 using nvblox costmap layers plus lidar fallback.
6. `racecarneo_cmd_vel_to_ackermann` converts Nav2 `/cmd_vel` into `/ackermann_cmd`; manual driving on `/cmd_vel_teleop` takes priority while it keeps arriving, and `/racecarneo/stop_request` latches a stop (`systems/racecarneo/config/cmd_vel_to_ackermann.yaml`).
7. `racecarneo_nicegui` serves a browser UI: the map with goals by tap, a drive joystick, Stop/Resume and pipeline status (`applications/nicegui/README.md`).
8. `foxglove_bridge` serves the ROS graph to Foxglove at `ws://localhost:8765`.

NiceGUI is available at `http://localhost:8080`.

The ROS containers do not share `/dev/shm`, so FastDDS runs over UDP only, with 4 MiB socket buffers (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4?sockets_size=4194304`). With the 208 KiB default, best-effort subscribers lost the 640x480 frames: with the simulator, bridge and Depth Anything running they received 74 % of the colour and 23 % of the depth frames (14.7 and 4.5 of 19.8 Hz), and Depth Anything published 3.8 Hz. With the larger buffers they receive every frame and Depth Anything keeps up with the camera (15-18 Hz). It subscribes with a one-frame queue, so it always infers on the newest frame: with the sensor-data queue of five its depth was a median 0.31 s old, now 0.11 s. The kernel caps the buffers at `net.core.rmem_max` and `net.core.wmem_max` (see "DDS transport" in `applications/aerostack2/README.md`).

### Depth for nvblox

Until 2026-10-11 nvblox integrated Depth Anything's depth (vkitti, max_depth
80), as the hardware composition still does. From the simulated car's
camera, 0.12 m above the floor, that depth reads a median 13 times
the bridge's depth over 0.5-5 m (15 times under 1 m; 195 frames while
circling Demo World, `applications/depthanything/README.md`, "Accuracy"):
anything farther than about 0.9 m lands beyond nvblox's 12 m integration
distance (in the Isaac arena the same error left nvblox's map without a
single obstacle and the obstacles carved out as free space,
`applications/isaac/RACECAR_TURRET.md`, "Depth for nvblox"). The indoor
hypersim model reads 2.9 times; aligned to the bridge's depth every
frame, either model still puts obstacles 3.5-5 m away at about half their
distance. The simulator's own depth has none of this,
so nvblox maps it, as a RealSense's depth would be mapped on the car.

nvblox counts everything 0.02-0.55 m above the floor as an obstacle, so it
needs the camera's mount to about a centimetre. The prefab's level
0.145 m (the URDF's earlier `sim_geometry` mount) put the floor 0.3-0.6 m
ahead 19 mm high and 44 % of it in that band: with the bridge's depth
nvblox filled the space in front of the car, and the goals (3.0, -1.5)
and (5.0, -1.5) aborted ("no valid path found"). The simulator's depth fits a flat floor
0.120 m below the camera with the camera pitched 0.81 degrees up (floor
residual under 2.4 mm from 0.3 to 4 m, 0-3 % in the band), which the URDF
now uses. With it, on 2026-10-11, two stacks: the stack validator passed
in both; the goals (3.0, -1.5, -30 deg) and then (5.0, -1.5, 0 deg)
succeeded in both (16.1 and 15.6 s for the first), and in the second the
goal (1.0, -3.0, -90 deg) after them too (53.6 s, turning around); and
89-99 % of SLAM's wall cells in the area nvblox had observed had an nvblox
obstacle within 0.15 m. In both the planner failed to replan ("no valid
path found") two or three times in the first 4 s of the first goal, while
the maps fill in, and the car kept to its first plan; not compared with
the earlier setting. More than half of nvblox's obstacle cells (53-80 %)
are not within 0.15 m of SLAM's walls; some are streaks along the
camera's rays a few metres out, and the Demo World has no ground truth to
say how many of the rest are objects off the lidar's plane or the inside
of thick obstacles. Not checked further.

Closed-loop status: the stack comes up unattended and Nav2 reaches goals in
Demo World. The goal tables below were measured with nvblox on Depth
Anything's depth (before the change above). `NavigateToPose` goals relative to the start pose, each on a
freshly started stack (SLAM, Depth Anything and nvblox running), measured on
2026-10-11 with this composition and its images:

| Goal (x, y, yaw) | Runs | Succeeded | Recoveries | Time to goal | Final error (SLAM map) | Final error (scan match) |
| --- | --- | --- | --- | --- | --- | --- |
| 3.0, -1.5, -30 deg | 4 | 4 | 0 | 15.3-16.1 s | 0.23-0.27 m, 1-4 deg | 0.17-0.24 m |
| 1.0, -3.0, -90 deg (right turn) | 4 | 4 | 0 | 16.3-17.6 s | 0.22-0.25 m, 1-9 deg | 0.19-0.24 m |
| 5.0, -1.5, 0 deg | 4 | 4 | 0 | 24.0-24.5 s | 0.23-0.24 m, 3-6 deg | 0.17-0.23 m |

The goal checker's tolerance is 0.25 m / 0.35 rad. The scan-match column
locates the car by matching its final scan against the SLAM map, without
odometry, on the same 0.05 m / 1 deg grid as the earlier tables; refined to
0.01 m / 0.1 deg it gives 0.17-0.23, 0.22-0.24 and 0.18-0.22 m, so every
run stopped inside the tolerance. The refined match put the car 0.02-0.06 m
from the pose the stack reported, in 10 of 12 runs nearer the goal. The
controller held its 15 Hz in every run (`/cmd_vel_nav` every 66.7 ms,
longest gap 80 ms, no "missed its desired rate" warnings), and no run logged
a recovery or an nvblox, costmap or SLAM warning after start-up.

SLAM Toolbox matches a scan, and so corrects `map -> odom`, every 0.2 m of
odometry travel (`minimum_travel_distance: 0.2` with
`check_min_dist_and_heading_precisely: true` in `slam_toolbox.yaml`), every
0.83-0.86 s at Nav2's 0.25 m/s. A correction moved the car's pose in the map
by up to 0.12 m in the first 5 s, while the map fills in, then by a median
0.02 m (90th percentile 0.06 m, at most 0.10 m), and in the last 2 s before
arrival by a median 0.02 m, at most 0.06 m.

With the 0.5 m default, two effects left the car outside the tolerance
(table below). Without the precise check SLAM Toolbox lets a scan through
after 0.45 m and its mapper drops it as under 0.5 m, so it matched only
every 0.9 m. `/odom` over-reports distance by 7-10%, more in turns, so
between matches the stack's pose ran ahead of the car: in four of the six
reruns, all on the two turning goals, the scan match put the car 0.03-0.09 m
farther from the goal than the stack did. And the controller checks the
goal in `odom` through the `map -> odom` at its current plan's timestamp,
while the plan is refreshed once a second, so it misses a correction in
that last second: in one rerun the goal check measured 0.245 m while the
corrected pose was 0.31 m from the goal. With 0.2 m spacing the first effect
is gone (the scan match was at most 0.01 m farther from the goal than the
stack) and the second is a few centimetres: the goal check missed up to
0.03 m of correction, and two runs ended 0.254 and 0.269 m from the goal in
the SLAM map while the refined scan match put them 0.233 and 0.231 m away.
A faster plan refresh would narrow this further, but each new plan already
causes most of the steering changes (below), and SLAM Toolbox stamps
`map -> odom` 0.2 s after its scan (`transform_timeout`), about 0.13 s ahead
of its arrival, so a plan still misses a correction from the 0.13 s before
it.

The cost: SLAM Toolbox used 4.6-5.3% of one CPU core while driving (2.9-3.3%
with the 0.5 m default; 2.3-2.6% idle with either), and Nav2 50-53% as
before. It keeps 16-27 scans for these drives instead of 3-6, so the map has
about twice as many occupied cells: walls are continuous instead of dotted
and the free space has no gaps between scans. Walls near the car stay one to
two cells thick, walls 6-9 m away spread over 3-6 cells as before, and none
is doubled; the final scan fits the map with a mean endpoint-to-wall
distance of 0.008-0.018 m (0.022-0.038 m before). A correction now changes
the commanded yaw rate half as much (rms 0.046 rad/s per 15 Hz cycle in the
0.5 s after one, 0.095 rad/s before) but four times as often, so over a run
the change per cycle is 0.026-0.056 rad/s rms (0.023-0.062 rad/s before),
and the car strays at most 0.05-0.09 m from its plan (0.05-0.17 m before).
Times to goal are unchanged. The pose graph gains a node every 0.2 m, so
long missions spend more on loop closure and on the map update (every 2 s,
over all scans); that is not measured beyond these 25 s drives.

Before the speed loop, the first goal succeeded once in two runs after five
recoveries; the other run stopped against a wall ~1 m from where the stack
believed it was. SLAM only maps where the lidar hits something, so in the
open Demo World a goal outside the mapped area is rejected as
`GOAL_OUTSIDE_MAP` (204).

Measured with the full stack while the 2026-10-10 goals ran (three stacks;
40 s before the goal and 12 s while driving):

| Topic | Rate | Age at receipt, median / p95 |
| --- | --- | --- |
| Bridge camera, depth, scan, IMU and odometry | 19.1-19.5 Hz, 18.4-18.6 Hz while driving | colour 25-27 / 31-35 ms |
| `/depth_anything/depth/image` | 16.1-17.6 Hz | 105-111 / 131-138 ms |
| `/nvblox_node/static_map_slice` | 9.1-9.4 Hz (`update_esdf_rate_hz: 10`) | 118-122 / 137-140 ms |
| `/local_costmap/costmap` | 1.6-1.7 Hz (2 Hz publishing on the 5 Hz update loop) | |

The stack used 1.6-1.8 GiB of GPU memory at 73-90% utilization.

**2026-10-10, before the SLAM change** (`minimum_travel_distance` at its
0.5 m default, without the precise check), with the same composition and
images:

| Goal (x, y, yaw) | Runs | Succeeded | Recoveries | Time to goal | Final error (SLAM map) | Final error (scan match) |
| --- | --- | --- | --- | --- | --- | --- |
| 3.0, -1.5, -30 deg | 5 | 5 | 0 | 15.3-16.3 s | 0.23-0.30 m, 3-5 deg | 0.17-0.30 m |
| 1.0, -3.0, -90 deg (right turn) | 3 | 3 | 0 | 16.6-17.4 s | 0.23 m, 2-10 deg | 0.27-0.32 m |
| 5.0, -1.5, 0 deg | 3 | 3 | 0 | 24.4-24.6 s | 0.23-0.27 m, 2-6 deg | 0.21-0.23 m |

Three runs stopped 0.27-0.30 m from the goal in the SLAM map although Nav2
reported success, and five 0.27-0.32 m by scan match. SLAM Toolbox corrected
every 3.6-4.3 s in steps of 0.04-0.21 m, mostly 0.06-0.12 m. Rerunning that
setting on 2026-10-11 alongside the runs above, two runs per goal: 6 of 6
succeeded without recoveries in 15.1-15.5, 17.8-18.1 and 23.7-24.1 s,
0.23-0.31 m from the goal in the SLAM map and 0.22-0.32 m by scan match
(0.21-0.32 m refined, four of six outside the tolerance), with a match every
0.89-0.91 m of odometry (3.6-4.7 s) moving the car 0.02-0.19 m.

**2026-10-10:** re-measured after the 4 MiB DDS socket buffers and Depth
Anything's one-frame queue. The earlier table, with the 208 KiB default
buffers and a five-frame queue, had 12 of 12 goals reached without
recoveries and 0.22-0.24 m final error (6, 3 and 3 runs). Rerunning that
configuration the same day (3 runs each of the first and last goal) gave
6 of 6, no recoveries, 0.22-0.23 m, the same 15 Hz controller and the same
times to goal (15.3-15.7 s and 23.9-24.5 s). In that configuration Depth
Anything also ran at 16.8-17.3 Hz with the full stack (the 3.8 Hz above was
measured with two extra camera subscribers on the bridge, one of them
reliable), but its depth was 0.30 s old while driving instead of 0.11 s.
No run in either configuration logged an nvblox, costmap or controller
warning after start-up. The wider final-error range of the 2026-10-10 table
comes from the SLAM correction timing, which does not depend on the depth
pipeline: both configurations corrected in the same steps every 3.6-4.3 s,
and in the six reruns the last correction before arrival happened to come
1.5-3.7 s early.

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
