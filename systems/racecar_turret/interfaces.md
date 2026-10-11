# RACECAR Turret: interface contract

The RACECAR Turret is the RACECAR Neo (`systems/racecarneo`) with the
PanTiltROS turret (`applications/pantiltros`) on a raised plate behind the
lidar. It drives (SLAM Toolbox + Nav2, as the RACECAR) and keeps a selected
object centred in the turret camera at the same time. One web page on the car
manages both. Isaac Sim simulates the whole vehicle and scores driving and
aiming together. This file is the contract between those parts; change it
only together with every part that uses it.

Hardware choices: the turret camera is a USB (UVC) camera on the tilt link,
and the PanTiltROS YOLO tracker runs on the car's Jetson next to Depth
Anything and nvblox. The Jetson drives the two ST3215 serial bus servos
itself, with no microcontroller: its 40-pin header UART (pins 8 TX and 10 RX,
`/dev/ttyTHS1`, 3.3 V) goes to a half-duplex bus servo adapter (Waveshare Bus
Servo Adapter (A) in UART mode), which also feeds the servos from the servo
supply. `turret_arduino_bridge.py`'s direct Feetech STS transport talks to the
servos with the same ROS interface and safety behaviour as its Arduino
transports. The car's front RealSense stays fixed and is used for navigation
only.

## Frames and geometry

`urdf/racecar_turret.urdf.xacro` includes the RACECAR URDF unchanged
(`base_link` on the ground under the chassis centre, x forward, z up;
`racecarneo/lidar_link` (0.08, 0, 0.18), `racecarneo/camera_link`
(0.18, 0, 0.22), `racecarneo/imu_link` (0, 0, 0.08)) and adds:

| Frame / joint | Parent | Origin (m) | Notes |
|---|---|---|---|
| `turret/base_link` (`turret_mount_joint`, fixed) | `base_link` | (-0.10, 0, 0.24) | Plate on four posts above the lidar |
| `turret/pan_link` (`pan_joint`, revolute) | `turret/base_link` | (0, 0, 0.045) | axis (0, 0, -1): positive pans right; limits ±1.5708 rad |
| `turret/tilt_link` (`tilt_joint`, revolute) | `turret/pan_link` | (0, 0, 0.055) | axis (0, -1, 0): positive tilts up; limits -0.65..0.75 rad |
| `turret/camera_link` (fixed) | `turret/tilt_link` | (0.035, 0, 0) | x forward, y left, z up |
| `turret/camera_optical_frame` (fixed) | `turret/camera_link` | rpy (-π/2, 0, -π/2) | REP-103 optical |

The joint names and signs are those of `/turret/joint_states` and the
PanTiltROS tracker and turret simulator: pan positive turns the camera
towards an object right of the image centre (+u), tilt positive towards one
above it (-v).
`robot_state_publisher` reads the turret joints from `/turret/joint_states`.

## Topics

The car's topics are those of `systems/racecarneo/config/topics.yaml`,
unchanged (`/cmd_vel`, `/ackermann_cmd`, `/odom`, `/scan`, `/imu/data_raw`,
`/camera/color/*`, `/camera/depth/*`), except that on this platform `/scan`
is the masked scan: the turret plate's four posts sit inside Nav2's robot
radius, so the lidar driver publishes `/scan_raw` and a laser_filters box
filter (`config/scan_filter.yaml`) removes the post returns into `/scan`. A
simulator that does not model the posts publishes `/scan` directly. The turret keeps every existing
`/turret/*` topic of `applications/pantiltros/README.md`. Added:

| Topic | Type | Producer → consumer | Content |
|---|---|---|---|
| `/turret/camera/image_raw`, `/turret/camera/camera_info` | `sensor_msgs/Image` (rgb8 or bgr8), `CameraInfo` | USB camera driver or Isaac → tracker, UI | `frame_id` `turret/camera_optical_frame`; 640x480 at 15 Hz or more |
| `/turret/command` | `std_msgs/String` (JSON) | UI → tracker | See "Turret commands" |
| `/turret/status` | `std_msgs/String` (JSON), reliable + transient-local, 2 Hz or more | tracker → UI, scorer | `{"mode": "idle" \| "auto" \| "manual" \| "estop", "target_id": "3" \| "", "target_class": "person", "locked": bool, "aim_error_deg": float \| null, "base_compensation": bool, "detections": int, "message": ""}` |
| `/turret/yolo/segments` | `std_msgs/String` (JSON), existing | tracker → UI | A JSON list, one object per detection, each with a stable `id` (string), `class`, `confidence`, `bbox_xywh` (centre x, centre y, width, height, in pixels of the `/turret/camera/camera_info` image), `stamp`, `image_width` and `image_height`, so the UI can draw and pick boxes |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | UI → `cmd_vel_to_ackermann` | Manual driving; takes priority over `/cmd_vel` while messages keep arriving (0.5 s timeout) |
| `/racecarneo/stop_request` | `std_msgs/Bool`, reliable + transient-local | UI → `cmd_vel_to_ackermann` | `true` latches zero speed (steering held) whatever `/cmd_vel` or `/cmd_vel_teleop` say; `false` releases |
| `/racecarneo/control_status` | `std_msgs/String` (JSON), 2 Hz | `cmd_vel_to_ackermann` → UI | `{"source": "nav2" \| "teleop" \| "none", "stopped": bool, "speed_mps": float, "steering_rad": float}` |

Isaac only:

| Topic | Type | Content |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Simulation time; every node runs with `use_sim_time` |
| `/simulation/truth` | `std_msgs/String` (JSON), 10 Hz or more | Ground truth in the `odom` frame at the sim stamp: `{"stamp": s, "vehicle": {"x", "y", "yaw", "speed_mps"}, "turret_camera": {"position": [x, y, z], "forward": [x, y, z]}, "targets": [{"name": "person_1", "class": "person", "position": [x, y, z], "centroid": [x, y, z], "visible": bool}], "collisions": int}` |

## Turret commands (`/turret/command`)

- `{"action": "lock", "target_id": "3"}`: track the detection with that ID.
- `{"action": "lock", "point": [u, v]}`: `u`, `v` in [0, 1] of the image;
  track the detection whose box contains the point (the nearest box centre
  if several do, none if none does: status message says so).
- `{"action": "release"}`: stop tracking and hold the current aim (mode
  `idle`).
- `{"action": "manual", "pan_rate": r, "tilt_rate": r}`: rad/s, clamped to
  the actuator limits, implies `release`; the UI repeats it at 10 Hz while
  the operator holds the control, and the turret holds position when it
  stops arriving for 0.5 s.
- `{"action": "center"}`: release and aim at pan 0, tilt 0.

A lock that matches no detection leaves the mode unchanged and says so in
the status message. A client that locks right after start-up (the Isaac
evaluator) waits until `/turret/status` reports `detections` > 0.

The existing safety gates are unchanged: nothing moves while
`/turret/emergency_stop` is true or `/turret/actuator_ready` is false.
The UI's turret arm and disarm buttons publish `/turret/emergency_stop`
(latched).

## Aiming from a moving base

The tracker compensates the base's rotation: with the base's angular
velocity (`sensor_msgs/Imu` on `base_imu_topic`, default `/imu/data_raw`,
rotated into `turret/base_link` through TF), a target that is still in the
world stays still in the image while the car turns. It also leads a target
whose bearing is changing (the car driving past it, or the target walking)
from the bearing rate it estimates from successive detections. Both can be
switched off (`base_compensation`, `target_lead`) so the evaluation can
measure what they are worth. Without an IMU message for 0.5 s the tracker
falls back to fixed-base behaviour and says so in `/turret/status`.

## Isaac evaluation

`compositions/racecar_turret_isaac.yaml` runs Isaac Sim with the vehicle
(Ackermann drive with the hardware's wheelbase 0.33 m and steering limit
0.42 rad, the RACECAR's sensors at the URDF mounts, the turret with ST3215
speed limits and the turret camera), the RACECAR stack (SLAM Toolbox, Nav2,
Depth Anything, nvblox, `cmd_vel_to_ackermann`) and the turret stack
(tracker, with the Isaac camera in place of the USB camera). A scenario
drives a Nav2 waypoint course through an arena with obstacles while people
walk; at the start the evaluator selects a target through `/turret/command`
exactly as the UI does. The scorer reads `/simulation/truth`,
`/turret/status` and the Nav2 result and reports, per run:

- driving: course completed, time, collisions, mean speed;
- aiming: the angle between the turret camera's optical axis and its line of
  sight to the selected target's centroid (mean, p95), the fraction of time
  it is under 5 degrees ("on target"), the fraction of time the tracker
  reports `locked`, and reacquisition times after occlusions;
- the same with `base_compensation` and `target_lead` off, for comparison.

Thresholds live in `validation/acceptance_thresholds.json`;
`tools/racecar_turret_isaac_test.sh` runs the whole evaluation and exits
non-zero when a threshold is missed.
