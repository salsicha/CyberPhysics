# PanTiltROS

This application owns the reusable ROS 2 perception and actuation layer for the
pan/tilt turret.  The same image and `pantilt_yolo_tracker.py` node are used in
simulation and on the physical robot.  The scene renderer is deliberately kept
in `applications/turret_sim`; ground truth never enters the controller.

The YOLO node consumes a ROS camera image, camera intrinsics, and pan/tilt joint
state.  It runs Ultralytics YOLOE segmentation, gives every detection an ID that
persists across frames, follows the selected target, and publishes masks, target
errors, status, diagnostics, and closed-loop `JointState` commands. The target is
chosen at start-up from the configuration or at run time on `/turret/command`.
Configuration is in `config/yolo_tracker.yaml` (simulator scene) and
`config/yolo_tracker_hardware.yaml` (physical turret: generic `person` prompt,
confidence 0.25, narrow association gate, commands only after the emergency
stop is released). On a moving base (the RACECAR Turret,
`systems/racecar_turret`) the tracker also compensates the base's rotation
from its IMU; that platform's profiles are
`systems/racecar_turret/config/turret_tracker.yaml` (car) and
`turret_tracker_isaac.yaml` (Isaac Sim, `use_sim_time`).

The image is built on the repository's ROS 2 Jazzy base (`ros2`, Python 3.12;
build it first with `make -C applications build_ros2`) and pins Ultralytics
8.4.52 with Torch 2.14.1 (CUDA 13 wheels, NVIDIA driver >= 580). It also builds
the upstream ST3215 package `pan_tilt_ros` (`pantiltros/`, with the
`SCServo_Linux` library fetched at the commit upstream pins), so
`ros2 launch pan_tilt_ros pan_tilt_launch.py` and `description_launch.py` are
available in the container.

### Control loop

Each detection's angular error is added to the turret position *when that
frame was captured* (measured joint states interpolated at the image stamp, or,
without measured feedback, the command in effect `actuator_latency_s` earlier)
to estimate the target direction; the command moves `kp` of the way there per
frame, limited to `max_rate_rad_s`. This makes the loop independent of camera,
inference and actuator latency; if the compensation is wrong the update
degrades to an integrator, stable for `kp < 2 sin(pi / (2 (2n + 1)))` with n
frames of delay (1.0, 0.62, 0.45, 0.35 for n = 1..4). The defaults (0.4
simulation, 0.3 hardware) stay stable for 3-4 uncompensated frames.

Commands are published only while all of these hold:

- `command_enabled` is true;
- the emergency stop is not engaged (`/turret/emergency_stop`,
  `std_msgs/msg/Bool`; with `require_estop_release` a `false` message must be
  received first);
- measured joint states arrived within `joint_state_timeout_s`, or the
  actuator reports ready on `/turret/actuator_ready` within
  `actuator_ready_timeout_s`. Without either, commands that nobody executes
  would wind the turret to a joint limit.

Non-finite errors and joint states are rejected.

### Moving base and target lead

With `base_compensation` the tracker integrates the base's angular velocity
(`sensor_msgs/Imu` on `base_imu_topic`, default `/imu/data_raw`) into the
base's attitude, after rotating it into `base_frame` (`turret/base_link`)
with the static TF transform from the messages' `frame_id` (or
`base_imu_frame` when the driver's is wrong or empty). The aim then lives in
that world-fixed frame: each frame's target direction (the turret position at
capture plus the image error, as above) and the previous aim are carried to
the time the command will take effect and turned back into joint angles
through the base's attitude at that time. That is a full rotation, so base yaw
maps to pan one to one at any tilt, and roll or pitch reaches pan and tilt
through the pan angle. Between frames a `control_rate_hz` (50 Hz) timer keeps
the aim on its world direction, so a still target stays still in the image
while the car turns. The time a command takes to move the turret is fitted to
the measured joint states twice a second while the turret moves
(`actuation_delay_s` in the diagnostics; `actuator_latency_s` until then or
without measured joint states). Joint states must therefore be stamped when
the position was measured.

With `target_lead` the tracker fits the target's line-of-sight rate (in the
same world-fixed frame) over the last `lead_window_s` of detections and feeds
it forward, which removes the lag of the aim filter for a walking target or
one the car drives past. It is used only with measured joint states, and it
coasts for at most 0.5 s after the target is lost.

Without an IMU message for `imu_timeout_s` (0.5 s), or without the IMU's
transform, the attitude freezes and aiming falls back to a fixed base;
`/turret/status` says so. With neither feature active the update is exactly
the fixed-base loop above. The fixed-turret profiles switch both off: lead
cut the error on the turret_sim plant, but with the simulator's noisy YOLOE
detections, coasting through an occlusion once carried the aim onto a
same-class decoy inside the start-up association gate.

`tests/test_turret_moving_base.py` runs the node against the turret_sim servo
model on a car (camera at 15 Hz, 70 ms inference, a servo 120 ms behind its
commands, gyro noise and bias, the hardware profile). Mean (p95) aim error:

| Run | Off | On |
|---|---|---|
| Slalom, yaw rate up to 0.94 rad/s, base compensation | 12.3 (19.9) deg | 0.8 (1.2) deg |
| The same with a 40 ms servo (profile assumes 150 ms) | 9.8 (15.7) deg | 0.8 (1.3) deg |
| Yaw and roll, target 0.3-0.4 rad above, IMU mounted turned | 11.3 (18.2) deg | 1.0 (1.7) deg |
| Person crossing at 1.5 m/s, 5 m away, target lead | 6.1 (6.8) deg | 0.2 (0.5) deg |
| Driving past a person 2.5 m away at 2 m/s, target lead | 6.5 (11.3) deg | 1.6 (3.2) deg |

### Simulation time

Every timing decision uses the node clock, so the tracker runs with
`use_sim_time` (Isaac publishes `/clock` and stamps images and joint states
with it), also when simulation time runs slower than real time: latency
compensation, the IMU and joint-state timeouts, the manual deadman and the
control timer all follow the stamps. Until the first `/clock` message nothing
is commanded (status "waiting for /clock"). Only `/turret/status` is published
on a wall-clock timer (`status_rate_hz`, 5 Hz), so a UI sees it however fast
the simulation runs; its `stamp` is node time. Inference latency in the
diagnostics is wall time.

## Runtime commands and status

`/turret/command` (`std_msgs/String`, JSON) selects and drives the turret at
run time:

- `{"action": "lock", "target_id": "3"}`: follow the detection with that ID.
- `{"action": "lock", "point": [u, v]}`: `u`, `v` in [0, 1] of the image;
  follow the detection whose box contains the point (the nearest box centre
  if several do). A lock that matches nothing keeps the current mode and
  says so in the status message.
- `{"action": "release"}`: stop tracking and hold the current aim (`idle`).
- `{"action": "manual", "pan_rate": r, "tilt_rate": r}`: rad/s, clamped to
  `max_rate_rad_s`, positions clamped to the joint limits; implies `release`.
  Repeat it (the UI does at 10 Hz): after `manual_timeout_s` (0.5 s) without
  one the turret holds.
- `{"action": "center"}`: release and move to pan 0, tilt 0 at
  `max_rate_rad_s`.

The safety gates above apply to all of them. A locked track that disappears
is picked up again by class within the association gate (it then gets the new
ID). With `auto_start` (default, the simulator and fixed-turret profiles) the
tracker starts by following `target_class` from `initial_selection_normalized`
as before; the RACECAR Turret profiles start `idle` and wait for a lock.

`/turret/status` (`std_msgs/String`, JSON, reliable and transient-local, 5 Hz
and after every command):

```json
{"mode": "auto", "target_id": "3", "target_class": "person", "locked": true,
 "aim_error_deg": 1.2, "base_compensation": true, "detections": 2,
 "message": "", "target_lead": true, "stamp": 1234.5,
 "command_state": "measured"}
```

`mode` is `idle`, `auto`, `manual` or `estop` (the emergency stop is
engaged); `target_id` is the followed detection's ID (`""` when not
tracking); `locked` means detected within `lock_tolerance_px` of the image
centre in the last second; `aim_error_deg` is the angle from the optical axis
to the target in the last frame (`null` when not detected);
`base_compensation` and `target_lead` say whether they are in use;
`command_state` is the safety gate (`measured`/`commanded` allow commands).
`message` joins the last command's result and what limits tracking (emergency
stop, actuator not ready, no camera images, no IMU, searching).

`/turret/yolo/segments` stays a JSON list with one object per detection; each
now also carries a stable `id` (string), `bbox_xyxy` (corners), `selected`
(the followed target), and the image's `stamp` (seconds), `image_width` and
`image_height`:

```json
[{"id": "3", "class": "person", "confidence": 0.87,
  "bbox_xywh": [412.0, 240.5, 96.0, 260.0], "bbox_xyxy": [364.0, 110.5, 460.0, 370.5],
  "centroid_px": [410.2, 236.8], "mask_area_px": 15873, "selected": true,
  "stamp": 1234.567, "image_width": 640, "image_height": 480}]
```

`bbox_xywh` is the box centre and size in pixels. IDs persist while a
detection of the same class continues within one box diagonal of the last
position, and through gaps up to `track_timeout_s` (1 s).

## Run

Run against the ROS camera and actuator topics already on the host:

```bash
docker compose -f compositions/pantiltros.yaml up --build
```

The annotated camera stream is published by default on
`/turret/yolo/annotated_image`. View it with RViz, Foxglove, or:

```bash
rqt_image_view /turret/yolo/annotated_image
```

To open the annotated OpenCV window directly from the container:

```bash
xhost +si:localuser:root
TURRET_SHOW_WINDOW=true \
docker compose -f compositions/pantiltros.yaml up --build
```

Press `q` or Escape to close only the window; ROS inference and publishers keep
running. Revoke the temporary X11 permission afterward with
`xhost -si:localuser:root`. Set `TURRET_PUBLISH_ANNOTATED=false` to disable the
annotated ROS image when bandwidth matters, or change its name with
`TURRET_ANNOTATED_TOPIC`.


Run the complete simulation:

```bash
docker compose -f compositions/turret_sim.yaml up --build
```

Run on the Arduino turret with an existing ROS camera:

```bash
docker compose -f compositions/turret_hardware.yaml up --build
```

With an Arduino UNO Q instead of the UNO R4 WiFi, the bridge runs on the
board and the tracker on the GPU host (same `ROS_DOMAIN_ID`); see
`hardware/turret.md`:

```bash
docker compose -f compositions/turret_uno_q.yaml --profile board up -d   # on the UNO Q
docker compose -f compositions/turret_uno_q.yaml --profile host up       # on the host
```

Add the repository's RealSense service when it supplies the camera:

```bash
docker compose -f compositions/turret_hardware.yaml \
  --profile realsense up --build
```

The hardware composition defaults to RealSense topics
`/camera/camera/color/image_raw` and `/camera/camera/color/camera_info`. Override
them with `TURRET_CAMERA_TOPIC` and `TURRET_CAMERA_INFO_TOPIC` for another camera.

The default `yoloe-26s-seg.pt` checkpoint is downloaded by Ultralytics on first
startup and persists in `cache/turret/models/`.  Override it with
`TURRET_YOLO_MODEL`; `TURRET_YOLO_DEVICE`, `TURRET_YOLO_IMAGE_SIZE`, and
`TURRET_YOLO_CONFIDENCE` are also supported.

## ROS contract

- Input: `/turret/camera/image_raw`, `/turret/camera/camera_info`,
  `/turret/joint_states` (measured positions only),
  `/turret/emergency_stop` and `/turret/actuator_ready` (`std_msgs/msg/Bool`,
  transient-local; the e-stop also accepts default-QoS publishers),
  `/turret/command` (JSON, above), and with `base_compensation` the base IMU
  (`/imu/data_raw`) and TF
- Perception: `/turret/yolo/segments`, `/turret/yolo/target_mask`,
  `/turret/yolo/target_bbox`, `/turret/yolo/target_confidence`,
  `/turret/yolo/annotated_image`
- Tracking: `/turret/target/pixel_error`, `/turret/target/angular_error`,
  `/turret/target/selected_id` (the configured `target_id` while following the
  start-up selection, the detection ID after a lock), `/turret/target/locked`
- Output: `/turret/joint_commands`, `/turret/status`, `/turret/diagnostics`

The Arduino bridge (`scripts/turret_arduino_bridge.py`) uses
`config/arduino.env` and `config/arduino_bridge.yaml`. It writes
`SET pan_joint <rad> tilt_joint <rad>` lines for joint commands and `STOP` when
the emergency stop engages, and publishes `/turret/joint_states` only from
`POS pan_joint <rad> tilt_joint <rad>` lines sent by the firmware; it never
echoes commands as joint states. Commands arriving while the port is closed,
the Arduino is resetting, or the e-stop is engaged are dropped, not queued.
`/turret/actuator_ready` is true only while commands are being written. The
firmware is `applications/arduino/scripts/turret_firmware` (UNO R4 WiFi and
UNO Q; compiled and unit tested, and run with this bridge on a real UNO Q
without servos; not yet run with servos or on an UNO R4); see
`hardware/turret.md`.

The bridge's `transport` parameter (`TURRET_ARDUINO_TRANSPORT`) selects the
link:

- `serial` (default): the line protocol above on `serial_port` at `baud_rate`
  (UNO R4 WiFi on USB).
- `uno_q_bridge`: run on an Arduino UNO Q's Linux side
  (`config/arduino_uno_q.env`, `compositions/turret_uno_q.yaml`). The same
  messages travel as msgpack-RPC through arduino-router's Unix socket
  (`router_socket`, default `/var/run/arduino-router.sock`) to the MCU:
  requests `turret/set [pan, tilt]` and `turret/stop`, notifications
  `turret/pos [pan, tilt]` and `turret/err [text]`. The bridge registers for
  the notifications, probes the MCU with `turret/get_position` every second,
  and is ready only while the MCU answers (an RPC unanswered for
  `rpc_timeout_s`, 0.5 s, or a sketch that is not running makes it not ready).
  The topics, e-stop handling and `/turret/actuator_ready` semantics are those
  of the serial transport; the diagnostics name the router socket instead of
  the port.
- `sts`: no Arduino. The bridge drives the two ST3215s itself from any Linux
  host with a UART or a USB bus servo adapter, in Feetech STS packets at
  1 Mbps on `serial_port` through a half-duplex adapter
  (`config/jetson_uart.env`: the Jetson's 40-pin header UART `/dev/ttyTHS1`
  and a Waveshare Bus Servo Adapter (A), as on the RACECAR Turret). It runs a
  Python port of the firmware's servo code (`src/sts_servo.*` and the
  `Controller` of `src/turret_core.*`), so `SET` and `STOP` behave exactly as
  with the firmware: joint limits clamp every command, the first `SET`
  enables torque, no command for 500 ms or a `STOP` holds the measured
  position, positions are read at 25 Hz and published only when both servos
  answered, a servo that does not answer or reports a fault bit gives an
  error at most once per second, nothing is written to the servos before the
  first `SET` or `STOP`, and adapters that echo transmitted bytes are
  handled. The firmware's settings are parameters in
  `config/arduino_bridge.yaml` (`pan_servo_id` 12, `tilt_servo_id` 13,
  `*_zero_steps` 2048 and `*_direction` -1 for the `pan_tilt_ros`
  convention, `servo_speed` 2000 steps/s, `servo_acceleration` 50,
  `servo_reply_timeout_s`, `stop_disables_torque`). The port opens
  exclusively and is ready at once (no board reset to wait for); topics,
  e-stop handling and `/turret/actuator_ready` are those of the serial
  transport, and the diagnostics add `servo_ids`, `servo_state` and
  `bad_packets`. `tests/test_turret_bridge_sts.py` checks its packets against
  the firmware's `SCServo_Linux` reference bytes and the compiled firmware
  code, and runs the firmware test's scenarios on a simulated servo bus.

The `uno_q_bridge` transport needs `python3-msgpack`, which the small `unoq`
image (`applications/unoq`) provides; the PanTiltROS image only carries
`pyserial` for the serial and `sts` transports.

### Servos straight from a Linux host (`sts`)

Wiring to a Jetson (details, bring-up and known issues in
`hardware/turret.md`, "Direct from a Linux host"): header pin 8 (TX) to the
adapter's TX, pin 10 (RX) to its RX, a ground pin (6) to its GND, jumper on A
(UART mode). Waveshare's wiki says "the connection must be RX-RX, TX-TX": the
labels are from the host's side; if the servos do not answer, swap TX and RX
first. Waveshare does not document the logic level of these pins and the
Jetson header is 3.3 V, not 5 V tolerant: power the adapter and measure its
TX pin's idle voltage before connecting, and put a bidirectional level
shifter rated for 1 Mbps in between if it is above 3.3 V. Enable pins 8/10
with `sudo /opt/nvidia/jetson-io/jetson-io.py` (Configure 40-pin header,
`uarta`) and check the port with a loopback (pins 8 and 10 shorted, adapter
off):

```bash
stty -F /dev/ttyTHS1 1000000 raw -echo
timeout 2 cat /dev/ttyTHS1 & sleep 0.5; echo hello > /dev/ttyTHS1; wait   # prints hello
```

On some JetPack releases the header UART's TX does not drive (reported for
JetPack 7 / L4T R39.2 on the Orin Nano Super developer kit). The fallback is
the same adapter on USB (jumper on B): only the device path changes,
`TURRET_ARDUINO_PORT=/dev/serial/by-id/...`, which also avoids the logic-level
question.

Safety difference: there is no microcontroller watchdog. The 500 ms hold and
`STOP` run in the bridge process; if the host or the bridge dies, or the UART
link breaks, the ST3215s keep their last goal (already within the joint
limits) with torque on. The physical emergency stop that cuts servo power is
the hardware safety.

The other host-side option is the upstream C++ node `pan_tilt_ctrl_node`
(`pan_tilt_ros`, built in this image with Feetech's `SCServo_Linux`; port in
`pantiltros/config/ctrl_config.yaml`). It speaks a different contract
(`/joint_commands` and `/joint_states`, no `/turret/emergency_stop` or
`/turret/actuator_ready`, a joystick button that turns torque off as its
stop, step limits). The bridge's `sts` transport is used instead so the
tracker, UI and compositions see one ROS contract and one safety behaviour
whether an UNO R4, an UNO Q or the host itself drives the servos.

Not yet run on hardware: no bus servo adapter or ST3215 was attached when it
was written.

Release (arm) or engage the emergency stop. The tracker and bridge keep the
last value they received; do not leave two publishers with different values
running at the same time:

```bash
ros2 topic pub --times 3 /turret/emergency_stop std_msgs/msg/Bool "{data: false}"  # release
ros2 topic pub --times 3 /turret/emergency_stop std_msgs/msg/Bool "{data: true}"   # stop
```

A dedicated e-stop node should publish with transient-local, reliable QoS so
nodes that start later receive the current state. With a default-QoS publisher
such as the commands above, ROS warns about an incompatible `DURABILITY` policy
for the latched subscription; the message still arrives through the second,
volatile subscription. The software e-stop is not a
substitute for a physical switch that removes servo power.

The original package is based on
[adityakamath/pan_tilt_ros](https://github.com/adityakamath/pan_tilt_ros).
