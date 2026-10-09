# Turret Hardware Shopping List

Goal: build a pan-tilt perception turret that can be simulated and validated
against the `applications/pantiltros` ROS 2 control stack. The turret should
segment a realistic camera scene, select a target object, and drive pan/tilt
servos to keep that object centered while it moves through the scene.

Baseline hardware

- 1 x two-axis pan-tilt mechanism compatible with `pan_tilt_ros`.
- 2 x Waveshare ST3215, Waveshare ST3215-HS, or Feetech STS3215 serial bus
  servos for pan and tilt.
- 1 x Arduino UNO R4 WiFi, Arduino SKU ABX00087, for the turret
  microcontroller. Buy the real Arduino board, not a clone, so the USB-C
  interface, 5 V logic, Wi-Fi/Bluetooth, and board package match the supported
  Arduino/micro-ROS tooling.
- 1 x USB-C data cable for the Arduino UNO R4 WiFi.
- 1 x half-duplex TTL serial interface, level shifter, or ST3215/STS3215
  Arduino shield/cape between the Arduino hardware UART and the serial bus
  servos. Do not power the servos from the Arduino board.
- 1 x optional USB serial bus servo adapter for one-time servo ID, midpoint,
  and limit setup before the Arduino firmware takes over runtime control.
- 1 x rigid base or tripod mount with enough stiffness to avoid camera shake.
- 1 x RGB camera mounted on the tilt payload link. Use a global-shutter camera
  if fast moving targets are required; otherwise a UVC camera or OAK RGB stream
  is acceptable for simulation parity.
- 1 x compute host capable of running ROS 2, PanTiltROS, and Ultralytics YOLO
  segmentation. Use GPU acceleration for high-resolution YOLO26 segmentation
  tests.
- 1 x regulated servo power supply sized for both servos at peak load.
- 1 x physical emergency-stop switch that removes servo power, plus a way to
  publish the software stop on `/turret/emergency_stop` (see below).
- Cabling, strain relief, mounting hardware, and optional cable chain so the
  pan/tilt range cannot pull camera or servo cables tight.

Arduino integration

Status: the ROS side exists; the turret firmware does not. Nothing in this
repository implements the serial protocol below on the Arduino yet
(`applications/arduino/scripts` only holds micro-ROS/UDP demos), and the
upstream ST3215 node `pan_tilt_ctrl_node` in `applications/pantiltros/pantiltros`
is not built in the PanTiltROS image: it needs the Feetech `SCServo_Linux`
library (`include/SCServo_Linux/libSCServo.a`), which is not vendored.

- `compositions/turret_hardware.yaml` runs the YOLO tracker with
  `applications/pantiltros/config/yolo_tracker_hardware.yaml` and the serial
  bridge `applications/pantiltros/scripts/turret_arduino_bridge.py`; the
  `arduino` profile adds the Arduino IDE container for flashing firmware.
- Serial protocol (ASCII lines, `\n` terminated, radians, 115200 baud by
  default, see `config/arduino.env`):
  - host to firmware `SET pan_joint <pan> tilt_joint <tilt>`: move to the
    position (already clamped to the joint limits by the bridge);
  - host to firmware `STOP`: stop moving and hold (or disable torque). Sent
    when the software e-stop engages, on bridge shutdown, and after
    (re)connecting while the e-stop is engaged;
  - firmware to host `POS pan_joint <pan> tilt_joint <tilt>`: measured joint
    positions, e.g. at 20-50 Hz. These are the only source of
    `/turret/joint_states`; the bridge never echoes commands;
  - firmware to host `ERR <text>`: shown in the bridge diagnostics.
- Firmware should also stop on its own when no `SET` arrives for ~0.5 s, apply
  its own servo limits, and drive the ST3215/STS3215 bus through the
  half-duplex interface.
- The bridge waits 2 s after opening the port (Arduino reset) and drops
  commands while disconnected, resetting or e-stopped, so stale commands are
  never replayed. Without `POS` feedback the tracker drives open loop and only
  while the bridge reports `/turret/actuator_ready`.
- Keep servo power on a separate regulated supply with a shared signal ground
  to the Arduino and a physical emergency-stop path that removes actuator
  power.

ROS contract

- Camera stream: the hardware composition defaults to the RealSense topics
  `/camera/camera/color/image_raw` and `/camera/camera/color/camera_info`
  (`TURRET_CAMERA_TOPIC`, `TURRET_CAMERA_INFO_TOPIC`).
- Segmentation output: `/turret/yolo/segments`, `/turret/yolo/target_mask`,
  `/turret/yolo/target_bbox`, `/turret/yolo/target_confidence`, and
  `/turret/yolo/annotated_image`.
- Tracker output: `/turret/target/pixel_error`,
  `/turret/target/angular_error`, `/turret/target/selected_id`, and
  `/turret/target/locked`.
- Joint commands: `sensor_msgs/msg/JointState` on `/turret/joint_commands`
  with `pan_joint` and `tilt_joint`.
- Measured joint state: `sensor_msgs/msg/JointState` on `/turret/joint_states`,
  published by the bridge from firmware `POS` lines only.
- Emergency stop: `std_msgs/msg/Bool` on `/turret/emergency_stop`. `true`
  stops the tracker from commanding and makes the bridge send `STOP` and drop
  commands. On hardware both start engaged and need an explicit `false`
  (`require_estop_release`), e.g.
  `ros2 topic pub --times 3 /turret/emergency_stop std_msgs/msg/Bool "{data: false}"`.
- Actuator readiness: `std_msgs/msg/Bool` on `/turret/actuator_ready`
  (transient-local), true only while the bridge is connected, settled, not
  e-stopped and writing commands.
- Diagnostics: `/turret/diagnostics` from the tracker and the bridge.

Simulation compatibility notes

- Start from `applications/pantiltros/pantiltros/urdf/pan_tilt_urdf.xacro`
  for geometry, joint names, limits, and the payload link.
- Preserve the current PanTiltROS default joint names: `pan_joint` and
  `tilt_joint`.
- Simulate servo position limits, speed, acceleration, quantization, backlash,
  current/temperature diagnostics, command latency, and emergency-stop torque
  disable behavior.
- The segmentation model should track the current Ultralytics YOLO
  instance-segmentation checkpoint at implementation time. As of June 2026,
  public Ultralytics YOLO26 material describes instance segmentation support.
  Use the latest available YOLO26 `-seg` checkpoint in the installed
  Ultralytics release for GPU validation, the smallest available YOLO26
  segmentation checkpoint for CPU smoke tests, and re-verify the exact
  checkpoint name before implementation.

Useful hardware and software references

- Arduino UNO R4 WiFi:
  https://store.arduino.cc/products/uno-r4-wifi
- micro-ROS Arduino:
  https://github.com/micro-ROS/micro_ros_arduino
- PanTiltROS:
  https://github.com/adityakamath/pan_tilt_ros
