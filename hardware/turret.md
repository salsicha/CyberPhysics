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
- 1 x emergency-stop input or joystick button mapped to the PanTiltROS
  `stop_button` parameter.
- Cabling, strain relief, mounting hardware, and optional cable chain so the
  pan/tilt range cannot pull camera or servo cables tight.

Arduino integration

- The hardware stack should run `compositions/turret_hardware.yaml`, which
  starts the PanTiltROS-compatible ROS side and the Arduino tooling profile.
- The ROS side publishes and consumes the same `pan_joint` and `tilt_joint`
  contract used in simulation, then forwards compact serial commands to the
  Arduino.
- The Arduino should run turret firmware that accepts host serial commands,
  applies configured servo limits, drives the ST3215/STS3215 bus through the
  half-duplex interface, and returns position/current/temperature/voltage
  feedback.
- Keep servo power on a separate regulated supply with a shared signal ground
  to the Arduino and a physical emergency-stop path that can remove actuator
  power.

ROS contract

- Manual command input: `sensor_msgs/msg/Joy` consumed by `pan_tilt_cmd_node`.
- Commanded joints: `sensor_msgs/msg/JointState` for `pan_joint` and
  `tilt_joint`.
- Servo controller output and feedback: `pan_tilt_ctrl_node` commands the
  ST3215 bus and publishes measured `sensor_msgs/msg/JointState` plus
  `diagnostic_msgs/msg/DiagnosticArray`.
- Camera stream: `/turret/camera/image_raw` with
  `/turret/camera/camera_info`.
- Segmentation output: `/turret/yolo/segments`, `/turret/yolo/target_mask`,
  `/turret/yolo/target_bbox`, and `/turret/yolo/target_confidence`.
- Tracker output: `/turret/target/pixel_error`,
  `/turret/target/angular_error`, `/turret/target/selected_id`, and
  `/turret/target/locked`.
- Actuator command output: `/turret/joint_commands` or the PanTiltROS
  `JointState` command topic used by the simulated controller.

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
