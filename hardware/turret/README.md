# Turret Hardware Shopping List

Goal: build a pan-tilt perception turret that can be simulated and validated
against the `applications/pantiltros` ROS 2 control stack. The turret should
segment a realistic camera scene, select a target object, and drive pan/tilt
servos to keep that object centered while it moves through the scene.

Baseline hardware

- 1 x two-axis pan-tilt mechanism compatible with `pan_tilt_ros`.
- 2 x Waveshare ST3215, Waveshare ST3215-HS, or Feetech STS3215 serial bus
  servos for pan and tilt.
- 1 x USB serial bus servo adapter compatible with the selected ST3215 servos.
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
