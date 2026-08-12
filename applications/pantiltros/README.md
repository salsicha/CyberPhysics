# PanTiltROS

This application owns the reusable ROS 2 perception and actuation layer for the
pan/tilt turret.  The same image and `pantilt_yolo_tracker.py` node are used in
simulation and on the physical robot.  The scene renderer is deliberately kept
in `applications/turret_sim`; ground truth never enters the controller.

The YOLO node consumes a ROS camera image, camera intrinsics, and pan/tilt joint
state.  It runs Ultralytics YOLOE segmentation, associates the configured target,
and publishes masks, target errors, diagnostics, and closed-loop `JointState`
commands.  Configuration is in `config/yolo_tracker.yaml`.

## Run

Run against the ROS camera and actuator topics already on the host:

```bash
docker compose -f compositions/pantiltros.yaml up --build
```

Run the complete simulation:

```bash
docker compose -f compositions/turret_sim.yaml up --build
```

Run on the Arduino turret with an existing ROS camera:

```bash
docker compose -f compositions/turret_hardware.yaml up --build
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
  `/turret/joint_states`
- Perception: `/turret/yolo/segments`, `/turret/yolo/target_mask`,
  `/turret/yolo/target_bbox`, `/turret/yolo/target_confidence`
- Tracking: `/turret/target/pixel_error`, `/turret/target/angular_error`,
  `/turret/target/selected_id`, `/turret/target/locked`
- Output: `/turret/joint_commands`, `/turret/diagnostics`

The Arduino bridge uses `config/arduino.env` and
`config/arduino_bridge.yaml`. It forwards the shared joint-command contract to
the physical controller and republishes joint state and diagnostics.

The original package is based on
[adityakamath/pan_tilt_ros](https://github.com/adityakamath/pan_tilt_ros).
