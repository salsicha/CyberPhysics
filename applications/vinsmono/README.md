# VINS-Mono

This application packages the ROS 2 VINS-Mono feature tracker, visual-inertial
estimator, and optional pose graph in a generic Ubuntu/ROS 2 Jazzy container.
It has no Raspberry Pi or sensor-vendor-specific dependencies.

The default container command launches:

- `feature_tracker`, which subscribes to the `sensor_msgs/msg/Image` topic in
  the mounted calibration file
- `vins_estimator`, which consumes tracked features and the configured
  `sensor_msgs/msg/Imu` topic
- `pose_graph` when `enable_pose_graph:=true`

## Run with Compose

The example composition mounts calibration and topic names from
`systems/vinsmono/calibration/vins_mono.yaml`:

```bash
docker compose -f compositions/vinsmono.yaml up --build
```

By default, the example subscribes to `/camera/image_raw` and `/camera/imu` and
writes estimator output under `generated/vinsmono/`. Publish compatible camera
and IMU messages on the same ROS domain, or change those topic names in the
system calibration file.

The supplied intrinsic, IMU noise, and camera-to-IMU values are illustrative.
Replace them with measurements from the actual sensor before using odometry for
control or navigation.

## Run the image directly

```bash
docker run --rm --network host --ipc host \
  -e ROS_DOMAIN_ID=0 \
  -v "$PWD/systems/vinsmono/calibration/vins_mono.yaml:/etc/vinsmono/calibration.yaml:ro" \
  -v "$PWD/generated/vinsmono:/data/vinsmono" \
  cyberphysics/vinsmono:latest
```

Launch arguments can select a tracker-only process or enable loop closure:

```bash
ros2 launch vins_estimator vins_mono.launch.py \
  config_file:=/etc/vinsmono/calibration.yaml \
  enable_estimator:=false \
  enable_pose_graph:=false
```

The primary estimator output is `/vins_estimator/odometry`. Feature tracks are
published on `/feature_tracker/feature` and the annotated image stream on
`/feature_tracker/feature_img`.

Camera and IMU subscriptions accept best-effort sensor publishers. Setting
`enable_pose_graph:=true` also enables loop closure, including the estimator's
keyframe and extrinsic outputs. Pose graph input queues retain at most 100
messages each when a matching stream is unavailable.

Run the sensor and shutdown smoke checks without hardware:

The checks include SIGINT and SIGTERM for the optional pose graph with loop
closure both enabled and disabled, including an open stdin pipe with no input.

```bash
docker run --rm --network none -v "$PWD:/repo:ro" cyberphysics/vinsmono:latest \
  python3 /repo/applications/vinsmono/scripts/runtime_smoke_test.py \
  --config /repo/systems/vinsmono/calibration/vins_mono.yaml
```

The bundled ROS 2 port is based on
[dongbo19/VINS-MONO-ROS2](https://github.com/dongbo19/VINS-MONO-ROS2) and the
original [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).
