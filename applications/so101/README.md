# SO-101 simulation

This package provides one SO-101 description and adapters for Gazebo Harmonic,
Genesis, NVIDIA Isaac Sim, and a lightweight ROS-only simulator. All backends
expose the same ROS 2 interface:

- State: `sensor_msgs/msg/JointState` on `/joint_states`
- Position command: `std_msgs/msg/Float64MultiArray` on `/so101/joint_commands`
- Simulated RGB camera: `sensor_msgs/msg/Image` on `/so101/camera/image_raw`
- Simulated depth camera: `sensor_msgs/msg/Image` on `/so101/camera/depth/image_rect_raw`
- Camera calibration: `sensor_msgs/msg/CameraInfo` on `/so101/camera/camera_info` and `/so101/camera/depth/camera_info`
- Camera IMU: `sensor_msgs/msg/Imu` on `/so101/camera/imu`
- Joint order: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`,
  `wrist_roll`, `gripper`

The model uses measured-scale primitive geometry and conservative inertial
estimates. It is appropriate for controller integration, joint-limit testing,
ROS graph validation, and initial policy evaluation. Replace the primitives and
inertias with production CAD and identified parameters before using contact
results or sim-to-real metrics as hardware predictions.

## Build

From the repository root:

```bash
make -C applications build_so101
```

The SO-101 hardware-specific description and simulator configuration live in
`systems/so101/`. The application image contains only the reusable ROS package
and adapter scripts. After editing `systems/so101/urdf/so101.urdf.xacro`,
regenerate the plain URDF in an environment with ROS 2 Xacro installed:

```bash
applications/so101/scripts/regenerate_urdf.sh
```

The compose files mount `systems/` at `/workspace/systems`, and the default
launch/script paths point at `/workspace/systems/so101`.

## Gazebo Harmonic

```bash
docker compose -f compositions/so101_gazebo.yaml up
```

Gazebo uses `gz_ros2_control` and a joint-group position controller. The default world is `/workspace/systems/so101/worlds/picking_table.sdf`, a tabletop pick-and-place scene mirrored by `/workspace/systems/so101/scenarios/picking_table.json`. The relay
maps the common command topic to the Gazebo controller topic.

## Genesis

Build the SO-101 controller image and the Genesis image, then run:

```bash
make -C applications build_so101 build_genesis
docker compose -f compositions/so101_genesis.yaml up
```

Use `--headless` by appending it to the Genesis service command for server runs.
Use `--backend cpu` on machines without a compatible GPU. The adapter loads
`systems/so101/urdf/so101.urdf` as a fixed-base robot and applies PD position
control at each simulation step.

## Isaac Sim

The composition uses the repository's Isaac Sim 5.1.0 image selection:

```bash
docker compose -f compositions/so101_isaacsim.yaml up
```

The standalone script uses Isaac Sim's base Python experience for headless runs,
enables the URDF importer and Isaac ROS 2 bridge, imports
`systems/so101/urdf/so101.urdf` as a fixed-base USD articulation, drives
articulation position targets, and publishes joint states plus a simulated RGB
camera. NVIDIA marks Isaac Sim 5.1 as unsupported, so validate the importer API
before changing the pinned image version.

## SO-101 + GR00T ROS Demo

This demo uses the same SO-101 ROS topics and GR00T bridge as the Isaac stack,
but runs a lightweight ROS simulator for environments where Isaac Sim cannot
start its RTX renderer:

```bash
make -C applications build_so101 build_groot
docker compose -f compositions/so101_groot_demo.yaml up
```

It publishes `/joint_states`, `/so101/camera/image_raw`, `/so101/camera/camera_info`, `/so101/camera/depth/image_rect_raw`, `/so101/camera/depth/camera_info`, `/so101/camera/imu`, and calibration TF, consumes `/so101/joint_commands`, and drives those
commands from the GR00T policy server through the bridge. The compose file sets
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` so ROS samples flow reliably between Docker
host-network containers.

## SO-101 + GR00T in Isaac Sim

The Isaac GR00T demo runs Isaac Sim headless, publishes SO-101 joint state plus
a simulated RGB camera stream, starts a GR00T-compatible policy server, and
bridges policy actions back to `/so101/joint_commands`:

```bash
make -C applications build_so101 build_groot
docker compose -f compositions/so101_groot_isaac.yaml up
```

By default the policy service uses `SO101_GROOT_MODE=mock`. This gives a
repeatable end-to-end demo without storing or downloading a finetuned SO-101
checkpoint. To use a real GR00T policy, provide a checkpoint that has been
finetuned for the SO-101 embodiment:

```bash
SO101_GROOT_MODE=real \
GR00T_MODEL_PATH=/models/so101-gr00t-checkpoint \
GR00T_EMBODIMENT_TAG=so101 \
docker compose -f compositions/so101_groot_isaac.yaml up
```

The Isaac adapter loads `/workspace/systems/so101/scenarios/picking_table.json`,
creates a rendered camera from its camera pose, and publishes RGB, depth, camera
info, IMU, and joint state topics. Set `SO101_ISAAC_CAMERA_SOURCE=synthetic` to
force deterministic generated images when the Isaac camera sensor cannot start.

The bridge reads `/so101/camera/image_raw`, optional depth, camera calibration,
and `/joint_states`, loads the selected task language from the scenario file,
sends joint history and observation metadata to the policy server, clips returned
joint targets to the SO-101 URDF limits, and publishes
`std_msgs/msg/Float64MultiArray` commands. Camera, scenario, and bridge
parameters live in `systems/so101/config/groot_demo.yaml`.

On this host, bounded startup testing reached Isaac Sim initialization but Isaac
Sim 5.1 crashed in `librtx.scenedb.plugin.so` before the SO-101 script could run.
The ROS demo above exercises the SO-101 camera, GR00T policy server, and bridge
path without that renderer dependency.

## Calibration And Dataset Checks

Validate the scenario camera, URDF camera/gripper alignment, and collision
geometry before collecting policy data:

```bash
PYTHONPATH=applications/so101/scripts \
python3 applications/so101/scripts/validate_calibration_and_dataset.py \
  --scenario systems/so101/scenarios/picking_table.json \
  --urdf systems/so101/urdf/so101.urdf
```

Record a short GR00T regression bag from a running SO-101 demo:

```bash
ros2 run so101_description record_groot_demo_bag.sh /tmp/so101_groot_demo_bag
```

After extracting topic timestamp arrays from the bag, validate synchronization
with the same checker using a manifest shaped like
`systems/so101/validation/groot_demo_sync_manifest.example.json`. The checker
fails if image, depth, camera_info, joint state, and command streams exceed the
configured skew budget.

## Picking Task Validation

Score scenario telemetry against the selected SO-101 picking task:

```bash
python3 applications/so101/scripts/score_picking_task.py \
  --scenario systems/so101/scenarios/picking_table.json \
  --telemetry /tmp/so101_pick_run.json \
  --task-id pick_red_block_left_bin
```

Telemetry JSON should include `duration_s`, optional `collisions`, optional
`failed_grasps`, and a `samples` array with `time`, `end_effector_xyz`,
`object_xyz`, and `gripper_width_m`. The scorer fails if the policy uses
simulator ground truth, if the arm never approaches and closes on the target, if
the object is not lifted and placed within task tolerance, if collisions exceed
the scenario limit, or if recovery after a failed grasp does not complete within
the configured window.

Generate deterministic domain-randomized scenario variants for nightly
validation:

```bash
python3 applications/so101/scripts/score_picking_task.py \
  --scenario systems/so101/scenarios/picking_table.json \
  --write-randomized-scenarios /tmp/so101_randomized \
  --num-randomizations 25 \
  --seed 101
```

## Acceptance Report

Aggregate scored SO-101 runs into the acceptance metrics from
`systems/so101/validation/acceptance_thresholds.json`:

```bash
python3 applications/so101/scripts/so101_acceptance_report.py \
  --metrics /tmp/so101_metrics/*.json \
  --thresholds systems/so101/validation/acceptance_thresholds.json
```

The report checks success rate by object class and clutter level, mean and worst
place error, total collisions, joint-limit violations, command saturation,
failed-grasp recovery rate, policy latency, and observation age.

## Manual motion test

With any backend running, publish one six-joint target:

```bash
ros2 topic pub --once /so101/joint_commands std_msgs/msg/Float64MultiArray \
  "{data: [0.4, -0.5, 0.8, -0.3, 0.5, 0.02]}"
```

Inspect feedback with:

```bash
ros2 topic echo /joint_states
```

The optional controller service in each composition returns the arm toward its
configured `goal` while keeping commands inside a margin from the URDF limits.
