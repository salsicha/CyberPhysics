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

## SO-101 + GR00T Task-Solving Demo

Build the reusable SO-101 and GR00T images, then select the simulation profile:

```bash
make -C applications build_so101 build_groot
docker compose -f compositions/so101_groot_isaac.yaml --profile sim up
```

The default `SO101_GROOT_MODE=demo` policy is a deterministic, GR00T-compatible
benchmark baseline. It uses the fixed task specification and measured joint state
to execute IK-generated pick, lift, transfer, place, and retreat waypoints. It does
not read live simulator object poses. Use this mode to verify the complete system;
use `real` mode for visual-policy evaluation.

Isaac now materializes the table, bins, target, distractors, and camera from
`systems/so101/scenarios/picking_table.json`. For deterministic demo behavior it
uses kinematic grasp attachment at the jaws, publishes JSON task progress on
`/so101/task_status`, and writes these host files:

- `data/groot/so101_task_telemetry.json`
- `data/groot/so101_task_telemetry_metrics.json`

The simulator is headless by default. On a Linux X11 host, show the Isaac window with:

```bash
xhost +si:localuser:root
SO101_HEADLESS=false docker compose \
  -f compositions/so101_groot_isaac.yaml --profile sim up
xhost -si:localuser:root
```

The final `xhost` command revokes the temporary local-root display permission after
the stack stops. To inspect camera, joints, commands, and task status in Foxglove:

```bash
docker compose -f compositions/so101_groot_isaac.yaml \
  --profile sim --profile observe up
foxglove-studio
```

Connect Foxglove to Rosbridge at `ws://localhost:9090`.

Set `SO101_TASK_ID=pick_green_block_right_bin` to run the second scenario task.

To use a fine-tuned SO-101 GR00T checkpoint stored under `data/groot/models/`:

```bash
SO101_GROOT_MODE=real \
GR00T_MODEL_PATH=/workspace/data/models/so101-checkpoint \
GR00T_EMBODIMENT_TAG=so101 \
docker compose -f compositions/so101_groot_isaac.yaml --profile sim up
```

### Optional Hardware In The Loop

The `hil` profile replaces Isaac and its simulation bridge with a RealSense camera,
hardware-topic bridge, and a safety gate. A physical motor driver must run separately,
publish `/so101/hardware/joint_states`, and subscribe to
`/so101/hardware/joint_commands`. Start HIL disarmed first:

```bash
docker compose -f compositions/so101_groot_isaac.yaml --profile hil up
```

The gate rejects every command while disarmed and also rejects commands when hardware
state is missing or older than 0.5 seconds. After checking the physical workspace,
driver, joint directions, limits, and emergency stop, explicitly arm a new run:

```bash
SO101_HIL_ARMED=true \
docker compose -f compositions/so101_groot_isaac.yaml --profile hil up
```

Override `SO101_HARDWARE_STATE_TOPIC`, `SO101_HARDWARE_COMMAND_TOPIC`, or the camera
topic variables when the physical driver uses different names. The HIL gate additionally
clips each requested joint step using `SO101_HIL_MAX_JOINT_STEP` (default `0.04`).

Isaac Sim 5.1 previously reached initialization but crashed in
`librtx.scenedb.plugin.so` on one tested host. The lightweight ROS demo remains useful
for transport/interface tests when that renderer-specific failure occurs.

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
