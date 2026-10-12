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
launch/script paths point at `/workspace/systems/so101`. The joint position
limits are read from `systems/so101/config/joint_limits.json` (their single
source of truth, see `systems/so101/README.md`), so every container that runs
these scripts, including the CBF controller and the HIL guard, needs that mount
(or `SO101_JOINT_LIMITS_FILE`); `so101_common.py` refuses to start without it.

## The scene

`systems/so101/scenarios/picking_table.json` is the one description of the
picking scene, and the one place its heights are written: the arm's spawn
pose, the pose and size of the table, the 31 mm riser (`arm_base_mount`), the
bins, objects, occluder and calibration board, and the fixed camera. Every
asset and object names the surface it stands on (`rests_on`), and
`so101_common.scene_support_errors` checks that it does: its underside on
that surface's top (the arm's underside is its spawn z, the URDF's root
link), nothing floating or sunk, nothing interpenetrating. So every simulator
starts the task from the same state as the printed station
(`hardware/parts/so101/`, built from the same file): Isaac keeps its objects
kinematic where the scenario puts them, Gazebo and Genesis simulate them as
free bodies, which stay put because they already rest on the table.

- Isaac builds the scene from the scenario at start-up (`so101_task_scene.py`).
- Gazebo loads `systems/so101/worlds/picking_table.sdf`, generated from the
  scenario: after editing the scenario, run
  `python3 applications/so101/scripts/scenario_world.py` (`--check` reports a
  stale world; the tests run it). `gazebo.launch.py` spawns the arm at the
  scenario's `robot_spawn` (`scenario_file` launch argument).
- Genesis builds the scene from the scenario (`--scenario`).
- `validate_calibration_and_dataset.py` and the simulators report an
  inconsistent scenario.

## Gazebo Harmonic

```bash
docker compose -f compositions/so101_gazebo.yaml up
```

Gazebo uses `gz_ros2_control` and a joint-group position controller. The default world is `/workspace/systems/so101/worlds/picking_table.sdf`, generated from `/workspace/systems/so101/scenarios/picking_table.json` (see "The scene"). The relay
maps the common command topic to the Gazebo controller topic. The world's RGB-D
sensor is the scenario camera; `gazebo.launch.py` bridges it to
`/so101/camera/image_raw`, `/so101/camera/camera_info`,
`/so101/camera/depth/image_rect_raw` and `/so101/camera/depth/camera_info`
(frame `groot_camera_rgb_optical_frame`; the depth image is registered to the
colour image). With `headless:=true` the server renders with
`--headless-rendering` (EGL), so the camera works without a display.

## Genesis

Build the SO-101 controller image and the Genesis image, then run:

```bash
make -C applications build_so101 build_genesis
docker compose -f compositions/so101_genesis.yaml up
```

Use `--headless` by appending it to the Genesis service command for server runs
(with `PYOPENGL_PLATFORM=egl` so the camera renders without a display).
Use `--backend cpu` on machines without a compatible GPU. The adapter loads
`systems/so101/urdf/so101.urdf` as a fixed-base robot at the scenario's spawn
pose, builds the scene from `--scenario` (fixed boxes for the table, riser,
bins, occluder and board; the objects as free bodies with the scenario's mass
and friction), applies PD position control at each simulation step, keeps
simulated time at wall-clock rate, and renders the scenario camera to the
common camera topics at its frame rate (`--no-camera` to skip it). It prints
the objects' positions every `SO101_LOG_HEARTBEAT_SECONDS` (30 s).

## Isaac Sim

The composition is pinned to Isaac Sim 6.0.1:

```bash
docker compose -f compositions/so101_isaacsim.yaml up
```

The standalone script uses Isaac Sim's base Python experience for headless runs,
enables the URDF importer and Isaac ROS 2 bridge, imports
`systems/so101/urdf/so101.urdf` as a fixed-base USD articulation, drives
articulation position targets, and publishes joint states plus a simulated RGB
camera. The Compose service configures the ROS 2 Jazzy bridge from
`isaacsim.ros2.core`, bypasses the container's streaming entrypoint, and keeps
6.0.1 caches separate from older releases.

Isaac Sim 6.0.1 runs as UID/GID 1234 in the container. The Compose files run a
network-disabled `isaacsim_cache_init` dependency first so newly created cache
mounts are automatically writable by that user. If you deliberately launch the
Isaac service with `--no-deps`, initialize the cache explicitly first:

```bash
docker compose -f compositions/so101_isaacsim.yaml run --rm isaacsim_cache_init
```

The GR00T composition instead defaults
`CYBERPHYSICS_ISAAC_CACHE` to
`/tmp/cyberphysics/isaac-sim/6.0.1/so101`; its `sim` profile initializes that
directory automatically, or the variable can point at another cache location.

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

## SO-101 + GR00T Inference

Build the reusable SO-101 and GR00T images, then select the simulation profile:

```bash
make -C applications build_so101 build_groot
docker compose -f compositions/so101_groot_isaac.yaml --profile sim up
```

The stack defaults to `SO101_GROOT_MODE=real` and expects a fine-tuned SO-100
or SO-101 `NEW_EMBODIMENT` checkpoint at
`data/groot/models/so101-checkpoint/`. The GR00T runtime is installed in the
container image, but model weights are deliberately stored outside the image.
The base `nvidia/GR00T-N1.7-3B` checkpoint cannot control SO-101 zero-shot:
SO-101 requires a checkpoint fine-tuned with its exact camera, joint-unit, and
action schema.

GR00T N1.7 also loads the gated `nvidia/Cosmos-Reason2-2B` backbone. Request
access on Hugging Face, then either export `HF_TOKEN` before starting Compose
or authenticate once into the mounted cache:

```bash
docker compose -f compositions/so101_groot_isaac.yaml \
  --profile sim run --rm --no-deps groot_policy hf auth login
```

Install or copy a fine-tuned checkpoint and start inference:

```bash
mkdir -p data/groot/models/so101-checkpoint
# Copy config.json, processor_config.json/statistics, and model weights here.

GR00T_MODEL_PATH=/workspace/data/models/so101-checkpoint \
GR00T_EMBODIMENT_TAG=NEW_EMBODIMENT \
docker compose -f compositions/so101_groot_isaac.yaml --profile sim up
```

The bridge queries the loaded policy's modality configuration. It supports the
official SO-101 `front`/`wrist`, `single_arm`/`gripper` schema and the
repository's legacy single-vector schema. If a checkpoint expects a wrist camera
but no `wrist_camera_topic` is configured, the bridge warns and duplicates the
front frame; provide the camera stream used during training for meaningful
inference.

For interface and simulator validation without model weights, explicitly select
the deterministic baseline:

```bash
SO101_GROOT_MODE=demo \
docker compose -f compositions/so101_groot_isaac.yaml --profile sim up
```

That baseline plans IK pick, lift, transfer, place, and retreat waypoints from
the scenario's object and bin poses (simulator ground truth, read from the
server's `--scenario-file`) and the measured joint state. It does not perceive
the scene, so its runs are reported with `policy_used_ground_truth: true` and
never count as a policy success. The bridge only forwards object poses to a
policy when started with `-p send_ground_truth:=true`, which also marks every
command as ground-truth assisted. Because the objects stand on the table, the
plan turns the wrist a quarter turn (the jaws open sideways) and closes the
jaws about 19 mm above the block's centre, so the 75 mm jaws stay 5 mm clear
of the table; the block keeps that offset while held and is released
standing on the bin.

Bridge timing and inputs:

- `command_rate_hz` (default 5, the `convert_rosbags_to_lerobot.py --fps`
  default) must equal the rate of the policy's training data; chunked policies
  replay their queued actions at this rate. The SmolVLA stack uses
  `SMOLVLA_POLICY_FPS` (default 30). `max_joint_step` is per command.
- Camera frames are sent at native resolution (`image_width`/`image_height`
  `0`), as the training videos were recorded, so the policy's own processor
  resizes them. Positive values letterbox (aspect-preserving) instead.
  The camera is 640x360 (see "Camera" below) in every simulator and on the
  hardware (`rgb_camera.color_profile:=640x360x30`).

### Camera

The scenario camera models the RealSense D435i's colour stream, the camera of
the physical station: 16:9, 69.4 deg horizontal and 42.5 deg vertical view,
published at 640x360 and 30 fps (CameraInfo fx = fy = 462.1 px, centred
principal point; depth registered to it). It is pitched 48.5 deg down, the
pitch the printed camera head is trimmed to (the station's assembly check
recommends it: the bins, objects, occluder, riser and arm base all lie within
88 % of the half-image), at the RGB centre the printed station puts there
(`hardware/parts/so101/`). The earlier simulated camera was 640x480 at
69 deg (54.5 deg vertical) and 45 deg pitch, a view the real camera cannot
produce: from 45 deg the D435i cuts off the near edge of the right bin.

Policies trained on the old 640x480 simulated camera see a different image:

- the aspect ratio is 16:9 instead of 4:3. The bridge sends native frames, so
  the policy's processor resizes them: SmolVLA pads to a square (a 640x360
  frame fills 512x288 of its 512x512 input instead of 512x384); a GR00T
  processor that resizes or crops to a square without padding stretches the
  scene differently;
- the view is narrower vertically (42.5 instead of 54.5 deg) and pitched
  3.5 deg further down: the new image is old rows 91-455, objects sit about
  28 px higher than in the old frame, and the far strip of the old view
  (behind the left bin) is gone. Horizontally the views match (69.4 vs
  69 deg).

Retrain or fine-tune on data recorded with the new camera (simulated or the
real D435i, which now give the same image size and field of view). To reuse
an old dataset, warp its frames into the new camera with the homography of
the pitch change, `H = K_old R_x(3.5 deg) K_new^-1` (the 3 mm move of the
lens is negligible), or approximately crop rows 90-453 of the 640x480 frame
and resize to 640x360 (about 4 px off on average, 11 px at the bottom
corners, which lie just outside the old image). An old checkpoint cannot be
given the view it was trained on: what the new camera does not see is lost.
Padding new frames into a 640x480 canvas at rows 91-455 keeps the objects
roughly where the checkpoint saw them, but expect lower success than on its
training data.
- The policy server is reset before the first request, after joint feedback
  resumes from a gap longer than `episode_reset_gap_s` (for example a simulator
  restart) or becomes invalid, and when `task_id`/`instruction` is changed with
  `ros2 param set`, so no stale queued actions run in a new episode.
- Each command's policy latency, observation age, step/limit saturation and
  ground-truth use are published as JSON on `/so101/policy_stats`; the Isaac
  task scene folds them into the metrics file.

Isaac now materializes the table, bins, target, distractors, and camera from
`systems/so101/scenarios/picking_table.json`. Its `camera` block is the single
camera definition: `xyz`/`rpy` are the pose of `groot_camera_link` in the world
(+X optical axis, +Y left, +Z up, Isaac's `camera_axes="world"`), CameraInfo
uses `fx = fy = (width/2)/tan(horizontal_fov_deg/2)` (462.1 px for 69.4 degrees at
640 px), and the URDF `base_to_groot_camera` joint must equal that pose relative
to `base_link`. The calibration validator and tests enforce all three. For deterministic demo behavior it
uses kinematic grasp attachment at the jaws (the target keeps the offset from
the jaw centre it had when they closed), publishes JSON task progress on
`/so101/task_status`, and writes these host files:

- `data/groot/telemetry/so101_task_telemetry.json`
- `data/groot/telemetry/so101_task_telemetry_metrics.json`

On a Linux X11 host, the simulator opens the Isaac window by default:

```bash
docker compose \
  -f compositions/so101_groot_isaac.yaml --profile sim up --force-recreate
```

Set `SO101_HEADLESS=true` for a remote or display-less run. Once the first
physics/render/publish step completes, the service becomes healthy and prints a
heartbeat every 30 seconds. Set `SO101_LOG_HEARTBEAT_SECONDS=0` to disable it.

The network-disabled `isaacsim_xauth_init` service copies the current Xauthority
cookie into a private Docker volume readable by Isaac Sim's UID 1234. This avoids
granting broad display access with `xhost`. Windowed runs automatically select
`isaacsim.exp.full.kit`; headless runs use `isaacsim.exp.base.python.kit`. Override
either choice with `SO101_ISAAC_EXPERIENCE` if needed. The GUI opens at `(0,0)`
with a 1440x900 window. Multi-monitor hosts can set `SO101_ISAAC_WINDOW_X`,
`SO101_ISAAC_WINDOW_Y`, `SO101_ISAAC_WINDOW_WIDTH`, and
`SO101_ISAAC_WINDOW_HEIGHT`. The launcher detects GNOME's truncated desktop-wide
work area, expands it only while Kit creates the startup window, and then restores
the original value. Set `SO101_ISAAC_X11_WORKAREA_FIX=false` to disable this guard.
After initialization, the arm's force position drive is assigned stiffness `400`
and damping `40` directly through the articulation controller, preventing gravity
sag at extended grasp poses. Advanced tuning is available through
`SO101_ISAAC_JOINT_DRIVE_TYPE`, `SO101_ISAAC_JOINT_STIFFNESS`, and
`SO101_ISAAC_JOINT_DAMPING`.
To inspect camera, joints, commands, and task status
in Foxglove:

```bash
docker compose -f compositions/so101_groot_isaac.yaml \
  --profile sim --profile observe up
foxglove-studio
```

Connect Foxglove to Rosbridge at `ws://localhost:9090`.

Set `SO101_TASK_ID=pick_green_block_right_bin` to run the second scenario task.

NVIDIA specifies at least 16 GiB GPU VRAM for N1.7 inference alone. Running
Isaac Sim and GR00T on the same GPU needs additional headroom. To use a policy
server on another machine, expose its TCP port and run only Isaac plus the
bridge locally:

```bash
SO101_GROOT_HOST=192.0.2.10 \
docker compose -f compositions/so101_groot_isaac.yaml \
  --profile sim up --scale groot_policy=0
```

### Physical station (`so101_hardware.yaml`)

The `policy` profile of `compositions/so101_hardware.yaml` routes the bridge's
commands to `/so101/hil/policy_commands`; `so101_hil_guard` forwards them to the
driver's `${SO101_HARDWARE_COMMAND_TOPIC:-/so101/joint_commands}` only when
started with `SO101_HIL_ARMED=true` and the driver's
`${SO101_HARDWARE_STATE_TOPIC:-/joint_states}` is fresher than
`SO101_HIL_STATE_TIMEOUT_S` (0.5 s). It starts disarmed:

```bash
docker compose -f compositions/so101_hardware.yaml --profile policy up
SO101_HIL_ARMED=true docker compose -f compositions/so101_hardware.yaml --profile policy up
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

The 6.0.1 pin replaces the unsupported 5.1 runtime that crashed in
`librtx.scenedb.plugin.so` with R595 drivers. The lightweight ROS demo remains
useful for transport/interface tests on hosts that do not satisfy Isaac Sim's
GPU, VRAM, or driver requirements.

## Calibration And Dataset Checks

Validate the scene (every asset and object resting on its surface, nothing
interpenetrating), the scenario camera, URDF camera/gripper alignment, and
collision geometry before collecting policy data:

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

The policy bridge rejects non-finite actions and incomplete, non-finite, or
out-of-range joint feedback. Invalid feedback blocks new commands until a valid
six-joint state arrives. Valid targets use the same limit and step checks as the
hardware guard.

Score scenario telemetry against the selected SO-101 picking task:

```bash
python3 applications/so101/scripts/score_picking_task.py \
  --scenario systems/so101/scenarios/picking_table.json \
  --telemetry /tmp/so101_pick_run.json \
  --task-id pick_red_block_left_bin
```

Telemetry JSON should include `duration_s`, `collisions` (a list, or `null` when
the simulator cannot detect contacts), `policy_used_ground_truth` (`null` when
unknown), optional `failed_grasps`, `joint_limit_violations`, `policy_commands`
(bridge report counts, saturation, maximum latency and observation age), and a
`samples` array with `time`, `end_effector_xyz`, `object_xyz`, and
`gripper_width_m`. The scorer fails if the policy uses simulator ground truth, if
the arm never approaches and closes on the target, if the object is not lifted
and placed within task tolerance, if collisions exceed the scenario limit, or if
recovery after a failed grasp does not complete within the configured window.
Checks that were not measured are listed in `unmeasured_checks` and make
`success` false; `task_completed` reports the physical outcome alone. The Isaac
scene does not register contact reporting, so its collisions are always
unmeasured, and the scripted demo always reports ground-truth use. Metrics are
also written when a run ends unsolved, including on `docker compose down`
(`run_isaacsim_so101.sh` forwards SIGTERM past Isaac's `python.sh`).

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

Each acceptance run must include measured `collision_count`,
`joint_limit_violations`, `command_saturation_fraction`, `policy_latency_ms`, and
`observation_age_ms`, along with placement error and a boolean
`failed_grasp_recovery` result (at the top level or under `checks`). The picking
scorer reports joint-limit violations measured by the simulator and the bridge's
latency, observation-age and saturation reports; it reports `collision_count`
as `null` when collisions were not measured. Missing or invalid
measurements produce failed checks, `null` aggregate metrics, and explanations
in `measurement_errors`; they cannot qualify a run for acceptance.

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
