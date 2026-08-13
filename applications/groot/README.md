# NVIDIA Isaac GR00T N1.7 for SO-101

This application contains a containerized, fail-closed workflow for downloading,
fine-tuning, evaluating, and deploying nvidia/GR00T-N1.7-3B with the SO-101
stack in compositions/so101_groot_isaac.yaml.

The base model is not an SO-101 controller. SO-101 is trained as
NEW_EMBODIMENT, and Compose must load the resulting numbered checkpoint.

## What the workflow enforces

The Isaac composition and every prepared dataset use this contract:

- Camera: one RGB stream named front.
- Joint order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
  wrist_roll, gripper.
- Arm state/action units: radians.
- Gripper state/action unit: opening in metres, from 0.0 closed to 0.04 fully
  open.
- Arm action representation: relative, configured by GR00T during training.
- Gripper action representation: absolute.
- Language key: annotation.human.task_description.
- Training embodiment: NEW_EMBODIMENT.

validate_so101_dataset.py requires meta/so101_units.json. Training will not start
merely because a dataset happens to contain six numbers; its schema and unit
contract must be explicit.

## Scripts

Run the shell wrappers from the CyberPhysics repository root.

| Script | Purpose |
| --- | --- |
| check_prerequisites.sh | Check GPU, Docker, Compose, disk, mounts, and image |
| download_models.sh | Download N1.7 and pre-cache its gated backbone |
| record_so101_bag.sh | Record one ROS 2 bag for one successful Isaac episode |
| convert_bags.sh | Convert Isaac bags to GR00T LeRobot v2 |
| prepare_dataset.sh | Convert/normalize a LeRobot dataset for Isaac units |
| validate_so101_dataset.py | Validate metadata, videos, vectors, units, and alignment |
| train_so101.sh | Validate and fine-tune NEW_EMBODIMENT |
| evaluate_so101.sh | Run open-loop evaluation |
| open_loop_eval_so101.py | Preserve one plot per trajectory while loading the model once |
| run_so101_isaac.sh | Run a numbered checkpoint in Isaac |
| normalize_so101_dataset.py | Low-level transactional unit converter |
| convert_rosbags_to_lerobot.py | Low-level ROS-bag converter |
| workflow_common.sh | Shared path-containment and Compose helpers |
| smoke_test.py | Container import and CUDA smoke test used at image startup |
| policy_healthcheck.py | Policy-server readiness probe |
| so101_policy_server.py | GR00T inference server used by the Isaac composition |
| so101_task_policy.py | Isaac-side SO-101 policy client and safety adapter |

All mutable artifacts remain outside the application image:

~~~text
data/groot/
├── bags/
├── datasets/
├── models/
├── telemetry/
└── training/
cache/groot/
~~~

Host paths below data/groot appear inside the GR00T container below
/workspace/data.

## 1. Hardware and software

NVIDIA recommends at least one 40 GiB GPU for default fine-tuning. Inference
alone requires at least 16 GiB, while concurrent Isaac Sim and policy inference
need additional headroom. The default fine-tune trains the projector and
diffusion action head; tuning the visual encoder or LLM requires substantially
more memory.

Required host components:

- Linux with an NVIDIA driver compatible with CUDA 12.8.
- Docker Engine and Docker Compose v2.
- NVIDIA Container Toolkit.
- Approximately 500 GB of SSD space for repeated datasets and checkpoints.
- Access to the gated nvidia/Cosmos-Reason2-2B model on Hugging Face.

Before building the image:

~~~bash
applications/groot/scripts/check_prerequisites.sh --skip-image
~~~

Use --min-vram-gib 16 when checking an inference-only machine.

## 2. Build

The image clones upstream NVIDIA/Isaac-GR00T recursively and installs its locked
environment. uv selects the Python version required by the selected upstream
ref.

~~~bash
make -C applications build_groot
make -C applications build_so101
~~~

To pin upstream explicitly:

~~~bash
docker build \
  --build-arg GR00T_REF=main \
  -t cyberphysics/groot:latest \
  applications/groot
~~~

Run the complete check after building:

~~~bash
applications/groot/scripts/check_prerequisites.sh
~~~

## 3. Authenticate with Hugging Face

First request access to
[nvidia/Cosmos-Reason2-2B](https://huggingface.co/nvidia/Cosmos-Reason2-2B).
Then store a read token in the persistent cache/groot mount:

~~~bash
docker compose \
  -f compositions/so101_groot_isaac.yaml \
  --profile sim \
  run --rm --no-deps groot_policy hf auth login
~~~

Do not put the token in a committed .env file. A process-scoped HF_TOKEN also
works, but the cache login is simpler for repeated container runs.

## 4. Download the model

~~~bash
applications/groot/scripts/download_models.sh
~~~

This creates data/groot/models/GR00T-N1.7-3B and pre-caches the gated Cosmos
backbone in cache/groot/huggingface. Both downloads are resumable. Use
--skip-backbone only when it is already cached.

## 5. Prepare demonstrations

There are two supported paths. Choose the one matching the intended deployment
camera and units. Mixing physical and simulated datasets requires transforming
both into exactly the same contract before using GR00T's multi-dataset support.

### Path A: Isaac ROS 2 bags

One bag represents one successful episode. The converter synchronizes rendered
RGB, measured joint state, and commanded joint targets, samples them at 5 Hz by
default, and rejects excessive timestamp skew.

For scripted pipeline validation, start Isaac and the demo policy without the
control bridge:

~~~bash
SO101_GROOT_MODE=demo SO101_HEADLESS=false \
docker compose \
  -f compositions/so101_groot_isaac.yaml \
  --profile sim up -d isaacsim groot_policy
~~~

In terminal 2, start recording before motion:

~~~bash
applications/groot/scripts/record_so101_bag.sh --name red_left_000
~~~

In terminal 3, start the bridge:

~~~bash
SO101_GROOT_MODE=demo \
docker compose \
  -f compositions/so101_groot_isaac.yaml \
  --profile sim up so101_groot_bridge
~~~

Stop the recorder with Ctrl-C after the successful episode. Recreate/reset the
simulation and repeat with unique bag names. Do not include failed, interrupted,
or partially reset episodes in the training command.

The deterministic demo is useful for testing the toolchain, not for producing a
general policy. A useful policy needs varied successful demonstrations: object
positions, initial arm poses, distractors, lighting, occlusion, recovery
behavior, and task wording must cover the intended evaluation distribution.

Convert all successful bags:

~~~bash
applications/groot/scripts/convert_bags.sh \
  --bag data/groot/bags/red_left_000 \
  --bag data/groot/bags/red_left_001 \
  --bag data/groot/bags/red_left_002 \
  --output-dataset data/groot/datasets/so101_isaac_red_left \
  --instruction "pick the red block and place it in the left bin"
~~~

The output is already in radians/metres and is validated automatically.

### Path B: LeRobot SO-101 data

Collect demonstrations with LeRobot using camera names and viewpoints that will
exist at deployment. Current LeRobot SO-101 data normally stores the five arm
joints in degrees and the gripper in its calibrated actuator scale. GR00T
requires LeRobot v2, while current recording may produce v3; the preparation
wrapper handles the v3-to-v2 conversion.

Determine the actual raw gripper values at fully closed and at the opening that
must map to 0.04 m. Do not assume 0 and 100, and do not infer direction from a
dataset maximum.

Example:

~~~bash
applications/groot/scripts/prepare_dataset.sh \
  --repo-id YOUR_HF_USER/so101_pick_red_block \
  --input-units degrees \
  --gripper-closed-value YOUR_CLOSED_VALUE \
  --gripper-open-value YOUR_OPEN_VALUE \
  --output-dataset data/groot/datasets/so101_pick_red_block_isaac
~~~

For an existing LeRobot v2 directory already below data/groot:

~~~bash
applications/groot/scripts/prepare_dataset.sh \
  --source-dataset data/groot/datasets/raw_v2 \
  --input-units degrees \
  --gripper-closed-value YOUR_CLOSED_VALUE \
  --gripper-open-value YOUR_OPEN_VALUE \
  --output-dataset data/groot/datasets/so101_pick_red_block_isaac
~~~

Use --input-units isaac only when the source already uses radians and a
0.0-0.04 m gripper opening. The normalizer writes a new dataset transactionally;
it never rewrites the source dataset.

The included modality config uses only front. Extra wrist videos may remain in
info.json, but they are not loaded. To train and deploy two cameras, add a wrist
camera to Isaac, update both files under config/, and configure
wrist_camera_topic in the ROS bridge.

## 6. Validate independently

Preparation scripts validate automatically. The same strict check can be run
again:

~~~bash
docker compose \
  -f compositions/so101_groot_isaac.yaml \
  --profile sim \
  run --rm --no-deps groot_policy \
  python /workspace/groot_scripts/validate_so101_dataset.py \
    --dataset-path /workspace/data/datasets/so101_pick_red_block_isaac
~~~

Inspect episodes visually before training. Validation catches structural and
numeric errors, not blurred images, poor demonstrations, target leakage, or
inconsistent task execution.

## 7. Run a training smoke test

Always prove the full data/model/checkpoint path with a small disposable run:

~~~bash
applications/groot/scripts/train_so101.sh \
  --dataset data/groot/datasets/so101_pick_red_block_isaac \
  --output-dir data/groot/training/so101_n17_smoke \
  --max-steps 200 \
  --save-steps 200
~~~

The wrapper performs dataset validation first, loads
config/so101_isaac_config.py, and invokes upstream examples/finetune.sh.

A successful smoke test must:

- Load every selected camera stream.
- Report single_arm and gripper state/action modalities.
- Download/load the gated backbone.
- Complete forward and backward passes.
- Write checkpoint-200.

## 8. Fine-tune

Single GPU:

~~~bash
applications/groot/scripts/train_so101.sh \
  --dataset data/groot/datasets/so101_pick_red_block_isaac \
  --output-dir data/groot/training/so101_n17 \
  --num-gpus 1 \
  --max-steps 10000 \
  --save-steps 1000 \
  --global-batch-size 32 \
  --state-dropout-prob 0.2
~~~

Multi-GPU:

~~~bash
applications/groot/scripts/train_so101.sh \
  --dataset data/groot/datasets/so101_pick_red_block_isaac \
  --output-dir data/groot/training/so101_n17_8gpu \
  --num-gpus 8 \
  --max-steps 10000 \
  --global-batch-size 256
~~~

The upstream launcher switches to torchrun automatically. Add --use-wandb after
authenticating Weights & Biases. Pass advanced upstream arguments after --, for
example:

~~~bash
applications/groot/scripts/train_so101.sh \
  --dataset data/groot/datasets/so101_pick_red_block_isaac \
  --output-dir data/groot/training/so101_n17_eval \
  --max-steps 10000 \
  -- --eval-strategy steps --eval-steps 500
~~~

Resume the latest checkpoint in an existing output directory:

~~~bash
applications/groot/scripts/train_so101.sh \
  --dataset data/groot/datasets/so101_pick_red_block_isaac \
  --output-dir data/groot/training/so101_n17 \
  --resume
~~~

## 9. Open-loop evaluation

Evaluate held-out trajectories, not only training episodes:

~~~bash
applications/groot/scripts/evaluate_so101.sh \
  --dataset data/groot/datasets/so101_pick_red_block_isaac \
  --checkpoint data/groot/training/so101_n17/checkpoint-10000 \
  --traj-ids 0 1 2 \
  --execution-horizon 16 \
  --steps 400
~~~

Plots are written below the checkpoint's open_loop/ directory unless
--output-dir is supplied. Check every joint, especially gripper direction and
scale. Low training loss alone is not deployment evidence.

## 10. Closed-loop Isaac evaluation

Run the exact numbered checkpoint:

~~~bash
applications/groot/scripts/run_so101_isaac.sh \
  --checkpoint data/groot/training/so101_n17/checkpoint-10000 \
  --headless
~~~

For a GUI plus rosbridge observation:

~~~bash
applications/groot/scripts/run_so101_isaac.sh \
  --checkpoint data/groot/training/so101_n17/checkpoint-10000 \
  --observe
~~~

The policy server queries the modality config saved in the checkpoint. The ROS
bridge then sends front, single_arm, gripper, and the selected task language. It
clamps requested motion against current state and URDF limits.

Stop and clean up:

~~~bash
docker compose \
  -f compositions/so101_groot_isaac.yaml \
  --profile sim --profile observe down
~~~

Score repeated runs using the task and acceptance tools documented in
applications/so101/README.md. Report closed-loop success rate, collisions, place
error, command saturation, joint-limit violations, and policy latency.

## Hardware deployment

The hil profile is deliberately disarmed by default. A hardware driver must
translate its native calibrated motor representation to this repository's ROS
contract: radians for the five arm joints and metres-opening for the gripper.
The inference camera must match the training camera name, viewpoint, resolution,
and preprocessing.

Start disarmed, validate the first predictions and emergency stop, and only then
start a new process with SO101_HIL_ARMED=true. See
applications/so101/README.md for the guarded HIL commands.

## Troubleshooting

### GatedRepoError or HTTP 401

The token either is absent or lacks accepted access to
nvidia/Cosmos-Reason2-2B. Repeat the cache-mounted login and run hf auth whoami
in the Compose service.

### Dataset validation reports missing so101_units.json

Raw six-dimensional LeRobot data is intentionally rejected. Use
prepare_dataset.sh, or use convert_bags.sh for Isaac recordings.

### Arm values exceed the radian range

A degree dataset was marked as Isaac units or was converted twice. Return to the
untouched source and prepare it once with --input-units degrees.

### Gripper is reversed or saturates

The raw open/closed calibration endpoints were wrong or reversed. Recreate the
prepared dataset from the source with measured endpoint values. Never patch only
the checkpoint statistics.

### Missing wrist camera

This workflow's included contract is front-only. A checkpoint trained with the
upstream two-camera SO-101 config expects front and wrist; duplicating the front
image into both inputs is only a transport fallback and is not meaningful
deployment.

### Out of GPU memory

Reduce batch size only as needed, use gradient accumulation through advanced
upstream arguments, avoid --tune-llm and --tune-visual, or move training to a
40+ GiB GPU. Running Isaac and GR00T simultaneously consumes more memory than
either one alone.

### Interrupted conversion

Converters never overwrite an output dataset. Remove or move only the failed
output/conversion workspace after inspecting it, then rerun. Source datasets and
bags are not modified.

## Upstream references

- [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T)
- [SO-100/SO-101 fine-tuning guide](https://github.com/NVIDIA/Isaac-GR00T/blob/main/examples/SO100/README.md)
- [Custom embodiment guide](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/finetune_new_embodiment.md)
- [Data preparation guide](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/data_preparation.md)
- [Hardware recommendations](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/hardware_recommendation.md)
