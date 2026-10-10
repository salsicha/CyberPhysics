# SmolVLA for SO101

This application builds a pinned Hugging Face LeRobot/SmolVLA image and serves a
SmolVLA checkpoint to the existing CyberPhysics SO101 ROS 2 bridge. The separate
[SO101 SmolVLA Isaac composition](../../compositions/so101_smolvla_isaac.yaml)
runs the policy against the rendered SO101 tabletop simulation.

The GR00T compositions are intentionally independent and unchanged. Use
`so101_groot_isaac.yaml` for GR00T and `so101_smolvla_isaac.yaml` for
SmolVLA.

## Included files

| File | Purpose |
| --- | --- |
| `Dockerfile` | CUDA 12.8 base pinned to LeRobot v0.6.0 and a Volta-compatible PyTorch CUDA 12.6 wheel |
| `scripts/so101_policy_server.py` | SmolVLA inference and SO101/Isaac unit adapter |
| `scripts/policy_healthcheck.py` | Compose readiness probe |
| `scripts/smoke_test.py` | Import-only image validation |
| `tests/test_so101_policy_server.py` | Unit and observation conversion tests |

## Hardware and software

SmolVLA is a 450M-parameter model and is a substantially better fit for the
available 32 GB V100 than GR00T N1.7. This image uses a CUDA 12.8 runtime base
and pins PyTorch's CUDA 12.6 wheel because PyTorch 2.11 CUDA 12.8 wheels omit
Volta (`sm_70`) kernels. Inference uses FP32 by default; the V100 does not need
FlashAttention or native BF16 support for this workflow.

Required host components:

- Docker Engine with Docker Compose v2.
- NVIDIA Container Toolkit.
- An NVIDIA driver compatible with CUDA 12.8.
- The existing `salsicha/cyberphysics:isaac-latest` and `salsicha/cyberphysics:so101-latest` images.
- Enough VRAM to run Isaac Sim and SmolVLA together. Headless Isaac reduces
  graphics overhead.

## Build

From the repository root:

```bash
make -C applications build_smolvla
```

Or build directly:

```bash
docker build \
  --build-arg LEROBOT_REF=v0.6.0 \
  -t salsicha/cyberphysics:smolvla-latest \
  applications/smolvla
```

The image installs the `smolvla`, training, and PEFT extras from the pinned
LeRobot source tree. Confirm the image without downloading a checkpoint:

```bash
docker run --rm salsicha/cyberphysics:smolvla-latest \
  python /workspace/smolvla_scripts/smoke_test.py
```

## Persistent directories

The model, training, and telemetry directories below
`applications/smolvla/smolvla` are tracked in Git. The compose stack mounts that
directory at `/workspace/data` and keeps the Hugging Face/torch cache in the
named Docker volume `smolvla_cache` at `/workspace/cache` (a missing host bind
directory would otherwise be created root-owned by Docker). The Docker image
also copies the data directory into `/workspace/data`, so the same layout is
available when the image is run without Compose.

## Download the base model

SmolVLA is public and does not normally require authentication:

```bash
docker compose \
  -f compositions/so101_smolvla_isaac.yaml \
  --profile sim run --rm --no-deps smolvla_policy \
  hf download lerobot/smolvla_base \
    --local-dir /workspace/data/models/smolvla_base
```

The server can also use `lerobot/smolvla_base` directly and let Hugging Face
cache the files on first start.

## Run the SO101 simulation

Build the reusable simulation images if needed:

```bash
make -C applications build_isaac build_so101 build_smolvla
```

Run a headless integration smoke test with the local base checkpoint:

```bash
SO101_HEADLESS=true \
SMOLVLA_MODEL_PATH=/workspace/data/models/smolvla_base \
docker compose \
  -f compositions/so101_smolvla_isaac.yaml \
  --profile sim up
```

To render the Isaac window, omit `SO101_HEADLESS=true`. Add
`--profile observe` to also start the rosbridge/Foxglove service.

> **Base-model warning:** `lerobot/smolvla_base` verifies model loading,
> observation preprocessing, and closed-loop transport, but it is not trained
> for this simulated red-block task. Its actions should not be expected to
> succeed. The ROS bridge still clamps joint limits and per-command motion, but
> the base model must only be exercised in simulation.

Stop the stack with:

```bash
docker compose \
  -f compositions/so101_smolvla_isaac.yaml \
  --profile sim down --remove-orphans
```

## Fine-tune for SO101

Use a LeRobot dataset recorded with the standard SO101 joint order:

1. `shoulder_pan`
2. `shoulder_lift`
3. `elbow_flex`
4. `wrist_flex`
5. `wrist_roll`
6. `gripper`

The default adapter (`SMOLVLA_CHECKPOINT_UNITS=so101`) expects data recorded by
the LeRobot `so101_follower` with **`use_degrees=True`**: the five arm joints in
degrees and the gripper in the 0–100 range. LeRobot's default
`use_degrees=False` stores each arm joint as -100..100 of its calibrated travel;
those values look like plausible degrees but are not, so the server refuses a
checkpoint whose state statistics saturate at exactly ±100
(`SMOLVLA_SKIP_UNIT_CHECK=true` overrides a false positive) and
`--checkpoint-units so101_range100` is rejected outright: converting it needs
the arm's calibration file.

LeRobot's 0° is the middle of each joint's calibrated range, while this
repository's URDF zero is the arm pointing straight up. The server maps
`urdf_deg = sign * lerobot_deg + offset_deg` per joint and the gripper's
`closed`/`open` readings to 0–0.04 m using
`systems/so101/config/lerobot_joint_mapping.json` (`SMOLVLA_JOINT_MAPPING`). Its
values are a nominal calibration derived from TheRobotStudio's
`so101_new_calib.urdf` (whose zero is LeRobot's calibrated mid-range), not a
measurement of your arm: verify the offsets and signs (pose it at known URDF
angles and read LeRobot's degrees). The server warns if the mapping is the
identity, and if the checkpoint's recorded state range, mapped this way, falls
outside the URDF joint limits (`systems/so101/config/joint_limits.json`, the
physical range of a LeRobot-calibrated arm in this URDF's convention). A typical
full fine-tune on a Hub dataset is:

```bash
docker compose \
  -f compositions/so101_smolvla_isaac.yaml \
  --profile sim run --rm --no-deps smolvla_policy \
  lerobot-train \
    --policy.path=lerobot/smolvla_base \
    --policy.device=cuda \
    --policy.use_amp=true \
    --policy.push_to_hub=false \
    --dataset.repo_id=YOUR_HF_USER/YOUR_SO101_DATASET \
    --batch_size=8 \
    --steps=20000 \
    --save_freq=2000 \
    --output_dir=/workspace/data/training/so101_smolvla \
    --job_name=so101_smolvla \
    --wandb.enable=false
```

Start at batch size 8 on the V100, then increase it only after observing peak
VRAM. The official SmolVLA guidance recommends roughly 50 varied, successful
episodes as a starting point.

LeRobot v0.6.0 also supports a smaller LoRA fine-tune:

```bash
docker compose \
  -f compositions/so101_smolvla_isaac.yaml \
  --profile sim run --rm --no-deps smolvla_policy \
  lerobot-train \
    --policy.path=lerobot/smolvla_base \
    --policy.device=cuda \
    --policy.use_amp=true \
    --policy.push_to_hub=false \
    --policy.optimizer_lr=1e-3 \
    --policy.scheduler_decay_lr=1e-4 \
    --dataset.repo_id=YOUR_HF_USER/YOUR_SO101_DATASET \
    --batch_size=8 \
    --steps=20000 \
    --output_dir=/workspace/data/training/so101_smolvla_lora \
    --job_name=so101_smolvla_lora \
    --peft.method_type=LORA \
    --peft.r=64 \
    --peft.lora_alpha=64 \
    --wandb.enable=false
```

The policy server accepts both normal LeRobot checkpoints and LeRobot PEFT
adapter checkpoints.

## Run a fine-tuned checkpoint

LeRobot writes deployable policy files below each checkpoint's
`pretrained_model` directory. For example:

```bash
SO101_HEADLESS=true \
SMOLVLA_MODEL_PATH=/workspace/data/training/so101_smolvla/checkpoints/last/pretrained_model \
docker compose \
  -f compositions/so101_smolvla_isaac.yaml \
  --profile sim up
```

If a checkpoint was trained on Isaac-native data where the arm is already in
radians and the gripper is already in metres, disable the default conversion:

```bash
SMOLVLA_CHECKPOINT_UNITS=isaac \
SMOLVLA_MODEL_PATH=/workspace/data/training/so101_smolvla/checkpoints/last/pretrained_model \
docker compose \
  -f compositions/so101_smolvla_isaac.yaml \
  --profile sim up
```

Do not set `SMOLVLA_CHECKPOINT_UNITS=isaac` for a normal dataset recorded by a
LeRobot SO101 follower.

## Camera mapping

The simulator publishes one rendered front camera. The server reads the image
feature names from the selected checkpoint:

- A one-camera fine-tuned checkpoint receives the front image on its camera key.
- The base checkpoint declares three cameras, so the server duplicates the front
  image across those three keys.
- For best task performance, fine-tune on images rendered from the same camera
  pose used during evaluation.

## Runtime settings

| Variable | Default | Meaning |
| --- | --- | --- |
| `SMOLVLA_MODEL_PATH` | `lerobot/smolvla_base` | Hub ID or path below `/workspace/data` |
| `SMOLVLA_DEVICE` | `cuda` | Torch inference device |
| `SMOLVLA_CHECKPOINT_UNITS` | `so101` | `so101` (use_degrees=True degrees/0–100) or `isaac` radians/metres |
| `SMOLVLA_JOINT_MAPPING` | `/workspace/systems/so101/config/lerobot_joint_mapping.json` | LeRobot degrees to URDF sign/offset and gripper range |
| `SMOLVLA_SKIP_UNIT_CHECK` | `false` | Accept a checkpoint whose state statistics look like -100..100 |
| `SMOLVLA_ACTION_HORIZON` | `8` | Cached action steps before replanning |
| `SMOLVLA_POLICY_FPS` | `30` | Bridge command rate; must equal the training dataset's fps so queued actions replay at their recorded rate |
| `SMOLVLA_PORT` | `5556` | Local ZeroMQ policy port |
| `SMOLVLA_LOCAL_FILES_ONLY` | `false` | Disable Hub access after precaching |
| `SO101_MAX_JOINT_STEP` | `0.08` | Maximum simulated joint change per command |
| `SO101_HEADLESS` | `false` | Run Isaac Sim without a window |

The SmolVLA server uses port 5556 so it does not collide with the GR00T server's
default port 5555.

The bridge resets the server (dropping its queued action chunk) before the first
request, when joint feedback resumes after a gap such as an Isaac restart, and
when the task changes. It sends camera frames at native resolution; SmolVLA's
own preprocessing resizes with padding exactly as during training.

## Validation

Render and validate the composition without starting it:

```bash
docker compose \
  -f compositions/so101_smolvla_isaac.yaml \
  --profile sim config
```

Run the adapter tests inside the image:

```bash
docker run --rm \
  -v "$PWD/applications/smolvla:/workspace/smolvla:ro" \
  -e PYTHONPATH=/workspace/smolvla/scripts \
  salsicha/cyberphysics:smolvla-latest \
  python -m unittest discover -s /workspace/smolvla/tests -v
```

## References

- [Official SmolVLA guide](https://github.com/huggingface/lerobot/blob/v0.6.0/docs/source/smolvla.mdx)
- [Official LeRobot SO101 rollout example](https://github.com/huggingface/lerobot/blob/v0.6.0/docs/source/smolvla.mdx#evaluate-the-finetuned-model-and-run-it-in-real-time)
- [Official PEFT guide](https://github.com/huggingface/lerobot/blob/v0.6.0/docs/source/peft_training.mdx)
