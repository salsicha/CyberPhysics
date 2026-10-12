# Arm and end-effector policy validation — 12 October 2026

SmolVLA and PushT run on this computer's GPU. None of the available workflows
currently demonstrates a complete VLA-controlled arm task with a trained
checkpoint and validated grasp physics.

The host has an NVIDIA RTX 4000 Ada Generation Laptop GPU, 12,282 MiB VRAM,
driver 610.57.04, and about 31 GiB system RAM. Tests used the repository's
Docker images, LeRobot 0.6.0 and Torch 2.11.0+cu126. GR00T's separate runtime
uses Torch 2.9.0+cu128. Compact results are in
[`validation/20261012`](validation/20261012/README.md).

| Workflow | Actual test | Result on this computer |
| --- | --- | --- |
| SmolVLA, 450,046,176 parameters | Strict pretrained loading and eight fresh action chunks from RGB, state and language | Runs: 933 MiB peak allocated / 976 MiB reserved Torch memory. The base checkpoint cannot supply calibrated SO101 joint commands. |
| PushT diffusion, about 263M parameters | Three complete seeded episodes, rendered observations and saved videos | All three succeeded. 1,099 MiB peak allocated / 1,180 MiB reserved. This is an image-conditioned 2D end effector, not a VLA or an articulated arm. |
| π0 | Actual example with the configured Hugging Face credential | Blocked by HTTP 403 for `google/paligemma-3b-pt-224`. No inference or measured GPU-fit claim. The example has placeholder observations and no arm loop. |
| GR00T N1.7 | Runtime/CUDA imports, repository VRAM preflight and authenticated backbone access | Imports work. The 16 GiB preflight rejects this GPU; `nvidia/Cosmos-Reason2-2B` also returns HTTP 403. No fine-tuned SO101 checkpoint is present, so real model inference was not tested. |
| SO101 scripted Isaac baseline | Rendered scene, real ROS/Zenoh bridge and deterministic IK policy | Geometric placement completes, with 2.50 mm final error. Correctly fails VLA acceptance because it uses ground truth and collision measurement is absent. |

Torch memory counters exclude CUDA context, graphics and simulator memory.
The rendered SO101 scene alone used approximately 2,187 MiB in `nvidia-smi`.
SmolVLA and PushT have substantial memory headroom here; these measurements do
not establish training batch sizes or GR00T/π0 fit. NVIDIA's supported GR00T
budget starts at 16 GiB for inference and 40 GiB for fine-tuning:
[upstream hardware requirements](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/hardware_recommendation.md).

## SmolVLA: runnable model, incomplete arm checkpoint

The tested public checkpoint is `lerobot/smolvla_base` at
`5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6`. Its visual backbone was cached at
`HuggingFaceTB/SmolVLM2-500M-Video-Instruct`, revision
`7b375e1b73b11138ff12fe22c8f2822d8fe03467`.

The checkpoint declares mean/std normalization for state and action but its
processor tensors contain embodiment-specific action keys such as
`so100.buffer.action`, with no matching `observation.state` or `action`
statistics. LeRobot 0.6 silently returns unchanged tensors when the feature's
statistics are absent. The former adapter could therefore interpret normalized
outputs as degrees and command the wrong pose. Its unit-range check also read
the constructor's empty `stats` field instead of the checkpoint's loaded
`_tensor_stats`. See the
[upstream normalizer implementation](https://github.com/huggingface/lerobot/blob/v0.6.0/src/lerobot/processor/normalize_processor.py).

Both defects are fixed. The server checks required statistics, shape and
finite values before loading model weights or opening its policy socket.
It rejects the public base for joint control. The unit-check override does
not bypass this requirement. Full weights load strictly, and PEFT adapter
revisions no longer get incorrectly applied to a different base-model repo.

The model-only benchmark explicitly uses normalized input/output space, without
joint decoding or commands. Eight finite action chunks passed; median latency
was 220 ms including first-call overhead, or 210 ms excluding the first call.
An Isaac flight ran concurrently, so these are not isolated latency estimates.
All three model camera slots received the same synthetic image; this tests
execution and memory, not perception or multi-camera task performance.

Use the documented [benchmark command](../applications/smolvla/README.md#measure-inference-before-running-a-scene).
To evaluate learned arm control, supply a fine-tuned checkpoint with matching
processors, calibrated joint units, task distribution and camera views. No
such checkpoint or task demonstration dataset was found in this checkout.
No statistics were invented and no scripted fallback was substituted.

## PushT: complete evaluation of its smaller task

Checkpoint: `lerobot/diffusion_pusht` at
`84a7c23178445c6bbf7e1a884ff497017910f653`. The old example used obsolete policy
and preprocessing assumptions. It now restores the checkpoint's saved
normalization tensors, verifies that input/output action scales agree, loads
all model weights strictly, and uses the current processor API.

| Seed | Success | Steps | Final reward | Episode wall time |
| --- | --- | ---: | ---: | ---: |
| 42 | Yes | 182 | 1.0 | 34.4 s |
| 43 | Yes | 208 | 1.0 | 37.8 s |
| 44 | Yes | 129 | 1.0 | 24.7 s |

No task threshold or diffusion inference-step count was reduced. Action-call
p95 was 1.39–1.45 s; most calls consume cached actions, while each new chunk
runs 100 denoising steps. These episodes establish functionality for these
three seeds, not a statistically strong generalization estimate.

The Compose service now preserves its model cache and reports/videos, exposes
the existing `HF_TOKEN` when configured, and mounts the current evaluator.
Run instructions are in the [SO101 README](../applications/so101/README.md#policy-evaluation-scope).
Local videos are under `generated/arm_validation/20261012/lerobot/pusht/`;
the compact JSON measurements are committed, while videos remain local.

## What remains incomplete in the arm simulation

The rendered baseline issued 94 commands and placed the object in 18.69 s,
with zero measured joint-limit violations. Its command saturation fraction
was 41.5%; this is a transport/scene baseline, not evidence of a learned
controller. The scene kinematically attaches an object when the jaws close
near it and snaps released objects to a supporting surface. It does not
simulate and measure a physical frictional grasp or collisions.

The scorer explicitly distinguishes `task_completed` from policy `success`.
The baseline reports `task_completed: true`, `success: false`, ground-truth
use, and `collision_count: null`. Full VLA acceptance still needs a trained
policy, physical grasp/contact modeling, measured collision behavior, varied
scenes and language, distractors, occlusion and failed-grasp recovery tests.

The runtime telemetry writer was also repaired: it replaces complete JSON
files atomically, so an earlier container UID's readable report does not
disable a new evaluation in a writable directory. The π0 example now checks
tokenizer access before downloading 14 GB of weights and propagates strict
loading failures instead of accepting the upstream loader's untrained fallback.

Regression tests cover missing/nonfinite/mismatched normalization statistics,
loaded tensor state-range checks, strict π0 loading failures and atomic
telemetry replacement. GPU and rendered tests above used actual runtimes;
mocked unit tests are not counted as model execution.
