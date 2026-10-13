# PerAct / Franka / Isaac validation — 2026-10-12

The compact, task-trained PerAct policy completed **20/20** held-out reaching
episodes in Isaac Sim. Both language commands passed 10/10 episodes. This is
an RGB-D-to-action neural policy executed through physical joint drives;
object poses enter demonstration labels and scoring only.

## Tested configuration

- NVIDIA RTX 4000 Ada Laptop, 12,282 MiB VRAM, driver 610.57.04.
- Isaac Sim 6.0.1, ROS 2 Jazzy with `rmw_zenoh_cpp`, PyTorch 2.11.0 / CUDA 12.6.
- Official PerAct encoder/voxelizer commit
  `02fb87681c5a47be9dbf20141bedb836ee2d3ef9`.
- Compact model: 40³ voxels, 128 latents, width 128, depth 2;
  **1,578,525 trainable parameters**. This is not the original benchmark-sized
  architecture or published RLBench multi-task checkpoint.
- 256 rendered training scenes, both commands per scene: 512 samples,
  placement/optimizer seed 123, 3,000 optimizer steps, 677 seconds of training
  loop time. Frozen CLIP RN50 features encode the two supported instructions.
- Checkpoint SHA-256:
  `f0541b341734e8e6831713088f5a7c7aa3a2e2a2fc9b3e8f133fb0bbfd52bf3a`.
  The locally generated checkpoint is excluded from Git; the preparation
  workflow recreates a task-trained model on a new checkout.
- Test seed 9876, 20 new scenes, alternating red and blue instructions.
  No test scenes were added to training. Each action is predicted once and
  executed with Lula IK and a minimum-jerk joint trajectory.

## Results

| Measurement | Result |
| --- | ---: |
| Successful physical reaches | 20 / 20 |
| Mean final distance to task target | 9.06 mm |
| Maximum final distance to task target | 14.93 mm |
| Mean learned position prediction error | 10.50 mm |
| Maximum learned position prediction error | 16.02 mm |
| Mean executor position tracking error | 5.86 mm |
| Maximum executor position tracking error | 8.76 mm |
| First inference after service startup | 365.4 ms |
| Warm inference median / p95 (19 requests) | 19.12 / 22.63 ms |
| Torch inference peak allocated memory | 98.42 MiB |
| Observed total GPU memory, Isaac + policy + desktop | 2,576 MiB |
| Observed GPU memory during training | about 2.5 GiB |

Success requires 30 consecutive physics steps within 15 mm of the model's
predicted pose, after the trajectory finishes, and a final task error below
25 mm. The simulator exited with code 0. These are headless acceptance results;
the optional desktop-window mode was not separately evaluated.

The initial RMPflow executor stalled on several otherwise accurate model
targets. Explicit IK trajectories and position-drive gains fixed the observed
tracking failures. Gravity, joint dynamics, and rigid-body physics remain
enabled; joints are reset only between episodes, not teleported to achieve a
predicted action. Joint goals and measured joint positions are saved in the
final report for inspection.

## Reproduction and checks

See [the demo README](../applications/peract/README.md). The final run used
the standard Makefile builds and `start.sh` launcher with default `latest`
image tags. `COMPOSE_PROJECT_NAME=peract_demo` reused this validation run's
named caches; a normal launch uses the isolated `franka_peract_isaac` project.

```bash
make -C applications build_isaac build_peract
./tools/prepare_peract_demo.sh
PERACT_HEADLESS=true ./start.sh franka_peract_isaac --profile sim
```

The full collection profile completed 256 scenes. The unprivileged training
profile was also smoke-tested for one optimizer step to a separate output,
preserving the validated 3,000-step checkpoint. This caught and fixed a missing
Unix user entry required by PyTorch's optimizer cache. Checkpoint replacement
is atomic; preparation stops an older policy service before retraining.

Additional checks passed:

- 47 executor and ROS middleware tests (`test_peract_contracts.py` and
  `test_ros_middleware.py`), including Zenoh defaults and explicit DDS fallback.
- Compose configuration, environment-default consistency, Python parsing,
  shell syntax, and Git whitespace checks.
- Actual `/franka/joint_states` reception by an independent ROS CLI process
  using Zenoh.
- Policy RPC rejected malformed data, unsupported instructions, wrong config
  digests, wrong image dimensions, and entirely invalid point clouds with HTTP
  400, then remained healthy.
- Identical camera observations produced different red/blue target predictions
  when only the instruction changed.

Local artifacts: `generated/peract/evaluation/report.json`, per-episode final
camera views, and `episode_000.gif` / `episode_001.gif` overview recordings.
Training data, checkpoints, simulation logs, and images remain outside Git.

## Scope

This establishes the basic PerAct-to-Franka simulation path for reaching in
the configured scene. It does not establish grasping, arbitrary instructions,
multi-step tasks, obstacle-aware planning, scene/domain robustness, or physical
robot readiness. Gripper-close and ignore-collision actions are explicitly
rejected by this reaching executor. Additional manipulation tasks require their
own demonstrations and an appropriate action executor.
