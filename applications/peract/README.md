# PerAct with Franka in Isaac

This is a learned, language-conditioned **reaching** demo. The Franka moves its
open gripper above either a red or a blue block, using rendered RGB-D and the
instruction. It exercises PerAct inference, sparse Cartesian actions, and
physical joint control in Isaac Sim. It does not yet demonstrate grasping,
multi-step manipulation, arbitrary language, or transfer to a physical robot.

The policy uses the unchanged [official PerAct encoder and voxelizer](https://github.com/peract/peract)
at commit `02fb87681c5a47be9dbf20141bedb836ee2d3ef9`, in a compact configuration:
40³ voxels, 128 latents of width 128, and two transformer layers. It is trained
on this Isaac scene, not loaded from the published 18-task RLBench checkpoint.
CLIP RN50 supplies frozen text features for the two supported instructions.
The container imports the encoder directly to avoid upstream's eager RLBench /
CoppeliaSim runner imports; those simulators are not dependencies of this demo.

## Build and prepare

Use an x86-64 NVIDIA GPU host with Docker Compose v2 and NVIDIA Container
Toolkit. The image uses CUDA 12.6 PyTorch; Isaac is the repository's 6.0.1 image
with external ROS 2 Jazzy and Zenoh. Internet access is needed for the image,
Franka USD asset and initial CLIP download. Accepting the NVIDIA image's EULA
and privacy choice follows the other Isaac compositions (`PRIVACY_CONSENT`).

From the repository root:

```bash
make -C applications build_isaac build_peract
./tools/prepare_peract_demo.sh
PERACT_HEADLESS=true ./start.sh franka_peract_isaac --profile sim
```

Preparation renders 256 random scenes (both instructions per scene), then
trains 3,000 optimizer steps. Collection and training run sequentially to free
the GPU between stages. It writes the dataset and checkpoint under ignored
`generated/peract/`; generated weights and caches are not committed. Repeat
preparation when changing the scene/model configuration. It intentionally
recollects and retrains rather than silently reusing a mismatched checkpoint.
Cold Isaac shader compilation can take several minutes.

For a desktop window omit `PERACT_HEADLESS=true`. Like the other Isaac stacks,
the composition forwards `DISPLAY` and copies `XAUTHORITY`; a working local X11
display is required. The demo runs 20 episodes and exits. Stop the policy service:

```bash
docker compose -f compositions/franka_peract_isaac.yaml --profile '*' down
```

## Architecture and configuration

- `applications/peract`: reusable model image, training, local HTTP policy
  service, and Isaac adapter. Isaac and PyTorch use separate containers.
- `systems/franka/config/peract_reach.json`: workspace, task instructions,
  randomization, model size, motion limits and acceptance thresholds.
- `systems/franka/config/joint_control.json`: Franka position-drive gains and
  trajectory timing. Explicit gains avoid the large tracking errors seen with
  the asset defaults; gravity and rigid-body dynamics remain enabled.
- `compositions/franka_peract_isaac.yaml`: `collect`, `train`, and `sim` profiles.
  The simulator starts only after the trained policy passes its health check.

The policy receives a 128×128 RGB image, its aligned world-frame depth point
cloud, four proprioceptive values, and one supported instruction. It predicts
a voxel-center position, discretized rotation, gripper state and collision
flag. Franka Lula inverse kinematics converts the accepted pose to a joint goal;
a minimum-jerk trajectory drives the physical joints at 60 Hz.
Isaac continues stepping while the independent policy process computes.

Block poses are used only for collecting supervised labels and scoring.
They are absent from the inference request. Invalid, low, out-of-workspace,
closed-gripper or unreachable predictions fail the episode; there is no
scripted target correction. This simple empty workspace does not establish obstacle-avoidance performance;
the executor is not a general collision-aware motion planner.

| Setting | Default | Purpose |
| --- | --- | --- |
| `PERACT_HEADLESS` | `false` | Hide Isaac's desktop window |
| `PERACT_EPISODES` | `20` | Number of test episodes |
| `PERACT_SEED` | `9876` | Test placements; training uses 123 |
| `PERACT_PORT` | `5562` | Loopback HTTP policy port |
| `PERACT_TRAIN_SCENES` | `256` | Training scenes, two labels each |
| `PERACT_TRAIN_STEPS` | `3000` | Optimizer updates |
| `PERACT_RESULTS_DIR` | `../generated/peract` | Compose-relative output mount |
| `PERACT_MODEL_DIR` | `../generated/peract/trained` | Checkpoint directory |
| `CYBERPHYSICS_TAG` | `latest` | Shared application image tag suffix |

If using a custom results directory, also point `PERACT_MODEL_DIR` at its
`trained` subdirectory for evaluation. A checkpoint/config digest mismatch is
an error. The policy binds to localhost and does not expose a remote-control API.

ROS 2 telemetry uses the shared Zenoh middleware configuration:
`/franka/joint_states` (`sensor_msgs/JointState`, 10 Hz),
`/franka/peract/target` (`geometry_msgs/PoseStamped`, world frame), and
`/franka/peract/status` (`std_msgs/String`, episode result JSON).

## Evaluation

The [recorded validation](../../tools/peract_validation_20261012.md) passed
20/20 held-out reaches on the 12 GiB RTX 4000 Ada Laptop: mean task error
9.1 mm, warm inference median 19.1 ms, and observed total GPU use about
2.5 GiB with Isaac and the policy together. The compact model has 1.58 million
parameters; these measurements do not describe the full published model.

The default evaluates 20 new scenes, alternating the two instructions.
A successful episode must settle within 15 mm of the predicted pose for
30 physics steps and finish within 25 mm of the selected block's reaching
point. Below 80% task success exits nonzero. This measures the complete
camera → learned policy → physical arm path, rather than just model loading.

Results are in `generated/peract/evaluation/report.json`: per-episode success,
prediction/tracking/task errors, inference latency, checkpoint hash and Torch
peak memory. `camera.png` is the initial policy view; `episode_*.png` are final
camera views. Hardware validation requires a GPU; CI runs the pure executor
contract tests in `tests/test_peract_contracts.py`.
The first two episodes also save overview GIFs of the arm's motion; this
separate observer camera is never sent to the policy.
