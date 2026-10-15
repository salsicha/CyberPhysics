# Act3D + Franka in Isaac Sim

A language-conditioned, sparse-pose **reaching** demo using the
[official Act3D implementation](https://github.com/zhouxian/act3d-chained-diffuser).
The Franka moves its open gripper above either a red or a blue block, selected by
the instruction. RGB and metric world-space points come from a rendered camera.
The policy receives the measured gripper pose, but no block poses or target labels.

This uses the same task, camera and Lula IK/minimum-jerk executor as the
[PerAct demo](../peract/README.md). It does not demonstrate grasping, obstacle-aware
motion planning, arbitrary instructions, or a complete household manipulation
policy. The upstream RLBench benchmark checkpoint is not substituted for a policy
trained on this scene.

## Validation

Validated on 2026-10-14 with Isaac Sim 6.0.1 and an RTX 4000 Ada laptop GPU (12 GiB).
The refined policy succeeded in **20/20 episodes across 10 held-out scenes**,
using both instructions in each scene. Mean physical reaching error was **14.3 mm**
(maximum 21.7 mm), within the 25 mm tolerance. Median inference after the first
request was **48.7 ms** while Isaac was rendering. Peak PyTorch allocation was
297.5 MiB; that excludes CUDA context and simulator memory.

The shared-executor PerAct regression passed 4/4 episodes, and 21 focused unit
tests passed. Live HTTP checks rejected mismatched configuration, unknown language
and incorrectly scaled depth. Changing only the instruction on an identical
observation selected the other block, with both predictions within 8 mm.
The checkpoint also loaded and ran in a container with networking disabled.
See [recorded metrics and provenance](validation_20261014.json).

The initial 6,000-step checkpoint missed the accuracy requirement; the second
training phase is necessary in the validated recipe. Run collection, training
and evaluation sequentially, as the preparation helper does. Simultaneous training
and multiple Isaac instances exhausted this host's 32 GiB RAM during the initial
attempt. These results cover this small reaching task, not RLBench performance
or physical robot deployment.

## Build and run

Requires an x86 Linux NVIDIA GPU host, NVIDIA Container Toolkit and Docker Compose
with GPU support. Isaac runs in its existing image. The Act3D image derives from
the PerAct image to reuse its pinned CUDA PyTorch runtime; it runs Act3D's own model.

```bash
make -C applications build_isaac build_act3d
./tools/prepare_act3d_demo.sh
ACT3D_HEADLESS=true ./start.sh franka_act3d_isaac --profile sim
```

Preparation collects 256 scenes (both instructions per scene) and trains in two
6,000-step phases: initial learning, then refinement with a fresh optimizer.
The initial checkpoint is retained as `act3d_initial.pt`; the refined checkpoint
used by the demo is `act3d_reach.pt`. Initial training downloads CLIP weights and requires internet access.
The resulting checkpoint includes the visual backbone, learned model, language
embeddings and configuration. Inference loads it offline and fails if it is
missing or incompatible; there is no untrained or ground-truth fallback.

The simulation evaluates 20 episodes on 10 new scenes, asking for both colors
on each scene. It writes `generated/act3d/evaluation/report.json`, camera images
and GIFs of the first two movements. An episode succeeds only when the physical
gripper settles and finishes within 25 mm of the requested target. Fewer than
80% successes produces a nonzero simulator exit status. To propagate that status
to a shell or CI job:

```bash
ACT3D_HEADLESS=true docker compose -f compositions/franka_act3d_isaac.yaml \
  --profile sim run --rm isaacsim
```

For a visible viewport, omit `ACT3D_HEADLESS=true`. Set `XAUTHORITY` to your X11
authority file when necessary. Stop only this demo with:

```bash
docker compose -f compositions/franka_act3d_isaac.yaml --profile sim down
```

## Configuration and architecture

- `systems/franka/config/act3d_reach.json`: scene, vocabulary and model settings.
- `systems/franka/config/joint_control.json`: shared physical controller tuning.
- `compositions/franka_act3d_isaac.yaml`: `collect`, `train` and `sim` profiles.
- `applications/peract/scripts/isaac_franka.py`: shared simulator/executor.
- `applications/act3d/scripts/`: training, upstream compatibility adapter and HTTP
  inference server. The server binds to loopback, default port 5563.

Act3D uses a frozen CLIP RN50 visual backbone, a trainable feature pyramid,
CLIP ViT-B/32 token embeddings, language cross-attention and three coarse-to-fine
ghost-point sampling stages. This configuration keeps the upstream 60-dimensional
features, four attention heads and 1,000/10,000 total training/evaluation sampling
budgets (integer division distributes points across the stages). It predicts a
continuous position, quaternion and gripper opening. The policy does not use a
color threshold or object detector to select a target.

Upstream source is pinned to `5004650a9771e7618f24f36a876bcb38e0fc068a`.
The CLIP text encoder is pinned to Hugging Face revision
`3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268`; RN50 downloads are hash-checked by CLIP.
The adapter fixes its 128-pixel feature-pyramid attribute typo and avoids importing
unneeded RLBench/ChainedDiffuser modules. Training caches only frozen RN50 features
in host RAM, stored as FP16; the feature pyramid and Act3D layers still train.
Frozen BatchNorm statistics stay fixed. Checkpoints include dataset hashes,
configuration, source revision, training seed and loss history.

Training uses labeled target positions to center fine sampling stages, following
upstream teacher forcing. At inference, every fine stage centers on the preceding
predicted position. No ground-truth action enters that call. The upstream model
has no collision head: the adapter returns `ignore_collisions=false`, and the
executor rejects out-of-workspace, low-reaching or closed-gripper predictions.
The scene is unobstructed; no collision-aware motion planner is included.

Isaac keeps stepping while inference runs in a worker thread, then executes the
returned pose with IK and a joint trajectory. This demo does not predict the next
observation or speculatively plan the action after the current action.

ROS telemetry uses the repository's Jazzy/Zenoh configuration and publishes
`/franka/joint_states`, `/franka/act3d/target` and `/franka/act3d/status`.
Use a distinct `ROS_DOMAIN_ID` when running alongside another Franka demo.

Useful overrides: `ACT3D_TRAIN_SCENES`, `ACT3D_TRAIN_STEPS`, `ACT3D_REFINE_STEPS`, `ACT3D_EPISODES`,
`ACT3D_SEED`, `ACT3D_PORT`, `ACT3D_RESULTS_DIR` and `ACT3D_MODEL_DIR`.
Paths in Compose are relative to `compositions/`; use absolute paths for custom
output/model locations. Changing camera, geometry or model configuration requires
recollection and retraining; mismatched requests are rejected.
