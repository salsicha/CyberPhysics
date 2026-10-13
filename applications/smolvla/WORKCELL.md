# Learned pick-and-place workcell

This runs a fine-tuned SmolVLA policy end to end: two rendered RGB views, six
joint positions and a language instruction produce joint targets for a
MuJoCo arm. The block is a free rigid body moved by frictional jaw contacts.
There is no kinematic attachment, scripted fallback, object pose or success
signal in the policy observation. The evaluator may inspect simulator state
to score the episode; the policy does not receive it.

The public checkpoint is trained for **one task: red block into blue tray**.
Its model name says `so101`, but the exact matching simulator uses MuJoCo
Menagerie's **SO-ARM100**. Do not feed its six radian actions (including the jaw)
into the repository's SO101 Isaac/LeRobot adapter: those joint conventions,
cameras and gripper units differ. The existing SO101 workflow still needs its
own matched training data/checkpoint. This demo establishes learned manipulation
in the trained workcell, not general instruction following or physical-arm
readiness.

## Run

Docker Compose v2, NVIDIA Container Toolkit and a compatible NVIDIA driver are
required. On the tested 12 GiB RTX 4000 Ada Laptop GPU, the model and simulation
fit comfortably; Torch reserved 956 MiB, excluding graphics/CUDA context.

```bash
make -C applications build_smolvla_workcell
mkdir -p generated/arm_validation/workcell
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) \
  docker compose -f compositions/smolvla_workcell.yaml run --rm workcell
```

The first run downloads the public model and backbone to a named Docker
volume. `HF_TOKEN` is optional. Later offline runs use `HF_HUB_OFFLINE=1
TRANSFORMERS_OFFLINE=1`. Results are written under
`generated/arm_validation/workcell`: `report.json`, all per-episode joint/action
and block trajectories, and one two-camera MP4 per condition. The UID/GID
settings keep bind-mounted results owned by the invoking user. Create the
output directory before running so Docker does not create it as root.

The default evaluates 20 paired initial poses in each of seven conditions:
nominal, visual randomization, physics randomization, combined randomization,
temporary front-camera occlusion, an external block force, and paraphrases
of the same task. Randomized mass/friction/lighting/cameras come from the
pinned environment. Occlusion covers an 85×80-pixel region during steps 15–44;
the disturbance applies 0.30 N for one control interval at step 35. All
conditions use the same initial-pose and policy seeds, starting at 123.

```bash
VLA_EPISODES=5 VLA_SEED=42 VLA_SCENARIOS=nominal \
  docker compose -f compositions/smolvla_workcell.yaml run --rm workcell
```

For a different output directory or experimental action horizon, override the
command with `python /workspace/smolvla_scripts/evaluate_workcell.py --help`.
The default reuses each 50-step action chunk, as configured by the checkpoint.
The simulated controller runs at approximately 15 Hz. This is an offline
evaluation: time does not advance while the GPU computes. A fresh chunk took
about 125 ms median, longer than one control interval; cached-action latency
alone must not be presented as a real-time control guarantee.

## Success and limitations

A trial only counts as task success after a measured bilateral jaw contact,
a lift of at least 6 cm, and a released block settled for 0.5 seconds with its
entire rotated footprint inside the tray. An airborne or overhanging block
does not count. This is stricter than the upstream center-position check.
Reports distinguish placement from collision-free acceptance and record
clipped commands, joint-limit excursions and solver contact forces. Exit 0
requires at least 80% success in every evaluated condition, nominal trials,
and collision-free success in every trial. Missing conditions are not inferred
as tested. Quality failures still save the full report and exit 1.

The measured 140-episode baseline completed **97/140** tasks, including
**14/20 nominal**, and **0/140 collision-free** tasks. All scenarios therefore
fail overall acceptance. This is a runnable learned arm demo with documented
policy limitations, not a robust or collision-free manipulation system.
The pretrained trajectory makes unwanted jaw/table contacts; solver forces
are sometimes large and have not been calibrated to real servos. Reducing
the action horizon did not reliably improve the small probe. Failed grasps
are handled only by whatever recovery the learned policy provides; the
evaluator does not repair them with privileged state or a scripted retry.

Improving these results requires retraining/validating the policy for broader
visual conditions and contact-aware demonstrations. Making the original
Isaac SO101 a comparable learned demo additionally requires its own contact
gripper and training embodiment. π0/GR00T gated-model access and GR00T's larger
supported GPU requirement remain separate limits; this checkpoint does not
resolve those workflows.

## Reproducibility

- [Checkpoint](https://huggingface.co/ahmedsohail2003/smolvla-so101-pickplace-v2):
  `f43f9f3a1c81b050f9747919c53019fcae2caec0`, weights SHA-256
  `e9cf64d37dba15549a2ec6ea6a996ecb78035340b79c106da5b98d96b88f066b`.
- [Matching environment](https://github.com/ahmedsohail2003/workcell-partA-imitation):
  `5822ac1dd508c92183fd8ecc99f91d6ec386549b`. Both projects declare Apache-2.0;
  bundled Menagerie assets retain their upstream license notices.
- LeRobot 0.6.0, Torch 2.11.0+cu126, MuJoCo 3.10.0. Checkpoint loading is strict
  and requires its actual saved state/action normalization tensors.
- [Validation report](../../tools/vla_embedded_validation_20261012.md) includes
  per-condition measurements, firmware results and remaining acceptance gaps.

CI runs the pure scoring/command validation tests. GPU rollouts were run on
this workstation; the hosted CI jobs do not claim GPU policy acceptance.
