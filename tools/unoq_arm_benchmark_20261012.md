# UNO Q: Depth Anything and compact PerAct benchmark

Measured on the USB-connected **2GB Arduino UNO Q**, 2026-10-12. All inference
ran on the board, with networking disabled in the benchmark containers. The
computer transferred files, converted models and collected results only.

The board can hold and execute this compact stack. The validated ONNX CPU
pipeline took **3.58 seconds** with 224×224 depth input, or **9.89 seconds**
with the default 518×518 depth input. These are compute results, not successful
robot manipulation: the monocular depth estimates did not produce usable
workspace geometry for this recorded scene.

## Hardware and scope

- QRB2210, four Cortex-A53-class cores, maximum reported clock 2.016 GHz;
  normal `schedutil` governor, no overclock or power-setting changes.
- 1,740 MiB Linux-visible RAM, 870 MiB existing zram swap. About 1,268 MiB
  available before setup. Normal desktop/Arduino services remained running;
  no other Docker workloads were active.
- Debian 13.1, kernel `6.16.7-g0dd6551ae96b`, aarch64.
- CPU tests used the board's existing
  `ghcr.io/arduino/app-bricks/python-apps-base:0.5.0` container, Python 3.13.9,
  PyTorch 2.11.0+cpu, torchvision 0.26.0+cpu, NumPy 2.2.6, OpenCV 4.11.0,
  einops 0.8.2, and ONNX Runtime 1.31.0. Dependencies were installed in a
  separate directory, not the board's system Python.
- Four inference threads, one inter-op thread. ONNX Runtime used only
  `CPUExecutionProvider`, with intra-op spinning disabled.
- Containers had no network, a 1,250 MiB memory limit and 128 MiB additional
  swap allowance. No successful run was killed for exceeding its memory limit.
- The board's Adreno 702 was visible through Mesa 25.2.6 OpenCL and Vulkan.
  `/dev/video0` and `/dev/video1` were codec devices, not a live camera.

Inputs were eight recorded 128×128 observations: four Isaac training scenes,
each with the red and blue instruction. Dataset indices were
`0,1,64,65,128,129,256,257`. This is a latency and numerical-parity sample,
not a new held-out task-success evaluation. The default-size depth test used
three timed observations after one warmup; the optimized 224 pipeline used
eight. Percentiles from these small samples are descriptive, not deadlines.

The PerAct model is the existing **1,578,525-parameter reaching policy**,
40³ voxels, width 128, 128 latents, two transformer layers, using upstream
commit `02fb87681c5a47be9dbf20141bedb836ee2d3ef9`. It is not the published
multitask RLBench model. The checkpoint contains cached CLIP features for the
two trained instructions; fresh language encoding is excluded.

Depth uses the repository's **Depth Anything V2 metric Hypersim Small**,
`vits`, max depth 20 m, upstream commit
`a561b849ebae10a6f5ef49e26c83cbbcd36c71bf`. The 224 input is an explicit
resolution experiment. Timing inputs were square; preserving a different
camera aspect ratio can increase the network dimensions and cost.

Checkpoint SHA-256:

- PerAct: `f0541b341734e8e6831713088f5a7c7aa3a2e2a2fc9b3e8f133fb0bbfd52bf3a`
- Depth: `b782898d8a3e8be1f639de33837ed85e9b4b73e40f8f5e5cd99067588d722545`

## Measured latency and memory

Each row ran in a fresh process. One warmup precedes the timed samples.
Pipeline time includes image preprocessing, depth inference, output resizing,
world-point projection, PerAct voxelization, policy inference and action
decoding. It excludes file loading, camera capture, ROS transport, IK,
collision/path planning and actuator execution. Both networks remain loaded
in the combined runs; they execute sequentially.

| Runtime and workload | Timed samples | Median | p95 | Peak process RSS |
| --- | ---: | ---: | ---: | ---: |
| PyTorch PerAct, existing RGB-D | 8 | 4.194 s | 4.255 s | 400 MiB |
| PyTorch depth, 518×518 | 3 | 10.485 s | 11.842 s | 713 MiB |
| PyTorch depth 224×224 + PerAct | 5 | 6.204 s | 6.240 s | 644 MiB |
| ONNX PerAct, existing RGB-D | 8 | **1.997 s** | 2.275 s | 448 MiB |
| ONNX depth 224×224 + PerAct | 8 | **3.580 s** | 3.607 s | **624 MiB** |
| ONNX depth 518×518 + PerAct | 3 | **9.887 s** | 9.917 s | **841 MiB** |

The optimized 224 pipeline's median stages were 1.324 s depth, 0.97 ms
projection and 2.252 s PerAct. For 518, they were 7.837 s, 0.94 ms and
2.031 s. Independent medians need not sum exactly to the total median.

During optimized combined runs, sampled Linux `MemAvailable` stayed above
822 MiB at 224 and 588 MiB at 518. Maximum sampled thermal-zone readings were
57.0°C and 60.2°C. Existing system zram use rose to approximately 84 MiB;
RSS is process memory, not total-board usage. This was a short benchmark,
not a sustained thermal or power-consumption test.

Optimized pipeline imports took about 7 seconds, followed by about 3 seconds
to load models. These startup costs are excluded from the warm timings.
The adapter retains PyTorch for the original voxelizer and output resizing;
this is not a minimal-memory standalone ONNX application.

## Numerical checks and failed acceleration attempts

With the original simulator-derived points, both PyTorch and ONNX produced
**8/8 identical decoded actions** to the desktop reference: translation,
rotation, gripper and collision flag. ONNX network outputs passed
`atol=1e-3, rtol=1e-3` against PyTorch. Maximum depth difference on the reference
input was 0.000034 m at 224 and 0.0000065 m at 518.

Network-only timings, using fixed preprocessed input, were 1.952 s PerAct,
1.307 s depth at 224, and 9.170 s depth at 518. These separate runs are not
added to estimate the combined pipeline; the actual combined measurements
above are authoritative for this test.

Two additional runtime paths were investigated and **excluded**:

- NCNN 1.0.20260526, Vulkan, explicitly selecting the Adreno GPU rather than
  llvmpipe: model loading completed, but inference failed with
  `vkWaitForFences failed -4` (device lost). This is not a successful GPU
  benchmark. The same exported 224 model on NCNN CPU ran in 0.978 s but
  differed from PyTorch by 1.238 m mean absolute depth, so that apparent
  speedup is invalid. PNNX emitted unsupported batch-permutation warnings.
- MNN 3.6.1: its prebuilt ARM64 wheel did not contain the OpenCL backend.
  Host validation of the depth conversion failed (1.649 m mean absolute
  difference), and its PerAct session crashed during input transfer. These
  exports were not accepted for deployment or counted as board speedups.

The valid results therefore establish **CPU inference**, not usable GPU
acceleration for these models. A more complete converter/driver port could
change performance; this benchmark does not establish a hardware limit.

## Functional limit: estimated depth is not interchangeable with simulator depth

Passing the numerical checks establishes that ONNX preserves the original
networks. It does not establish that Depth Anything supplies sufficiently
accurate metric geometry to PerAct.

For the first recorded scene, the original RGB-D has 1,896 points inside the
configured workspace. Reconstructing points from the uncalibrated Hypersim
predictions put **zero points inside that workspace**, at both 224 and 518.
Median absolute depth error on that image was 1.730 m and 0.567 m,
respectively. Projection used fixed camera rays and extrinsics; reference
depth scale was not supplied to the estimated-depth pipeline.

On the eight recorded observations, original-depth PerAct predictions had
8.3 mm median distance to their demonstration targets. Substituting 224
monocular depth increased it to **174 mm median, 301 mm maximum**. The three
timed 518 observations also produced incorrect target positions. These are
offline prediction errors on training scenes, not physical reaches or a
general depth-accuracy benchmark. An empty-workspace policy prediction must
not be treated as a valid arm command.

This stack needs near-field metric-depth validation/calibration and training
appropriate to its observations before controlling an arm. The existing
model only demonstrates reaching; it does not demonstrate grasping.

## Implication for a minimum-cost arm

The **2GB board fits the tested compact models**. For a slow arm, approximately
3.6 seconds per fresh low-resolution observation is a plausible compute
budget; camera, ROS, IK and planning need additional measurement and margin.
Planning from a predicted next scene could overlap roughly 2 seconds of
PerAct computation with the current movement, subject to validating that
prediction. No speculative scheduler was implemented or tested here.

Default-resolution depth increases the measured cycle to about 10 seconds.
The 4GB variant would provide memory headroom; RAM alone does not establish
faster inference. These measurements do not justify claiming subsecond
control, full-size PerAct support, or a working physical grasping system.

## Artifacts and reproduction

Scripts retained in this repository:

- `tools/benchmark_unoq_arm.py`: PyTorch and validated ONNX pipeline timing.
- `tools/export_unoq_benchmark_models.py`: fixed-shape model exports and
  PyTorch numerical references; MNN/NCNN exports are experimental and failed
  validation above. Conversion was performed using PyTorch 2.5.1, ONNX
  1.19.1, MNN 3.6.1 and PNNX 20260526 in a temporary depth-image container.
- `tools/benchmark_unoq_runtime.py`: network-only timing and numerical checks;
  exits with status 2 for an output mismatch.

Local, ignored artifacts are in `generated/unoq_benchmark_20261012/`:
`assets/` contains the pinned source slices, model weights and observation
sample; `export/` holds the exported networks and references; `wheels/`
holds the ARM64 dependencies; `results/`, `results-final.tar.gz` and
`summary.json` retain the raw measurements. The summary beside this report
preserves the principal numbers without committing model binaries.

The observation sample was extracted from the dataset produced by
`tools/prepare_peract_demo.sh`; the weights were copied from the existing
PerAct checkpoint and depth-image checkpoint. Sources were copied from the
pinned upstream revisions. ADB transferred the assets and offline wheels.
For an equivalent Python environment on the board, the core commands are:

```bash
python benchmark_unoq_arm.py --assets assets --mode peract --threads 4 \
  --runs 8 --output peract_t4.json
python benchmark_unoq_arm.py --assets assets --models optimized \
  --backend ort --mode pipeline --depth-size 224 --threads 4 --runs 8 \
  --output pipeline224_ort_full.json
python benchmark_unoq_arm.py --assets assets --models optimized \
  --backend ort --mode pipeline --depth-size 518 --threads 4 --runs 3 \
  --output pipeline518_ort_full.json
```

All temporary files installed on the UNO Q and both host helper containers
were removed after retrieval. Its firmware, existing images, applications,
services and system packages were not changed. The benchmark does not
connect to or issue commands to actuators.
