# Requested-task completion audit — 12 October 2026

The earlier work did not close every item. This follow-up found a failing VINS
CI runtime check, an x86 PyTorch image-build error, a Space ROS DDS exception,
and remaining turret perception/tracking failures. They were investigated and
corrected; physical acceptance and unavailable learned-arm checkpoints remain
explicit limits rather than successful tests.

## Request-by-request status

| Request | Result and remaining evidence |
|---|---|
| Review hardware STLs without editing them | Complete read-only review: 145 meshes pass closure, winding, finite-coordinate and net-volume checks. [Detailed feedback](printable_parts_review_20261012.md) covers purpose, fit-test priorities and launcher limitations. No printable geometry changed. |
| Native Jetson Orin images, JetPack 7.2 | Native ARM64 nvblox, CUDA YOLO and CUDA Depth Anything images all built successfully in [CI](https://github.com/salsicha/CyberPhysics/actions/runs/38307403741). Installation checks and actual-GPU smoke commands are supplied. Physical Orin memory/throughput tests require access to that device. |
| Persist 4 MiB receive buffers on installation | Installer writes and applies host sysctl settings, preserving higher limits; DDS preflight detects regressions. The Pi follow-up confirms both limits remain 4,194,304 bytes. Native Zenoh does not need the DDS UDP preflight. |
| Default Gazebo velocity-frame correction and new GPS-denied baselines | Complete: conversion enabled by default; full denial/recovery trials and negative control recorded. Both Gazebo and Isaac also passed after the Zenoh migration. [Baselines](gps_denied_sim_test.md). |
| Next-target selection and centered automatic triggering | Implemented and tested for standalone turret, car adapter and drone stand-in. Automatic requests remain simulation-only. Three real-model standalone simulated shots now complete at three distinct IDs. |
| Evaluate standalone segmentation, tracking and missed frames | Complete evaluation, plus fixes and a trained synthetic checkpoint. Five unchanged acceptance scenarios pass; every published frame receives a result. Target detection still has individual misses; see measured table below. |
| Clean up printable-part toolchain | Complete: surface freshness, partial/read-only checks, validation failure handling, cleanup and artifact paths fixed. Full CAD/metadata/docs suite passed 258 tests; follow-up focused checks passed. |
| Match aerial simulators to STL airframe and verify 3.17 kg | Both consume the same assembled geometry, COM, inertia and propulsion data. Loaded CAD/BOM estimate is **3.164661 kg**. Actual aircraft weighing remains unavailable. |
| Pi MAVROS CPU, offline-FC startup, Docker ownership, BlueOS logs | Complete and tested on 192.168.1.193: configured idle CPU about 4.8% of one core under Zenoh; detached startup returns with FC unavailable; ownership repaired; lossless log archival reclaimed 597 MiB with active maintenance timer. Existing BlueOS containers remain running. |
| Attached OAK-1 hardware test | Complete device/stream tests on the Pi and across the LAN: matched tracking timestamps, finite relative depth and no unmatched tracking frames. USB 2 was negotiated. An uncontrolled view cannot establish target/depth accuracy. |
| Replace ROS 2 DDS backend with Zenoh | Native `rmw_zenoh_cpp` is the shared default. Real topics, services, actions, restart recovery, Pi camera traffic and flight simulations passed. Space ROS core and message dispatchers are also rebuilt and tested. The micro-ROS XRCE agent/subscriber pair retains its explicit DDS boundary; it requires a distinct firmware/agent migration. |
| Test transformer end-effector/arm workflows and fix issues | SmolVLA inference fits (~933 MiB allocated); PushT completes 3/3 tested episodes (~1,099 MiB). Normalization, strict loading, APIs, telemetry and access checks fixed. π0 and GR00T hit authenticated gated-model denials; GR00T additionally requires a larger supported GPU budget. No matching trained SO101 checkpoint is available. [Full findings](arm_policy_validation_20261012.md). |
| Assess actuated launcher magazine/chambering mechanics | CAD/control review and simulated cycles complete. Design has an integrated six-dart clip, not a standard removable Nerf magazine. Chambering occurs during each next firing cycle. Reliable physical feeding, sealing and repeated launching require an assembled mechanism; the servo/sear model is not a feed-physics test. |

## Follow-up turret results

The YOLOE-26s fixture was trained on 480 randomized renderer scenes and validated
on 96 separate seeded scenes. The original acceptance trajectory/frames were
excluded. Held-out mask precision/recall are 97.9%/94.5%, mask mAP50 0.981 and
mAP50–95 0.723. These are synthetic-domain results; the hardware model default
has not been changed to this checkpoint. Reproduction, hashes and epoch metrics
are in [the model directory](../applications/turret_sim/models/README.md).

The tracker now recognizes promptable versus trained model heads without
using the filename, requests camera-resolution masks, suppresses near-identical
same-class duplicate boxes, and retains an existing startup target ID.
The simulator waits for graph discovery before its first frame. An observed
3.05-second discovery delay previously discarded 59 initial results; no
scenario frames are removed by the new startup check.

| Native Zenoh scenario | Processed / published | Selected-target detection | Mean mask IoU | Aim error p95 | Strict acceptance |
|---|---:|---:|---:|---:|---|
| Nominal | 361 / 361 | 90.9% | 0.798 | 1.32° | Pass |
| Low light | 361 / 361 | 93.5% | 0.805 | 1.45° | Pass |
| Glare / motion blur | 361 / 361 | 95.0% | 0.821 | 1.93° | Pass |
| Camera dropout | 349 / 349; 12 deliberately suppressed | 91.2% | 0.786 | 1.50° | Pass |
| Command dropout | 361 / 361 | 90.9% | 0.794 | 1.48° | Pass |

No acceptance threshold was relaxed. Capture-to-result p95 was 22–30 ms.
A stock-checkpoint comparison with the same tracker fixes also processed
361/361 frames, but had 124 empty masks among 341 visible frames and failed
strict acceptance (63.6% sampled detection, 0.504 mean IoU). The trained
fixture improves recognition within the synthetic domain; code fixes alone
do not establish a capable real-camera model.
These results distinguish delivery from recognition: the nominal aligned
measurement still has 31 empty selected-target masks among 341 visible frames.

With the simulated launcher fitted, the real tracker fired once each at the
blue cart, red box and green cart. Fire-to-release was 2.10 seconds each;
true aim errors were 1.31°, 0.26° and 0.71°. All three distinct IDs completed,
with zero unexplained actuator commands. The evaluator now matches shot truth
to the selected object after advancement, instead of incorrectly comparing
all later shots with the original blue cart. No physical actuator was fired.

## Follow-up builds and middleware

The x86 PanTiltROS build failed because `torch.__version__` includes a CUDA
suffix absent from its PyPI distribution version. Constraints now use installed
package metadata, and the actual image builds. CI now builds this x86 path as
well as ARM64. The VINS `--network none` runtime check now starts a local
loopback Zenoh router instead of relying on unavailable multicast interfaces.
The actual image passed feature flow, 40 odometry messages, 21 graph keyframes,
40 trajectory rows and all SIGINT/SIGTERM shutdown checks.

Space ROS required both a client-library rebuild and regeneration of its
message dispatchers; changing the environment variable or rebuilding only
`rcl` was insufficient. Exact source revisions come from the pinned base's
manifest. Its native-Zenoh full Gazebo test activated all six controllers and
received images, scans, joint states, clock and odometry, with responding
movement/stop services. The smaller lunar-rover graph also advanced waypoints
with 241 odometry messages and 240 nonzero commands in 12 seconds. See
[the middleware report](zenoh_validation_20261012.md).

## What cannot yet be claimed

- Physical Orin GPU acceptance: no Jetson address/access was provided.
- An actually weighed 3.17 kg aircraft or demonstrated jam-free launcher:
  no assembled aircraft/launcher was available.
- A complete learned SO101 manipulation demonstration: matching trained
  checkpoint/normalization data are absent, two backbones deny access, and
  the existing Isaac grasp is kinematic rather than a validated contact model.
  The scripted baseline is explicitly rejected as VLA success.

Those constraints cannot be resolved by passing unit tests or reporting a
scripted demonstration as learned behavior. All compact measurements are in
[the validation directory](validation/20261012/README.md).

Final non-CAD regression result: **2,145 passed, six skipped** in 73.17 seconds.
Python error-class lint, shell error checks and Compose environment-default
validation also passed. Printable geometry was unchanged during this audit.

The final Space ROS image built from the repository Dockerfile and passed
its native build-time message/service check, a real Fast DDS fallback
node/publisher check and the full six-controller Gazebo smoke test.
Image: `sha256:4d34a56ff757be3ae5d0cec437978cae9436539999d95ea0b41a64fc2c31fab9`.
