# Hardware and simulator validation — 12 October 2026

This records the changes and measurements made for the Jetson, Raspberry Pi,
printable-parts and simulator review. Raw reports remain under
`generated/pi_validation/20261012`, `generated/turret/evaluation_20261012` and
`generated/gps_denied_sim/airframe_20261012` in the validation checkout.
Compact measurement reports are retained in `tools/validation/20261012/`.

## Raspberry Pi 5 / BlueOS 1.4.2

The tested autopilot was ArduSub, disarmed in MANUAL. Tests did not arm it,
change its mode or send actuator commands. Camera testing used the attached
OAK-1 (IMX214, DepthAI 2.33.0), enumerating at USB HIGH / 480 Mb/s.

| Issue | Result |
|---|---|
| MAVROS idle CPU | Default 56-plugin process: 32.6–38.1% of one CPU. Configured 19-plugin process: 3.75–4.62%. Required navigation, GPS status, IMU, command and parameter interfaces remained available. At the configured ArduSub rates, CPU was 7.14% with attitude/raw IMU/pressure at 10 Hz and GPS at 5 Hz. |
| Compose blocked with FC offline | Hardware readiness moved inside the behavior-tree container. Actual staged stack returned from `up -d` in 3.24 s with an unreachable FC endpoint, and still waited without starting the tree 35 s later. |
| Root-owned Docker configuration | Empty `~/.docker` repaired to `pi:pi`. Installer repairs root-owned entries without following symlinks or changing other users' files. |
| Large BlueOS logs | Inactive `.tlog`/`.bin` files older than 24 h compressed and verified by SHA-256. Reclaimed 626,118,889 bytes (597 MiB); logs directory went from about 1.4 GiB to 775 MiB. Active recordings stayed intact. Hourly systemd timer installed. |

Install log maintenance on another BlueOS host with
`sudo bash tools/install_blueos_maintenance.sh`. Archives are retained; this is
lossless compression, not a hard disk quota. Restore an archive with `gunzip`
before opening it in a flight-log viewer. Long-running active logs can still
grow; they are intentionally not truncated or moved. Temporary camera/MAVROS test containers were removed
after measurement; the existing BlueOS containers stayed running.

Both `net.core.rmem_max` and `net.core.wmem_max` were initially 208 KiB and are
now 4 MiB, persisted by `tools/configure_dds_buffers.py`. Every ROS service in
the affected BlueOS compositions also loads the shared Fast DDS profile with
an 8 MiB SHM segment. Raising the host limits alone did not fix OAK images:
the default 512 KiB SHM segment was too small for a 2.76 MB high-resolution frame.

### OAK-1 delivery measurements

Each recording covers 60 seconds after a 10-second warmup. Rates are processed
pipeline rates, not the sensor frame rate. The camera was not aimed at a
controlled scene, so these results do not establish detection accuracy or
metric depth accuracy.

| Configuration | Relative depth + aligned RGB | High-resolution RGB | Tracking |
|---|---:|---:|---|
| Before transport correction, tracking active | 2.85 Hz | 11–13 of 30 expected frames, gaps up to 18 s | 5 Hz |
| Corrected, tracking active | 171 frames / 2.85 Hz | 29 frames / 0.483 Hz; max gap 2.012 s | 300 image/detection/calibration triplets; 300 matched timestamps, zero unmatched |
| Corrected, tracking paused | 270 frames / 4.50 Hz | 30 frames / 0.50 Hz; max gap 2.015 s | Not subscribed; inference pauses |

Depth was entirely finite and calibration matrices were valid. Tracking
capture-to-receipt age was about 0.31 s at the 95th percentile. Raw RGB/depth
headers are stamped at host publication, so their much smaller measured ages
must not be interpreted as sensor-to-result latency. USB 3 bandwidth was not
available on the tested connection.

## Standalone turret

The real YOLOE-26s segmentation model and tracker consumed the synthetic
warehouse camera at 20 Hz on the desktop GPU. The evaluator now matches every
output mask to its input timestamp and truth mask, including empty results;
its `frame_delivery` section separates missing results from target misses.

| Run | Published / processed frames | Visible target, empty result | Target overlap > 0.05 IoU | Result latency p95 |
|---|---:|---:|---:|---:|
| Nominal, old UDP buffers | 361 / 206 | 101 / 196 | 48.0% | 38.6 ms |
| Nominal, 4 MiB buffers | 361 / 361 | 132 / 341 | 60.1% | 30.1 ms |
| Injected camera dropout, corrected buffers | 349 / 349 (12 deliberately suppressed) | 143 / 329 | 55.9% | 29.6 ms |

The transport loss is fixed in these runs. The stock model and target-association pipeline still fail the
existing tracking acceptance thresholds: low recall and segmentation overlap
remain, even at a confidence threshold of 0.003. Missing selected-target masks
can arise from detector misses or target association; this is not a separate
measurement of raw detector recall. Correctly overlapping masks
average about 0.50 IoU. No thresholds were relaxed. Target-specific annotated
camera data and a trained/evaluated checkpoint are needed before claiming
reliable segmentation; synthetic warehouse results do not predict field recall.

The no-launcher simulator startup and emergency-stop QoS bugs were also fixed.
With its simulated launcher enabled, the detector lost the selected target
while priming; the controller aborted safely, trigger stayed at zero, and no
shot was produced. This is a valid fault-path test, not a successful firing
accuracy result.

The shared tracker/servo-plant tests also completed two centered targets with
exactly one simulated shot per ID, for both the drone and the turret adapter.
The drone stand-in requires simulation time; physical trackers only advance
selection and still need an explicit fire command. A real Jazzy ROS smoke test
of the car adapter delivered one automatic request for each of two IDs,
accepted the rack/trigger joint commands, and measured a virtual release and
joint feedback. Its Isaac profile now enables that virtual launcher; its
initial dart count remains zero until the operator reloads it. The drone
stand-in also passed a real Humble ROS smoke test: one automatic request per
ID with simulation time, zero automatic requests with a wall clock, and
working joint feedback/release events under explicit test joint commands.
The rendered car simulator also completed a 10-second smoke run, exited zero,
and reported no collisions. This checks scene/plant integration, not model
accuracy or a complete navigation course.

## Airframe and printable parts

The loaded-launcher CAD mass budget is **3.164661 kg**, about 5.3 g below the
3.17 kg claim. It includes five darts in the clip and one chambered.
This is an estimate; no assembled aircraft was available to weigh. Both
simulators now consume the same STL assembly, centre of mass, full inertia,
rotor layout, camera extrinsics and propulsion manifest. See
`hardware/aerodrone.md#simulator-geometry` for model assumptions.

The printable toolchain now catches changes to surfaces even when mesh bounds,
volume and triangle counts remain unchanged. Selected-part freshness checks,
manifest comparisons, read-only check mode, failed-validation handling and
temporary-directory cleanup were repaired. Downloaded manifests now resolve
paths within the artifact and do not advertise omitted STEP files. The full CAD/metadata/documentation
suite passed **258 tests** in 701 s; six focused surface-comparison, partial-check
and artifact-path tests passed after the final packaging changes. Committed printable STLs were not changed.

The full non-CAD CI-style suite passed **2,083 tests** with six skips. Unix
socket and firmware sanitizer tests required execution outside the restricted
sandbox. Python error-class lint, shell error checks and Compose environment
fallback checks passed. Every composition and component fragment rendered
successfully with Docker Compose. The final car/turret subset passed 209 tests
and the camera/flight composition subset passed 54 tests after integration
corrections.

## GPS-denied flight validation

The Gazebo velocity-frame conversion is now the default. New measurements use
the shared CAD airframe; see [the baseline table](gps_denied_sim_test.md#cad-airframe-baseline--12-october-2026).
An initial Isaac attempt was excluded because it ran the older image-baked
simulator against new camera settings. The composition now mounts the current
simulator source, and its runtime source/asset hashes were checked before
restarting. No navigation acceptance threshold was relaxed.

All three new-airframe Gazebo runs and the Isaac run passed the complete
150-second GPS denial and 60-second recovery. Gazebo's fused maximum error
was 11.71 m across the three runs; Isaac's was 6.13 m. The negative control
reached 144.2 m autopilot error and entered its EKF failsafe, as expected.
These measurements preserve the DDS baseline before the subsequently
requested Zenoh migration.

## Jetson and ROS middleware

Jetson images target JetPack 7.2 and ROS 2 Jazzy with Isaac ROS 4.6. Native ARM64
CUDA build paths, CI builds, installation checks and GPU smoke tests are in
place; see `applications/jetson.md`. The Pi has no NVIDIA GPU, so these tests do
not constitute physical Jetson GPU acceptance. Memory and throughput on the
actual Jetson remain to be measured.

The subsequently requested migration now selects native `rmw_zenoh_cpp` on
Ubuntu 24.04 / ROS 2 Jazzy throughout connected ROS graphs. Aerostack2 is
source-pinned, Isaac uses external Jazzy libraries, and DDS remains an explicit
fallback. The micro-ROS agent/subscriber pair remains a documented DDS exception.
The follow-up Space ROS source rebuild also migrates the rover graph to Zenoh. Deployment
instructions and required image rebuilds are in the
[ROS middleware guide](../systems/ros2/README.md); measured transport, flight
and hardware results are in the [Zenoh validation report](zenoh_validation_20261012.md).
These later changes do not turn the preceding DDS measurements into Zenoh
measurements, or establish physical Jetson GPU performance.

The separately requested transformer/arm evaluation is recorded in the
[arm-policy report](arm_policy_validation_20261012.md). It distinguishes real
model inference from the scripted arm baseline and documents the remaining
checkpoint, access and physics limitations.
