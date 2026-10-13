# VLA arm and embedded Zenoh follow-up — 12 October 2026

The repository now includes a runnable **trained arm policy with contact
physics**, and native Zenoh firmware replacing the default micro-ROS demos.
The firmware builds and host protocol tests pass. The learned policy completes
many placements, but **fails the measured reliability and collision-free
acceptance criteria**. No physical Pico, ESP32 or arm was available, and no
unrelated attached board was flashed.

## Learned arm

[`smolvla_workcell.yaml`](../compositions/smolvla_workcell.yaml) runs public
checkpoint `ahmedsohail2003/smolvla-so101-pickplace-v2` in its exact matching
MuJoCo workcell. Despite the checkpoint name, that workcell contains an
**SO-ARM100**, not this repository's SO101 Isaac URDF. The new route is separate
so its radian joint/jaw actions cannot silently be interpreted as SO101
degrees or linear gripper travel. Build/run instructions and immutable model
and environment revisions are in [WORKCELL.md](../applications/smolvla/WORKCELL.md).

The actual policy receives front RGB, wrist RGB, six joint angles and text.
Weights load strictly with the checkpoint's saved normalization tensors.
The block moves through rigid-body contact and friction; no attach, snap,
object-pose input, scripted planner or fallback is used. Simulator truth is
only used for scoring and applying the declared disturbance.

All seven conditions use the same 20 pose/policy seeds, beginning at 123.
The episode cap is 240 control intervals (15.84 simulated seconds). To count,
the block must have bilateral jaw contact during a lift of at least 6 cm,
then remain supported, released, still and wholly inside the tray for 0.5 s.
The upstream scorer could count a block still airborne or overhanging the tray;
this evaluator rejects those false positives and measures every contact step.

| Condition | Successful task placements | Collision-free task successes |
|---|---:|---:|
| Nominal | 14/20 | 0/20 |
| Visual randomization | 9/20 | 0/20 |
| Mass/friction randomization | 17/20 | 0/20 |
| Combined randomization | 10/20 | 0/20 |
| Temporary front-view occlusion | 14/20 | 0/20 |
| External block disturbance | 15/20 | 0/20 |
| Same-task language paraphrases | 18/20 | 0/20 |
| **Total** | **97/140** | **0/140** |

Placement is not the same as policy acceptance. The runner returns nonzero
unless nominal trials exist, every evaluated condition reaches 80% task
success, and every trial is collision-free. Failed reports and videos are
still saved. No condition is silently dropped and no threshold was lowered.
The policy makes unwanted table contacts and sometimes misses or drops the
block. Large reported contact forces are MuJoCo solver measurements, not
validated estimates of physical servo or gripper forces.

On the RTX 4000 Ada Laptop GPU (12,282 MiB), Torch used about **920 MiB peak
allocated / 956 MiB reserved**. Those counters exclude graphics and CUDA
context. Fresh 50-action chunks took about **125 ms median / 131 ms p95** in
the baseline. Most `select_action` calls return cached actions; their roughly
2–3 ms latency is not fresh model inference. The simulator pauses during
inference, so these offline rollouts do not establish deadline-safe 15 Hz
hardware control. A separate five-pose probe with an 8-action horizon scored
2/5 nominal, 2/5 combined, 4/5 occlusion and 3/5 disturbance; it did not justify
changing the checkpoint's default. Those probe seeds were separate from the
140-episode baseline.

The checkpoint was trained for only red-block-to-blue-tray. Paraphrases of
that task do not establish arbitrary instruction following, distractor
selection or a second learned task. The new demo therefore closes the absence
of a runnable learned/contact arm example, while **robust manipulation remains
unfinished**. Improving the pretrained behavior needs contact-aware recovery
and visual-variation training, followed by new held-out trials. The original
SO101 Isaac path additionally needs its own trained embodiment and contact
gripper. π0/GR00T access and memory constraints from the
[earlier report](arm_policy_validation_20261012.md) remain unresolved.

## Native MCU transport

[`applications/embedded_zenoh`](../applications/embedded_zenoh/README.md) ports
the existing LED and greeting topics to Pico-ROS/zenoh-pico. This removes the
XRCE/DDS boundary for these examples, preserving their ROS message types,
names and node name. It is a client firmware replacement, not a port of the
entire rclc/micro-ROS API. Existing flashed XRCE firmware still needs the
explicit `arduino_xrce.yaml` fallback and Fast DDS. Legacy flashing is now an
opt-in `flash` profile rather than part of ordinary startup.

| Target | Build result | Measured size |
|---|---|---|
| Pico 2 W / RP2350 USB | UF2 and ELF compiled | ELF text 451,944 B; BSS 187,132 B; data 0 B |
| ESP32 + LAN8720 Ethernet | PlatformIO firmware compiled | Static RAM 45,196 / 327,680 B; flash 859,977 / 1,310,720 B |
| Linux native C harness | Built from the same application sources | Passed actual Jazzy Zenoh interoperability |

These static section sizes do not measure live stack/heap margins. Both MCU
cross-builds were run on amd64; physical-board runtime acceptance is pending.
The ESP32 build is self-contained and no longer needs the full Arduino IDE
image. The router can use the existing ROS arm64 image on a Pi or Jetson.

The tests exercised real ROS graph discovery and two-way Int32/String traffic,
including signed extremes, UTF-8, the 120-byte name limit, both CDR byte
orders, oversized/malformed messages, router restart and client restart.
They ran with `--network none`, entirely inside the container's loopback
network. Serial tests use paired pseudo terminals and the actual Zenoh serial
framing. They cannot substitute for testing TinyUSB, MCU scheduling or PHY wiring.

| Topology | Main router recovery | Serial router recovery | Result |
|---|---:|---:|---|
| C client → TCP router → native ROS | 2.064 s | — | Pass |
| C client → serial router → TCP router → native ROS | 2.266 s | 10.279 s | Pass |

Tests exposed and fixed four transport problems in the pinned fork: TCP EOF
otherwise busy-loops until lease expiry; a lost serial startup handshake can
block forever; a serial RESET needs a fresh transport handshake; and unplugged
USB writes need a bounded deadline. The host tests verify EOF/read/reset
recovery. The analogous TinyUSB changes compile and still need hardware tests.
The default composition separates TCP and USB routers so an absent MCU does
not prevent the rest of the ROS graph starting. A USB device-node replacement
may require recreating the Docker serial service after unplugging.

## Evidence and regression

Compact checked-in records:

- [Arm rollouts](validation/20261012/arm_workcell_contact.json).
- [TCP](validation/20261012/embedded_zenoh_tcp.json) and
  [routed serial](validation/20261012/embedded_zenoh_serial.json) protocol results.
- [Image, firmware and source hashes](validation/20261012/embedded_zenoh_builds.json).

Local videos/trajectories are in `generated/arm_validation/workcell_release`;
compiled firmware is in `generated/embedded_zenoh/firmware`. Public weights
remain in the named model cache and can be downloaded from the pinned upstream
revision. Binary firmware and model caches are not added to Git.

The non-CAD regression suite passed **2,162 tests, six skipped** in 66.75 s.
The first sandboxed attempt could not create test Unix sockets or run
LeakSanitizer under ptrace; the unchanged tests passed with those restrictions
removed. Python error-class lint, Compose rendering/default checks and build
path regression checks passed. No printable meshes changed. CI now builds both
firmware targets and runs TCP/routed-serial tests; GPU policy rollouts remain
local workstation validation, and failed policy acceptance is retained.
