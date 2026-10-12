# Turret simulator

This application is the simulated environment for the shared PanTiltROS stack.
It renders a warehouse camera stream, simulates pan/tilt actuator dynamics, and
records/scores telemetry.  Production YOLO inference and closed-loop control do
not live here; both are provided by `applications/pantiltros` over ROS 2.

## Run the real YOLO path

```bash
git lfs pull --include='applications/turret_sim/models/*.pt'
make -C applications build_pantiltros
docker compose -f compositions/turret_sim.yaml up --no-build
```

The Makefile rebuilds the ROS base dependency as well, so an older local
image cannot silently leave the tracker without native Zenoh support.
The `pantilt_yolo` service warms the simulation-only
`models/warehouse_yoloe26s.pt` checkpoint. Once its healthcheck is ready and
the camera/state/result graph is discovered, `turret_sim` publishes the synthetic camera and joint state
and consumes the same commands used by hardware. Metrics and sampled frames are
written under `generated/turret/`.

The simulator exits when the 18-second scenario finishes; use Ctrl-C to stop the
remaining YOLO service, or clean up with:

```bash
docker compose -f compositions/turret_sim.yaml down
```

### Annotated preview window

On a Linux desktop using X11 or XWayland, grant the container temporary display
access and enable the YOLO node's annotated OpenCV window:

```bash
xhost +si:localuser:root
TURRET_SHOW_WINDOW=true \
docker compose -f compositions/turret_sim.yaml up --build
```

Press `q` or Escape to close the window without stopping inference. Revoke the
temporary display permission after the run with:

```bash
xhost -si:localuser:root
```

The preview is disabled by default, so headless runs continue to work without a
display server.

Override runtime settings without editing Compose:

```bash
TURRET_YOLO_MODEL=/workspace/cache/models/custom-turret-seg.pt \
TURRET_YOLO_DEVICE=0 \
TURRET_YOLO_IMAGE_SIZE=640 \
TURRET_YOLO_CONFIDENCE=0.25 \
TURRET_VARIANT=low_light \
docker compose -f compositions/turret_sim.yaml up --build
```

Run the acceptance scorer after the simulation has produced metrics:

```bash
docker compose -f compositions/turret_sim.yaml \
  --profile validation run --rm --no-deps turret_score
```

The simulation checkpoint was trained on separately randomized renderer frames.
See [its provenance and reproduction instructions](models/README.md). Hardware
model defaults are unchanged. To compare the stock open-vocabulary model, set
`TURRET_YOLO_MODEL=yoloe-26s-seg.pt TURRET_YOLO_CONFIDENCE=0.003`.
The acceptance thresholds are unchanged.

The plant also carries the foam dart launcher
(`systems/racecar_turret/interfaces.md`, "Foam dart launcher";
`TURRET_ACTUATOR_FITTED`, default true): `actuator_joint` (the priming rack)
and `trigger_joint` in the joint states and commands, moving at the
ST3215's goal speed (2000 steps/s; the rack on its 13.5 mm pinion,
`pantilt.launcher` in the scenario: the design's 0.038 m draw and 0.6 rad
fire angle), a sear that catches the plunger when the rack is driven past
0.03425 m and releases it (a shot) when the trigger is next driven past
0.52 rad (`sear_catch_m`, `trigger_release_rad`), the trigger returned to 0
and the rack stopped where it is by the emergency stop and when commands
stop for 0.5 s, as the firmware does, and `/turret/actuator_ready`. The
simulator plays the operator: from `TURRET_TAG_AT` seconds (default 2) on
it reloads `TURRET_DARTS` darts (default 3) and presses Fire
(`{"action": "tag", "target_id": <locked id>}`) whenever the tracker's
`/turret/status` says the button is enabled (`fire_block` empty), up to ten
times, until the clip is shot; Fire never moves the aim. A cycle stops
early (a fault) when the tracker loses the selected ID. The stock checkpoint
can still miss a synthetic target during the cycle. `TURRET_TAG_LOCK=true`
explicitly taps its box before firing and acknowledges a fault by disarming
and rearming; such a run changes tracking behavior and is not a comparable
tracking baseline. The default trained synthetic checkpoint completed three
shots at three different IDs in the October validation, with automatic
selection advancement. Near-identical duplicate boxes are suppressed before
ID assignment, and initial class tracking preserves an existing ID.
The model moves the
servos at the commanded speed, with no torque or load. The metrics file's
`launcher` section records the presses, the acknowledged faults, the
status's `last_tag` changes and launcher phases, each shot with its cycle
time (`press_to_shot_s`, Fire to the sear's release; `press_to_ready_s`,
Fire to ready again) and the true aim error when it left
(`true_aim_error_deg`: a shot at a mis-detected box shows here), the
rack's and trigger's travel, and `unexplained_command_count` (priming or
firing commands that no accepted Fire explains; it must be 0). The tracking
summary and its scoring are unchanged. `TURRET_ACTUATOR_FITTED=false` runs
the plant without it.

`command_saturation_fraction` counts received commands at or beyond a joint
limit, `emergency_stop_violation_count` counts commands that arrive more than
`pantilt.emergency_stop_grace_s` (default two frames) after the emergency stop
was raised, and the `command_dropout` variant drops commands during its
interval (`dropped_command_count`).

## Offline regression helper

`scripts/turret_sim.py` retains a deterministic synthetic detector solely for
fast renderer/control/scoring regression tests. It is not used by either Compose
stack:

```bash
python3 applications/turret_sim/scripts/turret_sim.py \
  --scenario applications/turret_sim/scenarios/warehouse_tracking.json \
  --output /tmp/turret_tracking_metrics.json

python3 applications/turret_sim/scripts/score_tracking_task.py \
  --metrics /tmp/turret_tracking_metrics.json
```

## Contents

- `scripts/turret_ros_sim.py`: ROS camera and actuator simulator used by Compose
- `scripts/turret_sim.py`: renderer plus explicit offline regression helper
- `scripts/score_tracking_task.py`: telemetry acceptance scorer
- `scenarios/warehouse_tracking.json`: scene, target, dynamics, and topic contract
- `validation/acceptance_thresholds.json`: strict tracking thresholds

### Frame delivery versus segmentation misses

The `frame_delivery` report matches each output mask to the input image stamp
and that frame's ground truth. Empty masks count as processed results. It
reports published/processed frames, deliberate camera drops, missing results,
visible targets without a mask, wrong masks, aligned overlap and result latency.
A two-second drain at the end allows queued results to arrive. These counts
are separate from the older sampled closed-loop summary.

The 12 October 2026 desktop run processed 361/361 nominal frames after the
4 MiB DDS UDP socket fix, versus 206/361 before it. The dropout run processed
349/349 published frames; 12 were deliberately suppressed. The stock model
still returned no target mask for 132/341 visible nominal frames and failed
strict tracking acceptance. See [the measured report](../../tools/hardware_validation_20261012.md).
Run `sudo python3 tools/configure_dds_buffers.py` on a new host so the kernel
permits the requested socket sizes.

### Trained checkpoint validation

The follow-up native Zenoh runs pass all existing tracking checks in nominal,
low-light, glare/motion blur, camera-dropout and command-dropout scenarios.
Every published image receives a result; selected-target masks can still be
missing on individual frames. Startup waits up to 30 seconds for camera,
calibration, joint-state and mask-result discovery before advancing the scene.
This wait is reported separately and does not remove any scenario frames.

The launcher report matches each shot to the selected scene object; automatic
advancement no longer measures every shot against the original blue cart.
`target_id` and `scene_object_id` identify that match, and missing truth remains
null. See the [follow-up report](../../tools/task_completion_20261012.md).
