# Turret simulator

This application is the simulated environment for the shared PanTiltROS stack.
It renders a warehouse camera stream, simulates pan/tilt actuator dynamics, and
records/scores telemetry.  Production YOLO inference and closed-loop control do
not live here; both are provided by `applications/pantiltros` over ROS 2.

## Run the real YOLO path

```bash
docker compose -f compositions/turret_sim.yaml up --build
```

The `pantilt_yolo` service first downloads and warms the default
`yoloe-26s-seg.pt` model if it is not already in `cache/turret/models/`. Once its
healthcheck is ready, `turret_sim` publishes the synthetic camera and joint state
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

The stock open-vocabulary model exercises actual inference and control, but the
strict thresholds are intended for a turret checkpoint trained on representative
camera data.

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
early (a fault) when the tracker loses the lock, and with the stock YOLOE
checkpoint it often does: the synthetic cart comes as several
low-confidence boxes, missing from about a third of the frames, and the
start-up selection jumps between their IDs. The fault latches, so this
operator stops there. `TURRET_TAG_LOCK=true` makes the operator tap the
target's box first (`{"action": "lock", "target_id": <id>}`), so the
tracker follows that one ID, and acknowledge a fault as on the page
(disarm for 0.5 s, then arm; a primed launcher is cleared by hand first, no
shot), so it fires again; that changes the tracking (a lock sticks to a
mis-detected box, the disarms stop the turret), so the tracking metrics of
such a run are not comparable. Even locked, the cart's ID often does not
last the 2 s from Fire to the shot with the stock checkpoint (the box is
lost while priming or chambering); the cycle time on this launcher model
with a steady target is checked in `tests/test_turret_launcher.py` (a shot
2.04 s after Fire, ready again after 2.47 s; one closed-loop run with the
lock timeout raised to 2 s fired 2.05 s after Fire). The model moves the
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
