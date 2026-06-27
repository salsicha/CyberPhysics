# Turret System

This system validates a PanTiltROS-compatible pan/tilt perception turret in a
headless deterministic simulation. It preserves the `pan_joint` and
`tilt_joint` actuator contract from `applications/pantiltros`, publishes the
same logical camera, segmentation, target, command, joint-state, diagnostics,
and emergency-stop streams in telemetry, and scores whether a selected moving
object is tracked through distractors and occlusions.

## Contents

- `scenarios/warehouse_tracking.json`: realistic warehouse/lab aisle scenario
  with moving target, distractors, occluders, camera model, YOLO segmentation
  contract, servo limits, and topic names.
- `scripts/turret_sim.py`: deterministic full-loop simulator for camera
  projection, YOLO-style instance segmentation output, target selection,
  closed-loop pan/tilt control, servo response, diagnostics, and metrics.
- `scripts/score_tracking_task.py`: acceptance checker for simulation telemetry
  or a directory of telemetry JSON files.
- `validation/acceptance_thresholds.json`: default turret tracking acceptance
  thresholds.

## Smoke Run

```bash
python3 systems/turret/scripts/turret_sim.py   --scenario systems/turret/scenarios/warehouse_tracking.json   --output /tmp/turret_tracking_metrics.json

python3 systems/turret/scripts/score_tracking_task.py   --metrics /tmp/turret_tracking_metrics.json
```

The simulator is intentionally offline-friendly. CPU smoke validation uses a
simulated YOLO26 segmentation backend driven by scenario ground-truth masks.
GPU/nightly validation should replace that backend with the current installed
Ultralytics YOLO26 `-seg` checkpoint and compare the model masks against the
same scenario masks and telemetry contract.

## Hardware Run

```bash
docker compose --env-file systems/turret/config/arduino.env \
  -f compositions/turret_hardware.yaml up
```

Use the `arduino` profile to open the Arduino application container for
firmware work:

```bash
docker compose --env-file systems/turret/config/arduino.env \
  -f compositions/turret_hardware.yaml --profile arduino up turret_arduino
```

The default hardware stack starts the PanTiltROS-compatible Arduino serial
bridge. The Arduino firmware should own the ST3215/STS3215 electrical bus and
return actuator feedback over the configured serial device.

## Topic Contract

The telemetry samples use these canonical names:

- `/turret/camera/image_raw`
- `/turret/camera/camera_info`
- `/turret/yolo/segments`
- `/turret/yolo/target_mask`
- `/turret/yolo/target_bbox`
- `/turret/yolo/target_confidence`
- `/turret/target/pixel_error`
- `/turret/target/angular_error`
- `/turret/target/selected_id`
- `/turret/target/locked`
- `/turret/joint_commands`
- `/turret/joint_states`
- `/turret/diagnostics`
- `/turret/emergency_stop`

`/turret/joint_commands` and `/turret/joint_states` use
`sensor_msgs/JointState` semantics with `pan_joint` and `tilt_joint`, matching
the PanTiltROS control stack. The control loop consumes simulated detection
centroids and camera intrinsics; ground-truth object pose is reserved for
rendering, synthetic YOLO smoke masks, and scoring.
