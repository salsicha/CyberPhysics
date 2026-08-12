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
