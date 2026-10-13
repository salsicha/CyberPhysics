# Validation records — 12 October 2026

See the [hardware report](../../hardware_validation_20261012.md),
[arm-policy report](../../arm_policy_validation_20261012.md), and
[Zenoh report](../../zenoh_validation_20261012.md) for setup, scope and
interpretation. These compact reports retain the recorded measurements and
acceptance decisions; per-frame recordings and images are in the ignored
`generated/` directories listed there.

- `pi_*`: OAK stream measurements, configured MAVROS rates and offline-FC
  Compose startup. The latter records include intermediate diagnostic commands.
- `turret_*`: strict scorer results and timestamp-aligned frame delivery.
  Failures of the stock segmentation/tracking model are deliberately retained.
- `gps_*`: GPS-denied simulator scores, with a separate negative control.
- `zenoh_*`: native middleware recovery/transport tests, hardware checks and
  new flight scores. The earlier `gps_*` and `pi_*` records use DDS.
- `arm_*`: real model evaluation, explicit access/prerequisite failures and
  the scripted scene baseline. A rejected base checkpoint is a recorded
  limitation, not a successful learned arm task.
- `drone_launcher_ros.json`: simulated-operator ROS wiring, including a
  wall-clock negative case. Explicit test joint commands exercised the
  virtual plant in both modes; they are separate from automatic requests.

Aircraft mass is a CAD/BOM estimate, and no physical Jetson GPU was tested.

Follow-up audit records:

- `stl_mesh_review.json`: all 145 hardware meshes and manifest fit-evidence counts.
- `image_build_ci.json`: all eight image-build jobs passing for the earlier migration commit.
- `turret_trained_*`: actual native-Zenoh, GPU-model tracking scores and three-target simulated launcher cycle.
- `zenoh_spaceros_gazebo.json`, `zenoh_spaceros_lunar.json`: rebuilt native Space ROS runtime checks; the older exception record is historical.
- `pi_followup_health.json`: ownership, persistent buffers, maintenance timer and preserved BlueOS containers.

See the [complete request ledger](../../task_completion_20261012.md).

The subsequent [VLA and embedded Zenoh report](../../vla_embedded_validation_20261012.md)
uses `arm_workcell_contact.json`, `embedded_zenoh_tcp.json`,
`embedded_zenoh_serial.json` and `embedded_zenoh_builds.json` in this directory.
It preserves failed VLA acceptance as well as successful MCU protocol checks.

Turret `summary.yolo_contract` retains the acceptance scenario's original
reference-model settings. Each follow-up wrapper records the checkpoint and
confidence actually used, plus hashes of the bind-mounted runtime source.
