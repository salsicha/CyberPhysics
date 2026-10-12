# Validation records — 12 October 2026

See [the report](../../hardware_validation_20261012.md) for setup, scope and
interpretation. These compact reports retain the recorded measurements and
acceptance decisions; per-frame recordings and images are in the ignored
`generated/` directories listed there.

- `pi_*`: OAK stream measurements, configured MAVROS rates and offline-FC
  Compose startup. The latter records include intermediate diagnostic commands.
- `turret_*`: strict scorer results and timestamp-aligned frame delivery.
  Failures of the stock segmentation/tracking model are deliberately retained.
- `gps_*`: GPS-denied simulator scores, with a separate negative control.
- `drone_launcher_ros.json`: simulated-operator ROS wiring, including a
  wall-clock negative case. Explicit test joint commands exercised the
  virtual plant in both modes; they are separate from automatic requests.

Aircraft mass is a CAD/BOM estimate, and no physical Jetson GPU was tested.
