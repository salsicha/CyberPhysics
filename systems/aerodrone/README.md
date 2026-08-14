# Aerodrone

This is the canonical definition of the repository one quadrotor. The physical deployment is `compositions/aerodrone_hardware.yaml`; Gazebo, Genesis, and Isaac compositions model this same vehicle and mount this directory for the same controller and interface configuration.

- `config/arducopter.params` is the baseline Navigator/ArduCopter vehicle configuration. A simulator may add backend-specific SITL tuning under its own application directory, but that does not define another drone.
- `config/topics.env` defines the stable Aerodrone and simulated sensor topic contract consumed by the navigation stack.
- `config/sensors.yaml` records the physical sensor equivalents, topics, and nominal operating limits. Serial-number-specific camera intrinsics and measured mounting extrinsics should be added here when calibration is performed on the built aircraft.

Simulation terrain, missions, models, scripts, and acceptance tests are not physical-system configuration and intentionally do not live in this folder.
