# Aerodrone

This is the canonical definition of the repository one quadrotor. The physical deployment is `compositions/aerodrone_hardware.yaml`; Gazebo, Genesis, and Isaac compositions model this same vehicle and mount this directory for the same controller and interface configuration.

- `config/arducopter.params` is the hardware baseline Navigator/ArduCopter configuration: the file to load on the aircraft. It keeps pre-arm checks and the RC/GCS failsafes enabled.
- `config/arducopter_sitl.params` holds the SITL-only relaxations (no arming checks, failsafes or auto-disarm, because the simulators have no RC transmitter or GCS). `applications/ardupilot_sitl/scripts/run_arducopter_sitl.sh` layers baseline, SITL relaxations, optional features, then backend physics compensation (`applications/{genesis,isaac}/config/aerodrone_sitl.params`). Never load the SITL file on hardware.
- `config/arducopter_gps_input.params` is the optional, experimental GPS-denied feature: DemNav/WildNav corrections reach ArduPilot as GPS 2 via MAVLink GPS_INPUT from `applications/aerostack2/scripts/ardupilot_navigation_bridge.py` (`GPS_INPUT_CORRECTIONS=true`). See `tools/gps_denied_sitl_test.md` for what it does and its measured limits.
- `CAMERA_MOUNT_YAW_DEG` in `config/topics.env` must match the physical OAK-1 mount (heading of the image top relative to the nose, counter-clockwise from above; 0 = image top toward the nose).
- `config/topics.env` defines the stable Aerodrone and simulated sensor topic contract consumed by the navigation stack.
- `config/sensors.yaml` records the physical sensor equivalents, topics, and nominal operating limits. Serial-number-specific camera intrinsics and measured mounting extrinsics should be added here when calibration is performed on the built aircraft.

Simulation terrain, missions, models, scripts, and acceptance tests are not physical-system configuration and intentionally do not live in this folder.
