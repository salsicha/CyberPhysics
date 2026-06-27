# Submarine

This application provides a lightweight ROS 2 bridge for a BlueROV2-class
submarine running BlueOS, ArduSub, Navigator, and MAVROS.

The image inherits from `cyberphysics/aerostack2` so it can run MAVROS and the
bridge from one ARM64-capable container image on a Raspberry Pi/BlueOS host.

## Build

```bash
make -C applications target_arm64 build_submarine
```

## Runtime Contract

The bridge subscribes to MAVROS telemetry and republishes CyberPhysics
submarine topics:

- `/sub0/sensor_measurements/gps`
- `/sub0/sensor_measurements/odom`
- `/sub0/sensor_measurements/imu`
- `/sub0/sensor_measurements/pressure`
- `/sub0/sensor_measurements/battery`

If `enable_velocity_setpoints` is set to `true`, it also converts
`/sub0/cmd_vel` into MAVROS velocity setpoints. Leave that disabled until
ArduSub mode, arming, leak detection, depth failsafes, and manual override have
been validated in a controlled test tank.

## Run

```bash
docker compose --env-file systems/submarine/config/blueos.env \
  -f compositions/submarine_hardware.yaml up
```

System-specific parameters live in `systems/submarine/config`.

## Simulation

The closed-loop simulation validates the same bridge and command path without
BlueOS hardware:

```bash
docker compose -f compositions/submarine_sim.yaml up
```

The scenario is a submerged pipeline inspection in
`systems/submarine/config/pipeline_inspection.yaml`. `bluerov2_sim.py`
publishes MAVROS-like GPS, odometry, IMU, pressure, and battery topics, while
`pipeline_waypoint_nav.py` consumes the bridged submarine odometry and publishes
`/sub0/cmd_vel`.
