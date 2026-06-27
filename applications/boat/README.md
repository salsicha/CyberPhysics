# Boat

This application provides a lightweight ROS 2 bridge for a BlueBoat-class
surface vessel running BlueOS, ArduRover, Navigator, and MAVROS.

The image inherits from `cyberphysics/aerostack2` so it can run MAVROS and the
bridge from one ARM64-capable container image on a Raspberry Pi/BlueOS host.

## Build

```bash
make -C applications target_arm64 build_boat
```

## Runtime Contract

The bridge subscribes to MAVROS telemetry and republishes CyberPhysics boat
topics:

- `/boat0/sensor_measurements/gps`
- `/boat0/sensor_measurements/odom`
- `/boat0/sensor_measurements/imu`
- `/boat0/sensor_measurements/battery`

If `enable_velocity_setpoints` is set to `true`, it also converts
`/boat0/cmd_vel` into MAVROS velocity setpoints. Leave that disabled until
ArduRover mode, arming, failsafes, geofence, and manual override have been
validated on the water.

## Run

```bash
docker compose --env-file systems/boat/config/blueos.env \
  -f compositions/boat_hardware.yaml up
```

System-specific parameters live in `systems/boat/config`.

## Simulation

The closed-loop simulation validates the same bridge and command path without
BlueOS hardware:

```bash
docker compose -f compositions/boat_sim.yaml up
```

The scenario is a harbor survey in `systems/boat/config/harbor_survey.yaml`.
`blueboat_sim.py` publishes MAVROS-like GPS, odometry, IMU, and battery topics,
while `harbor_waypoint_nav.py` consumes the bridged boat odometry and publishes
`/boat0/cmd_vel`.
