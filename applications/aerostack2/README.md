
# Aerostack2

This image provides the ROS 2 Humble Aerostack2 autonomy layer used by
CyberPhysics. It includes the runtime state estimator, motion controller,
motion behaviors, behavior-tree executor, and the `as2_platform_blueos` package
for controlling an ArduPilot vehicle through a BlueOS MAVLink endpoint and
MAVROS. Gazebo simulation packages are included for workstation simulation. The Pi/BlueOS
runtime should still run only the services it needs in compose.

See original project:
https://github.com/aerostack2/aerostack2/tree/main


./launch_as2.bash
./launch_ground_station.bash
./stop.bash

## BlueOS platform

BlueOS must expose the vehicle MAVLink stream to MAVROS. Start MAVROS, then:

```bash
ros2 launch as2_platform_blueos blueos_platform.launch.py \
  namespace:=drone0 mavros_namespace:=/mavros
```

The platform republishes MAVROS odometry, IMU, GPS, and battery data on the
standard Aerostack2 `sensor_measurements/*` topics. Aerostack2 commands are
translated to ArduPilot GUIDED-mode velocity or local-position setpoints.

The autonomy launch consumes `/navigation/odometry`. The WildNav navigation
fusion node publishes that topic at the MAVROS odometry rate while applying
accepted DemNav and WildNav horizontal corrections.

ArduPilot remains responsible for stabilization, motor mixing, EKF operation,
RC override, and vehicle failsafes.

## Map mission

`compositions/blueos_aerostack.yaml` runs the complete Humble stack. Open
`http://<blueos-host>:8080`, click a map location, choose an altitude, and send
the goal. The page converts the click to the DemNav local ENU frame and
publishes it to `/aerostack/map_goal`. An Aerostack2 behavior tree consumes the
goal and invokes the `GoTo` behavior.

The map page loads Leaflet, OpenStreetMap tiles, and roslib.js from their public
CDNs, so the browser needs network access.


## Gazebo Simulation

The hardware/simulation-specific Aerostack2 Gazebo files live in
`systems/aerostack2_gazebo/`. Start the base simulation with:

```bash
docker compose -f compositions/aerostack2_sim.yaml up
```

Start the same simulation with DemNav and WildNav consumers enabled:

```bash
docker compose --profile navsim -f compositions/aerostack2_sim.yaml up
```

The `navsim` profile uses simulated GPS, odometry, RGB, depth, and camera-info
topics. Override topic names with the variables documented in
`systems/aerostack2_gazebo/config/nav_topics.env` if the installed AS2 Gazebo
assets publish different names.
