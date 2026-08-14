
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


## Aerodrone simulation

`systems/aerodrone/` is the single physical quadrotor definition used by hardware and every simulation backend. Simulator assets and adapters are owned by `applications/gazebo/`, `applications/genesis/`, and `applications/isaac/`; the Mount Tamalpais mission and validation contracts are owned by this application.

Run a backend with one of:

```bash
docker compose -f compositions/aerostack2_sim.yaml up
docker compose -f compositions/aerostack2_genesis.yaml up
docker compose -f compositions/aerostack2_isaac.yaml up
```

All three use Navigator/ArduCopter through the same MAVROS and `as2_platform_blueos` boundary, build the same georeferenced terrain, and execute the generated behavior-tree mission. Topic names are defined in `systems/aerodrone/config/topics.env`; deployment values are in `applications/aerostack2/config/mount_tamalpais.env`.
