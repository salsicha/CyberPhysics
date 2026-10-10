
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
accepted DemNav and WildNav horizontal corrections (while ArduPilot absorbs
them as GPS 2, see below, it publishes ArduPilot's odometry unchanged).

`scripts/ardupilot_navigation_bridge.py` runs beside MAVROS in every
ArduPilot stack. It relays the ArduPilot EKF origin (MAVROS
`global_position/gp_origin`, WGS-84 ellipsoid altitude) to
`/navigation/origin` (reliable, transient-local), the single anchor of every
local navigation output: DemNav, WildNav and the fusion node express local
positions about it, and the fusion node converts DemNav/WildNav lat/lon fixes
into that frame with `synthetic_world`'s projection. In the ArduCopter
stacks it also feeds accepted corrections to ArduPilot as GPS 2 and switches
the EKF to its GPS-denied source set when the primary GPS loses its fix
(`GPS_INPUT_CORRECTIONS`, default true; false turns it off; see
`tools/gps_denied_sitl_test.md`). It does so only once the flight controller
reports `systems/aerodrone/config/arducopter_gps_input.params`, which SITL
loads automatically and the aircraft needs loaded by hand
(`systems/aerodrone/README.md`); the bridge never writes EKF source
parameters. The airplane stacks keep it off (not validated on ArduPlane).

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

### Navigation health check

`scripts/validate_aerodrone_navsim.py` checks that a running Aerodrone navigation simulation is healthy, and is the `navsim_validator` service (`validation` profile) of all three compositions:

```bash
docker compose -f compositions/aerostack2_genesis.yaml --profile validation up -d
docker compose -f compositions/aerostack2_genesis.yaml logs -f navsim_validator
```

It starts once `flight_ready` completes and passes (exit 0) when, at the same moment and over at least 10 s of observation:

- MAVROS GPS, odometry and IMU (`GPS_TOPIC`, `RAW_ODOM_TOPIC`, `IMU_TOPIC`) are fresh and finite;
- the downward image and camera_info WildNav uses (`WILDNAV_IMAGE_TOPIC`, `WILDNAV_CAMERA_INFO_TOPIC`) and the depth image and camera_info DemNav uses (`DEPTH_TOPIC`, `CAMERA_INFO_TOPIC`) are fresh and well formed, agree on frame and size (aspect ratio for the depth pair, which DemNav rescales), carry the stack's camera frame (Genesis/Isaac `downward_camera_optical_frame`), and at least 25 % of the depth pixels are finite (DemNav's own threshold); the rangefinder too when the stack has one (none of the three simulates one, so they pass `--rangefinder-topic ""`);
- `/navigation/origin` (latched) has arrived, finite and within 50 km of `INITIAL_LAT`/`INITIAL_LON`;
- `/navigation/odometry` and `/navigation/fix` are fresh and finite, with a known, positive, symmetric covariance whose horizontal sigma is at most 1 km, the fix status is not NO_FIX, and the fused odometry is in the `odom` frame;
- `/demnav/valid` and `/wildnav/valid` are alive (WildNav may be four `WILDNAV_RATE_HZ` periods old), `/wildnav/confidence` (and `/demnav/confidence` when DemNav matches are required) has arrived, and any confidence lies in [0, 1], any `/demnav/fix` or `/wildnav/fix` received is valid, and `/navigation/correction_source` and `correction_confidence` are fresh and valid;
- the TF links resolve and dynamic ones are under 10 s old: `earth -> aerodrone/map -> aerodrone/odom -> aerodrone/base_link` (the Aerostack2 state estimator) and MAVROS's `map -> map_ned` and `base_link -> base_link_frd`.

DemNav only publishes a confidence for an accepted terrain match. At the wilderness mission's 35-75 m AGL its depth footprint (about 50-100 m) spans too few posts of the 10 m SRTM DEM to be unique within its 900 m search radius, so DemNav does not match in these stacks and its confidence is only required with `NAVSIM_REQUIRE_VALID=demnav`; the check waits up to `NAVSIM_VALIDATION_TIMEOUT_S` (900 s). The fused fix turns NO_FIX after 300 s without an accepted correction, which the check also reports. Otherwise it exits 1 and lists each missing (with whether anything publishes the topic), stale or invalid item, for example:

```text
Aerodrone navsim validation FAILED after 45.0 s; 4 problem(s):
  - correction source [/navigation/correction_source]: invalid: dead_reckoning: no accepted DemNav/WildNav correction is active
  - DemNav valid [/demnav/valid]: missing: no message received (no publisher on this topic)
  - DemNav confidence [/demnav/confidence]: missing: no message received (no publisher on this topic)
  - DemNav valid [/demnav/valid]: never reported a valid match (data: true)
```

`NAVSIM_REQUIRE_VALID=demnav` (or `"demnav wildnav"`) additionally requires a valid match from those sources, and `NAVSIM_REQUIRE_CORRECTION=true` requires the fusion to be applying corrections (`correction_source` not `dead_reckoning`). Run `python3 scripts/validate_aerodrone_navsim.py --help` for every option; topic options default to the `topics.env` variables. The lite SITL stack runs the same script with its own topics once the vehicle is airborne (`tools/gps_denied_sitl_test.md`).
