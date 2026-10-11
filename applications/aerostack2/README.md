
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

### DDS transport

Every ROS container of the three compositions and of the lite SITL stack (`tools/gps_denied_sitl.yaml`) loads `systems/aerodrone/config/fastdds.xml` through `FASTRTPS_DEFAULT_PROFILES_FILE`. The containers share the host network and `/dev/shm` and run as root, so FastDDS (Humble, Fast DDS 2.6) sends between them through shared memory. Its default 512 KiB segment holds less than one camera frame (640x480 RGB is 0.9 MB, 32FC1 depth 1.2 MB): the sender reused buffers before the subscribers had read them, and DemNav, WildNav and the validator received 1-3 % of the Genesis frames (0.03-0.07 of 2.3 Hz), all of them with the profile. The profile gives every sender an 8 MiB segment and keeps UDP, with 4 MiB socket buffers, for peers that cannot use the shared memory. Isaac Sim runs as user 1234; FastDDS creates its shared-memory files mode 0644, so shared memory between it and the root containers delivers nothing, and it loads `fastdds_udp.xml` (UDP only, same buffers) instead. A ROS container added to these stacks needs the same profile, mount, host `/dev/shm` (`ipc: host` or a `/dev/shm` volume) and user; `tests/test_aerial_dds_transport.py` checks them.

The kernel caps the socket buffers at `net.core.rmem_max` and `net.core.wmem_max`. Shared memory does not depend on them, but Isaac Sim's camera does: on a host where they are below 4 MiB (208 KiB was the long-standing default; check with `sysctl net.core.rmem_max`), its UDP frames can still be dropped. Raising them is optional and not done by the stacks: `sudo sysctl -w net.core.rmem_max=4194304 net.core.wmem_max=4194304`. A killed process leaves its segment in `/dev/shm`; `docker run --rm --ipc host salsicha/cyberphysics:aerostack2-latest fastdds shm clean` removes the stale ones.

### Mission start

`flight_ready` (`scripts/wait_for_mavros_navigation.py`) completes once MAVROS has a GPS fix and local odometry and every server the mission tree calls can be discovered: it reads `wilderness_mission.xml` and waits for the action servers of its TakeOff, GoTo and Land nodes (`/aerodrone/TakeoffBehavior`, `GoToBehavior`, `LandBehavior`) and the services of its Arm, Offboard and Disarm nodes, with the availability checks the tree itself makes. `wilderness_mission` starts after it, through `scripts/run_behavior_tree.py`.

as2_behavior_tree (1.1.3) creates every node of the tree before the first tick, and each action node then gives its action server a hard-coded 1 s (`BtActionNode::createActionClient`; `server_timeout`, in milliseconds, only bounds the goal acknowledgement and the cancel). The exception it throws when that second runs out is built as `std::string("Action server %s not available", name)`, which C++ reads as a range between two unrelated pointers, so the process dies of `std::length_error` (`basic_string::_M_create`) instead, and `ros2 launch` still exits 0. In one Isaac run the tree's new process had not discovered `TakeoffBehavior`, up for 3.5 minutes by then, within that second. Nothing is commanded before the first tick, so `run_behavior_tree.py` starts the tree again (5 attempts, 5 s apart) when it aborted while being built and no node had left IDLE; a tree that ticked is never started again, and a tree process that died makes the service exit non-zero.

The BlueOS drone (`compositions/blueos_aerostack.yaml`) gates its map-goal tree (`map_goal.xml`, a GoTo inside a WaitForEvent) the same way, with `--no-require-gps` (the drone may start GPS-denied; local odometry is still required) and `--timeout 0` (wait as long as it takes rather than leave the drone without a tree after a slow start). Its tree also runs through `run_behavior_tree.py`, and `restart: always` starts it again after each map goal.

Stopping a stack mid-mission (`docker compose down` once the validator has passed, say) logs `Failed to cancel action server for GoToBehavior` (or whichever behaviour was running). The tree halts the running node on SIGINT and sends the cancel, and the behaviour cancels the goal (`Request to cancel goal received`, `Goal canceled`), but rclcpp has already shut down, so the tree's wait for the cancel response returns at once and logs the error; the behaviour warns that the client will not receive its response. A mission that runs to its end halts no running node and does not log it.

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

DemNav only publishes a confidence for an accepted terrain match, so its confidence is only required with `NAVSIM_REQUIRE_VALID=demnav`. Its depth footprint needs height: the simulated wilderness mission therefore flies its en-route waypoints at least `CRUISE_AGL_M` (250 m, `config/mount_tamalpais.env`) above ground, and DemNav matches once the mission has climbed there. The simulators run slower than real time, so the check waits up to `NAVSIM_VALIDATION_TIMEOUT_S` (1800 s). The fused fix turns NO_FIX after 300 s of simulated time (the simulator's `/clock`) without an accepted correction, which the check also reports. Otherwise it exits 1 and lists each missing (with whether anything publishes the topic), stale or invalid item, for example:

```text
Aerodrone navsim validation FAILED after 45.0 s; 4 problem(s):
  - correction source [/navigation/correction_source]: invalid: dead_reckoning: no accepted DemNav/WildNav correction is active
  - DemNav valid [/demnav/valid]: missing: no message received (no publisher on this topic)
  - DemNav confidence [/demnav/confidence]: missing: no message received (no publisher on this topic)
  - DemNav valid [/demnav/valid]: never reported a valid match (data: true)
```

`NAVSIM_REQUIRE_VALID=demnav` (or `"demnav wildnav"`) additionally requires a valid match from those sources, and `NAVSIM_REQUIRE_CORRECTION=true` requires the fusion to be applying corrections (`correction_source` not `dead_reckoning`). Run `python3 scripts/validate_aerodrone_navsim.py --help` for every option; topic options default to the `topics.env` variables. The lite SITL stack runs the same script with its own topics once the vehicle is airborne (`tools/gps_denied_sitl_test.md`).

### GPS-denied test

`tools/gps_denied_sim_test.sh <gazebo|genesis|isaac>` runs a composition with the `validation` profile and its `denial` profile, and scores the flight (`tools/gps_denied_sim_test.md`). The profile adds two services, both from this application's scripts: `gps_denier` (`scripts/gps_denier.py`) disables GPS 1 (`SIM_GPS_DISABLE=1`) `GPS_DENIAL_START_S` (60 s) after the mission reaches cruise, for `GPS_DENIAL_DURATION_S` (150 s), and records `GPS_DENIAL_RECOVERY_S` (60 s) more after restoring it, all of it simulation time (`/clock`; the defaults are in `config/mount_tamalpais.env`); `gps_denial_recorder` (`scripts/gps_denial_recorder.py`) records ground truth, ArduPilot's position, the fused fix and its covariance, the corrections and the DemNav/WildNav fixes. `tools/gps_denied_sim_scorer.py` reports fused and autopilot error, per-source fix errors, covariance consistency and route-keeping before, during and after the denial, and checks them against per-stack thresholds. The validator must pass as well (exit 3 otherwise), as in the lite test.
