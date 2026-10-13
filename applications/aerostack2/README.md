
# Aerostack2

This image provides the ROS 2 Jazzy Aerostack2 autonomy layer used by
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

The BlueOS stacks' MAVROS (`BLUEOS_FCU_URL`, default `tcp://127.0.0.1:5777`)
connects to the TCP server of BlueOS's MAVLink router (its protected
"Internal Link" endpoint), so no BlueOS setting changes. It joins as MAVLink
component 1:191 (onboard controller), which no BlueOS service uses
(mavlink2rest is 1:194, the camera manager 1:100-105), and it opens no serial
port: ArduPilot keeps the Navigator's UARTs. BlueOS restarts its router with
the autopilot (a firmware install, say); MAVROS retries the lost link every
30 s and reconnects. The previous default, `udp://0.0.0.0:14550@`,
received nothing on a stock BlueOS (1.4.2 checked on the Pi 5): its GCS Client
Link goes to the topside computer (192.168.2.1:14550) and its GCS Server Link,
a UDP server on 0.0.0.0:14550, is disabled. To use UDP anyway, add a BlueOS
MAVLink endpoint (UDP Client to 127.0.0.1:14550) and set `BLUEOS_FCU_URL`;
do not enable the GCS Server Link on the same Pi then, as it binds
0.0.0.0:14550 too and whichever of it and MAVROS starts second cannot open
the port (MAVROS logs `udp:bind: Address already in use` and retries every
30 s).

The stacks' rosbridge listens on 9092, not on its default 9090, which is
BlueOS's network manager (Cable Guy); the map page connects to 9092.

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

`compositions/blueos_aerostack.yaml` runs the complete Jazzy stack. Open
`http://<blueos-host>:8080`, click a map location, choose an altitude, and send
the goal. The page converts the click to the DemNav local ENU frame and
publishes it to `/aerostack/map_goal`. An Aerostack2 behavior tree consumes the
goal and invokes the `GoTo` behavior.

The page loads Leaflet 1.9.4 and roslib 1.4.1 from jsDelivr, pinned with
Subresource Integrity (a changed file is refused), and OpenStreetMap tiles, so
the browser needs network access.

## Object tracking UI

The same page (`src/as2_platform_blueos/web/`: `index.html`, `app.js`, and
`tracking_logic.js`, the logic without DOM or ROS) also follows a ground
object with the aerodrone, through the topics of
`systems/aerodrone/tracking.md`. It works on a phone or tablet (one column,
Stop pinned to the bottom of the screen) and on a desktop (camera and map
beside the controls).

- **Camera.** `/tracking/image/compressed` is drawn on a canvas with a box
  around every detection of `/tracking/detections`, labelled with its track ID,
  class and score. Boxes are in the pixels of `/tracking/camera_info`'s image
  and scaled to the displayed size; a click or tap is mapped back to those
  pixels and selects the smallest box under it, or the nearest within 14 screen
  pixels (small objects under a finger). Each frame is drawn with the
  detections of its own stamp, else the nearest within 0.5 s, fainter the
  further away they are; after 2 s without a new frame the view dims and taps
  are refused.
- **Follow.** Tapping a box publishes a `track` command on `/tracking/command`
  in the selected mode. *Centered* takes a height above ground of 15-120 m, or
  nothing to keep the current height (the field is then left out). *Slant
  range* takes the slant range and the depression angle (60-90 deg) and accepts
  only ranges whose height above the target, R sin(theta), is 15-120 m; it
  shows the resulting stand-off and height. Invalid values publish nothing.
  *Apply* sends the mode's current values to the current target, and *Stop
  (hover)* publishes `{"action": "stop"}`.
- **Status.** `/tracking/status`: state, mode, target (and whether it is in
  view), slant range and depression angle, target position and speed, message
  and age. The target's box is drawn thick amber; a tapped target the tracker
  has not yet reported is dashed amber.
- **Launcher.** With the foam dart launcher fitted (the status's `launcher`
  is not `"none"`; `systems/aerodrone/tracking.md`, "Foam dart launcher"):
  its state, *Arm launcher* and *Disarm*, the dart count and *Reload* (0-6,
  while disarmed), the fire checks, the last shot, and *Fire*, enabled only
  while the tracker's fresh status says every gate holds for the target it
  tracks (`fire_block` empty), with the reason when it does not. One press
  publishes one `{"action": "fire", "target_id": ...}` naming that target;
  the button then waits for the next status.
- **Map.** The click-to-goal above is unchanged. The map also shows the drone
  and the target, whose ENU position is converted the way the goal is, about the
  current `/navigation/fix` and `/navigation/odometry`.
- **Navigation badge.** The GPS, from MAVROS' raw fix
  (`/mavros/global_position/raw/fix`, status -1 without a fix), and the vision
  corrections the navigation fusion applies (`/navigation/correction_source`):
  "GPS", "GPS + demnav", "demnav (no GPS)", "dead reckoning (no GPS)", or "GPS
  unknown" after 5 s without a fix message. The fusion's source alone says
  `dead_reckoning` whenever no DemNav or WildNav correction is active, GPS or
  not, and the badge showed it while the GPS was healthy (in the Gazebo test
  flights the raw fix had a fix throughout while the source said
  `dead_reckoning` 88-100 % of the time).

Commands are published only while rosbridge is connected: roslib queues what
is published while disconnected and would send a stale command on
reconnection. The page reconnects every 2 s. `?prefix=/name` selects another
`TRACKING_TOPIC_PREFIX`.

`tools/tracking_ui_test.sh` runs the page in headless Chromium (Playwright)
against rosbridge and `scripts/mock_tracking_publisher.py`, a stand-in for the
detector and the tracker that publishes made-up detections, frames and status
and logs the commands it receives; it clicks and taps boxes in both modes,
checks the published JSON, Stop, the map goal and the phone layout, and saves
screenshots; a second mock with `--launcher` checks the launcher's card (Fire
enabled only while every gate holds, one command per press, disabled with the
reason by a person in view, a person as the target and disarming). `tests/test_aerodrone_tracking_ui.py` checks the page's wiring
and pinned libraries, the mock, and (with node) `tests/js/tracking_logic.test.js`.

## Object tracker

`scripts/aerodrone_tracker.py` (the `tracker` service of
`compositions/aerodrone_tracking.yaml`, `--profile tracking`) follows the object
picked in the page: it reads `/tracking/detections` and `/tracking/camera_info`
and publishes `/tracking/status` and `/tracking/target`
(`systems/aerodrone/tracking.md`). Its logic is plain Python in
`scripts/tracking_core.py`; it runs in the DemNav image, whose DEM loader,
geoid and camera geometry it uses.

- **Geolocation.** The selected track's box centre is a ray through the camera
  in its pose at the image stamp: position and attitude interpolated from
  `/navigation/odometry` (simulator-time stamps mapped through `/clock`, as in
  DemNav), the camera mount (`camera_mount_yaw_deg`, roll/pitch-stabilised
  cameras) and the intrinsics. The ray is marched to the ground of the DemNav
  DEM (`/data/demnav_cache`, the same cache DemNav fills; SRTM heights put on
  the ellipsoid with the geoid and expressed about `/navigation/origin`). The
  DEM knows the ground under the target, which a 60-degree slant view of a
  hillside needs (6 m of rise moves the fix 3.5 m); the aircraft's Benewake
  rangefinders reach 8-12 m and tracking flies 15-120 m above ground, so a
  fresh rangefinder reading only removes the DEM's offset below the aircraft.
  Without a DEM the ground is flat at the rangefinder's height, else at the
  home's (the status says so).
- **Filter.** A constant-velocity Kalman filter of the target's position, with
  Mahalanobis gating (five rejections in a row restart it where the target now
  is). A target unseen for `lost_timeout_s` (3 s) is lost.
- **Time.** The tracker runs on its node clock: the simulators' env files set
  `TRACKING_USE_SIM_TIME` (`use_sim_time`), so its timeouts, filter, lead and
  speed and yaw-rate limits are seconds of simulation, as ArduPilot's, in a
  simulator slower than real time (Isaac runs at about 0.2x); the aircraft
  keeps wall time. A detection's time is its image's stamp on that
  clock.
- **Track handover.** The detector's track IDs change when its image-space
  tracker loses an object, which the body-fixed camera of a manoeuvring
  aircraft makes common. When the selected ID is missing from a frame within
  `handover_window_s` (2.5 s) of the target's last placement, the tracker
  follows a new ID of the same kind (vehicle, two-wheeler, else the same
  class) and box size (within `handover_size_ratio`, 2) placed within
  `handover_max_m` (4 m) plus half the distance the target covered meanwhile
  of its position predicted along its arc, inside the filter's gate, if it is
  the only such one there; the status then carries the new ID ("handed over
  from ..."). A target occluded for longer is selected again by the
  operator. The selected ID counts only while its class is of the kind first
  seen (a car's ID moved onto a person is treated as gone).
- **Control.** The aircraft flies through Aerostack2's `FollowReference`
  behaviour (`as2_behaviors_motion`, started by `blueos_autonomy.launch.py`):
  the behaviour sends its position reference in `earth` (the navigation
  frame), the motion controller passes it through to the platform's
  POSITION/LOCAL_ENU mode, ArduPilot GUIDED position targets. The tracker moves
  the reference through the behaviour's `_behavior/modify` service at 10 Hz and
  cancels the goal to hover (the behaviour then sets HOVER). If the tracker dies
  the aircraft holds the last reference; it never keeps flying a velocity. The
  centered set-point is above the target at `altitude_agl_m` above its ground,
  yawed so a target moving at 2 m/s or more (`centered_yaw_min_speed_mps`)
  travels along the image's long axis, where the image reaches farthest ahead
  of and behind it (a walking or standing person keeps the yaw);
  the slant set-point is `R cos(theta)` short of the target on its bearing from
  the aircraft and `R sin(theta)` above it, yawed so the target lies on the
  image's long axis. ArduPilot trails a moving GUIDED position target (by
  about 3 s at 4 m/s in Gazebo: its position shaping brakes toward every
  new target), so both aim at the target predicted ahead, along its arc when
  it turns (a coordinated turn at the filtered heading's rate): `lead_s`
  (2.5 s) plus a lead learnt from how far the aircraft trails a target
  driving straight, followed centered (`lead_adapt_rate`, within
  `max_lead_extra_s` either way; the slant set-point lies on the bearing from
  the aircraft itself, so the aircraft's offset from it is no lag); a filter
  just started, whose velocity is still noise, leads less, growing to the full
  lead over `lead_ramp_s` (2 s), and a still target is not led (its filtered
  speed under `still_speed_mps`, 0.5 m/s, for `still_s`, 2 s, until it
  exceeds twice that: a parked car's filtered velocity is noise, and led in
  full it rocked the aircraft 10-16 deg either way above it in Gazebo;
  without the lead, under 2 deg). The reference
  moves at most `max_speed_mps` (ArduPilot flies position targets at
  `WPNAV_SPEED`, 5 m/s, at most anyway) and stays within
  `max_reference_lead_m` of the aircraft; heights stay 15-120 m above the
  ground under the set-point, and the depression is raised until the target
  stays `fov_margin_deg` inside the image's long axis (60 deg with the OAK-1,
  64.5 with Gazebo's 60-degree camera).
- **Pre-emption.** A track command stops a running `GoToBehavior` or
  `FollowPathBehavior` (their `_behavior/stop` service): the drone's map-goal
  tree then ends and `restart: always` starts it again for the next goal.
  Tracking ends (state `idle`, "pre-empted by ...") on a map goal
  (`/aerostack/map_goal`) or when any of those behaviours, `TakeoffBehavior` or
  `LandBehavior` starts. A track command is refused during take-off or landing
  and while the platform is not flying. A refused command leaves an active
  track as it is; while idle it shows as state `error` with the reason.
- **Foam dart launcher.** With `launcher_fitted` (`TRACKING_LAUNCHER_FITTED`;
  true in the simulators, false on the aircraft unless set) the tracker is the
  launcher's fire control (`scripts/launcher_core.py`; contract and gates in
  `systems/aerodrone/tracking.md`, "Foam dart launcher"): objects only, never
  people, one operator `fire` command per dart, every gate held from the
  command to the sear's release, the trigger at 0 and the rack stopped on any
  abort. On the aircraft the `launcher` profile adds the servo bridge
  (`turret_arduino_bridge.py`, `sts`, launcher only); the simulators run
  `scripts/launcher_standin.py`. The target is geolocated from the camera's
  position on the body (`camera_offset_m`), not the hub.

The parameters and their deployed values are in
`systems/aerodrone/config/tracking.env`; `aerodrone_tracking.yaml` repeats them
as the aircraft's fallbacks. The simulators include it with
`config/tracking_<simulator>.env` (DEM cache volume, camera mount) and add
`compositions/aerodrone_tracking_sim.yaml`: rosbridge and the page (the
aircraft runs its own) and `scripts/tracking_truth_detector.py`
(`--profile tracking_truth`, instead of the detector), which projects
`/simulation/targets` through the camera in the navigation pose into
detections, so the control can be tested apart from detection.
`tools/tracking_sim_test.sh <gazebo|isaac>` (with `TRACKING_TEST_SCENARIO=fire`:
the launcher's fire control, a shot at the halted car and two refusals, at a
person and with a person in view) flies and scores the
tracking end to end (`tools/tracking_sim_test.py`, layered on the composition
by `tools/tracking_sim_test.yaml` with `--profile tracking`): the YOLO detector
on the simulated camera, the tracker, and the scenario's car and people. Like
the operator in the page, the test picks a target by the ID of its box: it
pairs each detection frame with `/simulation/targets` projected through the
camera (the ground-truth detector, run beside the detector on
`/tracking/truth/detections`) and takes the nearest box of the target's kind
and size. A track command cancels a running GoTo, sent once the aircraft
holds above a point of the car's route; the car is followed centered 50 m
above it and slant (60 m at 70 deg) for 150 s each (`TRACKING_TEST_CENTERED_S`,
`TRACKING_TEST_SLANT_S`), the test clicking the car's new box whenever the
tracker has lost it for 1 s of simulation time; a person, tracked from 40 m, is hidden
under the terrain for 8 s (`/simulation/targets/hide`): lost and hovering,
then selected again; `stop` leaves the aircraft idle and hovering; a map goal
and a GoTo end tracking. `TRACKING_TEST_DETECTOR=truth` runs the
`tracking_truth` profile instead, the ground-truth detector's boxes named
after the targets (`TRACKING_TRUTH_SYNTHETIC=true`: its own targets).
`score.json` gives, per step and against simulator truth: the fraction of the
time tracking, the distance from the car (centered), the slant range and
depression errors, the target estimate's error, the detector's recall of the
target, its IDs and ID switches, the share of frames whose ID the tracker
followed, the operator's re-selections and the times to tracking again, the
lost, stop and pre-emption checks, the detection rate, and the host's and
containers' CPU (sampled every 15 s). `THRESHOLDS` in the script sets the pass
marks: the target well inside the image (centered within 0.25 / 0.35 of the
height, median / p95; slant range within 5 / 12 m, depression within 8 deg
and at most 8 deg shallower, p95), tracking 90 % of the time, recall 0.8, at
most 3 re-selections in 150 s.

Measured 2026-10-11 and 12 with the YOLO detector (`ROS_DOMAIN_ID=92`;
median / p95 against truth; the limit in brackets). Gazebo ran with
`GAZEBO_TWIST_IN_BODY_FRAME=true`, now the backend's default. Isaac runs at 0.2x real time, so its
follow steps were 120 s (`TRACKING_TEST_CENTERED_S=TRACKING_TEST_SLANT_S=120`)
to keep a run within 40 minutes of the shared machine (38 min; 150 s steps
take about 50). Every check passed in each run:

| | Gazebo, run 1 | Gazebo, run 2 | Isaac |
|---|---|---|---|
| centered: distance from the car, m [12.5 / 17.5] | 1.0 / 3.7 | 1.0 / 3.6 | 2.1 / 5.7 |
| centered: tracking [90 %], re-selections [3] | 100 %, 0 | 100 %, 0 | 100 %, 0 |
| slant: range error, m [5 / 12] | 1.0 / 2.1 | 1.0 / 3.1 | 1.5 / 3.9 |
| slant: depression error, deg [8 median]; shallower, p95 [8] | 1.5 / 3.3; 0.0 | 1.9 / 3.2; 1.8 | 4.1 / 5.6; 0.5 |
| slant: tracking, re-selections | 100 %, 0 | 99 %, 1 | 100 %, 0 |
| target estimate error (centered, slant), m | 0.5, 0.4 | 0.5, 0.5 | 0.4, 0.5 |
| car recall [0.8] (centered, slant); its IDs; followed [0.8] | 0.98, 0.97; 5, 4; 0.99, 1.00 | 0.98, 0.99; 4, 6; 0.99, 0.99 | 1.00, 1.00; 1, 8; 1.00, 1.00 |
| lost (hidden 8 s): lost after [4 s], drift [1 m], tracking again after shown [8 s] | 1.6 s, 0.05 m, 1.2 s | 2.9 s, 0.13 m, 1.2 s | 3.1 s, 0.04 m, 0.8 s |
| stop, map goal, GoTo | pass | pass | pass |
| detections (simulation / wall time) [4.5 Hz], per frame | 5.0 / 5.0 Hz, 73 ms | 5.0 / 5.0 Hz, 71 ms | 10 / 2.2 Hz, 73 ms |
| host CPU busy (20 cores) | 55 % | 55 % | 39 % |

The first results (2026-10-11, the first scenario: every stack failing on
re-selections, Isaac's slant tracking 72 % of the time with 49 of them) had
three causes, each fixed:

- **A scenario that could not be followed.** The first car lapped a
  26 m x 16 m circuit at the aircraft's own 5 m/s limit: the aircraft trailed
  it by 10-11 m (median; 16-17 m p95) and the car left the image. The car now
  drives at 3.5 m/s on a 116 m x 26 m oval (above): the aircraft is 1-2 m
  from it centered.
- **Wall-clock timeouts and limits.** `lost_timeout_s` and the handover window
  were wall time, 0.6 s of Isaac's simulation, and so were
  the filter's time steps, the lead and the reference's speed and yaw rate: in
  Isaac the lead was a fifth of what ArduPilot needs and the reference ran at
  five times the speed limit, which is why its slant lost the car at the image
  edge, the aircraft 5-15 deg shallower than commanded (p95 10 deg). The
  tracker now runs on the simulation clock (`TRACKING_USE_SIM_TIME`).
- **The camera's view of the car.** With a held yaw the car drove along the
  image's short axis half the time, where the image reaches least far and
  where the detector found it less often (in a Gazebo slant step, in 60 % of
  the frames against 92 % lying along the long axis). The centered mode now
  yaws the car's travel along the long axis.

Two of the Gazebo runs in between failed the slant step, and showed two
more: a lead learnt in slant (where the set-point lies on the bearing from
the aircraft itself) swung between -1.6 and +2.5 s, and each new filter after a
re-selection threw the reference about with its untrained velocity; either
rolled the aircraft 10-20 deg every few seconds and its body-fixed camera
lost the car again. The lead is now learnt only centered, and a new filter's
lead grows over `lead_ramp_s`. Raising the slant depression by the aircraft's
tilt toward the target, tried for the same losses, fed the oscillation (the
set-point jumps toward the target, the aircraft tilts further) and is not
used.

The scorer's thresholds are unchanged. The centered ones are fractions of the
height that keep the car well inside the narrowest image (0.25 and 0.35 of the
height against the 0.43 the 60-degree camera sees to either side); the
followable scenario now passes them by a wide margin (p95 3.6-5.7 m against
17.5 m), so they could be tightened to grade the control.

Gazebo's ground-truth twist is the body-frame velocity.
`gazebo_ardupilot_backend.py` now rotates it to ENU by default
(`GAZEBO_TWIST_IN_BODY_FRAME=true`), including outside tracking tests.
Without this conversion ArduPilot's GPS velocity turns with the yaw, and the
slant mode's yaw ended in a GPS glitch, EKF failsafe and emergency yaw reset.
The [GPS-denied baselines](../../tools/gps_denied_sim_test.md) also exercise
this default.


## Aerodrone simulation

`systems/aerodrone/` is the single physical quadrotor definition used by hardware and every simulation backend. Simulator assets and adapters are owned by `applications/gazebo/` and `applications/isaac/`; the Mount Tamalpais mission and validation contracts are owned by this application.

Run a backend with one of:

```bash
docker compose -f compositions/aerostack2_sim.yaml up     # Gazebo
docker compose -f compositions/aerostack2_isaac.yaml up   # Isaac Sim
```

Genesis support (`applications/genesis/`, `compositions/aerostack2_genesis.yaml`, the SO-101 adapter) was removed: Gazebo and Isaac are the simulators, and Genesis appears only in older commits.

Both use Navigator/ArduCopter through the same MAVROS and `as2_platform_blueos` boundary, build the same georeferenced terrain, and execute the generated behavior-tree mission. Topic names are defined in `systems/aerodrone/config/topics.env`; deployment values are in `applications/aerostack2/config/mount_tamalpais.env`.

The two simulated airframes now share the printable assembly's geometry and
loaded-launcher mass budget (3.1647 kg), centre of mass, inertia and square-X
rotor positions. See [simulator geometry](../../hardware/aerodrone.md#simulator-geometry)
for the generator, assumptions and calibration limits.

Hardware startup waits for MAVROS and behavior servers inside the
`behavior_tree` container, then replaces the waiter with the mission runner.
`docker compose up -d` therefore returns while an offline autopilot is awaited.
The separate `flight_ready` completion dependency below applies to simulations.

### ROS middleware

ROS 2 Jazzy uses native `rmw_zenoh_cpp` by default. The image builds Aerostack2
from upstream commit `97ade9f1d43ac760da8f69312e8ef9d2b4bf2328` into
`/opt/aerostack2`, because Jazzy binaries are not published. This revision
includes Jazzy and BehaviorTree.CPP 4 support. Rebuild this image and its
DemNav, WildNav, OAK-1, boat and submarine descendants together.

Every ROS service loads `systems/ros2/middleware.env`. Local/LAN peer discovery
works without a separate router; see the [middleware guide](../../systems/ros2/README.md)
for routed networks, migration and the native MCU firmware path. ROS domain IDs
still isolate vehicles. MAVLink between MAVROS and the autopilot is unchanged.

The existing Fast DDS profiles remain for an explicit DDS fallback. Their
8 MiB shared-memory segments and UDP-only Isaac profile address large-image
loss and different container UIDs. `./install.sh` still persists at least
4 MiB for both host socket-buffer limits; `start.sh` checks these only when
DDS is selected. Native Zenoh's default TCP transport does not depend on the
DDS UDP tuning. Do not mix DDS and native Zenoh nodes in one graph.

### Mission start

`flight_ready` (`scripts/wait_for_mavros_navigation.py`) completes once MAVROS has a GPS fix and local odometry and every server the mission tree calls can be discovered: it reads the mission tree (`wilderness_mission.xml`, or `tracking_mission.xml` with `AERODRONE_MISSION=tracking`, "Tracking targets and scenario") and waits for the action servers of its TakeOff, GoTo and Land nodes (`/aerodrone/TakeoffBehavior`, `GoToBehavior`, `LandBehavior`) and the services of its Arm, Offboard and Disarm nodes, with the availability checks the tree itself makes. `wilderness_mission` starts after it, through `scripts/run_behavior_tree.py`.

as2_behavior_tree (the pinned Jazzy source retains this 1.1.3 behavior) creates every node of the tree before the first tick, and each action node then gives its action server a hard-coded 1 s (`BtActionNode::createActionClient`; `server_timeout`, in milliseconds, only bounds the goal acknowledgement and the cancel). The exception it throws when that second runs out is built as `std::string("Action server %s not available", name)`, which C++ reads as a range between two unrelated pointers, so the process dies of `std::length_error` (`basic_string::_M_create`) instead, and `ros2 launch` still exits 0. In one Isaac run the tree's new process had not discovered `TakeoffBehavior`, up for 3.5 minutes by then, within that second. Nothing is commanded before the first tick, so `run_behavior_tree.py` starts the tree again (5 attempts, 5 s apart) when it aborted while being built and no node had left IDLE; a tree that ticked is never started again. The service exits non-zero when the tree process died or the tree ended in FAILURE (as2_behavior_tree itself exits 0 then; the wrapper reads the root's last status transition from the tree's log), so scripts and CI can tell a failed mission from a finished one.

The BlueOS drone (`compositions/blueos_aerostack.yaml`) gates its map-goal tree (`map_goal.xml`, a GoTo inside a WaitForEvent) the same way, with `--no-require-gps` (the drone may start GPS-denied; local odometry is still required) and `--timeout 0` (wait as long as it takes rather than leave the drone without a tree after a slow start). Its tree also runs through `run_behavior_tree.py`, and `restart: always` starts it again after each map goal.

Stopping a stack mid-mission (`docker compose down` once the validator has passed, say) logs `Failed to cancel action server for GoToBehavior` (or whichever behaviour was running). The tree halts the running node on SIGINT and sends the cancel, and the behaviour cancels the goal (`Request to cancel goal received`, `Goal canceled`), but rclcpp has already shut down, so the tree's wait for the cancel response returns at once and logs the error; the behaviour warns that the client will not receive its response. A mission that runs to its end halts no running node and does not log it.

A GoTo succeeds once the vehicle is within `go_to_threshold` (3D) of its target in the navigation frame, ArduPilot's own (`/navigation/odometry`). `blueos_autonomy.launch.py` takes it as a launch argument, Aerostack2's 0.2 m by default, which the drone's map goals keep; both simulations pass 3.0 m. While GPS 1 was denied ArduPilot held its targets up to 2.1 m off in that frame (0.3 m median), and at 0.2 m the wilderness mission hovered at one waypoint for most of the denial; its waypoints are 42 m or more apart and accepted within 4-8 m (`tools/gps_denied_sim_test.md`, "Mission progress and the GoTo threshold").

The generated tree (`applications/demnav/scripts/build_georeferenced_terrain.py`) sends each waypoint's GoTo twice, 2 s apart. as2 1.1.3's GoTo first checks its distance to the target 100 ms after accepting a goal (its 10 Hz run timer starts then), but computes that distance only when the next state arrives, against the previous goal until then. Where states come less often than that (ArduPilot's local position is 20 Hz of simulation time: every 260 ms of wall time in Isaac at 0.19x), a goal succeeded at once whenever the vehicle was within `go_to_threshold` of the previous waypoint, and every other waypoint was skipped. The second GoTo's previous goal is the same waypoint, so it flies there unless the vehicle already is. Each GoTo is followed by a delay because the behaviour rejects a goal (`there is no localization`, failing the mission) until a state has arrived since the last goal ended.

### Navigation health check

`scripts/validate_aerodrone_navsim.py` checks that a running Aerodrone navigation simulation is healthy, and is the `navsim_validator` service (`validation` profile) of both compositions:

```bash
docker compose -f compositions/aerostack2_sim.yaml --profile validation up -d
docker compose -f compositions/aerostack2_sim.yaml logs -f navsim_validator
```

It starts once `flight_ready` completes and passes (exit 0) when, at the same moment and over at least 10 s of observation:

- MAVROS GPS, odometry and IMU (`GPS_TOPIC`, `RAW_ODOM_TOPIC`, `IMU_TOPIC`) are fresh and finite;
- the downward image and camera_info WildNav uses (`WILDNAV_IMAGE_TOPIC`, `WILDNAV_CAMERA_INFO_TOPIC`) and the depth image and camera_info DemNav uses (`DEPTH_TOPIC`, `CAMERA_INFO_TOPIC`) are fresh and well formed, agree on frame and size (aspect ratio for the depth pair, which DemNav rescales), carry the stack's camera frame (Isaac `downward_camera_optical_frame`), and at least 25 % of the depth pixels are finite (DemNav's own threshold); the rangefinder too when the stack has one (neither simulation has one, so both pass `--rangefinder-topic ""`);
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

`tools/gps_denied_sim_test.sh <gazebo|isaac>` runs a composition with the `validation` profile and its `denial` profile, and scores the flight (`tools/gps_denied_sim_test.md`). The profile adds two services, both from this application's scripts: `gps_denier` (`scripts/gps_denier.py`) disables GPS 1 (`SIM_GPS_DISABLE=1`) `GPS_DENIAL_START_S` (60 s) after the mission reaches cruise, for `GPS_DENIAL_DURATION_S` (150 s), and records `GPS_DENIAL_RECOVERY_S` (60 s) more after restoring it, all of it simulation time (`/clock`; the defaults are in `config/mount_tamalpais.env`); `gps_denial_recorder` (`scripts/gps_denial_recorder.py`) records ground truth, ArduPilot's position, the fused fix and its covariance, the corrections and the DemNav/WildNav fixes. `tools/gps_denied_sim_scorer.py` reports fused and autopilot error, per-source fix errors, covariance consistency and route-keeping before, during and after the denial, and checks them against per-stack thresholds. The validator must pass as well (exit 3 otherwise), as in the lite test.

### Tracking targets and scenario

The two simulations, Gazebo and Isaac, render moving ground targets for the object tracker (`systems/aerodrone/tracking.md`), all from one definition, `missions/tracking_targets.json`: in the open clearing 60 m north-west of home, a car (`car_1`, Gazebo Fuel's "Hatchback blue") laps a 116 m x 26 m oval along the clearing, two 90 m straights joined by 13 m-radius turns, at 3.5 m/s (74.8 s a lap), a person (`person_1`, "Walking person") walks a 12 m x 6 m loop in its middle at 1.5 m/s (21.7 s) and another (`person_2`, "Casual female") stands between them, 12.6 m from the walker's loop. Each starts its loop at simulation time 0 and goes round at constant speed. The car is a fair test of following: it drives at 70 % of the aircraft's 5 m/s speed limit (`TRACKING_MAX_SPEED_MPS`, ArduPilot's `WPNAV_SPEED`), so the aircraft can catch up after a lag; its turns ask the aircraft above it for 0.9 m/s², under half its `WPNAV_ACCEL`; and each straight is four slant stand-offs long (60 m at 70 deg: 20.5 m), so a slant follower drives behind it. The first scenario's car lapped a 26 m x 16 m circuit with 6.5 m corners at the speed limit itself: the aircraft trailed it by 11 m (median) even with the ground-truth detector, and left it at the image edge; on a 60 m x 24 m oval the slant follower sat inside the loop with the car circling it, crossing the image sideways (`tests/test_tracking_targets.py` checks all three).

- **Models.** The `target_assets` service downloads each pinned Fuel model version into the stack's `tracking_targets_<simulator>_cache` volume (the first run needs the network, like the DEM and imagery seeds) and checks its SHA-256 and its licence on Fuel (all CC0, `THIRD_PARTY_NOTICES.md`). `terrain_assets` (`--targets`) bakes each into one textured mesh, metres, z up, front along +x, its lowest point at z = 0, with every material in one texture atlas (`applications/demnav/scripts/target_models.py`): `model.obj` and `<model>_atlas.png` for Gazebo, the same triangles in `model_mesh.npz` for Isaac, under `/data/navsim_assets/models/<model>`.
- **Routes on the ground.** The builder resolves each route on the terrain the cameras see, the render mesh's triangles (`terrain_mesh.npz`, `terrain.obj`), into a track sampled every 0.2 m (`tracking_targets.json` in the assets; `applications/demnav/scripts/tracking_targets.py`): position, ground height and heading, and for the car the pitch and roll of the ground under its wheels. Every simulator places each target at its track pose for its own simulation time: Isaac in its lock-step loop just before each camera frame (visual-only meshes, no physics); Gazebo through the `gazebo_targets` service (`applications/gazebo/scripts/gazebo_tracking_targets.py`), which sets the visual-only, gravity-free models' poses with the world's `set_pose` service every 0.02 s of `/clock` (the builder puts them in the world at their start).
- **Ground truth.** `/simulation/targets` follows the contract: Isaac publishes it with each camera frame (10 Hz), Gazebo's driver every 0.02 s; `stamp` is simulation seconds (`/clock`). Positions are ENU metres about the navigation frame's origin, ArduPilot's EKF origin, which is the vehicle's home (`manifest.json` `home_enu_m`, the reference the simulators give ArduPilot positions about), not INITIAL_LAT/INITIAL_LON: the field is about (-30, 46) there. `up` is the ground under the target and `yaw` its heading, counter-clockwise from east. Projected through the camera, the truth falls on the rendered target. From a fixed camera 40 m above the field, the projected model silhouette and the target's rendered pixels overlap with an IoU of 0.92-0.98 for the car, their centroids within 1 px (Isaac) and 1.7 px (Gazebo, where a frame can show a pose up to one 0.02 s update old). In the first tracking scenario, through the drone's ground-truth pose at the hold, the car's centroid was within 0.4 px (Isaac) and 2 px (Gazebo) of the projected truth.
- **Tracking scenario.** `AERODRONE_MISSION=tracking docker compose -f compositions/aerostack2_<...>.yaml up` flies `tracking_mission.xml` instead of the wilderness mission (the `wilderness_mission` service runs it; `flight_ready` reads the same tree): arm, take off to 35 m, fly to the middle of the car's oval, 50 m above the ground there and nose east (GoTo sent twice, like the wilderness waypoints, fixed yaw), then hover until stopped, commanding nothing more so the tracker can take over. From 50 m the downward camera covers 58 m x 43 m of ground (Gazebo, Isaac: 60 degrees across 640 x 480); whatever the yaw, the people stay inside its 43 m square with 2 m to spare, and the car crosses it on every lap (the middle of its straights). The car is about 44 x 24 px there and the people about 7 x 10 px.
