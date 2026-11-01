# Aerodrone hardware-in-the-loop test (Gazebo + the BlueOS Raspberry Pi 5)

`tools/aerodrone_hil_test.sh <mission|gps_denied|tracking|fire>` runs the
Aerodrone's companion stack on the Raspberry Pi 5 that runs BlueOS, as it
runs on the aircraft (`compositions/aerodrone_hardware.yaml`, the arm64
images), while the vehicle it flies is simulated on the workstation: Gazebo,
ArduPilot SITL and the moving targets. It is companion-computer-in-the-loop:
the Pi's CPU, memory, temperature, its Docker, BlueOS and BlueOS's own
ArduPilot (running beside the stack, as on the aircraft) and the network
between the companion computer and the rest are real; the flight
controller, the camera and the rangefinder are not.

In short (2026-10-31 and 11-01; single runs, each against the same scenario
run all on the workstation the same day; details in "Results"):

- **Mission:** flies end to end from the Pi (run 2: tree SUCCESS, 40 of 40
  GoTos, every waypoint within 0.3 m horizontally in truth, landed and
  disarmed after 21 min 26 s; the control: 0.04 m, 19 min 9 s), though
  ArduPilot's height drifted about 1 m low during the cruise. A first run
  flew all 40 GoTos but climbed in Land after repeated ArduPilot EKF
  warnings on the workstation's SITL. Not the companion's late GPS_INPUT
  corrections (ruled out: "6. The missions' height"): a later HIL run that
  sent none did it again, and SITL's log showed the simulated vehicle's
  attitude loop oscillating from the take-off, which biases ArduPilot's
  vertical velocity by up to 2 m/s. Three re-runs with the hardened bridge
  (late, bursty corrections; GPS 2 only in source set 2) all oscillated:
  one landed normally, one tipped over at touchdown, one (without
  corrections) timed out; why HIL runs fall into it and all-local runs do
  not is not established.
- **GPS-denied: fails in HIL** (fused 11.7 / 91.9 / 135.8 m median / p95 /
  max while denied; the control 7.1 / 11.5 / 12.7 m, the documented
  baseline 6.6 / 10.9 / 12.7 m). The Pi throttles at 85 °C to 1.5-1.9 GHz,
  WildNav's matching takes 8 s instead of 1.2 s, the corrections thin out
  and the EKF drifts. With WildNav's lighter profile it **still fails**
  (11.8 / 94.1 / 147.9 m), though WildNav now turns a still into a fix in
  0.2-0.5 s on the Pi and the Pi runs at 45 % instead of 73 % of its cores:
  the drift is no longer WildNav's latency ("7. GPS-denied with the lighter
  WildNav profile"). The same profile all on the workstation passes
  (5.3 / 8.3 / 9.6 m).
- **Tracking:** centered following, losing and re-finding a person, stop and
  both pre-emptions pass; **slant fails** (43 % tracking, 8 re-selections;
  the control 99 %, 1). The page served by the Pi works in a browser on the
  workstation (4.5 frames/s, 0.26 s from the camera to the screen).
- **Launcher:** no dart at a person in any run; HIL refused the scenario's
  shot at the car (1.1 m from the bore against the 1 m gate); the
  person-target step fails all-local too.
- **Pi:** 62-73 % of the 4 cores (median), about 90 % at p95 and up to 95 %
  with WildNav at cruise, 2.6 GB of 8 used, throttled in 87-100 % of
  samples; BlueOS's ArduSub rises from 8 % to 18-22 % of a core. With
  WildNav's lighter profile the GPS-denied flight ran at 45 / 59 % (median /
  p95), throttled in 57 % of samples, at 2.3 GHz median.
- **Link:** one Zenoh session carries 1.0-1.4 MB/s to the Pi (11-16 % of the
  measured 9 MB/s), MAVLink 3.5 kB/s; MAVLink round trip 19-45 ms median,
  85-141 ms p95; camera to autopilot about 0.25-0.3 s median; nothing
  dropped; nothing on the wired LAN.

## Design

| | Workstation (`hil_sim`) | Pi 5 (`hil_pi`) |
|---|---|---|
| Composition | `compositions/aerostack2_sim.yaml` + `tools/aerodrone_hil_sim.yaml` (+ `tools/tracking_sim_test.yaml` + `tools/aerodrone_hil_test.yaml` for the tests) | `compositions/aerodrone_hardware.yaml` + `tools/aerodrone_hil_pi.yaml` |
| Runs | Gazebo, its bridges and targets, ArduPilot SITL (ArduCopter 4.6.3, JSON lock-step), the OAK-1 stand-in, the YOLO detector (the OAK-1's on-camera network), the launcher stand-in, the test harness (recorder, GPS denier, validator, tracking driver, truth boxes) | MAVROS, the navigation bridge, DemNav, WildNav and the fusion, Aerostack2, the rate keeper, flight_ready, the mission tree, the tracker with the launcher's fire control, rosbridge and the page |
| Not run | the companion stack (scale 0) | the OAK-1 and the rangefinder (scale 0), the launcher's servo bridge |
| Images | `*-latest` (amd64) | `*-arm64`, built on the Pi |
| Settings | `applications/aerostack2/config/aerodrone_hil.env` (both), after `systems/aerodrone/config/topics.env` and `applications/aerostack2/config/mount_tamalpais.env` (the simulated site) | the same |

BlueOS, its ArduPilot (the Navigator's ArduSub 4.7.1 on the bench), its
endpoints and settings are left as they are: nothing in the HIL stack talks
to them. The vehicle that flies is the workstation's SITL.

### Flight controller link (MAVLink)

ArduPilot SITL stays beside Gazebo: its JSON backend exchanges a packet every
physics step in lock-step, which cannot run over a Wi-Fi round trip. The Pi's
MAVROS connects to SITL's first serial port over TCP,
`BLUEOS_FCU_URL=tcp://<workstation>:5760` (the aircraft's own MAVROS launch
file and plugin list, `blueos_mavros.launch.xml`). TCP rather than UDP:
SITL's own server (no SITL output to configure toward the Pi), ordered and
reliable over a lossy radio, MAVROS reconnects by itself, and it is the
aircraft's transport (`tcp://127.0.0.1:5777`, BlueOS's router). Over Wi-Fi a
lost segment delays what follows it by a retransmission instead of dropping
it; the measured round trips are below.

### ROS link (native Zenoh over Wi-Fi only)

The stacks run native Zenoh (`systems/ros2/README.md`), not Fast DDS, so
the link is configured in Zenoh terms (`aerodrone_hil.env`,
`ROS_DOMAIN_ID=68`):

- one Zenoh router per host (`hil_router`, port 7468: 7447 is BlueOS's, and
  7448/7449 belong to another project's routers on the workstation's wired
  interface). The workstation's listens on loopback and its Wi-Fi address;
  the Pi's listens on loopback only and connects out to it. That router to
  router TCP session is the only ROS session across the network: each topic
  crosses once, however many nodes subscribe on the other side;
- every ROS node is a Zenoh peer listening on loopback only, connected to its
  host's router, and reaches the other nodes of its host directly through
  gossip (as on the aircraft, where nodes are peers without a router); the
  routers gossip to their own peers only, and peers connect only to peers
  they hear of, so no node opens a session to the other host;
- no multicast scouting (it would also find the other project's peers on the
  workstation, which use domains 42 and 43), no `0.0.0.0` listener, no
  address of the wired 10.0.0.0/24 LAN anywhere: the runner's socket audit
  checks that every HIL session (7468, 5760, 9092, 8080) is between the two
  Wi-Fi addresses.

Measured before the runs (one TCP session each way over the same Wi-Fi):
9 MB/s to the Pi, 11-15 MB/s from it, 8 ms round trip (ping, 0 % loss). The
wired path (the workstation's 100 Mb/s USB adapter at 10.0.0.18, the Pi's
eth0 at 10.0.0.2) is not used: it lost 7-25 % of packets with up to 1 s
spikes, its USB adapter is saturated by another project's traffic (about
12 MB/s, 94 Mb/s, measured during these runs), and the Pi's IPv4 default
route through it does not reach the internet at all (100 % loss to 8.8.8.8,
so DNS fails on the Pi; its IPv6 route over Wi-Fi works).

### Camera, rangefinder and time

The workstation plays the aircraft's OAK-1 and rangefinder
(`applications/aerostack2/scripts/hil_oak1_standin.py`, service
`hil_oak1_standin`), from Gazebo's downward cameras, so that the Pi's DemNav,
WildNav and tracker read the topics, sizes and rates of
`applications/oak1/README.md`:

| Topic (to the Pi) | Aircraft (OAK-1) | HIL |
|---|---|---|
| `/oak1/relative_depth`, `/oak1/camera_info` | MegaDepth relative depth 256x192, 4.6 Hz, 2.9 Hz while the detector runs | Gazebo's metric depth resized to 256x192 (nearest neighbour), 4.6 Hz of simulation time, 2.9 Hz while `/tracking/detections` has a subscriber; DemNav runs with `depth_is_metric` |
| `/oak1/image_highres`, `/camera_info` | 1280x720 stills, 0.5 Hz | Gazebo's 640x480 colour camera, 0.5 Hz |
| `/oak1/image_raw` | colour at the depth's size and rate | published on demand (nothing on the Pi takes it) |
| `/tracking/detections`, `/image/compressed`, `/camera_info` | YOLO and the ObjectTracker on the camera, 5 Hz, MJPEG 640x352 | `sim_tracking_detector.py` (the network's ONNX on the workstation's CPU), 5 Hz, JPEG 640x480 |
| `/aerodrone/rangefinder` | TFmini-S, about 100 Hz, 0.1-12 m | the depth image's centre, 5 Hz, 0.1-12 m, ±inf outside |

Every frame is stamped in wall time, as the OAK-1 stamps its frames with
its capture time on the host clock: the stand-in maps each frame's
simulation stamp to the wall time at which `/clock` reached it. The Pi
therefore runs entirely on wall time, like the aircraft (the tracker with
`use_sim_time` false, its aircraft default), and `/clock` (200 Hz, 2.4 kB/s)
never needs to cross; Gazebo runs at 0.9-1.0x real time, so wall-time
timeouts differ from simulation time by the real-time factor (reported per
run). The workstation's harness keeps simulation time, and the scorers map
between the two (`tools/gps_denied_sim_scorer.py` now scores a wall-stamped
camera fix against the truth at that wall time). The two hosts' clocks are
NTP-synchronised; the runner measures the Pi's offset at the start and the
end of each run (between -2 and +24 ms here, to ±2 ms) and the latency
figures below are corrected by it.

Simulated-sensor settings the Pi's DemNav and WildNav get as in the Gazebo
stack, otherwise their commands are the aircraft's
(`tests/test_aerodrone_hil.py` holds each to the aircraft's arguments): metric
depth on the 10 m DEM the world is built from (`depth_is_metric`,
`dem_resolution_m`), and WildNav's covariance fitted to imagery that is the
rendered ground itself (`attitude_sigma_deg` 0.5, `match_sigma_floor_m` 1).
Aerostack2 takes the simulations' `go_to_threshold` of 3 m and take-off
height. The DEM and tile caches the workstation seeds, and the mission tree it
generates, are copied to the Pi at each run.

### What the runner does

1. Checks that no `hil_` container is up on either host and that the images
   are there; syncs the files the Pi's stack mounts (compositions, systems,
   tools and `applications/aerostack2`'s config, scripts and page) to the
   Pi's copy (`HIL_SYNC=false` skips it).
2. Measures the Pi's clock offset; starts the Pi's resource sampler
   (`tools/aerodrone_hil_resources.py`, host-side, standard library: CPU per
   container and process, ArduPilot's own, memory, temperature,
   `vcgencmd get_throttled` and the firmware's ARM clock, interface and
   HIL-session bytes) and samples the Pi idle (BlueOS and its ArduPilot
   alone) for `HIL_IDLE_S` (60 s) before anything starts.
3. Brings the workstation side up, copies the mission tree and caches, brings
   the Pi side up over SSH and waits for the Pi's flight_ready.
4. Runs the scenario; at a fixed point probes the topics that cross on both
   hosts (`tools/aerodrone_hil_probe.py`: rate, bytes, gaps, age at arrival;
   on the Pi also MAVROS's TIMESYNC round trips) and lists the hosts' TCP
   sessions; in the tracking scenario loads the page from the Pi in a
   headless browser here (`tools/aerodrone_hil_page_check.py`).
5. Collects both sides' logs and tears both down, volumes included, also on
   failure or interruption, then scores and writes `hil_summary.json`.

```bash
# Images: the workstation's amd64 ones (applications/Makefile), and on the Pi
# the aircraft's arm64 aerostack2, demnav and wildnav (see "Images" below).
( flock 9; tools/aerodrone_hil_test.sh mission ) 9>/path/to/simulator.lock
tools/aerodrone_hil_test.sh gps_denied
tools/aerodrone_hil_test.sh tracking
tools/aerodrone_hil_test.sh fire
HIL_KEEP_VOLUMES=true tools/aerodrone_hil_test.sh mission   # keep the seeded caches
tools/aerodrone_hil_test.sh control gps_denied   # the same scenario all on the workstation
```

Variables: `HIL_PI` (ssh destination), `HIL_PI_REPO` (the Pi's copy,
`/home/pi/cyberphysics`), `HIL_SYNC`, `HIL_OUTPUT`
(`generated/aerodrone_hil/<scenario>`), `HIL_TIMEOUT_S` (2400),
`HIL_PROBE_S` (60), `HIL_PROBE_DELAY_S`, `HIL_PAGE_S` (60), `HIL_IDLE_S`
(60), `HIL_KEEP_VOLUMES`, and the GPS-denied and tracking tests' own.
`control <mission|gps_denied|tracking|fire>` runs the scenario all on the
workstation with the same middleware (`tools/aerodrone_hil_control.yaml`),
through `tools/gps_denied_sim_test.sh` and `tools/tracking_sim_test.sh` or,
for the mission, the Gazebo stack with the recorder, scored like a HIL
mission (output `generated/aerodrone_hil/control_<scenario>`). Exit status:
0 pass, 1 a check failed, 2 no usable run, 3 the score passed but the
navigation validation did not. Addresses, ports and the domain are in
`aerodrone_hil.env`.

### Images

The Pi needs `salsicha/cyberphysics:{aerostack2,demnav,wildnav}-arm64` of
the current Jazzy/Zenoh stack (the images there from before the Zenoh
migration were Humble, which native Zenoh cannot talk to). They are built
natively on the Pi from its copy of the repository (`git archive` of HEAD
plus its Git LFS objects), with `docker build --network host`, because the
Pi resolves names over its IPv4 route, which goes through eth0 and does not
reach the internet. On this Pi (2026-11-01) `aerostack2-arm64` was built from HEAD's
Dockerfile with its Aerostack2 source-build step taken from the Pi's BuildKit
cache of 2026-10-12: the same pinned revision (`97ade9f`) on the same base
digest, compiled with `BUILD_JOBS=2`; HEAD only adds `MAKEFLAGS` (a compiler
job limit) to that step, so the step was matched without it rather than
recompiling Aerostack2 on the Pi for an hour or more (about 90 s in all).
`demnav-arm64` and `wildnav-arm64` were built from HEAD (43 s and 28 s) with
`SEED_NAV_ASSETS=0` (HIL mounts the simulated site's caches) and `--add-host`
entries giving the package mirrors' IPv6 addresses. Host entries are part of
BuildKit's cache key, so the aerostack2 build was split at its first network
step and only the tail used them. These build steps were run by hand from a
scratch script on the Pi; they are not part of the repository.

## Results (2026-10-31 and 11-01)

Code: HEAD `907948d` with this change. Workstation: i9-13900H (20 threads),
amd64 images built from HEAD that evening, Gazebo with software rendering,
shared with another project's simulator (Isaac on its GPU, a load average of
about 6.5 throughout). Pi: Raspberry Pi 5, 8 GB, BlueOS 1.4.2 on kernel
6.6.31 with the Navigator's ArduSub 4.7.1 running, in its bench enclosure.
`ROS_DOMAIN_ID=68`. Every run was compared with the same scenario all on the
workstation run the same day with the same middleware (`control`, see
"Controls" below) and with the documented results.

### Controls

`tools/aerodrone_hil_test.sh control <scenario>` runs the scenario all on the
workstation (`tools/aerodrone_hil_control.yaml`: the Gazebo stack unchanged,
its nodes loopback Zenoh peers of a local router). The stock configuration
(multicast peers) could not be used as the control on this workstation: in a
first attempt, right after flight_ready, MAVROS logged `Unable to push non
droppable network message` and lost the autopilot's heartbeat, most likely
because its Zenoh session also reached the other project's multicast peers
on the same host. The documented baselines were made with the stock
configuration.

### 1. Mission

The Gazebo wilderness mission end to end, with flight_ready, the mission
tree and Aerostack2 on the Pi, recorded on the workstation and scored by
`tools/aerodrone_hil_mission_score.py` (each commanded waypoint's closest
approach in truth while it was the target; a waypoint counts as reached
within the stacks' 3 m `go_to_threshold`, in 3D).

| | HIL run 1 | HIL run 2 | Control (same day) | Documented full mission (2026-10-11, DDS, before the CAD airframe and Zenoh) |
|---|---|---|---|---|
| Mission tree | still in Land at the run's 38 min limit | **SUCCESS** (exit 0) | **SUCCESS** (exit 0) | SUCCESS |
| GoTos succeeded | 40 / 40 | 40 / 40 | 40 / 40 | 40 / 40 |
| Waypoints reached in truth (3 m, 3D) | 9 / 20 (all within 1 m horizontally, 4-6 m off vertically) | 20 / 20 | 20 / 20 | 20 / 20 |
| Closest approach, horizontal, median / max | 0.20 / 0.99 m | 0.08 / 0.30 m | 0.01 / 0.04 m | within 0.1 m |
| Closest approach, 3D, median / max | 4.25 / 5.77 m | 1.52 / 2.15 m | 0.03 / 0.08 m | within 0.7 m |
| Armed to disarmed (wall; simulation) | 24 min 8 s from take-off to Land, no landing | 21 min 26 s; 20 min 58 s | 19 min 9 s; 18 min 9 s | 18 min 44 s (the tree's run) |
| Landing | climbed in Land from 13 m to 306 m over 13 min | landed and disarmed | landed and disarmed | landed and disarmed |
| ArduPilot warnings | vibration compensation 8, GPS glitch 8, EKF emergency yaw reset 2 | none | none | |
| Navigation validation | passed | passed | passed | passed (111 s) |
| flight_ready after the stack started | 108 s | 108 s | 73 s | |
| Real-time factor | 0.99 | 0.98 | 0.95 | 0.97 |

Run 2 is the result: the mission flies end to end from the Pi, every
waypoint within 0.3 m horizontally in truth, 2 min 17 s slower than the
control. Its 3D figure (1.5 m median, against 0.03 m in the control) is
height: ArduPilot's own height (SITL, on the workstation) drifted from the
truth between the second and the sixth minute, at cruise, to about 1 m low
(0.83 m median while holding height, against 0.01 m in the control) and
stayed there to the landing, so every GoTo, accepted within 3 m of
ArduPilot's position, ended about 1-2 m off in height. No autopilot message
accompanied it, it was not seen in the HIL GPS-denied run's first minutes,
and its cause is not established. Over the same flight the navigation
bridge on the Pi sent 80 corrections in 16 minutes, the last ones with a
horizontal accuracy of 65-98 m, where the control's sent 100 in 75 s at
10-23 m: the Pi's DemNav and WildNav (see "2. GPS-denied") barely corrected
the cruise, though with GPS 1 healthy ArduPilot ignores these (GPS 2 is
disabled until GPS 1 is lost). Run 1 flew all 40 GoTos too, but
ArduPilot (SITL, on the workstation) reported 8 vibration-compensation and
8 GPS-glitch events and 2 EKF emergency yaw resets from the fourth minute
on; its height was 4-6 m off at nine waypoints, and in Land its vertical
velocity estimate was biased (it read -2.1 m/s while the vehicle climbed
0.4 m/s), so the descent command climbed the vehicle until the run's time
limit. None of this recurred in run 2, in any other HIL run, in the
controls, or in a take-off-and-land and a GoTo-then-land check over HIL;
nothing on the Pi's side differed between the runs (same images, the same
commands, MAVLink round trip 19 / 99 ms against 22 / 101 ms median / p95).
It came back in a later HIL run that sent no GPS_INPUT at all, whose SITL
log shows the mechanism, an attitude oscillation of the simulated vehicle
from the take-off: "6. The missions' height and the GPS_INPUT corrections".

### 2. GPS-denied

`tools/gps_denied_sim_test.sh`'s scenario: the wilderness mission, GPS 1
denied 60 s of simulation time after reaching cruise for 150 s, DemNav,
WildNav, the fusion and the navigation bridge on the Pi. Errors in metres,
median / p95 / max while GPS 1 was denied.

| | HIL | Control (same day) | Zenoh baseline (2026-10-12) | DDS CAD baselines (2026-10-12) |
|---|---|---|---|---|
| Result | **fail**: fused and autopilot p95 and max, route | pass | pass | pass (3 runs) |
| Fused fix | 11.7 / 91.9 / 135.8 | 7.1 / 11.5 / 12.7 | 6.6 / 10.9 / 12.7 | 6.2-6.7 / 9.2-9.9 / 11.2-11.7 |
| Autopilot | 11.7 / 91.7 / 135.3 | 7.1 / 11.5 / 12.7 | | |
| Route distance, max | 109.2 | 7.4 | 7.4 | 7.0-7.7 |
| DemNav fixes while denied (n), error | 347: 8.3 / 14.2 / 36.9 | 454: 6.6 / 10.1 / 15.3 | | |
| WildNav fixes while denied (n), error | 13: 2.4 / 4.8 / 7.0 | 31: 2.3 / 3.1 / 3.4 | | |
| Corrections, error; longest gap | 8.3 / 13.9 / 69.2; 9.1 s | 4.2 / 6.9 / 8.4; 4.9 s | | |
| Fused beyond 2 / 3 sigma | 14.2 / 4.7 % | 2.7 / 0.0 % | | |
| Camera to fix, whole run: DemNav; WildNav (s) | 0.29 / 0.40 / 0.55; 8.0 / 10.2 / 11.5 | 0.22 / 0.33 / 0.42; 1.2 / 1.5 / 1.6 | | |
| Navigation validation | passed | passed | passed | passed |
| Real-time factor | 0.99 | 0.97 | 0.96 | 0.94-0.98 |

The HIL run held 7-12 m for 110 s of the denial, then lost it: from 104 s
into the denial the corrections came 8.2 and 9.1 s apart (ArduPilot dropped
and re-detected its GPS 2 each time), the EKF drifted, fewer fixes passed
the fusion's gate (289 corrections from 360 fixes while denied, 80 %,
against 475 from 485, 98 %, in the control), and the autopilot was 134 m off
at the restore. The cause is on the Pi: WildNav's
SIFT matching of each still against the tile cache took 8.0 s (median) on
the throttled Pi against 1.2 s on the workstation, so WildNav, set to
match every 5 s, matched every 8-11 s on stills 8-11 s old; DemNav's fixes
came 70 ms later and less often (347 against 454 while denied; the
aircraft-rate input, 4.6 Hz at 256x192 instead of Gazebo's 5 Hz at 320x240,
accounts for about 8 % of that). The control, with the same images, site
and middleware, passed with the documented errors.

### 3. Tracking and the launcher

`tools/tracking_sim_test.sh`'s scenarios with the tracker on the Pi (wall
time, as on the aircraft), the detector on the workstation, the page served
by the Pi. Distances in metres, median / p95; thresholds in brackets.

| | HIL | Control (same day) | Gazebo baselines (2026-10-11/12, doc) |
|---|---|---|---|
| Result | **fail**: slant | pass | pass (2 runs) |
| Centered: distance from the car [12.5 / 17.5] | 1.5 / 5.4 | 0.8 / 3.4 | 1.0 / 3.6-3.7 |
| Centered: tracking [90 %], re-selections [3] | 100 %, 0 | 100 %, 0 | 100 %, 0 |
| Centered: target estimate error | 0.6 / 1.4 | 0.4 / 0.8 | 0.5 |
| Centered: car recall [0.8], IDs, followed [0.8] | 0.98, 5, 1.00 | 0.97, 5, 0.98 | 0.98, 4-5, 0.99 |
| Slant: tracking [90 %], re-selections [3] | **43 %, 8** | 99 %, 1 | 99-100 %, 0-1 |
| Slant: range error [5 / 12] | 1.1 / 10.8 | 0.9 / 2.4 | 1.0 / 2.1-3.1 |
| Slant: depression error; shallower p95 [8] | 3.6 / 14.1; **14.1** | 1.7 / 3.2; 1.7 | 1.5-1.9 / 3.2-3.3; 0-1.8 |
| Slant: recall, IDs, followed | **0.72**, 17, **0.76** | 0.99, 4, 0.99 | 0.97-0.99, 4-6, 0.99-1.00 |
| Lost (hidden 8 s): lost after [4 s], drift [1 m], tracking again [8 s] | 1.4 s, 0.01 m, 4.0 s | 0.0 s, 0.00 m, 3.2 s | 1.6-2.9 s, 0.05-0.13 m, 1.2 s |
| Stop, map goal, GoTo | pass | pass | pass |
| Detections (simulation / wall) [4.5 Hz], per frame | 5.0 / 4.8 Hz, 71 ms | 5.0 / 4.5 Hz, 80 ms | 5.0 / 5.0 Hz, 71-73 ms |
| Landing after the test | succeeded | succeeded | |

In slant the aircraft lost the car every 4-5 s (`not seen for 3 s`; 8
re-selections, the last one waiting 64 s for the car to come back into the
view): the slant set-point puts the car near the image's edge, and the
camera stamp reached the tracker 0.18-0.27 s late (median, p95; on the
workstation the same frames reach the tracker about 0.14 s after capture)
with the set-point another 0.07-0.17 s from ArduPilot, while the tracker ran
on wall time (the aircraft's choice; the control runs on simulation time).
The centered mode, which keeps the car in the middle of the image, held it
throughout. The README's earlier slant failures in Gazebo (the oscillation
that rolls the aircraft and its body-fixed camera off the car) are this
step's known weakness; the extra latency of HIL brought it back.

Fire control (the foam dart launcher's gates, `TRACKING_TEST_SCENARIO=fire`):

| | HIL | Control (same day) |
|---|---|---|
| Shot at the halted car (expect fired) | **refused**: "the target is 1.1 m from the bore's ground point (at most 1 m)" (0.64 m when the gates were checked) | fired 2.1 s after the command, 0.13 m from the car's centre, on the car |
| Fire at a standing person (expect refused by the person gate) | refused, but "not tracking (tracker lost)": **fail** | the same: **fail** |
| Fire at the car with a person in view (expect refused by that gate) | refused, but "not hovering: 2.0 m/s": **fail** | refused: "a person is in view": pass |
| Darts fired at people | none | none |

The person step fails in the control too: the tracker loses the standing
person before the fire command, so the refusal comes from the tracking
gate, not the person gate the step tests (a scenario problem, not HIL's).
In HIL the hover over the car was looser (bore 0.64-1.1 m from the car
against 0.18 m) and the aircraft was still moving when the third command
came: every refusal was right, but the shot the scenario expects did not
happen.

Page (`tools/aerodrone_hil_page_check.py`, loaded from the Pi in headless
Chromium on the workstation for 60 s of the centered follow): passed. Page
loaded in 0.95 s, rosbridge connected and first frame at 0.99 s; 271 frames
at 4.5 frames/s with the boxes drawn on 242; the tracker's state shown
(tracking, lost, acquiring); frame age from the camera stamp to the frame on
the page (detector, Wi-Fi to the Pi's rosbridge, Wi-Fi back to the browser)
0.26 / 0.34 / 0.56 s (median / p95 / max); no console errors; desktop and
phone screenshots in the run directory.

### 4. The Pi's resources

Sampled every 5 s (`aerodrone_hil_resources.py`), with BlueOS and its
ArduSub running throughout. Host CPU in % of the 4 cores; containers and
ArduSub in % of one core; median / p95 / max.

| | Idle (BlueOS, ArduSub) | Mission, run 2 (21 min) | GPS-denied (6 min) | Tracking (9 min) | Fire (8 min) |
|---|---|---|---|---|---|
| Host CPU | 1.7 / 2.0 / 2.0 | 67.0 / 89.6 / 92.1 | 72.6 / 90.4 / 95.4 | 61.9 / 69.5 / 85.7 | 63.8 / 68.3 / 78.5 |
| Load average (1 min) | 0.7 | 6.3 / 8.1 / 8.8 | 6.7 / 8.7 / 8.9 | 4.7 / 5.5 / 6.3 | 4.6 / 5.7 / 5.9 |
| Memory used (of 8 GB) | 1.2 GB | 2.6 / 2.6 / 2.6 GB | 2.5 / 2.6 / 2.6 GB | 2.5 / 2.6 / 2.6 GB | 2.5 / 2.6 / 2.6 GB |
| Temperature | 67.8-71.6 °C | 85.3 / 86.5 / 87.0 °C | 84.8 / 85.9 / 86.5 °C | 84.8 / 85.9 / 86.5 °C | 84.8 / 85.9 / 86.5 °C |
| Throttled now (share of samples) | 0 % | 95 % | 87 % | 99 % | 100 % |
| ARM clock (firmware) | 1.7-2.4 GHz | 1.5 GHz median | 1.9 GHz median | 1.5 GHz median | 1.5 GHz median |
| ArduSub (`ardupilot_navigator`) | 7.7-8.9 % | 20.5 / 24.6 / 27.0 % | 21.9 / 25.2 / 27.5 % | 17.8 / 19.0 / 22.2 % | 18.3 / 19.0 / 20.9 % |
| blueos-core (with ArduSub) | 12.3-13.7 % | 27.8 % | 29.3 % | 24.1 % | 25.0 % |
| WildNav (and the fusion) | | 123.2 / 197.3 / 213.8 % | 130.3 / 190.2 / 198.3 % | 74.2 / 84.3 / 92.3 % | 77.5 / 84.4 / 85.9 % |
| DemNav | | 43.2 / 54.2 / 59.4 % | 51.6 / 57.0 / 60.4 % | 31.5 / 34.2 / 35.6 % | 33.1 / 36.0 / 37.7 % |
| Tracker | | | | 60.8 / 66.4 / 68.8 % | 63.0 / 68.5 / 71.3 % |
| Aerostack2 | | 22.5 / 26.0 / 28.4 % | 23.8 / 26.2 / 29.5 % | 19.1 / 22.3 / 26.6 % | 19.7 / 22.4 / 22.9 % |
| MAVROS | | 15.7 / 18.1 / 19.0 % | 17.9 / 19.9 / 21.9 % | 13.0 / 13.9 / 16.3 % | 13.2 / 13.9 / 14.4 % |
| Zenoh router (HIL only) | | 6.2 / 7.6 / 11.5 % | 7.5 / 8.9 / 9.2 % | 5.8 / 6.7 / 7.5 % | 6.2 / 6.9 / 10.5 % |
| Behaviour tree, rate keeper, bridge, rosbridge | | 6.6, 3.4, 1.1, 0.3 % | 7.1, 3.5, 1.4, 0.3 % | -, 2.7, 0.9, 0.6 % (13.6 p95 with the page) | -, 2.9, 1.0, 0.3 % |

- **Thermal throttling is the limit, not memory.** Within minutes of the
  stack starting, the bench Pi reached its 85 °C limit and the firmware
  held the cores at 1.5-1.9 GHz of 2.4 for nearly every sample of every run
  (`vcgencmd get_throttled` 0xe000e: frequency capped, throttled, soft
  temperature limit, now and since boot). The idle Pi already sat at 71-79 °C.
- **What saturates:** in the GPS-denied cruise the throttled CPU ran at
  90-95 % of the four cores at p95 with a load of 7-9: WildNav's SIFT bursts
  (up to 2.1 cores) beside DemNav (0.5 core), Aerostack2, MAVROS and
  ArduSub. WildNav's latency grew sevenfold and the navigation failed (above).
  In tracking the tracker takes 0.6 core and the host stays at 62-70 % (p95),
  with little headroom left for WildNav's bursts.
- **ArduPilot** (the Navigator's ArduSub, its loop running beside the
  stack) went from 8 % of a core idle to 16-22 % under the load, on the
  slowed clock.
- **Memory** is ample: 2.5-2.7 GB used, at least 5.4 GB available.
- The topic probe itself cost the Pi 0.6-0.8 core for its 60 s windows (it
  subscribes to the depth and stills); it is not counted above.

### 5. Network

Topics crossing to the Pi, as they arrived there (probe on the Pi; age from
the camera stamp, corrected by the measured clock offset; 60 s windows):

| Topic | Rate (mission / tracking) | Size | Bandwidth | Age at the Pi, median / p95 / max | Longest gap |
|---|---|---|---|---|---|
| `/oak1/relative_depth` | 4.6 Hz / 2.8 Hz | 197 kB | 0.91 / 0.56 MB/s | 173-214 / 248-318 / 529-1416 ms | 0.50-0.53 s |
| `/oak1/image_highres` | 0.5 Hz | 922 kB | 0.46 MB/s | 155-225 / 225-298 / 283-1023 ms | 2.1-2.2 s |
| `/aerodrone/rangefinder` | 5.0 Hz | 56 B | 0.3 kB/s | 158-195 / 228-295 / 529-1223 ms | 0.31-0.37 s |
| `/tracking/detections` | 4.9 Hz | 0.6 kB | 2.8 kB/s | 182 / 274 / 656 ms (fire: 216 / 307 / 565) | 0.39 s |
| `/tracking/image/compressed` (page open) | 4.9 Hz | 12.6 kB | 62 kB/s | 185 / 273 / 656 ms | 0.39 s |
| `/aerodrone/launcher/joint_states` | 25 Hz | 92 B | 2.3 kB/s | 5 / 70 / 479 ms | 0.25 s |

| Link (Pi's `ss -ti` byte counters, 5 s samples) | GPS-denied cruise | Tracking |
|---|---|---|
| Zenoh router to router, to the Pi | 1.39 / 1.48 / 1.49 MB/s | 0.97 / 1.17 / 1.22 MB/s |
| Zenoh, from the Pi | 15 / 59 / 71 kB/s | 19 / 20 / 21 kB/s |
| MAVLink (SITL to MAVROS / back) | 3.5 / 1.7 kB/s | 3.4 / 1.6 kB/s |
| Pi's wlan0 (received / sent) | 1.45 / 0.04 MB/s | 1.02 / 0.05 MB/s (0.15 p95 with the page) |
| Share of the measured 9 MB/s to the Pi | 15-16 % | 11-13 % |

- **Latency, camera stamp to the tracker's set-point:** the detections reach
  the Pi 182 ms after the camera stamp (median; 274 ms p95, 656 ms max) in
  the tracking run, of which about 70 ms is the detector's inference on the
  workstation and 60-70 ms the simulated camera's own pipeline; the tracker
  then moves its reference at the next tick of its 10 Hz loop (0-100 ms) and
  the platform's set-point reaches ArduPilot half a MAVLink round trip later
  (22-45 ms median, 70 ms p95): about 0.25-0.3 s median, 0.45 s p95, from the
  camera to the autopilot.
- **MAVLink round trip** (MAVROS TIMESYNC, MAVROS on the Pi to SITL on the
  workstation over TCP on Wi-Fi), median / p95 / max: 19 / 99 / 215 ms
  (mission run 1), 22 / 101 / 211 ms (mission run 2), 22 / 85 / 234 ms (GPS-denied), 45 / 141 / 247 ms (tracking),
  44 / 126 / 318 ms (fire). On the aircraft this link is TCP over loopback
  to BlueOS's router.
- **Drops:** none seen. Every probed topic arrived at its publisher's rate,
  the longest gaps were one or two frame periods (0.5 s for 2.9-4.6 Hz depth,
  2.2 s for 0.5 Hz stills), and the occasional late frame (1.4 s once for a
  depth frame, 1.0 s for a still) arrived late rather than not at all
  (Zenoh's reliable TCP link). The page saw 271 frames in 60 s at 4.5
  frames/s, the longest gap 0.44 s.
- **Only Wi-Fi:** the socket audit of every run listed exactly two HIL
  sessions, both between 192.168.1.116 and 192.168.1.193 (the Zenoh routers
  on 7468 and MAVLink on 5760), and none on 10.0.0.0/24; the Pi's eth0 carried
  only BlueOS's own 7 kB/s (its GCS link to 192.168.2.1), idle or not.
- **Clocks:** the Pi's clock minus the workstation's was between -2 and
  +24 ms over the runs (±2 ms, the exchange's half round trip), drifting a
  few milliseconds within a run.

### 6. The missions' height and the GPS_INPUT corrections (2026-11-01)

Run 1 climbed in Land from 13 m to 306 m with ArduPilot's vertical speed at
-2.1 m/s while the vehicle rose at 0.4 m/s, after 8 vibration-compensation,
8 GPS-glitch and 2 EKF yaw-reset messages; run 2's height ran 1 m low at
cruise. Only MAVLink from the Pi reaches SITL, and it carries the
navigation bridge's GPS_INPUT corrections, which on the throttled Pi were
built from camera frames 0.3 s to several seconds old. They are not the
cause. The same signature came back in a later HIL run (gi1) that sent no
corrections at all, and its SITL log shows the mechanism: an attitude
oscillation of the simulated vehicle from the take-off. Run 1's own SITL
log was not kept (the runner keeps it now), so for run 1 that rests on the
matching signature.

**GPS_INPUT cannot reach the vertical estimate** (ArduCopter 4.6.3, commit
`3fc7011a` of the `Plane-4.6.3` tree in the SITL images; details and the
source in `tools/gps_denied_sitl_test.md`, "Late and bursty corrections"):

- with GPS 1 healthy the bridge keeps `GPS2_TYPE=0`, and `AP_GPS::handle_msg`
  hands GPS_INPUT only to the drivers of enabled instances: ArduPilot
  discards it. Run 2's bridge log shows `GPS2_TYPE=0` at the start, no
  `GPS2_TYPE=14` and no source-set switch all flight, 80 corrections sent
  (run 1's logs were overwritten by run 2's; same code, same scenario);
- with GPS 2 enabled the bridge's GPS_INPUT carries horizontal position
  only (altitude, velocity and their accuracies flagged absent) and source
  set 2 takes height from the barometer and fuses no velocity;
- standalone SITL (`tools/gps_input_sitl_study.py`), every ingredient
  separately: corrections 0.5-5 s late, released every 3 or 8 s, stamped
  at capture, when sent or not at all, claiming zero or true velocity,
  zero or true altitude, with GPS 1 healthy or lost and `GPS2_TYPE` 0 or 14.
  None moved the vertical estimate (vertical velocity error at most
  0.18 m/s, height 0.41 m; every landing descended at 0.5 m/s); lateness
  cost horizontal position (lateness times speed) and 8 s gaps the EKF
  failsafe. Vibration compensation and a GPS glitch appeared only when
  GPS 2 was enabled before ArduPilot ran source set 2 and claimed a (zero)
  vertical velocity: GPS 2 fused in source set 1;
- the Gazebo stack, all on the workstation with the HIL middleware, with
  `tools/gps_input_delay.yaml` making the corrections late and bursty
  (table below): the mission with every GPS_INPUT reaching ArduPilot
  0.3-2.3 s late, in bursts every 8 s, flew all 20 waypoints within 0.09 m
  and landed (vertical velocity error 0.013 m/s p95), as the control does;
- **HIL run gi1 sent none**: an FC-check bug in the first version of the
  fix (below) kept forwarding off for the whole flight, so no GPS_INPUT
  left the Pi, and the run 1 signature came back (yaw resets, vibration
  compensation, GPS glitches, lane switches; ArduPilot's height 1.4 / 2.8 m
  off, median / p95, its vertical velocity 2.3 m/s off in median all
  flight), and the vehicle, stuck at the fifth waypoint, timed out after
  4 of 20.

**What it is.** ArduPilot SITL's dataflash log, which the runner now keeps
(`sitl/` in the run directory; `tools/sitl_log_summary.py` summarises one,
with pymavlink), shows it in gi1. Gazebo's truth, SITL's GPS 1 and its
barometer agree with each other; the EKF does not. From six
seconds after arming, three metres into the take-off, the roll and pitch
rate loop diverged (rate doubling every 0.4 s) into a limit cycle that
lasted the whole flight: roll and pitch 21-26 deg standard deviation about
level, rates 150-250 deg/s, motor outputs swinging by 300 PWM (standard
deviation) between their limits, throttle 0.38 instead of 0.24. Holding
height while tilting back and forth needs more thrust, so the IMU's mean
vertical specific force was 11.6-11.8 m/s^2 instead of 9.81 (Gazebo's own
IMU sensor read the same, 11.77). EKF3 put that in its accelerometer bias
state until it saturated at `EK3_ACC_BIAS_LIM`, 1 m/s^2
(`AP_NavEKF3_core.cpp`), about 3 minutes into the flight, and the rest into
its vertical velocity, 2.2-2.4 m/s downward, held against the barometer by
a standing innovation: hence "Vibration compensation ON" (ArduCopter's
`ekf_check.cpp`: vertical velocity and position innovations both positive
with a filtered velocity variance over 1), "GPS Glitch" and the GSF's
emergency yaw resets. In Land (Aerostack2 descends in GUIDED at 0.5 m/s)
a vehicle that believes it is already descending at more than 2 m/s
throttles up, and climbs: run 1.
A smaller oscillation would leave the bias state room to absorb it and a
smaller height offset (run 2's metre); without its log that is the likely
reading, not established. All-local runs show none of it: roll and pitch
2-5 deg, motor outputs within about 50 PWM, the IMU at 9.81-9.84 m/s^2 and
the EKF's vertical velocity within 0.02-0.04 m/s of the truth (p95; every
Gazebo run below with a SITL log, the all-local mission included).

**Why the HIL runs, not established.** Every HIL mission run with the
runner since its SITL logs are kept oscillated from the take-off (gi1, gi2,
gi3 below: roll-rate standard deviation 110-210 deg/s); none of the nine
all-local Gazebo flights of the same day with SITL logs did (0.4-6 deg/s:
the runs in the table below with logs, the all-local mission included).
SITL's parameters were identical (the dataflash parameter lists differ only
in counters), the Pi sends the same velocity set-points (`GUIP` type 4) at
the take-off as the all-local stack, and the simulator side's configuration
differs only by the open-file cap and the OAK-1 stand-in. Measured during
gi1 and an all-local mission with a passive gz-transport listener in the
Gazebo container, the motor commands reached Gazebo 0.4 ms after the
backend sent them (median; p99 20 ms, max 100 ms) and Gazebo's IMU came at
197-198 Hz, identically in both. An all-local run with the HIL's open-file
cap on the simulator services did not oscillate. One five-minute HIL flight
did not either (H1, run by a scratch driver with an instrumented copy of
`gazebo_ardupilot_backend.py` that does a little more work per frame): from
the same commands it stayed level where gi1, gi2 and gi3 diverged about one
second after lift-off. The loop SITL closes through Gazebo is the suspect:
`gazebo_ardupilot_backend.py` answers each of SITL's 400 Hz servo frames
with the latest of three ROS topics, Gazebo's pose (100 Hz, whose stamp
sets SITL's clock, so SITL's physics advances in 10 ms steps; 3 of 4 frames
repeat it), its IMU (200 Hz, typically 5 ms newer than the pose) and its
twist, and the phase between those free-running clocks and the backend's
timer, which the workstation's load shifts, may decide whether the rate
loop is stable at lift-off. HIL's looser hover in the fire scenario
(0.64 m against 0.18 m) and its slant tracking may share it; their logs
were not kept.

Corrections, late and bursty, in the Gazebo stack (`gps_denied_sim_test.sh
gazebo` with the HIL control middleware; errors while GPS 1 was denied,
median / p95 / max, in metres; ArduPilot's vertical velocity and height
against the truth from SITL's log):

| Run | Bridge | Corrections | Result | Fused | Autopilot | Vertical velocity error p95 / max | Height error max |
| --- | --- | --- | --- | --- | --- | --- | --- |
| control (11-01, `control gps_denied`) | before | on time | pass | 7.1 / 11.5 / 12.7 | 7.1 / 11.5 / 12.7 | | |
| B1 | before | 0.3-1.0 s late to the bridge | pass | 9.2 / 14.4 / 17.1 | 9.2 / 14.4 / 17.1 | 0.05 / 0.27 m/s | 0.04 m |
| B2 | before | 0.3-2.3 s late, released every 8 s | fail: none reached ArduPilot (all over 1 s), EKF variance failsafe and LAND 11 s into the denial | 0.7 / 2.9 / 3.2 (until it stopped) | 80.6 / 129.8 / 131.2 | 0.02 / 0.03 m/s | 0.09 m |
| A0 (without `GPS2_DELAY_MS`) | after | on time | pass | 7.3 / 11.9 / 13.0 | 7.3 / 11.9 / 13.1 | 0.04 / 0.07 m/s | 0.04 m |
| U1 (A0 with the HIL's open-file cap) | after | on time | pass | 6.5 / 10.9 / 12.8 | 6.5 / 11.0 / 12.8 | 0.03 / 0.08 m/s | 0.04 m |
| acceptance (`control gps_denied`, final code) | after | on time | pass | 7.0 / 11.0 / 12.6 | 7.0 / 10.9 / 12.7 | | |
| A0f (final code, through the delay layer) | after | on time | pass | 7.0 / 10.9 / 15.7 | 7.0 / 10.9 / 15.8 | 0.03 / 0.18 m/s | 0.04 m |
| A1 | after | 0.3-1.0 s late to the bridge (as B1) | pass; 9 % dropped as over 1 s, bursts collapsed to their newest, never suspended | 7.9 / 13.2 / 15.6 | 7.9 / 13.2 / 15.5 | 0.05 / 0.13 m/s | 0.05 m |
| Mission, all-local (`control mission` with the delay layer) | before | every GPS_INPUT 0.3-2.3 s late to MAVROS, every 8 s (GPS 1 healthy) | pass: 20 / 20 waypoints, 0.03 / 0.09 m in 3D, landed and disarmed after 19 min 23 s | | | 0.013 / 0.50 m/s (touchdown) | 0.24 m |

With the bridge forwarding up to 1 s late corrections the Gazebo stack
passes, before (B1) and after (A1, slightly better) the gate; later ones the
bridge already dropped, and HIL-like bursts of 8 s left ArduPilot without
corrections and its EKF failsafe landed it (B2), the same outcome as no
corrections at all (a first B2 attempt whose injector failed: 79.3 / 131.1
/ 132.5 m, LAND). Nowhere did late corrections touch the vertical. The
GPS-denied acceptance passes with the final code against the committed
baselines (Zenoh 6.6 / 10.9 / 12.7 m, the same day's control 7.1 / 11.5 /
12.7 m): corrections 4.1 / 7.1 / 9.0 m, DemNav 445 and WildNav 32 fixes
while denied (control 454 and 31). In a healthy pipeline the gate drops
nothing as stale: corrections reached the bridge 5-23 ms after their stamp
at least, 23-45 ms in median and at most 0.12 s; it held back 3-4 % that
came within 0.2 s of the previous one and 4-5 with the same stamp as the
last sent (two fixes accepted on one odometry sample).

**Fixed in the bridge anyway** (`ardupilot_navigation_bridge.py`, tested in
`tests/test_navigation_ardupilot_bridge.py`):

- GPS 2 exists only while source set 2 runs: the bridge asks for set 2 and
  sets `GPS2_TYPE=14` once ArduPilot accepted it, and on recovery sets
  `GPS2_TYPE=0` and asks for set 1 once that is confirmed. Before, both
  requests left in the same callback; in the HIL and control GPS-denied
  runs ArduPilot confirmed `GPS2_TYPE=14` 5-9 ms before set 2, so GPS 2
  existed in set 1 only briefly, but a lagging companion or a refused or
  lost command would stretch that window, and three seconds of it gave
  "GPS Glitch" and the EKF failsafe in SITL;
- a correction gate (`CorrectionGate`): corrections more than 1 s old at
  the bridge (`max_correction_age_s`, as before) or stamped more than 0.5 s
  ahead are dropped, as is one not newer than the last sent; at most one
  goes out per 0.2 s (the newest of a burst); when 5 of the last 10 arrive
  stale forwarding stops (`GPS_INPUT corrections SUSPENDED: ... the
  companion is lagging`) until 5 arrive in time in a row, so a slow
  companion makes ArduPilot dead-reckon and, with GPS 1 lost, land through
  its EKF failsafe, rather than feed it a sporadic, misdated stream;
- `GPS2_DELAY_MS=20` (`arducopter_gps_input.params`): ArduPilot dates a
  GPS_INPUT by its GPS time plus the transport delay of the fastest
  recent message, less this lag (the MAV driver's 200 ms when 0); the
  bridge's stamp is the odometry sample the fusion propagated the fix to,
  and the shortest stamp-to-bridge age measured in the Gazebo stack is
  5-23 ms (median 23-46 ms, at most 0.22 s). ArduPilot hides the parameter
  while `GPS2_TYPE=0`, so the bridge cannot check it through MAVROS: the
  first version required it, and in gi1 waited for it all flight;
- every 20 corrections the bridge logs their age on arrival (min, median,
  p95, max) and what it dropped.

**HIL re-runs** (`tools/aerodrone_hil_test.sh mission`, 2026-11-01, the
Pi and set-up as in "Results"; SITL logs in `generated/aerodrone_hil/
mission_gi*/sitl`, summarised by `tools/sitl_log_summary.py`; the mission
scores re-run with the crash check below):

| | gi1 | gi2 | gi3 |
| --- | --- | --- | --- |
| Bridge | first version of the fix: its FC check waited all flight for `GPS2_DELAY_MS`, so no GPS_INPUT was sent | final | final |
| Corrections sent; age at the bridge (min / median / p95 / max of the logged windows) | none | 100: 10-16 / 30-42 / 72-97 / 200 ms; nothing dropped as stale, never suspended, `GPS2_TYPE` stayed 0 | 120: 14-26 / 37-48 / 66-83 / 94 ms; the same |
| Attitude loop from the take-off: roll-rate / motor-output standard deviation | 206 deg/s / 305 PWM | 156 / 283 | 126 / 262 |
| ArduPilot's vertical velocity error, median / p95 | 2.29 / 2.59 m/s | 0.64 / 1.10 | 0.23 / 0.55 |
| ArduPilot's height error, median / p95 | 1.42 / 2.76 m | 0.94 / 1.43 | 0.34 / 0.46 |
| Warnings | vibration compensation 3, GPS glitch 3, emergency yaw reset 2, lane switch 2 | none until touchdown | none |
| Mission | timed out at 40 min, 4 / 20 waypoints (stuck at the fifth) | tree SUCCESS, 20 / 20 (3D 1.61 / 2.58 m), but the vehicle tipped over at touchdown: "Crash: Disarming: AngErr=150>30", 20 min 34 s after arming | tree SUCCESS, 20 / 20 (3D 0.62 / 1.10 m), **landed and disarmed** after 20 min 30 s |
| MAVLink round trip, median / p95 | | 28 / 111 ms | 29 / 119 ms |

So with the fix the companion's corrections reached the bridge 10-50 ms after
their stamp (100 ms at p95 at most, 200 ms once) on the throttled Pi, well
inside the 1 s bound, and ArduPilot ignored them all with GPS 1 healthy, as
it must. The landing failed in two of three HIL runs anyway, from the
oscillation, whose size decided the outcome: the EKF absorbs a small one in
its bias state (gi3), a larger one leaves about a metre of height error
(gi2, and most likely run 2) and in gi2 a tip-over at touchdown, a larger
one still saturates the bias state and leaves 2 m/s (gi1, and most likely
run 1). `tools/aerodrone_hil_mission_score.py` used to count gi2 as landed
(disarmed on the ground); it now counts ArduPilot's warnings, and a
`Crash:` text means no landing (`tests/test_aerodrone_hil.py`).

### 7. GPS-denied with the lighter WildNav profile (2026-11-01)

WildNav on the companion computers now runs a lighter profile
(`tools/wildnav_profile.md`; `compositions/blueos_nav_core.yaml`, mirrored by
`tools/aerodrone_hil_pi.yaml`):
- the tile search ends at the first tile nearest the prior that matches
  with confidence 0.5 (`match_stop_confidence`);
- OpenCV runs on two threads (`opencv_threads`);
- the tiles' features stay in memory between matches.

The Pi's `wildnav-arm64` was rebuilt with it on the Pi.

In the HIL run (`generated/aerodrone_hil/gps_denied_lite`) the
workstation's ArduPilot SITL already ran without the attitude oscillation
of section 6:
- SITL's IMU rate estimate was 100-109 Hz (683-841 Hz in gi1-gi3);
- roll and pitch varied by 3.2 / 5.8 deg over the flight (21-27 deg before);
- the EKF tilt stayed within 0.6 deg of the truth for the first 108 s of
  the denial.

The control ran all on the workstation the same day, through `control
gps_denied` from HEAD with only the WildNav change. Errors in metres,
median / p95 / max while GPS 1 was denied.

| | HIL, lighter profile | HIL before ("2. GPS-denied") | Control, lighter profile | Control before |
|---|---|---|---|---|
| Result | **fail**: fused and autopilot p95 and max | fail | **pass** | pass |
| Fused fix | 11.8 / 94.1 / 147.9 | 11.7 / 91.9 / 135.8 | 5.3 / 8.3 / 9.6 | 7.1 / 11.5 / 12.7 |
| Autopilot | 11.8 / 89.3 / 145.0 | 11.7 / 91.7 / 135.3 | 5.3 / 8.3 / 9.5 | 7.1 / 11.5 / 12.7 |
| DemNav fixes while denied (n), error | 240: 9.4 / 23.8 / 743.7 | 347: 8.3 / 14.2 / 36.9 | 444: 6.3 / 9.5 / 12.7 | 454: 6.6 / 10.1 / 15.3 |
| WildNav fixes while denied (n), error | 27: 5.0 / 31.9 / 48.4 | 13: 2.4 / 4.8 / 7.0 | 30: 2.0 / 3.1 / 3.6 | 31: 2.3 / 3.1 / 3.4 |
| Corrections, error; longest gap | 6.6 / 13.9 / 40.2; 9.9 s | 8.3 / 13.9 / 69.2; 9.1 s | 3.5 / 5.0 / 6.1; 5.2 s | 4.2 / 6.9 / 8.4; 4.9 s |
| Fused beyond 2 / 3 sigma | 18.0 / 4.0 % | 14.2 / 4.7 % | 0.0 / 0.0 % | 2.7 / 0.0 % |
| WildNav, camera to fix, whole run (s; nearest-rank p95, from the recordings) | 1.9 / 3.8 / 6.3 | 8.0 / 9.8 / 11.5 | 0.30 / 0.41 / 0.56 | 1.19 / 1.51 / 1.59 |
| WildNav's match on the Pi (its log) | 0.2-0.5 s, usually 1 of 121 tiles | about 8 s, every tile | | |

The Pi while flying, median / p95 / max:

| | Lighter profile | Before |
|---|---|---|
| Host CPU | 44.9 / 58.5 / 77.0 % | 72.6 / 90.4 / 95.4 % |
| WildNav container (with the fusion) | 50.8 / 57.9 / 157.9 % | 130.3 / 190.2 / 198.3 % |
| Temperature | 82.3 / 84.8 / 84.8 °C | 84.8 / 85.9 / 86.5 °C |
| Throttled now (share of samples) | 57 % | 87 % |
| ARM clock, median | 2.3 GHz | 1.9 GHz |

- **WildNav's latency is fixed.** The camera-to-fix time in HIL is now
  mostly the still's age: the Pi's stills come at 0.5 Hz over Wi-Fi and
  WildNav matches the newest every 5 s. The two oldest stills were 5-6 s
  old, waiting on the link. All-local, the camera-to-fix time is a quarter
  of what it was.
- **The early stop did not pick wrong tiles.** WildNav's worst fixes in
  HIL were real matches: 100-340 inliers at confidence 0.8-0.9, mostly on
  the first tile tried.
  - The 14-19 m errors came in the denial's last 40 s, when the EKF's pitch
    was 3.5-3.8 deg off. 220 m above the ground that is 14-15 m at the nadir,
    which the fix's sigma (attitude 0.5 deg) does not cover.
  - The 29-33 m errors were the two stills 5-6 s old, while the EKF's
    attitude was still within 0.35 deg.

  All-local, the profile's WildNav fixes were as good as before.
- **The navigation still fails in HIL.** The fused fix drifted from about
  20 s into the denial, while WildNav's fixes were still within 1-8 m. Two
  things are worse in HIL, in both HIL runs:
  - **DemNav's fixes** were worse than all-local both times. Its input in
    HIL is the stand-in's 256 x 192 depth at 4.6 Hz, against Gazebo's
    320 x 240 at 5 Hz. This run also had an outlier of 744 m, and DemNav's
    camera-to-fix time reached 4.5 s at most (0.6 s before).
  - **The corrections** came up to 9-10 s apart.

  What makes the HIL navigation drift is not established. It is no longer
  WildNav's compute, and no longer the SITL oscillation.

## Findings, fixes and limits of these runs

Fixed in the repository:

- `applications/aerostack2/scripts/ardupilot_navigation_bridge.py`: GPS 2
  exists only while EKF source set 2 runs (configured after ArduPilot
  accepts set 2, removed before set 1 is asked for); a correction gate
  drops stale, future and out-of-order corrections, sends at most one per
  0.2 s and stops forwarding while the companion lags; `GPS2_DELAY_MS=20`;
  corrections' ages logged ("6. The missions' height").
- `tools/aerodrone_hil_test.sh` keeps ArduPilot SITL's dataflash log and
  console (`sitl/`), and makes the Pi's `generated/aerodrone_hil` as the SSH
  user (a bind mount had created it as root after it was deleted).
- `tools/aerodrone_hil_mission_score.py` counts ArduPilot's warnings and
  no longer counts a vehicle its crash check disarmed as landed.
- `tools/gps_denied_sim_test.sh` takes several override files;
  `tools/gps_input_delay.yaml` and `tools/gps_input_delay_injector.py` make
  the corrections late and bursty for tests; `tools/gps_input_sitl_study.py`
  is the standalone SITL study; `tools/sitl_log_summary.py` summarises a
  SITL log (EKF against the simulator, the attitude loop, warnings).

- `tools/tracking_sim_test.yaml`: its two ROS services (the test driver and
  the truth boxes) did not load `systems/ros2/middleware.env`, so they
  ignored `ZENOH_CONFIG_OVERRIDE` and kept the image's multicast-peer
  default: with a router (the routed setup of `systems/ros2/README.md`, or
  this HIL link) the tracking test could not reach the stack. They load it
  now, like every other ROS service (`tests/test_aerodrone_hil.py`).
- `tools/gps_denied_sim_scorer.py`: a camera fix stamped in wall time beside
  a simulation-time truth (Gazebo's) was scored against the truth at its
  arrival, not at its camera stamp, adding the fix's latency times the speed
  to its error. It is now scored against the truth at that wall time (by
  the truth's arrival), as the HIL stand-in stamps frames
  (`tests/test_gps_denied_sim_scorer.py`).

Found and not changed (reported):

- **The bench Pi throttles.** At the HIL load it sits at 85 °C and the
  firmware holds the cores at 1.5-1.9 GHz in 87-100 % of samples; WildNav's
  matching then took 8 s and the GPS-denied navigation failed. WildNav's
  lighter profile (section 7) brought the matching to 0.2-0.5 s and the
  throttled share to 57 %; the GPS-denied navigation still fails in HIL. Cooling (an
  active cooler or the aircraft's airflow) has to be checked before trusting
  the aircraft's GPS-denied flight; on the aircraft WildNav's matching cost
  also competes with the tracker.
- **The Pi's IPv4 default route** goes through eth0 (10.0.0.1), which does
  not reach the internet, and its only DNS server is 8.8.8.8, so name
  resolution fails on the Pi (builds here used host entries over IPv6). A
  system setting, not changed.
- **`tests/test_ros_open_file_limit.py`'s list of Jazzy images is stale:**
  since the Zenoh migration the aerostack2 family (aerostack2, demnav,
  wildnav, oak1) is Jazzy too, so the aircraft compositions' MAVROS,
  DemNav, WildNav, Aerostack2 and OAK-1 services lack the open-file cap the
  test means to enforce. Harmless on this Pi (Docker gives 1024 / 524288
  already); the HIL layers set it on every ROS container.
- **The fire scenario's person step fails all-local too:** the tracker loses
  the standing person before the fire command, so the refusal comes from
  the tracking gate, not the person gate the step tests.
- **The stock all-local stack's multicast peers** can tangle with another
  Zenoh graph's peers on the same host (MAVROS lost its heartbeat here);
  `tools/aerodrone_hil_control.yaml` isolates a run.
- The wired 10.0.0.0/24 path: 7-25 % loss with spikes up to 1 s, and the
  workstation's USB adapter saturated by another project; not used.

What failed and why:

- GPS-denied in HIL (the control passed): the Pi's throttled CPU (above).
- Slant tracking in HIL (the control passed): latency from the camera to the
  autopilot of 0.25-0.3 s median and 0.45 s p95, with the tracker on wall
  time, revived the slant step's oscillation; the car left the view every
  4-5 s.
- The fire scenario's car shot in HIL: refused by the 1 m bore gate (1.1 m),
  the hold looser than all-local (0.64 m against 0.18 m); and the person-in-
  view step refused for motion (2 m/s) instead of the person gate.
- The HIL missions' height: run 1's EKF warnings, biased vertical velocity
  and climb in Land are an attitude oscillation of the simulated vehicle
  (seen in SITL's log of a later HIL run that sent no GPS_INPUT), not the
  GPS_INPUT corrections; run 2's 1 m drift is likely a milder case. What
  starts the oscillation in HIL runs and not all-local is not established
  ("6. The missions' height and the GPS_INPUT corrections").

Not done:

- **Isaac.** Not run: the Isaac image was not built on this workstation, and
  its GPU was busy with another project's Isaac session throughout (100 %
  utilisation). The HIL layers carry nothing Gazebo-specific beyond the
  camera topic names, but an Isaac HIL layer was not written or tested.
- No run was repeated except the mission; results rest on single runs.

## Limits: what this HIL covers and what it does not

- **In the loop:** the Pi 5's CPU, memory and temperature under the
  aircraft's companion stack with BlueOS and its ArduPilot running; the
  aircraft's arm64 images, compositions, MAVROS plugin list, rate keeper,
  navigation bridge, DemNav, WildNav, fusion, Aerostack2, tracker and fire
  control, rosbridge and page; wall-time operation as on the aircraft; a
  real network between the companion computer and everything else.
- **Not in the loop:** the Navigator's sensors and the ArduPilot binary that
  flies (SITL ArduCopter 4.6.3 on the workstation, with the simulations'
  relaxed arming and failsafes; the bench Navigator runs ArduSub and is
  never armed or commanded); the OAK-1 (its MegaDepth relative depth, its
  0.5 s depth latency and receive-time stamps, the 1280x720 stills, the
  on-camera YOLO and its USB transport: the stand-in sends Gazebo's metric
  depth, 640x480 stills and a CPU detector on the workstation); the
  rangefinder's rate (5 Hz instead of about 100 Hz) and noise; the servo
  bridge of the launcher; the ground station.
- **The link is not the aircraft's.** On the aircraft MAVLink and the
  cameras are local to the Pi (TCP loopback, USB); here MAVLink and every
  camera frame cross Wi-Fi, which adds latency and load the aircraft does
  not have, and the Wi-Fi between the workstation and the Pi is the local
  Wi-Fi network (one access point), not the Wi-Fi HaLow link to the ground station
  (`hardware/aerodrone.md`, "Radio"): its bandwidth, range and loss are not
  measured here. The page's figures are the workstation's browser over this
  Wi-Fi.
- **Zenoh routers.** The aircraft's nodes are multicast peers without a
  router; here each host has a router and the link carries one session. The
  Pi's router's CPU is reported separately.
- **Thermal.** The bench Pi's cooling is not the aircraft's (no prop wash);
  the temperatures and throttling above are the bench's.
- **Latency** from the camera to the autopilot is the sum of measured
  stages (camera to the Pi, the tracker's loop period, half the MAVLink
  round trip), not an end-to-end trace of one frame.
- Gazebo with software rendering only; Isaac was not run (see "Not done").
