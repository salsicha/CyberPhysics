# Aerodrone object tracking: interface contract

The aerodrone follows a ground object picked in the web UI, in one of two
modes, on the real aircraft (BlueOS: Raspberry Pi 5 + Navigator + downward
OAK-1) and in the Gazebo and Isaac simulations. This file is the
contract between the parts; change it only together with every part that
uses it.

## Camera and models

- Tracking uses the existing nadir camera (the OAK-1 on the aircraft, the
  downward RGB camera in the simulators). No second camera.
- On the aircraft both networks run on the OAK-1's Myriad X: MegaDepth
  (unchanged, for DemNav) and a YOLO detector (COCO-class or aerial-trained,
  chosen by measurement), with DepthAI's on-device ObjectTracker giving
  stable track IDs. The Pi only relays and geolocates.
- In the simulators, where there is no OAK-1, a host-side detector runs the
  same YOLO model on the simulated camera and a host-side tracker assigns
  IDs, publishing exactly the topics below.

## Topics (prefix `TRACKING_TOPIC_PREFIX`, default `/tracking`, in `config/topics.env`)

| Topic | Type | Producer | Content |
|---|---|---|---|
| `/tracking/detections` | `vision_msgs/msg/Detection2DArray` | detector (OAK-1 node or sim detector) | `header.stamp` = the source image's stamp; `header.frame_id` = the camera optical frame. Per detection: `id` = track ID string, stable while the object stays tracked; `bbox.center.position.x/y`, `bbox.size_x/size_y` in pixels of the image described by `/tracking/camera_info`; `results[0].hypothesis.class_id` = class name (`person`, `car`, ...), `.score` in [0, 1]. Target rate 5 Hz or more. |
| `/tracking/image/compressed` | `sensor_msgs/msg/CompressedImage` (JPEG) | detector | The frame the detections refer to (same stamp), at most 640 px wide, at least 2 Hz, for the UI. |
| `/tracking/camera_info` | `sensor_msgs/msg/CameraInfo` | detector | Intrinsics for that image's resolution. |
| `/tracking/command` | `std_msgs/msg/String` (JSON) | UI | `{"action": "track", "target_id": "7", "mode": "centered", "altitude_agl_m": 40}`; `{"action": "track", "target_id": "7", "mode": "slant", "slant_range_m": 60, "depression_deg": 70}`; `{"action": "stop"}`. Optional fields default as below. The launcher's commands, fire, launcher_arm, launcher_disarm and reload: "Foam dart launcher" below. |
| `/tracking/status` | `std_msgs/msg/String` (JSON), reliable + transient-local, 2 Hz | tracker | `{"state": "idle" \| "acquiring" \| "tracking" \| "lost" \| "error", "mode": ..., "target_id": ..., "target": {"east": .., "north": .., "up": .., "speed_mps": ..}, "slant_range_m": .., "depression_deg": .., "message": ".."}`, and the launcher's fields ("Foam dart launcher" below), also on every change of `fire_block` (at most 5 Hz). |
| `/tracking/target` | `geometry_msgs/msg/PointStamped` | tracker | The selected target's estimated position in the navigation frame (ENU about `/navigation/origin`). |
| `/tracking/shots` | `std_msgs/msg/String` (JSON), reliable | tracker (launcher fitted) | One message per dart released ("Foam dart launcher" below). |

Simulators only:

| Topic | Type | Content |
|---|---|---|
| `/simulation/targets` | `std_msgs/msg/String` (JSON), 10 Hz or more | Ground truth: `{"stamp": <sim s>, "targets": [{"name": "car_1", "class": "car", "east": .., "north": .., "up": .., "yaw": ..}]}`, ENU metres about the same origin as the navigation frame. A hidden target (below) keeps its route position and has `"hidden": true`. |
| `/simulation/targets/hide` | `std_msgs/msg/String` | Comma-separated target names the simulator hides under the terrain, out of the camera's sight (an occlusion); an empty string shows them all. |
| `/simulation/targets/halt` | `std_msgs/msg/String` | Comma-separated target names the simulator stops where they are, marked `"halted": true` in `/simulation/targets`; left out of a later message, they go on along their routes from there (the launcher fires only at a still target). |
| `/simulation/launcher/shots` | `std_msgs/msg/String` (JSON) | The launcher stand-in's shots, the moment its sear released: `{"stamp": <sim s>, "shot": <count>}`. |

## Modes

The target's ground position comes from the detection's pixel ray through
the camera pose at the image stamp (attitude interpolated as in
`applications/*/camera_geometry.py`), intersected with the terrain.

- **centered**: keep the target at the image centre, i.e. hover directly
  above it at `altitude_agl_m` (default: the current height above ground,
  clamped to the limits below) and follow it.
- **slant**: keep slant range `slant_range_m` and depression angle
  `depression_deg` (angle below the horizontal from the drone to the target):
  horizontal stand-off `R cos(theta)`, height above the target `R sin(theta)`,
  on the target's current bearing from the drone; yaw so the target lies along
  the image's long axis. A nadir camera with the OAK-1's ~69 deg horizontal
  field of view sees the target only within about 30 deg of nadir (with
  margin), so `depression_deg` is limited to [60, 90]; 90 equals centered.

Limits (parameters): minimum height above ground 15 m, maximum 120 m;
maximum horizontal speed; a target lost for `lost_timeout_s` (default 3 s)
makes the drone hover in place (state `lost`); `stop` hovers in place
(state `idle`). Starting a track pre-empts any running GoTo; a map goal or a
mission step pre-empts tracking.

## Settled details

These were fixed while the parts were built and tested:

- Positions: `/tracking/target` is a `PointStamped` in the odometry's frame;
  `/simulation/targets` positions are ENU metres about the navigation
  frame's origin, the vehicle's home (ArduPilot's EKF origin), with `up` at
  ground level under the target.
- Slant defaults: `slant_range_m` 60, `depression_deg` 70. The depression is
  raised above 60 deg when the camera's field of view is narrower than the
  OAK-1's, so the target stays 4.5 deg inside the image.
- Centered yaw: a target moving at 2 m/s or more (`centered_yaw_min_speed_mps`)
  is followed with its direction of travel along the image's long axis (the
  end nearer the current yaw), where the image reaches farthest ahead of and
  behind it; a slower one keeps the yaw (spun about a walking person's tight
  turns, the detector found the person less often). The VisDrone detector
  also finds a car lying along the long, horizontal, axis more often than
  one lying along the short axis: in 92 % of the frames against 60 % in the
  Gazebo test flights.
- Status: `slant_range_m` and `depression_deg` report the measured geometry
  from the drone to the target, in both modes.
- Commands: a numeric `target_id` is accepted. A rejected command sets state
  `error` when idle and leaves an active track unchanged, with "rejected
  command: ..." in `message`. A target never seen after a track command goes
  `lost`; one that is seen but cannot be placed on the ground goes `error`.
- Track handover: detector track IDs change under camera motion (a
  body-fixed camera on a manoeuvring aircraft moves the image by tens of
  pixels a frame; in the Gazebo test flights the followed car got a new ID
  every 8-14 s).
  When the selected ID is missing from a frame at most `handover_window_s`
  (2.5 s, within `lost_timeout_s`) after the target was last placed, the
  tracker follows a new ID of the same kind of object (vehicle: car, van,
  truck, bus; two-wheeler: bicycle, motorcycle, tricycle; any other class
  only itself), with a box within `handover_size_ratio` (2) of the target's
  last one, placed within `handover_max_m` (4 m) plus half the distance the
  target covered meanwhile of its position predicted along its arc, inside
  the filter's gate, when it is the only such one there. `status` then
  carries the new `target_id`, and `message` says "handed over from <old>".
  An occluded target comes back later than that, too uncertain to tell from
  a look-alike: the operator selects it again. A look-alike that appears
  within those few metres while the target is hidden is taken for it. The
  selected ID counts only while its class is of the kind first seen: a
  car's ID that the detector moves onto a person is treated as gone.
- Lead: a still target is not led: one whose filtered speed has stayed
  under `still_speed_mps` (0.5 m/s) for `still_s` (2 s), until it exceeds
  twice that. A parked car's filtered velocity is noise (0.1-0.4 m/s), and
  the full 2.5 s lead swung the reference a metre either way; hovering 17 m
  above it in Gazebo the aircraft rolled and pitched 10-16 deg either way
  (median 12 deg), and under 2 deg (median 0.5-1.3) without the lead.
- Speed: the effective horizontal limit is the lower of
  `TRACKING_MAX_SPEED_MPS` and ArduPilot's `WPNAV_SPEED`.
- Time: the tracker runs on its node clock, simulation time under
  `use_sim_time` (the simulators set it: their /clock may run slower than
  real time, Isaac's at about 0.2x), wall time on the aircraft.
  `lost_timeout_s`, `handover_window_s`, the target filter's time steps and
  velocities, the lead and the reference's speed and yaw-rate limits are
  seconds of that clock, the one ArduPilot and the targets move in; a
  detection's time is its image's stamp on that clock. (They were wall
  time at first: 3 s was 0.6 s of Isaac's simulation, and there the
  reference ran at five times the speed limit, held back only by
  `max_reference_lead_m`.)
- Profiles: `tracking` runs the detector and tracker; `tracking_truth`
  replaces the detector with one that projects `/simulation/targets` into
  the camera (simulators only).
- Detector: a YOLOv8n trained on VisDrone (classes `person`, `car`, `van`,
  `truck`, `bus`, `bicycle`, `motorcycle`, `tricycle`), confidence 0.2,
  the same ONNX in the simulators (ONNX Runtime on the CPU, host-side IDs)
  and compiled for the OAK-1 (DepthAI ObjectTracker IDs). It is
  CC BY-NC-SA 3.0 (VisDrone) and AGPL-3.0 (Ultralytics): non-commercial use
  only (`THIRD_PARTY_NOTICES.md`).
- MegaDepth on the OAK-1 keeps its topics and network but not its rate: the
  two networks share the Myriad X, so MegaDepth runs at about 2.9 Hz while
  the detector runs (4.6 Hz otherwise). The detector runs only while a
  `/tracking/*` topic has a subscriber.
- Image size: the aircraft's `/tracking/image/compressed` and camera_info are
  640x352 (the OAK-1's 16:9 frame); the simulators publish their 640x480
  frame. Consumers must use camera_info, not assume a size.
- Stamps: the aircraft's `/tracking/*` topics carry the frame's capture time
  (about 0.16 s before the host receives it); its `/oak1/*` topics still carry
  the host receive time.

## Foam dart launcher

The optional payload of hardware/aerodrone.md ("Foam dart launcher"): the
turrets' spring-plunger launcher for Elite-size foam darts, toy power only,
pointing straight down under the hub, its bore the vertical line through
the hub centre and the centre of mass. Its mechanism and joints are the
turrets' (systems/racecar_turret/interfaces.md, "Foam dart launcher"): an
ST3215 rack (bus ID 14, `actuator_joint`, metres, 0 forward) primes the
plunger, an ST3215 trigger (ID 15, `trigger_joint`, radians, 0 safe)
releases the sear. On the aircraft both are driven through a Waveshare Bus
Servo Adapter (A) on the Pi 5's USB by the turret bridge's `sts` transport
with no pan and tilt (`turret_arduino_bridge.py`, `pan_tilt_fitted` false;
`compositions/aerodrone_tracking.yaml`, profile `launcher`, service
`launcher_bridge`, with `TRACKING_LAUNCHER_FITTED=true` and the adapter's
`/dev/serial/by-id/` path in `TURRET_ARDUINO_PORT`). The simulators run a
stand-in (`aerodrone_tracking_sim.yaml`, `tracking_launcher`:
`applications/aerostack2/scripts/launcher_standin.py`) with the same topics
and safety behaviour, the joints at the ST3215's speed on the simulation
clock, and no dart physics.

**Objects only, never people; the operator fires every shot.** The fire
control runs in the tracker (`applications/aerostack2/scripts/launcher_core.py`,
which gives the reasons for every value below).

Topics (prefix `/aerodrone/launcher`, the bridge's or the stand-in's):

| Topic | Type | Producer → consumer | Content |
|---|---|---|---|
| `joint_commands` | `sensor_msgs/msg/JointState` | tracker → bridge | `actuator_joint` (absent: the rack stops where it is) and `trigger_joint` goals. Sent at once when a cycle starts or stops (an abort: the trigger at 0, no rack goal), then every 0.1 s while the launcher is off rest; none at rest. |
| `joint_states` | `sensor_msgs/msg/JointState` | bridge → tracker | The measured rack and trigger, 25 Hz, when both servos answer. |
| `emergency_stop` | `std_msgs/msg/Bool`, reliable + transient-local | tracker → bridge | True while the launcher is disarmed (from the start): the bridge then returns the trigger to 0, stops the rack and drops commands. |
| `actuator_ready` | `std_msgs/msg/Bool`, reliable + transient-local | bridge → tracker | True while commands reach the servos (port open, emergency stop released). |

Commands (`/tracking/command`):

- `{"action": "launcher_arm"}` and `{"action": "launcher_disarm"}`: the
  launcher's arm state, latched, disarmed at start; arming again after a
  fault resets it (the launcher returns to rest). The hardware safety catch
  is mechanical and independent of it.
- `{"action": "reload", "darts": 6}`: the darts in the clip, a whole
  number from 0 to 6 (0 at start), only while disarmed and no cycle runs.
- `{"action": "fire", "target_id": "7"}`: one dart at the tracked target:
  prime (the rack back to the sear, verified from the measured position),
  rack forward, trigger, trigger back to 0, then record the shot. One
  command, one cycle, at most one dart. Nothing else primes or fires the
  launcher: there is no automatic, timed or on-lock firing, and no option
  that enables it.

A fire command runs only when every gate holds, otherwise it is refused
with the reason in the status's `fire_block` (the first gate that does not
hold, in this order):

1. the launcher fitted, armed, and the bridge's `actuator_ready` true and
   fresh (1 s);
2. the tracker tracking, in centered mode, the target the command names
   (a stale tap for another target is refused);
3. the target's class known and not a person's (`person`, and VisDrone's
   `pedestrian` and `people` if a detector reports them);
4. no person anywhere in the camera's view: none in the latest
   `/tracking/detections` frame, which must be at most
   `fire_detection_timeout_s` (0.8 s) old, and none in any frame of the
   last `fire_person_clear_s` (3 s); a detection without a class counts as
   a person;
5. the target placed in a frame of the last 0.8 s;
6. hovering: the aircraft's horizontal speed at most `fire_max_speed_mps`
   (0.5 m/s, over the last 0.5 s of odometry) and its tilt at most
   `fire_max_tilt_deg` (5 deg), over a target moving at most
   `fire_max_target_speed_mps` (0.5 m/s);
7. the height above ground (the rangefinder, else the DEM under the
   aircraft, as the tracker uses; refused when neither is known) within
   [`fire_min_agl_m`, `fire_max_agl_m`] = [8, 20] m. A dart's fall never
   ends faster than the muzzle velocity (foam darts fall at 13-15 m/s at
   most), so the window is set by where the dart lands and what the camera
   can check: from 20 m a 5 m/s wind and 3 deg of dispersion put it within
   about 3.3 m of the bore's ground point, under half of the 7.6 m the
   OAK-1 sees to either side across its image, and a person still spans
   about 10 px of its 640 px, the small end of what the detector finds;
   below 8 m the camera checks less than 3 m around the landing point. The
   tracker's own minimum is 15 m ("Limits"): with the default limits the
   aircraft fires from 15-20 m;
8. the target's estimated ground position within `fire_max_offset_m` (1 m)
   of the bore's ground point: where the bore's line, from the muzzle
   (body FLU (0, 0, -0.3065) m from the hub) along the body's -z axis,
   meets the terrain. The target is geolocated from the camera, not the
   hub: `camera_offset_m`, the OAK-1's lens at (0.137, 0, -0.080) m
   (`hardware/parts/aerodrone/_frame.py`; Gazebo (0, 0, -0.05), Isaac
   (0, 0, -0.08));
9. the launcher at rest (rack forward, trigger safe, measured within
   0.5 s) without a latched fault, no cycle running, darts left, and
   `fire_min_interval_s` (2 s) since the last cycle ended.

Gates 1-8 hold from the command to the release of the sear, and the
target's track must not restart meanwhile (the tracker starting again
elsewhere when the detector gives the target's id to another object of its
kind): one that stops holding aborts the cycle (a person in a detection
frame aborts it at once), and the trigger leaves 0 only while they hold. Safe state, as the turrets':
the trigger at 0 and the rack stopped where it is, on any abort (disarm, a
stop or track command, a pre-emption, a lost target, a person, a fault, the
tracker shutting down) and by the bridge itself on its emergency stop and
500 ms without commands. An abort before the release latches a fault until
the launcher is disarmed and armed again. The rack is drawn back only
inside a fire cycle. A cocked launcher stays cocked when its power is cut.

Status fields added to `/tracking/status`: `"launcher": "none" | "ready" |
"priming" | "chambering" | "firing" | "empty" | "fault"`, `"launcher_armed"`,
`"launcher_ready"` (the bridge), `"darts"` (remaining), `"shots"`,
`"target_class"` (the tracked target's kind, as first seen),
`"fire_block"` (the reason a fire command at the tracked target would be
refused now, `""` when every gate holds), `"fire_checks"` (`{gate: bool}`
for `launcher`, `armed`, `ready`, `tracking`, `target`, `not_person`,
`no_person`, `target_seen`, `hover`, `height`, `on_target`,
`launcher_rest`, `darts`, `interval`), `"last_fire"` (`{"target_id",
"result": "refused" | "running" | "fired" | "fault", "reason", "stamp"}`),
`"agl_m"`, `"bore_offset_m"` (the target from the bore's ground point) and
`"launcher_message"`. Without a launcher (`TRACKING_LAUNCHER_FITTED` false,
the aircraft's default) `launcher` is `"none"` and every fire command is
refused ("no launcher").

`/tracking/shots`, one message per dart released: `{"stamp", "shot",
"target_id", "target_class", "result", "reason", "darts_left",
"at_command": {...}, "at_shot": {...}}`, the last two with the height
above ground, the offset, the aircraft's speed and tilt and the aircraft's,
target's and bore's ground positions when the command was accepted and at
the shot.

The web page shows the launcher's state, Arm and Disarm, the dart count
and Reload, and a Fire button enabled only while `fire_block` is empty and
the status is fresh, with the reason when it is not.

Operating rules (hardware/aerodrone.md, "Foam dart launcher"): foam darts
only, toy power (the 1.5 N/mm spring, every chronograph shot under 30 m/s);
eye protection for everyone nearby; never fly the launcher over people.
**Dropping or firing objects from a drone is regulated**: in the US, FAA
Part 107.23(b) forbids dropping an object in a manner that creates an
undue hazard to persons or property, the operations-over-people rules
(Part 107, subpart D) apply, and Section 363 of the FAA Reauthorization Act
of 2018 (49 U.S.C. 44802 note) forbids operating a drone equipped with a
dangerous weapon. The operator must comply with these and with local rules.
