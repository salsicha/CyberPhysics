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
| `/tracking/command` | `std_msgs/msg/String` (JSON) | UI | `{"action": "track", "target_id": "7", "mode": "centered", "altitude_agl_m": 40}`; `{"action": "track", "target_id": "7", "mode": "slant", "slant_range_m": 60, "depression_deg": 70}`; `{"action": "stop"}`. Optional fields default as below. |
| `/tracking/status` | `std_msgs/msg/String` (JSON), reliable + transient-local, 2 Hz | tracker | `{"state": "idle" \| "acquiring" \| "tracking" \| "lost" \| "error", "mode": ..., "target_id": ..., "target": {"east": .., "north": .., "up": .., "speed_mps": ..}, "slant_range_m": .., "depression_deg": .., "message": ".."}` |
| `/tracking/target` | `geometry_msgs/msg/PointStamped` | tracker | The selected target's estimated position in the navigation frame (ENU about `/navigation/origin`). |

Simulators only:

| Topic | Type | Content |
|---|---|---|
| `/simulation/targets` | `std_msgs/msg/String` (JSON), 10 Hz or more | Ground truth: `{"stamp": <sim s>, "targets": [{"name": "car_1", "class": "car", "east": .., "north": .., "up": .., "yaw": ..}]}`, ENU metres about the same origin as the navigation frame. A hidden target (below) keeps its route position and has `"hidden": true`. |
| `/simulation/targets/hide` | `std_msgs/msg/String` | Comma-separated target names the simulator hides under the terrain, out of the camera's sight (an occlusion); an empty string shows them all. |

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
