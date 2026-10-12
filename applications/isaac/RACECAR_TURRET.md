# RACECAR Turret in Isaac Sim

The RACECAR Turret (`systems/racecar_turret/interfaces.md`) drives with SLAM
Toolbox and Nav2 and keeps a selected object centred in the turret camera
at the same time. This simulation runs the whole stack against NVIDIA Isaac
Sim 6.0.1 and scores both tasks together: a Nav2 waypoint course through an
arena with obstacles while people walk, the turret locked on one of them.

## Running it

```bash
# The stack, idle (select a target with /turret/command, send Nav2 goals):
docker compose -f compositions/racecar_turret_isaac.yaml up

# The scored evaluation: one run with base-rotation compensation and target
# lead (the tracker as configured), one with both off, then the scorer.
tools/racecar_turret_isaac_test.sh
tools/racecar_turret_isaac_test.sh compensated        # one run only
```

Images: `isaac`, `racecarneo`, `nvblox`, `depthanything`, `pantiltros`
(built locally; `make -C applications build_<name>`). The YOLOE checkpoint
is read from `cache/turret/models` as in `compositions/turret_sim.yaml`. The
first run downloads the people's Gazebo Fuel models (CC0, pinned versions and
SHA-256 in `applications/aerostack2/missions/tracking_targets.json`).

`tools/racecar_turret_isaac_test.sh` brings up a fresh stack per run (compose
project `rt_isaac`, `RACECAR_TURRET_TEST_PROJECT`), waits for the evaluator,
saves `recording.jsonl.gz`, `stack.log.gz`, `resources.log` (GPU memory and
load, host memory, per-container CPU and memory every 20 s), `score.json`,
nvblox's map slice and Nav2's global costmap every 5 s (`maps/`) and their
obstacle coverage (`map_coverage.json`, "Depth for nvblox")
under `generated/racecar_turret_isaac/<run>/`, tears the stack down with its
volumes (also on failure or Ctrl-C), and writes `summary.json` with both runs
and their comparison. It exits 0 when every threshold of
`systems/racecar_turret/validation/acceptance_thresholds.json` is met, 1 when
one is missed, 2 when a run is unusable.

## What is simulated

| Part | Model | Where |
|---|---|---|
| Vehicle | `racecar_turret.urdf.xacro` with `sim_vehicle:=true`: 1/10 Traxxas Slash 4x4-class chassis, wheelbase 0.33 m, track 0.30 m, tyre radius 0.055 m, 3.5 kg with the turret; one PhysX articulation, four velocity-driven wheels (tyre friction 1.0/0.9, sphere colliders), two position-driven steering knuckles in Ackermann geometry | `racecar_turret_isaac.py` |
| ESC and steering servo | `/ackermann_cmd` (SI) clamped to 0.42 rad; speed through a 0.12 s lag within 2 m/s² acceleration and 3 m/s² braking; steering through a 0.09 s lag within 6.5 rad/s (20 kg-cm servo); 0.5 s command watchdog (simulation time) | `racecar_turret_sim.DriveTrain` |
| Wheel odometry | rear wheel spin × nominal tyre radius in 2 mm encoder ticks with 2.5 % slip noise; the true tyre is 2 % larger than nominal and the steering trim is off by 0.004 rad, so `/odom` drifts as the car's does; yaw rate from the measured steering angle (vesc_to_odom); `/odom` 60 Hz, odom → base_link through `racecar_turret_odom_tf.py` | `WheelOdometry` |
| IMU | `racecarneo/imu_link`, 120 Hz (every physics step): chassis angular rate and specific force (filtered finite differences + gravity), bias and noise of `sensor_models.yaml` | `ImuModel` |
| 2D lidar | `racecarneo/lidar_link`: 720 rays (0.5°) cast with PhysX scene queries every 100 ms, 0.15–12 m, 15 mm noise, 0.5 % dropout, no return = inf; published directly on `/scan` (the turret plate's posts are not modelled, so no `/scan_raw` and no laser_filters box filter as on the car) | `Lidar`, `scan_ranges` |
| Front RGB-D | `racecarneo/camera_*_optical_frame`, 640×480, fx = fy = 617.1 px (`systems/racecarneo/calibration/camera.yaml`), bgr8 colour and 32FC1 depth (15 mm noise, 1 % dropout, NaN beyond 10 m), 15 Hz | `CameraRig` |
| Turret | pan/tilt joints driven by two ST3215 models: goal speed 3.07 rad/s and acceleration 7.67 rad/s² of `turret_config.h`, 30 ms command latency, 4096-step encoder, `/turret/joint_states` at 25 Hz, each published 10 ms after the reading and stamped with the reading's time (`turret_arduino_bridge.py` stamps on arrival), 0.5 s hold watchdog, hold under `/turret/emergency_stop`; `/turret/actuator_ready` latched true | `TurretActuator` |
| Turret camera | `turret/camera_optical_frame`, 640×480, 70° horizontal field of view (fx = fy = 457 px, the nominal `systems/racecar_turret/calibration/turret_camera.yaml`), rgb8, 15 Hz | `CameraRig` |
| Truth | `/simulation/truth` at 15 Hz in the odom frame (the spawn pose): the car, the turret camera's position, forward and up, every person's floor position, centroid, `visible` (clear line of sight from the turret camera to at least half of five points up the body; obstacles and the other people occlude) and `in_view` (the centroid projects into the image), and the count of collisions (car footprint 0.50 × 0.34 m against obstacles and people) | `truth_message` |

Simulation time: the physics steps exactly once per loop at 120 Hz
(`World.step(render=False)`) and `/clock` follows it; the stage is rendered
once every eighth step (15 Hz, `World.render()`, which does not step
physics). The annotators hand over each frame one render later, so the
images reach ROS 67 ms after the time they show, about a USB camera's
latency, stamped with that time (`IsaacReadSimulationTime`). Every ROS node
runs on `/clock`; the simulator never runs faster than real time
(`ISAAC_MAX_REAL_TIME_FACTOR`). 120 Hz and 240 Hz physics gave the same
straight-line speed (0.300 m/s for a 0.3 m/s command) and full-lock turning
circle (0.758 m radius at base_link, 0.757 m from the Ackermann geometry);
each physics step costs about 3.6 ms of Kit overhead whatever the rate.

## The stack

`compositions/racecar_turret_isaac.yaml`: the simulator; `robot_state_publisher`
on `racecar_turret.urdf.xacro` (simulator optical frames, turret joints from
`/turret/joint_states`); `cmd_vel_to_ackermann` with the hardware car's
`cmd_vel_to_ackermann.yaml` (0.33 m, 0.42 rad, 0.35 m/s); SLAM Toolbox, Nav2
(SmacPlannerHybrid DUBIN, RPP), Depth Anything (vits, vkitti metric) and
nvblox with `systems/racecarneo/config` unchanged except `use_sim_time`,
nvblox integrating the front camera's depth rather than Depth Anything's
("Depth for nvblox"); the
PanTiltROS tracker with `systems/racecar_turret/config/turret_tracker_isaac.yaml`;
and, in the `evaluation` profile, `tools/racecar_turret_evaluator.py`.

## Scenario

`systems/racecar_turret/scenarios/isaac_arena.json`: a 20 m × 15 m walled
arena (1.2 m walls). The car starts at the origin facing east and laps the
course of `systems/racecar_turret/missions/isaac_course.json` (10 poses,
about 37 m) counter-clockwise around a central island (a 4 m wall segment,
two crates, a pillar and a barrel). Between the course and the outer loop
stand a pillar on the south side, a 1.4 m wall segment on the east, a crate
on the north and a pillar on the west.

- `person_1`, the target (Gazebo Fuel "Walking person"), walks the outer loop
  in the car's direction at 0.33 m/s, starting 8 m ahead-right of it (about
  12 m once the car sets off): the obstacles between the loops hide them for
  a few seconds at a time, and the car's corners swing the turret.
- `person_2` ("Casual female") walks back and forth across the arena at
  0.8 m/s west of the island, across the course twice per lap.
- `person_3` ("Casual female") strolls a small loop on the island, a
  distractor of the same class.

People give way to the car as pedestrians do (`racecar_turret_sim.Walkers`):
a person's clock along their scripted route stops while their next step
would take them within 0.25 m of the car's footprint and closer to it, so
nobody walks into the car; the car stops for people in its way itself
(lidar, Nav2). Without it, `person_2` walked into the side of the turning
car in a development run (one collision, then Nav2 aborted: its start cell
was occupied).

The people's models are the aerial tracking work's: baked once by
`applications/demnav/scripts/target_models.py` and moved along routes resolved
by `applications/demnav/scripts/tracking_targets.py` (both imported by
`racecar_turret_assets.py`), visual meshes with a kinematic capsule collider
the lidar sees.

The evaluator waits for the stack, selects `person_1` as the UI does (the
`/turret/yolo/segments` box containing the target's centroid projected into
the turret camera, `{"action": "lock", "target_id": ...}` on `/turret/command`,
repeated until `/turret/status` reports it), drives the course with
`NavigateThroughPoses` and records the run. SLAM's map covers only what the
lidar has seen (12 m), and Nav2 rejects a goal outside it ("outside bounds"),
so each goal holds the course's next poses inside the current map and is
replaced by a longer one (Nav2 pre-empts the running goal) as the map grows.
The lap ends at the south-west turn, short of the start: Nav2's goal checker
compares the car with the last pose only, so a last pose on the way out
ended the course after 12 s.

The evaluator also plays the operator watching the turret image: when the
turret has been more than 10° off the target for 3 s while the target is in
sight and within the turret's reach, because the tracker lost it or follows
another person, it steers the turret toward the target with the UI's
joystick command (`{"action": "manual", ...}` at 10 Hz) until the target is
in the image, and selects it there again the same way (a `reselected`
event). Without this, one identity switch decided the rest of a run: in a
development run `person_2`, crossing between the car and the target, took
over the track 26 s into the course and the turret followed them to the
end. The number of re-selections is a metric of its own.

## Metrics

`tools/racecar_turret_scorer.py`, over the course (goal sent to result):

- driving: completed (Nav2 succeeded), time, collisions, distance and mean
  speed of the true path, SLAM pose error against the truth;
- aiming: the true angle between the turret camera's optical axis and its
  line of sight to the target's centroid, mean and p95, and the fraction
  under 5°, over the samples where the target can be aimed at (visible and
  within the pan and tilt range: "trackable") and over all samples; the
  fraction of trackable samples `/turret/status` reports `locked` while the
  turret is not aimed at another person; every loss of the target (occluded
  or out of reach for 0.5 s or more) and the time after it to aim within 5°
  again (reacquisition) and to report `locked` (relock); the number of
  operator re-selections;
- following: the same aim statistics over the trackable samples on which
  the tracker follows the target, that is, its selected detection in
  `/turret/yolo/segments` contains the target's centroid projected into
  that frame. They measure the aiming loop (latency, base-rotation
  compensation, lead) apart from the tracker's identity switches, which
  the aiming statistics above include.

## Results

Two evaluations (`tools/racecar_turret_isaac_test.sh`, each a run with
base-rotation compensation and target lead and a run without), 2026-10-11,
the tracker of commit b21cfec, an RTX 4000 Ada Laptop (12 GB) with 20 cores
and 30 GB of RAM, the GPU to itself. Per configuration: first evaluation /
second.

| | Compensation and lead on | Off |
|---|---|---|
| Course | completed, 148.5 / 148.3 s | completed, 149.1 / 148.7 s |
| Collisions | 0 / 0 | 0 / 0 |
| Mean speed | 0.247 / 0.246 m/s | 0.246 / 0.247 m/s |
| SLAM pose error, max | 0.13 / 0.11 m | 0.16 / 0.13 m |
| Target trackable (in sight and reach) | 89 / 88 % | 89 / 89 % |
| Aim error, mean (trackable) | 2.7 / 3.3° | 2.5 / 2.9° |
| Aim error, p95 (trackable) | 17.7 / 21.4° | 9.8 / 17.1° |
| On target (< 5°) | 89 / 88 % | 94 / 91 % |
| Locked | 87 / 86 % | 93 / 90 % |
| Aimed at another person | 7.5 / 5.7 % | 2.6 / 2.5 % |
| Longest reacquisition after an occlusion | 3.6 / 3.5 s | 2.5 / 3.5 s |
| Operator re-selections | 3 / 4 | 1 / 3 |
| Following the target | 90 / 89 % | 94 / 91 % |
| Aim error while following, mean | **1.0 / 1.0°** | **1.3 / 1.3°** |
| Aim error while following, p95 | **1.4 / 1.5°** | **3.8 / 3.5°** |
| On target while following | 98.7 / 98.9 % | 99.4 / 99.4 % |

```json
{
  "compensated": [
    {"completed": true, "time_s": 148.47, "collisions": 0, "mean_speed_mps": 0.247, "slam_error_max_m": 0.132, "trackable_fraction": 0.891, "mean_error_deg": 2.69, "p95_error_deg": 17.67, "on_target_fraction": 0.893, "locked_fraction": 0.871, "aimed_at_other_fraction": 0.075, "max_reacquire_s": 3.6, "unrecovered_losses": 1, "reselections": 3, "following_fraction": 0.904, "following_mean_error_deg": 0.99, "following_p95_error_deg": 1.41, "following_on_target_fraction": 0.987},
    {"completed": true, "time_s": 148.34, "collisions": 0, "mean_speed_mps": 0.246, "slam_error_max_m": 0.107, "trackable_fraction": 0.883, "mean_error_deg": 3.33, "p95_error_deg": 21.37, "on_target_fraction": 0.879, "locked_fraction": 0.864, "aimed_at_other_fraction": 0.057, "max_reacquire_s": 3.53, "unrecovered_losses": 0, "reselections": 4, "following_fraction": 0.889, "following_mean_error_deg": 0.97, "following_p95_error_deg": 1.54, "following_on_target_fraction": 0.989}
  ],
  "uncompensated": [
    {"completed": true, "time_s": 149.07, "collisions": 0, "mean_speed_mps": 0.246, "slam_error_max_m": 0.156, "trackable_fraction": 0.886, "mean_error_deg": 2.49, "p95_error_deg": 9.79, "on_target_fraction": 0.939, "locked_fraction": 0.926, "aimed_at_other_fraction": 0.026, "max_reacquire_s": 2.53, "unrecovered_losses": 0, "reselections": 1, "following_fraction": 0.944, "following_mean_error_deg": 1.3, "following_p95_error_deg": 3.77, "following_on_target_fraction": 0.994},
    {"completed": true, "time_s": 148.73, "collisions": 0, "mean_speed_mps": 0.247, "slam_error_max_m": 0.131, "trackable_fraction": 0.886, "mean_error_deg": 2.9, "p95_error_deg": 17.11, "on_target_fraction": 0.909, "locked_fraction": 0.895, "aimed_at_other_fraction": 0.025, "max_reacquire_s": 3.53, "unrecovered_losses": 0, "reselections": 3, "following_fraction": 0.912, "following_mean_error_deg": 1.26, "following_p95_error_deg": 3.48, "following_on_target_fraction": 0.994}
  ]
}
```

While the tracker follows the target, base-rotation compensation and target
lead cut the 95th percentile of the aim error from 3.5–3.8° to 1.4–1.5°,
and the mean from 1.3° to 1.0°: without them the aim lags whenever the car
turns (up to 0.33 rad/s at the corners). The overall aim statistics are
decided by identity instead: the tracker's track jumps to another person
who passes close to the target in the image (`person_2` crossing,
`person_3` on the island), and the operator had to take over 1–4 times per
lap. In both evaluations the run with compensation and lead was on another
person more often (5.7–7.5 % of the time, against 2.5–2.6 %), which is
where its worse overall p95 comes from; two evaluations are too few to say
whether lead makes such switches likelier, but it is worth a look in the
tracker's association. Occlusions by the obstacles mostly cost nothing:
the turret held its aim through them and was on the target again as soon
as it reappeared (no reacquisition time in most losses; at most 3.6 s).

Driving is the same with and without: every run completed the lap in
148–149 s without collisions, SLAM within 0.16 m of the truth.

Resources, all runs alike: 4.1 GB of GPU memory at the peak (Isaac Sim
2.0 GB, nvblox 1.0 GB, YOLOE 0.6 GB, Depth Anything 0.44 GB), the GPU about
half busy; 6.9 GB of memory in the containers (host total 6.2 → 14.3 GB);
about 5.4 cores (Isaac 2.3, Nav2 0.7, Depth Anything 0.6, the tracker 0.4).
The whole stack fits, so nothing was cut. Simulation ran at 0.51–0.59 of
real time; a run takes about 5.5 minutes of wall clock, an evaluation 12.

`systems/racecar_turret/validation/acceptance_thresholds.json` holds limits
set from these runs with margin. The `following_*` limits (p95 under 2.5°)
are what tell the aiming loop apart: the runs without compensation and lead
fail them.

## Depth for nvblox

Until 2026-10-11 nvblox integrated Depth Anything's depth (vits, vkitti
metric, max_depth 80), as on the car. Scored against the rendered depth on
150 frames of a lap (`depth_accuracy record` alongside the evaluation,
`applications/depthanything/README.md`, "Accuracy"), over 0.5-5 m:

| Depth | Estimate / true, median | Abs. error median / p95 | Rel. error median / p95 | Floor lifted into nvblox's slice | Obstacle pixels in place | GPU per frame |
|---|---|---|---|---|---|---|
| a) vkitti, max_depth 80 (the setting) | 9.07 | 9.03 / 21.2 m | 807 / 909 % | 0 % | 0.04 % | 48 ms |
| b) hypersim, max_depth 20 | 1.31 | 0.39 / 1.15 m | 31 / 81 % | 2 % | 3 % | 47 ms |
| c) vkitti, aligned per frame to the camera's depth | 0.99 | 0.06 / 1.85 m | 7 / 48 % | 25 % | 3 % | 48 ms |
| c) hypersim, aligned per frame | 0.99 | 0.05 / 1.29 m | 6 / 30 % | 18 % | 15 % | 47 ms |
| d) the camera's own depth | 1 | 15 mm noise (the sensor model) | | 0 % | 100 % | none |

The last two columns: nvblox takes points 0.02-0.55 m above the floor for
obstacles (`systems/racecarneo/config/nvblox.yaml`), so from 0.22 m a floor
point's depth must be within about 9 % to stay out; "in place" is an
obstacle pixel kept in that band within 0.15 m. Scale alignment fixes the
median but not the shape (obstacles 3.5-5 m away at 0.63-0.75 of their
distance), so the composition feeds nvblox the camera's depth
(`/camera/depth/image_rect_raw`). Depth Anything still runs and publishes
`/depth_anything/*` as on the car; `RACECAR_TURRET_NVBLOX_DEPTH=
/depth_anything/depth/image` maps it again. GPU time is `infer_image` on
the RTX 4000 Ada Laptop GPU. 151 frames of a later lap on the final tree
(an evaluation of d below) gave the same within 0.02 of each median ratio
(vkitti 9.05, hypersim 1.32).

### Obstacle maps

`tools/racecar_turret_map_coverage.py record` (the `evaluation` profile's
`racecar_turret_map_recorder`) saves nvblox's map slice, in `map` through
TF at its stamp, and Nav2's global costmap every 5 s of a run; `score`
(run by `tools/racecar_turret_isaac_test.sh` into `map_coverage.json`)
compares them with the scenario: the share of the obstacles' outlines
(every 5 cm; the arena walls' inner faces) with an occupied cell (nvblox:
within half a voxel of a surface; costmap: lethal) within 0.15 m at some
point of the run; for nvblox also the share it observed at all, free or
occupied ("carved free": observed, never occupied); and phantom obstacles,
occupied cells more than 0.3 m from every obstacle and from where the
people walked.

With Depth Anything's depth nvblox observed 85 % of the outlines and
mapped every one of them as free space: the vkitti depth put each surface
nine times too far, past nvblox's 12 m, and carved the space in front of
it. Nav2's global costmap had the obstacles anyway, from SLAM's map and the
lidar (99.8 %).

### People in nvblox's map

With the camera's depth nvblox also maps the people. In its static mode (as
on the car) a walking person left a trail of obstacles until the camera saw
the floor there again or the TSDF decayed (about 15 s with nvblox's default
0.95 every 0.2 s), and nothing clears the 0.75 m in front of `base_link`,
below the camera's view. In the first evaluation (b, below) `person_2`
crossed just in front of the car, stayed in the map there, and Nav2 aborted
the course ("Start occupied"). A faster decay of what the camera no longer
sees (0.8 every 0.2 s, about 3.5 s; c) did not stop it: `person_2`, waiting
for the car as the scenario's people do, stood 0.6 m from it, nvblox mapped
their whole body (the lidar sees a 0.22 m capsule), and after two minutes
of recoveries Nav2 aborted again ("Start occupied").

nvblox's dynamic mode (`systems/racecar_turret/config/nvblox_isaac.yaml`:
NVIDIA's `nvblox_dynamics.yaml` values, the static slice's heights, the
faster decay) puts people who walk into space it has seen free into a
dynamic occupancy layer that decays instead. With Nav2 on the combined
(static and dynamic) slice (d) 5 of 6 runs completed; the sixth stopped
where `person_1`'s and `person_2`'s routes cross the course, behind the
trails the lidar leaves in SLAM's map and the obstacle layer ("exceeded
maximum iterations" planning past them), with no nvblox obstacle within
1 m of the car. The composition now gives Nav2 the static slice (e): the
arena's obstacles from nvblox, the people from the lidar as before.
`RACECAR_TURRET_NVBLOX_SLICE=/nvblox_node/combined_map_slice` gives Nav2
nvblox's people too; `RACECAR_TURRET_NVBLOX_DEPTH=/depth_anything/depth/image`
with `RACECAR_TURRET_NVBLOX_OVERRIDES=/workspace/config/nvblox.yaml` is the
earlier setting.

### Evaluations

`tools/racecar_turret_isaac_test.sh` (a run with base-rotation compensation
and target lead, and one without), 2026-10-11, with the tracker in the tree
at the time (uncommitted changes on top of b21cfec, so the aim figures
differ from the Results above). a) is the earlier setting on the tree before
the turret's dart launcher (one run, alongside the depth recording),
a') the same on the final tree:

| | a) / a') Depth Anything vkitti | b) camera depth, static map | c) b + faster decay | d) camera depth, dynamic mode, combined slice | e) dynamic mode, static slice (the composition) |
|---|---|---|---|---|---|
| Runs | 1 + 2 | 2 | 2 | 6 (3 evaluations) | 4 (2 evaluations) |
| Course completed | 3 of 3, 148.7-150.7 s | 1 of 2 (aborted at 130 s) | 1 of 2 (aborted at 276 s) | 5 of 6, 148.3-151.1 s (one aborted at 111 s) | 4 of 4, 148.2-155.6 s |
| Collisions | 0 | 0 | 0 | 0 | 0 |
| Mean speed (completed runs) | 0.245-0.246 m/s | 0.232 | 0.248 | 0.243-0.246 | 0.236-0.248 |
| nvblox: obstacle outlines mapped | 0 % | 68, 82 % | 80, 93 % | 79-83 % (48 % in the aborted run) | 81-83 % |
| nvblox: observed but carved free | 84-85 % | 1-2 % | 1-4 % | 1-4 % | 0-2 % |
| nvblox: phantoms in the arena | 0 | 0.05, 0.10 m² | 0 | 0-0.01 m² | 0 |
| Costmap: obstacle outlines lethal | 99.7-99.8 % | 100 % | 100 % | 97-100 % (94 % in the aborted run) | 99.9-100 % |
| Aim while following, compensated: mean / p95 | 0.77-1.08 / 1.13-1.44° | 0.82 / 1.33° | 0.76 / 1.11° | 0.99-1.04 / 1.40-1.44° | 0.92-0.95 / 1.42-1.46° |
| Every threshold met | a' (a missed the aim mean and p95) | no | no | 2 of 3 evaluations | 2 of 2 evaluations |

"Outlines mapped" is the share of the obstacles' outlines nvblox had as an
obstacle at some point of the lap (the camera sees about 85 % of them);
nvblox forgets what is more than 12 m away (`map_clearing_radius_m`), so at
the end of a lap it holds 6-8 %. The costmap has the obstacles in every
configuration, from SLAM's map and the lidar; with the camera's depth
nvblox adds the arena's obstacles to it as on the car it would add what
the lidar's plane misses. The aim figures differ between runs by identity
switches (the operator's re-selections), as in the Results, not by the
depth. Resources: nvblox 0.89 GB of GPU memory (1.0 GB with the vkitti
depth, which allocated blocks out to 12 m) and 0.32-0.48 cores in dynamic
mode (0.17-0.19 in static mode); the GPU's peak 4.0-4.2 GB.

The car's own setting (`systems/racecarneo/config/nvblox_hardware.yaml`:
e with both mappers integrating to 5 m, for the RealSense's error), run
here on the camera's depth with `RACECAR_TURRET_NVBLOX_OVERRIDES=
/workspace/config/nvblox_hardware.yaml`, one run (compensated) on
2026-10-12: course completed in 150.3 s, no collision, 0.245 m/s, every
threshold met, no planner failure. nvblox mapped 38 % of the outlines (e: 81-83 %):
the obstacles along the course 78-100 %, the island's pillar and barrel
(in the camera's view from 3.8-4.0 m) 79 and 53 %, the arena walls 40 %,
and the island's wall and crates (in view only from 6.9 m and more, or
not at all) none; 2 % carved free, no phantoms; the costmap 100 %. nvblox took
0.21-0.52 cores, the GPU's peak 4.0 GB. This checks the range, not a real
stereo camera's error.

## Fidelity gaps

- **Simulation runs at about half real time** (real-time factor 0.5 with the
  whole stack; Isaac's Kit loop is CPU-bound at about 3.6 ms per physics
  step, the GPU about half busy). Every node runs on `/clock`, and the
  simulated latencies (camera 67 ms, servo command 30 ms, joint feedback,
  ESC and steering lags) are exact in simulation time, but the computing
  time of the YOLO tracker, Depth Anything, SLAM and Nav2 is wall-clock
  time, so in simulation time it is about halved: the turret loop here
  reacts faster than on the Jetson.
- **Depth for nvblox.** nvblox maps the simulated camera's depth: the
  rendered depth with the RealSense noise model (15 mm, 1 % dropout,
  nothing beyond 10 m), not a real stereo camera's errors and holes. The
  car's hardware composition maps the RealSense's aligned depth the same
  way, in dynamic mode, but integrates it to 5 m rather than 12 m for a
  stereo camera's error (`systems/racecarneo/config/nvblox_hardware.yaml`).
- **Lidar**: an instantaneous scan (no rotation skew), PhysX collision
  shapes rather than visual surfaces, ideal reflectivity, and no turret
  posts (so no `/scan_raw` and no laser_filters stage, which the car has).
- **People** are static walking poses gliding along scripted routes (no
  limb motion); they give way to the car, and the lidar sees them as
  capsules.
- **Rendering**: path-traced RTX lit by a uniform dome; ideal pinhole
  cameras without distortion, motion blur, rolling shutter or auto exposure.
- **Vehicle**: sphere tyres with Coulomb friction (no tyre slip curve);
  each wheel's speed is commanded (ideal differentials) within a 1 N m motor
  torque per wheel; flat floor, no suspension.
- **Sensor errors** (odometry tyre radius and trim, IMU bias and noise,
  depth and lidar noise) follow `systems/racecarneo/config/sensor_models.yaml`
  or are chosen, not identified on the car.
- **Turret**: the servos' motion profile is modelled and the joints follow
  it stiffly: no backlash, compliance or load-dependent speed. The foam
  dart launcher is not simulated (the assets expand the URDF with
  `launcher:=false`; any further joint would be held at zero).
- **Truth**: collisions are counted from the footprint (0.50 × 0.34 m)
  against the scenario's obstacle and people shapes, not from contact
  forces; visibility uses the same shapes.
