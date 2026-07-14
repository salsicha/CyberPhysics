TODO:

Complete simulation validation plan for the hardware configurations in
`hardware/`: `racecarneo`, `so101`, `aerodrone`, and `turret`.

## Cross-platform simulation foundation

1. Define a simulation readiness matrix for every hardware sensor and actuator.
   Include simulated topic name, message type, frame_id, update rate, latency,
   noise model, drop-out model, calibration source, and the matching physical
   hardware item from the relevant `hardware/` platform entry.
2. Add a reusable simulation validation harness that can launch each supported
   composition headlessly, wait for required ROS topics, assert TF connectivity,
   record a short rosbag, and fail on stale topics, NaNs, impossible covariance,
   missing camera_info, or missing actuator feedback.
3. Add deterministic scenario assets with seeded randomness. Every scenario
   should support nominal, degraded-sensor, poor-lighting/visibility, and
   recovery/failsafe variants.
4. Add golden telemetry checks for each scenario:
   - Localization drift and covariance bounds.
   - Perception frame rate, timestamp skew, and calibration consistency.
   - Planner success rate and time-to-goal.
   - Collision, near-miss, workspace-limit, and geofence violations.
   - Command saturation, actuator lag, and emergency-stop behavior.
5. Add CI entry points that run fast smoke tests without a GPU and longer
   nightly validation jobs for GPU simulators, Isaac Sim, Gazebo, Unity, Depth
   Anything, GR00T, nvblox, DemNav, and WildNav.
6. Standardize rosbag and metrics output under a generated data directory that
   is ignored by git. Include scripts to summarize results as markdown tables.
7. Add Foxglove layouts and topic documentation for each scenario so failures
   can be inspected without rebuilding the launch graph.

## RACECAR Neo realistic navigation simulation

Goal: validate the full 1/10-scale RACECAR Neo stack in a realistic driving
environment with sensor-driven waypoint navigation.

1. Build or integrate a realistic closed-course environment.
   - Include road lanes, cones/barriers, curbs, grass, parked vehicles, signs,
     buildings, trees, shadows, uneven pavement, and narrow passages.
   - Provide at least one indoor lab variant and one outdoor campus/parking-lot
     variant.
   - Add ground-truth map, semantic labels, collision geometry, and spawn poses.
2. Add a waypoint mission set for Nav2.
   - Include simple loop, slalom, lane-change, obstacle-avoidance, reverse or
     recovery, and long multi-lap missions.
   - Store waypoints with frame, pose, tolerance, speed limit, expected route
     length, and success criteria.
   - Drive missions through `NavigateToPose` or `NavigateThroughPoses`, not by
     teleporting or using ground-truth control.
3. Simulate all hardware sensors and control interfaces from
   `hardware/racecarneo.txt`.
   - Forward RGB/RGB-D camera on `/camera/color/image_raw`,
     `/camera/color/camera_info`, optional depth fallback, and realistic lens
     distortion, exposure, motion blur, rolling/global shutter selection,
     low-light behavior, glare, and vibration.
   - Depth Anything output on `/depth_anything/depth/image` and
     `/depth_anything/points` with validation against simulator ground-truth
     depth.
   - 2D lidar on `/scan` with range noise, angular resolution, missing returns,
     reflective/absorbing surfaces, and occasional packet drops.
   - IMU on `/imu/data_raw` with bias, noise density, gravity alignment errors,
     vibration, and timestamp skew.
   - Wheel odometry or VESC telemetry with slip, encoder quantization, steering
     servo lag, speed saturation, and battery-voltage effects.
   - Ackermann command output on `/ackermann_cmd`, plus actuator feedback,
     manual override state, and emergency-stop state.
4. Finish and harden the existing stack integration.
   - Keep `compositions/racecarneo.yaml` as the full-path launch target:
     Unity simulator -> bridge -> Depth Anything -> nvblox -> SLAM Toolbox ->
     Nav2 -> `cmd_vel_to_ackermann` -> simulator.
   - Complete the current RealSense, nvblox, and Nav2 working demo with the same
     topics and frames used by the hardware.
   - Add explicit checks for `map -> odom -> base_link -> camera/lidar/imu`
     transforms and for nvblox map slices feeding the Nav2 costmap.
5. Add sensor-driven behavior tests.
   - Block direct ground-truth pose use by the planner except for scoring.
   - Verify the car can localize, build/consume obstacle maps, avoid dynamic
     obstacles, and recover after temporary camera, lidar, IMU, or odom loss.
   - Validate that degraded camera depth cannot produce unsafe throttle through
     stale nvblox/costmap data.
6. Add Racecar Neo acceptance metrics.
   - At least 95 percent waypoint completion across seeded nominal runs.
   - Zero collisions in nominal runs and bounded low-speed contact behavior in
     recovery tests.
   - Lateral tracking error, stop-line error, localization drift, and costmap
     freshness thresholds recorded for every run.

## SO-101 realistic GR00T picking simulation

Goal: validate the SO-101 arm, depth camera, calibration, and GR00T policy path
in a realistic tabletop picking scenario.

1. Build realistic manipulation scenes for Isaac Sim and the existing Gazebo or
   Genesis adapters where useful.
   - Include a rigid table, arm mount, bins/trays, clutter, target objects,
     distractors, occluders, varied materials, labels/AprilTags for
     calibration, and realistic lighting.
   - Add object sets with simple primitives, common household parts, deformable
     or reflective edge cases where the simulator supports them, and known
     ground-truth poses.
2. Simulate all hardware sensors and interfaces from `hardware/so101.txt`.
   - Joint states on `/joint_states` with servo limits, quantization, backlash,
     latency, thermal/current saturation, and command clipping.
   - Joint command interface on `/so101/joint_commands` with trajectory and
     hold-position behavior matching the hardware adapter.
   - RealSense-style RGB stream on `/so101/camera/image_raw` and
     `/so101/camera/camera_info`.
   - Depth stream and camera_info for the selected D435i/D405-equivalent setup,
     including close-range invalid depth, stereo holes, multipath/noise,
     exposure changes, and optional IMU data when used by calibration or policy.
   - Static and hand-eye extrinsics for table, base, camera, and gripper frames.
3. Upgrade the GR00T demo path.
   - Replace the current synthetic-image publisher in the Isaac adapter with a
     rendered camera product using `groot_camera_rgb_optical_frame`.
   - Extend `compositions/so101_groot_isaac.yaml` so GR00T receives rendered
     RGB, optional depth, joint history, task language, and calibration metadata.
   - Add a scenario/task loader for prompts such as "pick the red block and
     place it in the left bin" with target object IDs and scoring rules.
4. Add picking task validation.
   - Score reachability, grasp approach, grasp closure, lift stability,
     collision-free motion, place accuracy, and recovery after failed grasp.
   - Verify GR00T is not using simulator ground truth except through rendered
     observations and joint feedback.
   - Include domain-randomized validation for camera pose perturbation, object
     pose, clutter density, friction, mass, lighting, and depth noise.
5. Add calibration and data checks.
   - Generate camera intrinsics and extrinsics from the simulator and compare
     them with the ROS camera_info and TF tree.
   - Add automated tests for base-to-camera, base-to-table, end-effector, and
     gripper collision geometry alignment.
   - Record demonstration bags suitable for GR00T regression testing and verify
     that image, depth, joint, command, and task streams are synchronized.
6. Add SO-101 acceptance metrics.
   - Pick-and-place success rate by object class and clutter level.
   - Mean and worst-case place error.
   - Collision count, joint-limit violations, command saturation, and failed
     grasp recovery rate.
   - Policy latency and observation age at every command.

## Aerodrone realistic wilderness navigation simulation

Goal: validate the quadrotor navigation stack in a wilderness scenario using
GPS/odom, downward OAK-1 RGB, OAK-1 relative depth, rangefinder, DEM data, and
satellite maps.

1. Build a georeferenced wilderness scenario.
   - Use a real or realistic mountainous/forested test area with lat/lon origin,
     UTM/local frame conversion, terrain elevation, trees, clearings, trails,
     rocks, water, shadows, and no-fly/geofence regions.
   - Seed matching DEM and satellite tile caches for the same area, following
     the generated-asset policy from `applications/demnav` and
     `applications/wildnav`.
   - Include day, dusk, low-texture, canopy-shadow, wind, and GPS-degraded
     variants.
2. Simulate all hardware sensors and interfaces from `hardware/aerodrone.txt`.
   - GPS fix and covariance on the Aerostack topic used by
     `systems/aerostack2_gazebo/config/nav_topics.env`, with multipath,
     dropouts, and degraded-accuracy modes.
   - Odometry on `/drone0/sensor_measurements/odom` or the simulation namespace
     equivalent, including velocity, acceleration, covariance, and time sync.
   - Flight-controller IMU, compass, and barometer noise models, with vibration
     and bias drift.
   - Downward OAK-1 high-resolution RGB on `/oak1/image_highres` or
     `/drone_sim_0/sensor_measurements/downward_rgb/image_raw`, with matching
     camera_info and realistic nadir camera intrinsics/extrinsics.
   - OAK-1 MegaDepth relative depth on `/oak1/relative_depth` or
     `/drone_sim_0/sensor_measurements/downward_rgbd/depth`, including scale
     ambiguity, invalid pixels, motion blur, terrain texture failures, and
     camera footprint limits.
   - Downward rangefinder altitude with saturation, minimum range, terrain slope
     effects, and dropouts.
   - Battery, RC failsafe, geofence, and emergency/land behavior signals.
3. Integrate DemNav and WildNav into the simulation.
   - Run the `navsim` profile in `compositions/aerostack2_sim.yaml` against the
     wilderness asset set.
   - Verify DemNav consumes relative depth, camera_info, GPS, and odometry, then
     publishes `/demnav/fix`, `/demnav/odometry`, `/demnav/metric_depth`,
     `/demnav/points`, confidence, scale, and valid flags.
   - Verify WildNav consumes downward RGB, camera_info, prediction odometry, raw
     odometry, GPS, and DemNav corrections, then publishes `/wildnav/*` and the
     fused `/navigation/odometry`, `/navigation/fix`,
     `/navigation/correction_source`, and `/navigation/correction_confidence`.
   - Confirm satellite imagery licensing and cache behavior for offline tests.
4. Add wilderness mission plans.
   - Include takeoff, climb, ridge crossing, valley following, waypoint loiter,
     return-to-home, and emergency landing missions.
   - Use DEM-derived altitude constraints and satellite-map visual matching to
     correct drift during GPS degradation.
   - Score route completion, geofence compliance, terrain clearance, localization
     correction quality, and safe landing behavior.
5. Add failure and robustness tests.
   - GPS degraded or denied segments where DemNav/WildNav must bound drift.
   - Low-texture terrain where WildNav confidence should drop cleanly.
   - Flat terrain where DemNav should reject weak DEM correlation.
   - Canopy, shadows, motion blur, wind gusts, rangefinder dropouts, and stale
     map cache tests.
   - Ensure navigation falls back to ArduPilot/Aerostack estimates when DemNav or
     WildNav validity is false.
6. Add Aerodrone acceptance metrics.
   - Horizontal and vertical localization error against simulator ground truth.
   - DemNav depth-scale error, terrain-correlation confidence, and rejection
     correctness.
   - WildNav match count, inlier ratio, correction error, and false-positive
     rate.
   - Terrain clearance, waypoint success, battery reserve, geofence violations,
     and emergency landing success.

## Turret realistic object-tracking simulation

Implementation status: CPU smoke simulation and validation are implemented in
`systems/turret`, with a compose entry at `compositions/turret_sim.yaml`. The
GPU/nightly path should replace the deterministic smoke segmentation backend
with the latest installed Ultralytics YOLO26 segmentation checkpoint and record
the exact checkpoint in run metadata.

Goal: validate a PanTiltROS-based pan/tilt turret that segments a realistic
scene, selects a configured target object, and tracks that object with closed
loop pan/tilt control while distractors and occluders move through the camera
view.

1. Define the turret hardware and ROS contract.
   - Use `applications/pantiltros` as the starting control stack and preserve
     its `pan_joint` and `tilt_joint` names, ST3215 servo assumptions, joystick
     emergency-stop behavior, and measured joint-state/diagnostics feedback.
   - Track the hardware definition in `hardware/turret.md`, including
     camera, servo bus, power, emergency stop, and ROS topic expectations.
   - Standardize simulated topics for `/turret/camera/image_raw`,
     `/turret/camera/camera_info`, segmentation outputs, target lock state,
     angular error, joint commands, joint states, diagnostics, and emergency
     stop.
2. Build a realistic moving-object scene.
   - Include an indoor lab or warehouse aisle with textured walls, shelves,
     benches, windows, shadows, reflective surfaces, partial occluders, and
     background clutter.
   - Add multiple moving object classes such as person, cart, box, drone,
     vehicle, and distractor objects with seeded trajectories, velocities,
     stops, crossings, and occlusions.
   - Provide ground-truth object IDs, class labels, segmentation masks,
     3D poses, target visibility state, and the camera-to-target angular error
     used only for scoring.
3. Integrate YOLO segmentation.
   - Use the current Ultralytics YOLO instance-segmentation model at
     implementation time. As of June 2026, public Ultralytics YOLO26 material
     describes instance segmentation support; use the latest available YOLO26
     `-seg` checkpoint in the installed Ultralytics release for GPU/nightly
     validation, and the smallest available YOLO26 segmentation checkpoint for
     CPU smoke tests. Re-verify the exact checkpoint name before
     implementation.
   - Publish instance masks, class IDs, confidences, bounding boxes, mask
     centroids, and per-frame inference latency.
   - Validate segmentation against simulator masks for target IoU, false
     positives, missed detections, confidence calibration, and mask latency
     under motion blur, low light, glare, and partial occlusion.
4. Add target selection and tracking behavior.
   - Load a scenario target contract that identifies the object to track by
     class plus stable simulator object ID, initial mask, or configured visual
     prompt.
   - Maintain target identity through brief occlusions and class-level
     distractors using mask overlap, appearance embedding or color histogram,
     predicted image motion, and joint-angle history.
   - Convert target pixel centroid error into pan/tilt angular commands using
     camera intrinsics, joint limits, velocity limits, deadband, and PID or
     model-predictive smoothing.
   - Drive the turret through PanTiltROS-compatible `JointState` commands; do
     not use simulator ground truth for tracking control.
5. Simulate actuator and camera failure modes.
   - Model ST3215 position quantization, speed/acceleration limits, backlash,
     overshoot, latency, dropped commands, bus disconnects, current/temperature
     saturation, and emergency-stop torque disable.
   - Model camera exposure changes, rolling-shutter artifacts, lens distortion,
     frame drops, timestamp skew, compression artifacts, and vibration from
     fast pan/tilt motion.
   - Verify safe behavior when the target is lost, the segmentation model is
     stale, the commanded joint would exceed limits, or emergency stop is
     asserted.
6. Add Turret acceptance metrics.
   - Target lock percentage, reacquisition time after occlusion, and ID-switch
     count.
   - Mean and 95th-percentile pixel error and angular error while visible.
   - Segmentation mask IoU, false-positive rate, false-negative rate, and
     inference latency.
   - Joint command saturation, overshoot, settling time, diagnostic fault
     count, emergency-stop response time, and zero use of simulator ground truth
     by the tracker.

## Deliverables

1. Scenario assets and launch files for all four platforms.
2. Sensor model configuration files matching every item in the `hardware/`
   platform entries.
3. Mission/waypoint/task definitions and scoring scripts.
4. Headless smoke tests and GPU/nightly robustness tests.
5. Documentation for running each simulation, inspecting results, and mapping
   simulated topics to physical hardware topics.
