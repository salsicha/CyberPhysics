# Aerostack2 Gazebo System

This system describes a Gazebo-starting simulation setup for an Aerostack2
quadrotor with GPS, a downward RGB camera, and a downward RGB-D camera. The
payload is intended to provide the ROS topics needed by DemNav and WildNav.

Files:

- `config/world_gazebo.yaml`: AS2 Gazebo world, drone, and payload selection.
- `config/config_gazebo.yaml`: shared ROS parameters for Aerostack2 nodes.
- `config/pid_speed_controller.yaml`: PID speed-controller parameters.
- `config/nav_topics.env`: expected simulated topic names and Mount Tamalpais test origin for DemNav/WildNav.
- `config/sensor_models.yaml`: hardware-to-simulation GPS, OAK-1, rangefinder, battery, geofence, RC failsafe, and emergency-land topic contract.
- `scenarios/mount_tamalpais_wilderness.json`: georeferenced wilderness scenario contract for simultaneous GPS, DemNav, WildNav, and fused-navigation evaluation.
- `missions/wilderness_missions.json`: takeoff, climb, ridge crossing,
  valley following, loiter, return-home, and emergency-landing mission suite.
- `scripts/score_wilderness_mission.py`: validates the mission contract and
  scores extracted telemetry JSON against the mission criteria.
- `validation/navigation_robustness_tests.json`: GPS, map, camera, weather,
  rangefinder, cache, and fallback failure cases.
- `scripts/score_navigation_robustness.py`: validates robustness contracts and
  scores extracted telemetry JSON against the failure-case criteria.
- `validation/acceptance_thresholds.json`: final acceptance thresholds for
  localization, DemNav, WildNav, missions, safety, and robustness.
- `scripts/aerodrone_acceptance_report.py`: validates the acceptance threshold
  contract and evaluates metrics JSON into JSON or Markdown reports.

Start the base Aerostack2 simulation headlessly:

```bash
docker compose -f compositions/aerostack2_sim.yaml up
```

The Gazebo services run without X11 and publish ROS 2 topics for GPS, odometry,
downward RGB, downward depth, and camera calibration. Expected topic names are
listed in `config/nav_topics.env`; the default sensor topics are:

- `/drone_sim_0/sensor_measurements/gps`
- `/drone_sim_0/sensor_measurements/odom`
- `/drone_sim_0/sensor_measurements/downward_rgb/image_raw`
- `/drone_sim_0/sensor_measurements/downward_rgb/camera_info`
- `/drone_sim_0/sensor_measurements/downward_rgbd/depth`
- `/drone_sim_0/sensor_measurements/downward_rgbd/depth/camera_info`
- `/navigation/gps_standard`
- `/aerodrone/imu/data_raw`
- `/aerodrone/compass/magnetic_field`
- `/aerodrone/barometer`
- `/oak1/image_highres`
- `/oak1/image_highres/camera_info`
- `/oak1/relative_depth`
- `/oak1/camera_info`
- `/aerodrone/rangefinder`
- `/aerodrone/battery_voltage`
- `/aerodrone/battery`
- `/aerodrone/rc_failsafe`
- `/aerodrone/geofence_violation`
- `/aerodrone/emergency_land`

Start it with DemNav and WildNav consumers enabled:

```bash
docker compose --profile navsim -f compositions/aerostack2_sim.yaml up
```

The `navsim` profile seeds named Docker volumes for `/data/demnav_cache` and
`/data/wildnav_cache`, starts the simulated Aerodrone hardware topic bridge,
then runs DemNav and WildNav against `/navigation/gps_standard`, `/oak1/*`, and
the raw Aerostack odometry topic. To run the topic-level validator as well:

```bash
docker compose --profile navsim --profile validation -f compositions/aerostack2_sim.yaml up aerodrone_nav_validator
```

If the AS2 Gazebo assets publish camera topics under different names on your
installed package version, override the topic variables from `nav_topics.env`
or edit the compose environment.

Validate the tracked mission contract without telemetry:

```bash
python3 systems/aerostack2_gazebo/scripts/score_wilderness_mission.py
```

Score a mission after extracting telemetry samples from a bag or simulator log:

```bash
python3 systems/aerostack2_gazebo/scripts/score_wilderness_mission.py \
  --mission-id ridge_crossing_gps_degraded \
  --telemetry generated/aerodrone/ridge_crossing_telemetry.json
```

Validate the robustness test contract:

```bash
python3 systems/aerostack2_gazebo/scripts/score_navigation_robustness.py
```

Score a robustness run after extracting telemetry samples:

```bash
python3 systems/aerostack2_gazebo/scripts/score_navigation_robustness.py \
  --test-id gps_degraded_ridge_crossing \
  --telemetry generated/aerodrone/robustness_telemetry.json
```

Validate the final acceptance threshold contract:

```bash
python3 systems/aerostack2_gazebo/scripts/aerodrone_acceptance_report.py
```

Evaluate aggregated metrics and write reports:

```bash
python3 systems/aerostack2_gazebo/scripts/aerodrone_acceptance_report.py \
  --metrics generated/aerodrone/acceptance_metrics.json \
  --output-json generated/aerodrone/acceptance_report.json \
  --output-markdown generated/aerodrone/acceptance_report.md
```


## DemNav/WildNav Simulation Notes

Gazebo is the starting point for Aerostack2 integration because AS2 already
ships Gazebo platform and asset packages. This setup is good for verifying ROS
topic flow, timing, odometry/GPS plumbing, controller behavior, and whether
DemNav/WildNav nodes can consume simulated camera topics.

For algorithm-quality DemNav tests, replace `empty_gps` with a georeferenced
terrain world and align `INITIAL_LAT`, `INITIAL_LON`, and `ORIGIN_ALT` with that
world. DemNav compares downward depth against terrain elevation, so a flat empty
world only validates wiring.

For algorithm-quality WildNav tests, the downward RGB stream must visually match
the satellite tile cache. The default Gazebo world is not enough; use textured
terrain/orthophoto tiles, AirSim/Unreal, or Isaac Sim with georeferenced terrain
assets when evaluating matching quality.

Practical simulator choices:

- Gazebo/GZ Sim: best first target for Aerostack2 because AS2 provides Gazebo
  assets and a Gazebo platform plugin. Good for ROS integration and DemNav depth
  plumbing.
- AirSim/Unreal: useful for photorealistic downward RGB/depth and external
  flight-controller workflows, but upstream AirSim is effectively legacy.
- Isaac Sim: useful when high-fidelity sensors, ROS 2 bridging, and synthetic
  data matter more than direct Aerostack2 support.
- Bag replay: still the fastest regression path for DemNav/WildNav once real or
  simulator image/depth/GPS/odom topics have been recorded.

## Mount Tamalpais Wilderness Navsim Default

The `navsim` Compose profile uses a georeferenced wilderness test origin near
Mount Tamalpais so standard GPS, DemNav, WildNav, and fused navigation can be
evaluated against the same area:

- `INITIAL_LAT=37.9234`
- `INITIAL_LON=-122.5967`
- `ORIGIN_ALT=280.0`

The tracked scenario contract is
`systems/aerostack2_gazebo/scenarios/mount_tamalpais_wilderness.json`.
The Gazebo compose file includes `demnav_assets` and `wildnav_assets` services
that seed named Docker volumes before the navigation nodes start. This keeps DEM
data, satellite tiles, descriptors, terrain meshes, bags, and logs out of git.
Confirm the terms for the configured `WILDNAV_TILE_URL` provider before caching
or replaying satellite imagery offline. Change Docker build args, runtime
environment variables, or cache volumes when evaluating a different test area.
