# Aerodrone GPS-denied SITL test

`tools/gps_denied_sitl_test.sh` flies ArduCopter (the Aerodrone) in GUIDED,
disables its GPS mid-flight, and checks that it keeps flying GUIDED on
DemNav/WildNav corrections fed back to ArduPilot, scoring position error
against simulator ground truth. It is meant for nightly or local runs, not
per-push CI.

```bash
tools/gps_denied_sitl_test.sh                               # defaults below
GPS_INPUT_CORRECTIONS=false tools/gps_denied_sitl_test.sh   # control run
tools/gps_denied_sitl_test.sh --max-fused-rmse-m 20         # scorer options
```

Needs the `salsicha/cyberphysics:navigator-latest`, `aerostack2`, `demnav` and `wildnav`
images (`make -C applications build_navigator build_demnav build_wildnav`).
The working tree's Python sources are mounted over the images' installed
packages, so a stale image still runs the repository's code. No GPU, Gazebo
or network access is needed. A run takes about 10 minutes; telemetry,
the score and all container logs land in `generated/gps_denied_sitl/`.

Variables: `ROS_DOMAIN_ID`, `SITL_INSTANCE` (MAVLink on 5760 + 10 N),
`GPS_DENIED_DENY_AFTER_S` (60), `GPS_DENIED_DENIED_S` (300),
`GPS_DENIED_RESTORE_S` (30), `GPS_DENIED_TAKEOFF_ALT_M` (150),
`GPS_DENIED_LEG_M` (300), `GPS_DENIED_OUTPUT`, `GPS_DENIED_TIMEOUT_S`.

## Stack (`tools/gps_denied_sitl.yaml`)

The lightest stack that exercises ArduPilot, MAVROS, DemNav, WildNav,
navigation fusion and the ArduPilot navigation bridge together:

- ArduCopter SITL with built-in multicopter physics, parameters layered as
  on every simulator: `arducopter.params` (hardware baseline),
  `arducopter_sitl.params`, `arducopter_gps_input.params`;
- `applications/ardupilot_sitl/scripts/sitl_ground_truth.py`: simulator
  truth (MAVLink SIM_STATE) on `/aerodrone/ground_truth/{fix,odom}`;
- `systems/airplane/scripts/satellite_camera_sim.py` rendering the shared
  synthetic world at the **true** pose (so the camera keeps working when
  the GPS is gone), matched by DemNav (synthetic DEM) and WildNav
  (synthetic tiles);
- the fusion node and `applications/aerostack2/scripts/ardupilot_navigation_bridge.py`;
- `tools/gps_denied_sitl_mission.py`: waits for GPS/EKF/origin, GUIDED,
  arm, take off to 150 m, flies a 300 m square of local setpoints, sets
  `SIM_GPS_DISABLE=1` (ArduPilot 4.6: GPS 1 then reports no fix) after 60 s,
  restores it after 300 s, and records 1 Hz telemetry;
- `tools/gps_denied_sitl_scorer.py`: scores the denied phase.

## How corrections reach ArduPilot

1. **Shared origin.** The bridge relays the EKF origin from MAVROS
   `global_position/gp_origin` to `/navigation/origin` (reliable,
   transient-local). MAVROS converts ArduPilot's AMSL origin altitude to the
   WGS-84 ellipsoid (SITL: 781.0 m AMSL home -> 748.77 m, geoid -32.2 m). The
   fusion node converts DemNav/WildNav lat/lon fixes into ArduPilot's local
   ENU frame about that origin and compares them with the raw odometry at the
   fix's camera timestamp (interpolated between samples up to 1.5 s apart;
   rejected with no sample within 0.25 s; simulator stamps are mapped
   through `/clock`).
2. **Only accepted fixes, once.** Each accepted fix is published once on
   `/navigation/correction_fix`, propagated from camera time to the latest
   odometry sample with the odometry *velocity* (not position, which jumps
   when the EKF absorbs an earlier correction). It is never the fused
   estimate: that is ArduPilot's odometry plus an offset, and feeding it back
   makes the EKF chase its own offset.
3. **GPS 2 = GPS_INPUT.** The bridge sends each one through the MAVROS
   gps_input plugin with fix type 3D, GPS week/ms from the stamp, honest
   horizontal accuracy (fix sigma, HDOP = sigma / 5 m), 6 satellites (the
   EKF's alignment minimum, below any real receiver so GPS 1 always wins),
   and altitude, velocity and vertical accuracy flagged absent (visual fixes
   carry the prior's altitude; height stays on the barometer).
4. **EKF source sets.** GPS_INPUT without velocity is fused as zero velocity
   while GPS is a velocity source. With GPS 1 lost the bridge selects EKF
   source set 2 (GPS position only, no velocity, baro height, compass yaw)
   and returns to set 1 after GPS 1 has held a 3D fix for 10 s.
   `EK3_SRC_OPTIONS=0` is required: the default "fuse all velocities"
   keeps fusing set 1's GPS velocity.
5. **Arming.** A configured GPS 2 without a fix fails the "GPS 2: Bad fix"
   pre-arm check. `arducopter_gps_input.params` therefore leaves
   `GPS2_TYPE=0`; the bridge sets `GPS2_TYPE=14` when GPS 1 is lost in flight
   and back to 0 on disarm, so pre-arm GPS checks stay enabled. If the bridge
   dies with GPS 2 configured, the next arming is refused until
   `GPS2_TYPE` is reset (fail-safe).
6. While forwarding is enabled, the fusion node stops applying its own
   offsets (the EKF already absorbs the corrections).

The feature is off by default (`GPS_INPUT_CORRECTIONS=false`) in every
stack; this test turns it on.

## Measured in ArduCopter 4.6.3 SITL (pinned `Plane-4.6.3` tree)

Prototype with pymavlink, GPS 1 disabled mid-flight in GUIDED, GPS_INPUT
built from simulator truth plus Gaussian noise (sigma) every interval:

| Configuration | Result |
| --- | --- |
| No corrections | "EKF variance" failsafe -> LAND 8 s after GPS loss |
| GPS_INPUT, default EKF sources (GPS velocity fused) | velocity innovation check fails at the first GPS 2 sample -> LAND within 2 s |
| Set 2 position-only, `EK3_SRC_OPTIONS=0`, 5 s / sigma 8 m | GUIDED for 240 s, error RMSE 9.2 m, max 20 m |
| Same, exactly the bridge's message fields | GUIDED 240 s, RMSE 11.0 m, max 26.5 m |
| Same, `ARMING_CHECK=1`, `GPS2_TYPE` set at runtime | arms; GUIDED 180 s, RMSE 11.9 m, max 23.8 m |
| 3 s / sigma 15 m, GPS restored after 120 s | GUIDED; RMSE 13.2 m, max 30 m; back to set 1, 0.0 m after restore |
| 5 s / sigma 20 m | GUIDED; RMSE 18.2 m, max 48 m |
| 10 s / sigma 8 m | "EKF variance" failsafe -> LAND ~8 s after a fix |

So corrections must arrive less than about 7 s apart: EKF3 declares
position aiding lost after roughly 8 s without a position measurement.

Full stack (`tools/gps_denied_sitl_test.sh`, DemNav corrections at 2 Hz):

| Run | Denied | Fused error RMSE / p95 / max | EKF error RMSE | GUIDED | Coverage |
| --- | --- | --- | --- | --- | --- |
| corrections on, 120 s | 118 s | 14.0 / 27.3 / 30.2 m | 13.9 m | 100 % | 94 % |
| corrections on, 300 s (defaults), two runs | 298-299 s | 8.2-8.4 / 17.0-17.4 / 20.3-20.6 m | 8.1-8.3 m | 100 %, no failsafe text | 99 % |
| control (`GPS_INPUT_CORRECTIONS=false`) | 128 s | 51.4 / 83.1 / 86.1 m | 57.2 m | 6 % (EKF failsafe -> LAND after 8 s) | 100 % |

WildNav matched rarely in this stack (a handful of fixes with 11-15
inliers and confidence 0.15-0.30 at 150 m AGL over the synthetic tiles,
several 100-750 m off and rejected by its jump gate or the fusion
confidence gate), so DemNav supplied almost every correction; its
horizontal sigma (up to ~70 m) dominates the error.

## Provisional thresholds

The scorer's defaults (`--max-fused-rmse-m 25`, `--max-fused-p95-m 45`,
`--max-fused-max-m 60`, `--max-ekf-rmse-m 25`, `--min-correction-coverage
0.8`, `--min-guided-fraction 1.0`, no EKF failsafe / GPS glitch text,
`--min-denied-s 240`) are about 1.5-3x what the runs above achieved: tight
enough to catch the control run and frame/clock regressions, loose enough
for run-to-run variation. They are proposals, not a validated requirement.

## Limits

- SITL's IMU is near-perfect, so dead reckoning between fixes is far better
  than on a real Navigator; real drift will need more frequent or more
  accurate fixes. Nothing here has flown on hardware.
- GPS loss is detected from GPS 1's fix type only; a spoofed or jammed GPS
  that still reports a 3D fix is not handled.
- In Gazebo/Isaac stacks camera stamps are simulator time and MAVROS stamps
  are wall time; the fusion node maps them through `/clock`
  (unit-tested, not run in Gazebo here).
- No GitHub workflow: the run needs the ArduPilot SITL, Aerostack2/MAVROS,
  DemNav and WildNav images (about 10 GB, well over an hour to build) on
  every run, which does not fit GitHub-hosted runners without a registry or
  self-hosted runner.
