# Aerodrone GPS-denied SITL test

`tools/gps_denied_sitl_test.sh` flies ArduCopter (the Aerodrone) in GUIDED,
disables its GPS mid-flight, and checks that it keeps flying GUIDED on
DemNav/WildNav corrections fed back to ArduPilot, scoring position error
against simulator ground truth. It is meant for nightly or local runs, not
per-push CI.

```bash
tools/gps_denied_sitl_test.sh                               # defaults below
GPS_INPUT_CORRECTIONS=false tools/gps_denied_sitl_test.sh   # control run
GPS_DENIED_DENIED_S=0 GPS_DENIED_DENY_AFTER_S=300 \
    tools/gps_denied_sitl_test.sh                           # normal flight on GPS
tools/gps_denied_sitl_test.sh --max-fused-rmse-m 20         # scorer options
```

Needs the `salsicha/cyberphysics:navigator-latest`, `aerostack2`, `demnav` and `wildnav`
images (`make -C applications build_navigator build_demnav build_wildnav`).
The working tree's Python sources are mounted over the images' installed
packages, so a stale image still runs the repository's code. No GPU, Gazebo
or network access is needed. A run takes about 10 minutes; telemetry,
the score, the navigation validation and all container logs land in
`generated/gps_denied_sitl/` (`telemetry.json`, `score.json`,
`validation.log`, `stack.log`).

Variables: `ROS_DOMAIN_ID`, `SITL_INSTANCE` (MAVLink on 5760 + 10 N),
`GPS_DENIED_DENY_AFTER_S` (60), `GPS_DENIED_DENIED_S` (300),
`GPS_DENIED_RESTORE_S` (30), `GPS_DENIED_TAKEOFF_ALT_M` (150),
`GPS_DENIED_LEG_M` (300), `GPS_DENIED_OUTPUT`, `GPS_DENIED_TIMEOUT_S`,
`GPS_DENIED_VALIDATION_TIMEOUT_S` (90), `GPS_DENIED_PROJECT` (compose
project name, default `gps_denied_sitl_<SITL_INSTANCE>`),
`GPS_DENIED_COMPOSE_OVERRIDE` (an extra compose file, for example to layer
another parameter file on the SITL through `ARDUCOPTER_PARAM_FILE`).

`GPS_DENIED_DENIED_S=0` never disables the GPS: the mission is a normal
flight of `GPS_DENIED_DENY_AFTER_S + GPS_DENIED_RESTORE_S` and the scorer
checks that it armed, took off and finished in GUIDED with no pre-arm,
`GPS 2` or failsafe text from arming on, with the corrections enabled as in
every stack.

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
- `tools/gps_denied_sitl_scorer.py`: scores the denied phase;
- `validator` (`validation` profile,
  `applications/aerostack2/scripts/validate_aerodrone_navsim.py`, the
  `navsim_validator` of the Gazebo/Genesis/Isaac compositions with this
  stack's topics): run once by the test script when the mission logs
  `EVENT at_altitude`, before the GPS is denied. It needs, over at least
  10 s, fresh and valid MAVROS GPS/odometry/IMU, the camera image, depth and
  camera_info DemNav and WildNav consume (frame
  `aerodrone/downward_optical_frame`), the latched `/navigation/origin`
  near home, fused `/navigation/odometry` and `/navigation/fix` with a
  bounded covariance, live DemNav/WildNav `valid` streams, at least one
  valid DemNav match, `correction_source` not `dead_reckoning`, and MAVROS's
  static TF (`map -> map_ned`, `odom -> odom_ned`,
  `base_link -> base_link_frd`). This stack has no rangefinder. A failed
  validation fails the test with exit code 3 (when the score passed) and
  `validation.log` lists every missing, stale or invalid item. To run it by
  hand against a running stack:
  `docker compose -p <project> -f tools/gps_denied_sitl.yaml run --rm -T validator`.

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
2. **Accepted fixes, combined.** Each accepted fix publishes, on
   `/navigation/correction_fix`, the inverse-variance combination of every
   source's latest accepted fix (within `correction_timeout_s` and
   `max_fix_age_s`), each propagated from camera time to the latest
   odometry sample with the odometry *velocity* (not position, which jumps
   when the EKF absorbs an earlier correction). It is never the fused
   estimate: that is ArduPilot's odometry plus an offset, and feeding it back
   makes the EKF chase its own offset. EKF3 gives every GPS sample the
   largest accuracy reported in the last few seconds (a decaying peak hold)
   and fuses each as independent, so it cannot weight interleaved sources
   (the denser DemNav would outvote the more accurate WildNav); sending only
   the more accurate source instead starves it (position every few seconds:
   3 deg of tilt drift in Gazebo, which moves every camera fix). The
   variance sent is the combination's times n, the number of corrections
   sent in the last `correction_correlation_s` (60 s, the fixes' measured
   integral correlation time: see "Correction correlation time" below).
   The EKF fuses every sample as independent, and n = rate x correlation
   time samples carry one independent sample's information. At their own
   variance the EKF jumps at every sample, the controller answers with tens
   of degrees of tilt and the camera fixes, which use the EKF's attitude,
   degrade with it (Gazebo: 20-70 m autopilot error). A correction whose
   inflated sigma would exceed `max_correction_sigma_m` (100 m, where EKF3
   clips the GPS accuracy it fuses) is held back and not counted, so the
   stream thins out instead of being fused as more accurate than it is
   (up to 0.9 % of the corrections recorded in Isaac, Gazebo and Genesis
   went out at 100-175 m before; in Isaac it now holds back DemNav-only
   corrections that would go out at 117-122 m). Fixes more than
   `correction_consistency_sigma` (5) combined sigma apart are not
   combined; the one farther from the fused position is left out (a false
   WildNav match, halved and sent with that variance, passed the EKF's gate
   and pulled it 50 m in this test).
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
6. While forwarding is active, the fusion node stops applying its own
   offsets (the EKF already absorbs the corrections).
7. **Flight-controller check.** The bridge never writes the EKF source
   parameters. It reads the values of `arducopter_gps_input.params` through
   MAVROS (`param/get_parameters`) and arms forwarding (GPS_INPUT, source
   switching, `GPS2_TYPE` management) only when every one matches;
   otherwise it logs each wrong parameter as an error and forwards nothing.
   `run_arducopter_sitl.sh` loads the file, so SITL passes (about 10 s after
   MAVROS connects, once it has read the parameters); on the aircraft the
   file must be loaded by hand (`systems/aerodrone/README.md`).

The feature is on by default in every ArduCopter stack (Gazebo, Genesis,
Isaac, this test and the BlueOS hardware stacks: `GPS_INPUT_CORRECTIONS`
defaults to true, `GPS_INPUT_CORRECTIONS=false` turns it off). The airplane
stacks keep it off: ArduPlane has not been validated.

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
| on by default with the FC parameter check (forwarding armed 15 s after start) | 299 s | 8.1 / 17.7 / 20.5 m | 8.0 m | 100 %, no failsafe text | 98 % |
| combined corrections, variance x corrections in 60 s (item 2) | 298 s | 19.5 / 29.4 / 34.6 m | 19.4 m | 100 %, no failsafe text | 100 % |
| same, with the 100 m clip guard (item 2), two runs | 298 s | 15.5 / 29.7 / 30.0 and 14.3 / 28.3 / 28.6 m | 15.4 and 14.2 m | 100 %, no failsafe text | 100 % |
| experiment: `correction_correlation_s` 20 s (corrections at ~25 m) | 298 s | 14.4 / 22.7 / 28.2 m | 14.3 m | 100 %, no failsafe text | 100 % |

With the variance scaling the corrections go out at ~42 m (DemNav's 3.9 m
sigma, 120 corrections per 60 s). Run to run the result spreads by 5 m of
RMSE at the same setting; one run with a 20 s window landed inside that
spread (with a 6 m lower p95). What sets the error in every run, before
and after the scaling: within 10-30 s of the GPS loss the EKF's velocity
runs away from the truth by up to 3 m/s while the DemNav fixes are 1-3 m
off (the vehicle slows from 5 to under 3 m/s, MAVROS odometry still
reports 5 and later 8 m/s while the EKF position advances at 5 m/s),
putting it 16-30 m off; position-only GPS_INPUT corrects position, not
that velocity, so it converges over one to two minutes. Later in the
denial DemNav's terrain-fixed biases (up to 10-16 m on parts of the
square) add to it. The earlier 8 m runs had a per-fix sigma of ~22 m
(SITL's noisy gyro rate, which a stabilised camera does not see), about a
16 s window: three times as confident as this stack's measured correlation
supports.

### Correction correlation time

`correction_correlation_s` is the integral correlation time of the fixes'
errors against simulator truth: twice the area under their
autocorrelation up to its first zero (for an exponential, twice the 1/e
lag), on the simulator clock. n = rate x that time samples then carry one
independent sample's information, which is what the variance scaling
tells the EKF. Measured in the recorded runs (lag 1/e / integral time):

| Stack, recording | DemNav | WildNav | Corrections sent |
| --- | --- | --- | --- |
| this test, GPS on throughout (336 s) | 27 / 44 s (rms 5.0 m, sigma 3.9 m) | too few matches | 27 / 44 s |
| this test, denied (3 runs x 300 s, windows 60 and 20 s) | 28-29 / 45-47 s | too few matches | 28-29 / 44-47 s |
| Genesis, GPS on (2 x 820-850 s) | 33-38 / 62-66 s | 19 / 30-36 s | 30-37 / 52-55 s |
| Gazebo, GPS on after a denial (8 x 250 s) | 52-69 / 82-126 s (4-7 m biases) | 59-94 s or longer / 93-184 s (1-2 m bias) | 57-94 / 86-177 s |
| Gazebo, denied (8 x 150 s) | 17-70 / 36-113 s | sparse (0.2/s) | 14-68 / 21-92 s |
| Isaac, denied (4 x 150 s) | 9-22 / 14-48 s | 7 s once its 1-2 m bias is removed | 22-54 / 66-91 s |

DemNav's error is a terrain-fixed bias: binned by position over this
test's square it is the same with GPS on and denied (+10.1 and +11.0 m
east in one 75 m cell, (+4.0, -3.5) and (+4.1, -3.5) m in another), and it
fades over about a footprint flown (1/e after 130-140 m at 5 m/s here).
The integral time does not change with the window (45-47 s here at 60 and
at 20 s), so it is a property of the fixes, not of the loop. The same
follows without an exponential model: the error of the inverse-variance
mean of the corrections over 60 s, against what it would be for
independent samples, was 0.3-0.9 of the 120-fold inflation the window
applies here, 0.9-1.7 of it in Gazebo (eight runs: its corrections are as
confident as their errors support, or more), 0.2-0.4 in Isaac and 0.15 in
Genesis. So 60 s is the measured value for this stack and Genesis and a
lower bound for Gazebo; no single shorter window is supported, and lite's
slower recovery is the honest cost of position-only corrections whose
errors persist for most of a minute. Separate windows per source are not
either: WildNav's errors are small against its sigma but carry a steady
1-2 m bias, and in Gazebo the combined stream is already overconfident.

Normal flight with the corrections enabled (`GPS_DENIED_DENIED_S=0
GPS_DENIED_DENY_AFTER_S=300`, plus `ARMING_CHECK=1` layered through
`GPS_DENIED_COMPOSE_OVERRIDE` so pre-arm checks run as on the aircraft):
armed at the first attempt with no pre-arm or `GPS 2` text, 324 s in GUIDED
over the full square, 680 GPS_INPUT messages sent while `GPS2_TYPE` stayed 0
and the EKF stayed on source set 1 (ArduPilot ignores them), EKF error RMSE
0.5 m.

WildNav matched rarely in this stack (a handful of fixes with 11-15
inliers and confidence 0.15-0.30 at 150 m AGL over the synthetic tiles,
several 100-750 m off and rejected by its jump gate, the fusion
confidence gate or, for one 225 m off at confidence 0.22, the fusion's
consistency gate), so DemNav supplied almost every correction.

## Pass criteria

A run passes with fused navigation error RMSE <= 25 m, p95 <= 45 m and
max <= 60 m over at least 240 s of GPS denial, 100 % of it armed in GUIDED
(`--max-fused-rmse-m 25`, `--max-fused-p95-m 45`, `--max-fused-max-m 60`,
`--min-denied-s 240`, `--min-guided-fraction 1.0`), plus `--max-ekf-rmse-m
25`, `--min-correction-coverage 0.8` and no EKF failsafe / GPS glitch text.
These are the agreed pass criteria for the SITL test (about 1.5-3x what the
runs above achieved: tight enough to catch the control run and frame/clock
regressions, loose enough for run-to-run variation). They stay scorer
parameters so a run can be scored against other limits.
The navigation validation at altitude must pass too (exit code 3
otherwise).

## Limits

- SITL's IMU is near-perfect, so dead reckoning between fixes is far better
  than on a real Navigator; real drift will need more frequent or more
  accurate fixes. Nothing here has flown on hardware: a supervised flight
  test (systems/aerodrone/README.md) is required before relying on it.
- The flight-controller check runs once per bridge start; parameters
  changed on the autopilot after it passed are not re-checked.
- GPS loss is detected from GPS 1's fix type only; a spoofed or jammed GPS
  that still reports a 3D fix is not handled.
- In Gazebo/Isaac stacks camera stamps are simulator time and MAVROS stamps
  are wall time; the fusion node maps them through `/clock`
  (unit-tested, not run in Gazebo here).
- No GitHub workflow: the run needs the ArduPilot SITL, Aerostack2/MAVROS,
  DemNav and WildNav images (about 10 GB, well over an hour to build) on
  every run, which does not fit GitHub-hosted runners without a registry or
  self-hosted runner.
