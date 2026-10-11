# Aerodrone GPS-denied SITL test

`tools/gps_denied_sitl_test.sh` flies ArduCopter (the Aerodrone) in GUIDED,
disables its GPS mid-flight, and checks that it keeps flying GUIDED on
DemNav/WildNav corrections fed back to ArduPilot, scoring position error
against simulator ground truth. It is meant for nightly or local runs, not
per-push CI. The Gazebo, Genesis and Isaac stacks have their own GPS-denied
test, `tools/gps_denied_sim_test.sh` (`tools/gps_denied_sim_test.md`).

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
| fused covariance from the distance to the fixes ("Fused fix uncertainty"), two runs | 298 s | 14.1 / 28.5 / 29.0 and 14.1 / 28.3 / 29.0 m | 14.0 and 14.0 m | 100 %, no failsafe text | 100 % |

With the variance scaling the corrections go out at ~42 m (DemNav's 3.9 m
sigma, 120 corrections per 60 s). Run to run the result spreads by 5 m of
RMSE at the same setting; one run with a 20 s window landed inside that
spread (with a 6 m lower p95). What sets the error in every run, before
and after the scaling: within 10-30 s of the GPS loss the EKF's velocity
runs away from the truth by up to 3.4 m/s (median 2.9 m/s over the first
minute) while the DemNav fixes are 1-3 m off (the vehicle slows from 5 to
under 3 m/s, MAVROS odometry still reports 5 and later 8 m/s while the EKF
position advances at 5 m/s), putting it 16-30 m off; position-only
GPS_INPUT corrects position, not that velocity, so it converges over one
to two minutes ("EKF velocity after GPS loss" below has the cause and the
fixes tried). Later in the denial DemNav's terrain-fixed biases (up to
10-16 m on parts of the square) add to it. The earlier 8 m runs had a
per-fix sigma of ~22 m (SITL's noisy gyro rate, which a stabilised camera
does not see), about a 16 s window: three times as confident as this
stack's measured correlation supports.

### EKF velocity after GPS loss

**Cause.** With source set 2 the EKF has no velocity measurement, and the
corrections, sent at ~42 m, barely observe its tilt. ArduCopter's default
`EK3_MAG_CAL=3` fuses all three magnetometer axes in flight, and they act
on roll and pitch too. In this SITL model a residual of 20-40 mGauss (on
the body Z axis, which the EKF had been absorbing into its compass bias
state while GPS velocity pinned the tilt) tilted the attitude by 1.0-1.4
deg within 15 s of the GPS loss. A degree of tilt error is 0.17 m/s^2 of
acceleration the EKF does not see, and the position controller, flying on
that attitude, accelerates the vehicle the other way: 3 m/s in 15-20 s.
It is not the bridge or the switch: the GPS_INPUT fields, flags and times
are as intended, ArduPilot fuses no velocity in set 2 (the absent GPS 2
velocity is only used by an EKF velocity reset, and none happens), GPS 2
is fused within 1.5 s of the loss with no EKF reset or lane switch, and
the MAV driver's 0.2 s lag misdates a correction by under 0.2 s. Nor is it
the corrections (perfect positions do the same) or SITL's gyro drift (it
happens without it).

Standalone ArduCopter 4.6.3 SITL (built-in quad, pymavlink, no ROS), the
same 300 m square at 150 m and 5 m/s, GPS 1 disabled, GPS 2 = GPS_INPUT at
2 Hz from SIM_STATE truth plus an error, sent at 42 m horizontal accuracy
as the stack does; EKF velocity and position against SIM_STATE while
denied:

| Run (error added to truth) | EKF velocity error median / p95 / max | Position RMSE / max | Max tilt error |
| --- | --- | --- | --- |
| stack parameters, 2 m white + 4 m bias, 120 s | 1.7 / 3.3 / 3.3 m/s | 17.9 / 30.9 m | 1.2 deg |
| same without SITL's gyro drift (`SIM_DRIFT_SPEED=0`) | 1.1 / 3.4 / 3.4 m/s | 17.3 / 31.2 m | 1.4 deg |
| same, perfect positions | 1.5 / 2.8 / 2.9 m/s | 15.3 / 24.7 m | 1.2 deg |
| same, positions sent at 99 m (unaided) | 2.6 / 4.4 / 4.4 m/s | 33.7 / 53.1 m | 1.4 deg |
| same with `EK3_MAG_CAL=2` (compass heading only) | 0.5 / 0.9 / 0.9 m/s | 6.6 / 10.4 m | 0.3 deg |
| this test's recorded DemNav errors replayed, 150 s / 300 s | 1.7 / 2.8 / 2.9 and 1.8 / 2.4 / 2.5 m/s | 13.5 / 25.1 and 13.3 / 20.6 m | |
| same with `EK3_MAG_CAL=2` | 0.9 / 1.4 / 1.5 and 1.0 / 1.5 / 1.6 m/s | 4.9 / 7.5 and 7.2 / 16.2 m | |
| replayed errors, velocity from the fixes (3 s fit) also sent, at the fit's time | "GPS Glitch", EKF variance failsafe -> LAND | up to 18 m | |
| same, carried to the send time with the EKF's own velocity change (2 s / 4 s window) | 0.15 / 0.5 / 0.8 and 1.1 / 2.4 / 2.7 m/s; 4 s at 4 Hz: EKF failsafe | 1.8 / 4.0 and 3.3 / 6.7 m; 72 m | |
| same, carried with the IMU's accelerations (RAW_IMU, attitude) | 0.17-0.23 / 0.45-0.68 / 0.67-1.11 m/s | 2.1-2.6 / 4.6 m | |

**Fixes tried in the full stacks; none ships.**

- `EK3_MAG_CAL=2` (with the velocity below, sent at ~0.84 m/s): this test
  2.5 / 4.1 / 4.3 and 2.3 / 3.7 / 3.8 m (RMSE / p95 / max), the EKF's
  velocity within 0.20 m/s (median) and 0.55 m/s of the truth. But
  Gazebo's and Genesis' compasses (SITL's JSON backend) agree with the
  attitude, and there three-axis fusion is what holds the tilt: 150 s of
  denial ended 13.4 / 22.8 / 25.3 and 20.9 / 45.1 / 51.9 m off in Gazebo
  (median / p95 / max, the second denied 30 s after the validation rather
  than 60 s after cruise; committed code 5.7-7.5 / 11.2-19.8 / 12.3-22.4 m,
  the EKF's velocity 2.2 m/s off instead of 0.4-0.9) and 6.0 / 6.7 / 6.9 m
  in Genesis (2.0 / 2.5 / 2.6 m): `tools/gps_denied_sim_test.md`.
- A velocity from successive camera fixes, sent in the same GPS_INPUT with
  a third source set (GPS position and velocity): per source a 3 s
  straight-line fit to the accepted fixes, carried from the window's mean
  time to the correction's with the IMU's accelerations (never the EKF's
  velocity: its change contains the corrections it already fused, counted
  again), its accuracy the fit's standard error or the measured scatter of
  non-overlapping fits, whichever is larger. ArduPilot complicates it: it
  gates GPS velocity at five times `EK3_VELNE_M_NSE` (0.3 m/s) plus its
  own uncertainty, not at the reported accuracy, reports "GPS Glitch" above
  1-1.5 m/s of filtered speed accuracy, reuses the previous GPS_INPUT's
  velocity for one without (so corrections without a velocity must be
  held back while it fuses them), and drops GPS 2 after 4 s without a
  message. With the default compass it did not hold here: sent
  conservatively (0.9 m/s) the run ended at 14.3 / 24.6 / 25.9 m, as
  before; sent at its own accuracy (0.35 m/s), the fits through a turn
  erred by up to 1.7 m/s (p95), the EKF's velocity ran 2-9 m/s off within
  20 s and the EKF failsafe landed the vehicle in both runs (79 and 84 m
  off; in one, corrections held back for 4 s also made ArduPilot drop
  GPS 2). In the heavy stacks the camera rides the vehicle and the fixes
  are projected with the EKF's attitude: in Gazebo the fits scatter by
  0.7-1.2 m/s,
  none qualified and the test passed as with the committed code (Isaac's
  recordings scatter as much); in Genesis they qualified (0.45 m/s) but
  grew to 0.54 / 1.49 m/s off
  (median / p95, 27 % beyond 2 sigma) once the EKF lost GPS, and the
  denial ended 4.5 / 13.4 / 17.3 m off, failing the test. With
  `EK3_MAG_CAL=2` but the fit's own standard error as accuracy Gazebo
  went 34.9 / 75.7 / 84.9 m off.

So the lite stack's runaway comes from this SITL model's compass and
position-only aiding, and both remedies regress the heavier simulators.
A fix that holds everywhere needs a velocity whose errors do not follow
the EKF (optical flow, which no stack has, or visual odometry), or a
compass model and calibration the EKF agrees with. The lite numbers above
stay what they are.

### Fused fix uncertainty

`/navigation/fix` (and `/navigation/odometry`) is ArduPilot's odometry plus
the fusion's offset, none while ArduPilot absorbs the corrections, so its
error is how far the EKF is from the truth. Its covariance used to be the
latest fix's variance plus dead-reckoning growth: about 4 m sigma here
while the EKF was 12-30 m off, 70-75 % of the denied samples beyond 2 sigma
and 44-55 % beyond 3. It is now, per axis, the exponentially weighted mean
square of (accepted fix - fused estimate at the fix's time) over
`fix_error_time_constant_s` (5 s), plus `fix_variance_weight` (0.5) times
the fixes' variance, plus the 0.5 m/s dead-reckoning growth since the last
fix. The distance measures how far the estimate lags the corrections; the
fixes' own error enters with a weight because an EKF that follows the
fixes shares it (Gazebo, Isaac) while one that dead-reckons between them
does not, and the fixes' sigma is conservative in some stacks. The weight
and time constant were chosen by replaying the model on recordings of
every stack (simulator truth; share beyond 2 / 3 sigma while denied, a
consistent 2D Gaussian puts 13.5 / 1.1 % there):

| Recordings | weight 0 | 0.25 | 0.5 (chosen) | 1 |
| --- | --- | --- | --- | --- |
| this test, two runs (committed navigation) | 24 / 9, 16 / 3 % | 21 / 6, 13 / 0 % | 17 / 3, 10 / 0 % | 11 / 0, 4 / 0 % |
| Gazebo, three runs (`tools/gps_denied_sim_test.sh`) | 10-31 / 0-2 % | 5-19 / 0 % | 1.5-3.7 / 0 % | 0-2 / 0 % |
| Genesis, two runs | 34-42 / 0 % | 0 / 0 % | 0 / 0 % | 0 / 0 % |
| Isaac | 69 / 51 % | 30 / 0 % | 0 / 0 % | 0 / 0 % |

0.5 keeps every stack at or under 3.2 % beyond 3 sigma (0.25 leaves 6 %
here) and is conservative in Genesis and Isaac (their errors are 2-3 m,
the sigma 2.3-2.8 m). A 10 s time constant lagged the error's growth after the GPS
loss here (19 / 3 and 13 / 0 % at 0.5). Live, on the final code:

| Run | Fused error while denied (median / p95 / max) | Fused sigma (median) | Beyond 2 / 3 sigma | Before (committed) |
| --- | --- | --- | --- | --- |
| this test, two runs | 11.0 / 28.4 / 29.4 and 11.0 / 28.3 / 29.1 m (RMSE 14.1) | 7.4-7.6 m | 6.1-6.8 / 0 % | 3.9 m, 70-75 / 44-55 % |
| Gazebo | 7.2 / 12.5 / 13.5 m | 6.8 m | 4.0 / 0 % | 5.9-6.1 m, 10-17.5 / 3.3-8.4 % |
| Genesis | 2.0 / 2.5 / 2.7 m | 2.8 m | 0 / 0 % | 3.7 m, 0 / 0 % |
| Isaac | 3.2 / 5.4 / 5.9 m | 2.3 m | 0 / 0 % | 3.1 m, 0.1 / 0 % |

Navigation is unchanged (the fused covariance does not reach ArduPilot;
DemNav and WildNav size their searches with it): DemNav still matched
every frame here
(2.0 fixes/s while denied), and the search radius, the configured radius
plus three sigma, changed by about 10 m at most (312 -> 322 m median here,
within 3 m in the simulators). With GPS the sigma stays conservative (2-4 m
against 0.1-0.2 m of error): the fixes' own error dominates it.

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
