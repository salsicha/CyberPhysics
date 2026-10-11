# Aerodrone GPS-denied simulation test (Gazebo, Genesis, Isaac)

`tools/gps_denied_sim_test.sh <gazebo|genesis|isaac>` flies the wilderness
mission of an Aerodrone simulation composition, denies GPS 1 for a window of
**simulation** time during the cruise at `CRUISE_AGL_M` (250 m above
ground), and scores the fused navigation fix, ArduPilot's own position, the
DemNav/WildNav fixes and the corrections against simulator ground truth. It
is the heavy-stack sibling of the lite SITL test
(`tools/gps_denied_sitl_test.md`, ArduCopter's built-in physics and a
synthetic camera), meant for nightly or local runs on a host with a GPU and
the stack's images, not per-push CI.

```bash
tools/gps_denied_sim_test.sh gazebo                    # about 10 minutes
tools/gps_denied_sim_test.sh genesis                   # about 15 minutes (0.55x real time)
tools/gps_denied_sim_test.sh isaac                     # about 35 minutes (0.19x real time)
GPS_DENIAL_DURATION_S=60 tools/gps_denied_sim_test.sh gazebo   # shorter denial
GPS_DENIED_COMPOSE_OVERRIDE=tools/gps_denied_sim_control.yaml \
    tools/gps_denied_sim_test.sh gazebo                # control run: must fail
tools/gps_denied_sim_test.sh isaac --max-fused-p95-m 8 # scorer options
python3 tools/gps_denied_sim_scorer.py generated/gps_denied_sim/isaac   # re-score
```

The script brings the composition up with the `validation` and `denial`
profiles, waits for the denier to finish its window (the stack is torn down
early if the mission ends before reaching cruise, or after
`GPS_DENIED_TIMEOUT_S`), saves the logs, tears the stack down (with its
volumes unless `GPS_DENIED_KEEP_VOLUMES=true`: the next run seeds the DEM and
satellite tiles again) and scores the run. Exit status: 0 pass, 1 score
failed, 2 no usable run (stack did not come up, no denial, no recording), 3
score passed but the navigation validation failed or did not finish.

Variables: `ROS_DOMAIN_ID` (0), `GPS_DENIED_PROJECT` (compose project name,
`gps_denied_<stack>`), `GPS_DENIED_COMPOSE_OVERRIDE` (an extra compose file),
`GPS_DENIED_OUTPUT` (`generated/gps_denied_sim/<stack>`),
`GPS_DENIED_TIMEOUT_S` (wall clock: gazebo 1800, genesis 2700, isaac 3300),
`GPS_DENIED_KEEP_VOLUMES`, `NAVSIM_REQUIRE_VALID` (`demnav wildnav`) and
`NAVSIM_REQUIRE_CORRECTION` (`true`) for the validator, and the denial window
`GPS_DENIAL_START_S` (60), `GPS_DENIAL_DURATION_S` (150) and
`GPS_DENIAL_RECOVERY_S` (60), whose defaults live in
`applications/aerostack2/config/mount_tamalpais.env`. The stacks use host
networking, fixed SITL/MAVROS ports and the GPU: run one at a time per host.

The run directory holds `recording.jsonl.gz` (the recorder's samples),
`denial_events.jsonl` (the denier's steps), `score.json` (the summary),
`validation.log`, `stack.log.gz` (every container's log) and `up.log`.

## The `denial` profile

Each of `compositions/aerostack2_sim.yaml` (Gazebo),
`aerostack2_genesis.yaml` and `aerostack2_isaac.yaml` has two opt-in
services, in the aerostack2 image with the working tree's
`applications/aerostack2/scripts` mounted and `GPS_DENIED_OUTPUT` on `/out`:

- `gps_denier` (`applications/aerostack2/scripts/gps_denier.py`): waits for
  the mission to reach cruise, the vehicle armed and 140 m above the EKF
  origin (MAVROS local position; the en-route waypoints are 170-255 m above
  home), then waits `GPS_DENIAL_START_S`, sets `SIM_GPS_DISABLE=1` (GPS 1
  reports no fix), restores it after `GPS_DENIAL_DURATION_S` and exits
  `GPS_DENIAL_RECOVERY_S` later. Every interval is simulation time from
  `/clock` (ArduPilot's lock-step time), never wall time: Isaac runs at about
  0.2x and Genesis at about 0.5x real time, so a wall-clock window would
  deny a different stretch of the mission on every stack. Each step goes to
  `denial_events.jsonl` with both clocks. With the defaults the denial
  covers the climb's last leg and the ridge crossing (about 60-210 s after
  cruise), where the scratch runs this test replaces denied it.
- `gps_denial_recorder` (`gps_denial_recorder.py`): from MAVROS start-up,
  one compact JSON line per sample: simulator ground truth
  (`/aerodrone/simulation/ground_truth/odom` in Genesis and Isaac,
  `/aerodrone/ground_truth/pose` in Gazebo, the pose its bridge publishes and
  ArduPilot's JSON backend flies; east/north/up metres about
  `INITIAL_LAT`/`INITIAL_LON`), `/clock`, ArduPilot's position (MAVROS
  `global_position/global`), `/navigation/fix` with its covariance, every
  `/navigation/correction_fix`, `/demnav/fix` and `/wildnav/fix` with their
  covariances, and changes of the correction source, the autopilot mode,
  GPS 1's fix type and the MAVROS position setpoint (the mission's GoTo
  targets), autopilot status texts and the EKF origin. Truth is kept at
  10 Hz and the autopilot/fused fixes at 5 Hz of simulation time: a
  35-minute Isaac run records 1.6 MB (under 300 kB compressed).

The `navsim_validator` (`validation` profile) runs alongside as in any
validation run, with DemNav and WildNav matches and an applied correction
required; it normally passes before the denial starts.

## Scorer (`tools/gps_denied_sim_scorer.py`)

Pure Python (runs on the host). Phases are cut at the denier's events:
**before** (cruise on GPS, from reaching cruise to the denial), **during**
(GPS 1 denied) and **after** (from the restore to the denier's exit). For
each it reports:

- the horizontal error of the fused fix and of ArduPilot's position against
  the truth at each sample's arrival: median, p95 (linear interpolation, as
  numpy), max and RMS;
- the error of every DemNav and WildNav fix against the truth at its camera
  stamp (the truth's and the camera's stamps share a clock in each stack:
  wall time in Genesis and Isaac, simulation time in Gazebo; a fix stamped
  on another clock is scored at arrival) and of every correction at arrival;
- covariance consistency: the share of fused fixes, DemNav/WildNav fixes and
  corrections beyond 2 and 3 sigma, as the Mahalanobis distance over the
  reported horizontal covariance (a consistent 2D Gaussian puts 13.5 %
  beyond 2 sigma and 1.1 % beyond 3). The corrections go out with their
  variance multiplied by the number sent per correlation time
  (`tools/gps_denied_sitl_test.md`), so they are almost never beyond
  2 sigma; their share is reported, not checked;
- route-keeping: the true track's distance from the commanded leg, the
  straight line from where the vehicle was when a MAVROS position setpoint
  arrived to that setpoint (ArduPilot's GUIDED path), the waypoints reached
  (setpoint changes) and the distance flown. Legs start at the vehicle, not
  at the previous waypoint, because in Isaac every other GoTo of the tree
  succeeds at once and the vehicle flies straight to the next one;
- correction coverage (share of fused samples while the fusion reports an
  active correction source) and the longest gap between corrections in
  simulation seconds (EKF3 declares position aiding lost after about 7 s);
- while denied: the autopilot modes, GPS 1's fix types and any failsafe
  text (the lite scorer's `FAILSAFE_PATTERN`, imported from it).

It writes everything to `score.json`, prints a table and exits 0 (pass), 1
(fail) or 2 (unusable: no recording, no truth, no projection origin, or GPS 1
never denied).

### Pass criteria

| Check | Gazebo | Genesis | Isaac |
| --- | --- | --- | --- |
| fused and autopilot error while denied, median / p95 / max | 20 / 35 / 45 m | 5 / 6 / 7 m | 6 / 12 / 15 m |
| route distance while denied, max | 45 m | 7 m | 15 m |
| fused fixes beyond 3 sigma while denied | 15 % | 10 % | 10 % |

and, for every stack: GPS 1 denied and restored, at least 95 % of the
configured simulation time; GPS 1 lost its 3D fix; armed in GUIDED
throughout with no failsafe text; at least one scored fused sample per
simulated second; fused median before the denial at most 1 m (frames and
clocks agree); correction coverage at least 0.8; autopilot median after the
restore at most 2 m.

The error limits are about twice the worst run of the committed navigation
code (below; `tests/test_gps_denied_sim_scorer.py` keeps them between 1.5
and 4 times it), the route limit is the error's maximum. They pass the
committed code's run-to-run spread (Gazebo's p95 11-20 m over five runs)
and fail the control run, the regressions seen while these stacks were being
tuned (Gazebo p95 50-170 m and max 87-248 m, Isaac 8.7 / 16.3 / 18.9 m with
earlier fusion settings) and the three work-in-progress runs below. The
beyond-3-sigma share is 1.1 % for a consistent Gaussian, but over one 150 s
window it is noisy, since the errors stay correlated for about a minute;
10 % catches a fused covariance several times too confident. Gazebo's limit
is 15 %: before the fused covariance followed the autopilot's lag, its
fused sigma stayed at the fixes' 5-6 m while the autopilot drifted up to
22 m, 8.4 % beyond 3 sigma in one run; with it (below) a Gazebo run was
6.8 m and 0 % beyond 3 sigma, so this limit can tighten after more runs. Mission
progress (waypoints reached while denied) is reported, not checked: see the
Isaac run below. Every limit is a scorer option (`--max-fused-p95-m`, ...),
and the limits are for the default 150 s window: score a longer or shorter
denial against limits of its own.

## Measured

Default window (GPS 1 denied 60 s of simulation time after cruise, for
150 s, recorded 60 s after), `ROS_DOMAIN_ID=89`, 2026-10-10/11. Committed
navigation code: HEAD between `04d2b86` and `f1b574b` (bridge, fusion and
ArduPilot parameters unchanged since `471b50a`) with this test's files; the
HEAD runs were made
from a `git archive` of HEAD with HEAD's `navigation_fusion.py` mounted over
the wildnav image, while another session's uncommitted bridge, fusion and
parameter changes were in the working tree. Errors in metres, median / p95 /
max while GPS 1 was denied unless marked; "after" is the autopilot after the
restore. Every validation passed 35-100 s before the denial (110-115 s
after it started in Gazebo, 155 s in Genesis, 617 s in Isaac), except the
control run's, which cannot.

| Run | Real time | Fused before | Fused | Autopilot after | DemNav fixes (n) | WildNav fixes (n) | Corrections | Fused beyond 2 / 3 sigma (sigma) | Route distance | Result |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Gazebo 1 | 0.97x | 0.1 / 0.4 / 0.5 | 7.5 / 11.2 / 12.3 | 0.2 / 2.1 / 12.2 | 8.2 / 13.3 / 21.6 (448) | 2.5 / 4.7 / 5.3 (31) | 5.1 / 8.1 / 9.5 | 10 / 3.3 % (5.9 m) | 5.0 / 10.9 / 13.3 | pass |
| Gazebo HEAD 1 | 0.97x | 0.1 / 0.5 / 0.6 | 5.7 / 12.6 / 14.5 | 0.2 / 1.2 / 5.8 | 8.7 / 15.0 / 19.3 (384) | 3.0 / 5.0 / 5.3 (31) | 3.9 / 10.1 / 11.9 | 13.5 / 3.3 % (6.1 m) | 5.8 / 12.3 / 14.0 | pass |
| Gazebo HEAD 2 | 0.96x | 0.1 / 0.1 / 0.2 | 7.3 / 19.8 / 22.4 | 0.2 / 0.9 / 10.5 | 6.5 / 10.9 / 13.8 (468) | 2.2 / 4.0 / 4.5 (31) | 4.0 / 9.9 / 15.3 | 17.5 / 8.4 % (5.9 m) | 4.9 / 11.9 / 12.6 | pass |
| Genesis 1 | 0.57x | 0.1 / 0.2 / 0.2 | 2.0 / 2.5 / 2.6 | 0.1 / 0.1 / 1.5 | 2.2 / 5.9 / 6.8 (1501) | 1.0 / 1.7 / 2.3 (53) | 1.3 / 1.6 / 2.2 | 0 / 0 % (3.7 m) | 2.0 / 2.5 / 2.7 | pass |
| Genesis HEAD | 0.57x | 0.1 / 0.2 / 0.2 | 2.1 / 2.5 / 2.6 | 0.1 / 0.4 / 1.5 | 2.2 / 5.9 / 6.8 (1500) | 1.0 / 1.6 / 1.8 (52) | 1.3 / 1.8 / 2.0 | 0 / 0 % (3.7 m) | 2.1 / 2.5 / 2.6 | pass |
| Isaac | 0.19x | 0.1 / 0.2 / 0.2 | 3.2 / 5.5 / 6.0 | 0.1 / 1.2 / 1.7 | 3.7 / 9.5 / 11.4 (1437) | 1.5 / 2.2 / 2.5 (156) | 2.4 / 3.9 / 4.6 | 0.1 / 0 % (3.1 m) | 3.5 / 5.9 / 6.4 | pass |
| Gazebo control (`gps_denied_sim_control.yaml`) | 0.97x | 0.1 / 0.2 / 0.3 | 1.6 / 5.3 / 6.0 (first 16 s); autopilot 45.7 / 83.5 / 84.0 | 0.0 / 0.1 / 79.8 | none | none | none | 0 / 0 % (99 m) | 38.6 / 77.2 / 77.6 | fail (exit 1) |

The autopilot's error matches the fused fix's within 0.1 m in every run
but the control (the fusion publishes ArduPilot's odometry while ArduPilot
absorbs the corrections). The same scorer on the scratch recordings this
test replaces (converted to the recorder's format) gives the numbers
reported for them: Isaac 2.6 / 5.7 / 6.0 m, Gazebo 10.6 / 13.7 / 15.2 and
7.8 / 13.7 / 15.8 m, with the same DemNav/WildNav/correction errors and
consistency shares.

Runs with another session's uncommitted changes in the working tree (a new
EKF source set 3 with a camera velocity, `EK3_MAG_CAL=2`, and the bridge
that forwards the velocity, mounted from the tree; in Gazebo 3 also the
rebuilt wildnav image's fusion, which sent velocities): all three fail.

| Run | Fused | Corrections | Fused beyond 2 / 3 sigma | Route distance | Failed checks |
| --- | --- | --- | --- | --- | --- |
| Gazebo 2 (no velocity sent) | 13.4 / 22.8 / 25.3 | 8.1 / 13.6 / 15.6 | 56 / 16.7 % | 9.7 / 22.3 / 23.9 | fused consistency |
| Genesis 2 (no velocity sent) | 6.0 / 6.7 / 6.9 | 1.8 / 2.9 / 3.5 | 0.8 / 0 % | 4.4 / 7.6 / 7.8 | fused and autopilot median and p95, route |
| Gazebo 3 (velocity sent, source set 3 for 8 s at the loss) | 34.9 / 75.7 / 84.9 | 18.8 / 43.7 / 63.8 | 37 / 9.5 % | 36.5 / 90.1 / 97.7 | every error check, route |

- **Control run.** With DemNav and WildNav stopped nothing reaches ArduPilot
  as GPS 2: "EKF variance" and "EKF Failsafe: changed to LAND Mode" 11 s
  after the denial, the vehicle descends in LAND up to 78 m off its leg
  (ArduPilot thinks it is 84 m from where it is), and the fusion stops
  publishing `/navigation/fix` 16 s in, once the EKF has no position and
  MAVROS no local odometry (75 fused samples where 150 are required). It
  fails eight checks: `stayed_guided`, `no_failsafe`, `fused_samples`, the
  three autopilot errors, `correction_coverage` and `route_keeping`. The
  validator, which needs DemNav matches, never passes, so the test would
  exit 3 even if the score had passed.
- **Run to run.** Genesis repeats to 0.1 m (2.0 and 2.1 m median). Gazebo
  spreads: DemNav's terrain-fixed biases (up to 14-22 m) pull the
  corrections and the EKF, so its p95 ranged from 11 to 20 m over five
  committed-code runs.
- **Work in progress.** In Genesis 2 the autopilot drifted 6 m within 30 s
  of the loss and stayed there for two minutes while the corrections
  averaged 1.5 m off, three times the committed code's error on the same
  flight; in Gazebo 3 it reached 20 m 20 s after the loss and 42 m at 30 s,
  following 8 s on source set 3, and the camera fixes, which use the EKF's
  attitude, degraded with it.
- **Fused covariance.** The runs in the tables above predate the fused
  covariance that includes the lag between the fixes and the fused estimate
  (`tools/gps_denied_sitl_test.md`, "Fused fix uncertainty"). Before it the
  fused sigma followed the fixes (3-7 m) rather than the autopilot's drift:
  in Gazebo up to 8.4 % of the fused fixes were beyond 3 sigma (17 % in
  work-in-progress run 2), Genesis and Isaac under 1 %. With it, while
  denied: Gazebo 7.2 / 12.5 / 13.5 m at a 6.8 m sigma, 4.0 / 0 % beyond 2 /
  3 sigma; Genesis 2.0 / 2.5 / 2.7 m, 0 / 0 %; Isaac 3.2 / 5.4 / 5.9 m,
  0 / 0 %. All pass.
- **Mission progress in Isaac.** A GoTo completes when Aerostack2's state is
  within `go_to_threshold` (0.2 m, the as2 default) of the target. While GPS 1
  was denied, ArduPilot held 0.3-0.4 m from the ridge-entry waypoint in its
  own EKF frame, so the mission hovered there (on target in truth) for
  110 s of the 150 s denial and moved on 2 s before the restore. Gazebo and
  Genesis passed one or two waypoints while denied. Hence waypoints are
  reported, not checked. In Isaac every other GoTo of the tree also
  succeeds at once (the vehicle flies straight from 0, 0 to 200, 100 and
  from 460, 140 to 1060, 110), which is why route legs start at the vehicle.
- **Seeding.** One run never started: a satellite tile failed to download
  (120 of 121), `terrain_assets` refused the holed orthophoto and the
  wrapper exited 2. `GPS_DENIED_KEEP_VOLUMES=true` reuses a seeded cache.
- Recordings are 220-300 kB compressed, stack logs 25-210 kB; a run holds
  the host for 8 (Gazebo), 14 (Genesis) or 35 (Isaac) minutes.

## Limits

- One run per stack takes the host's GPU and the fixed SITL/MAVROS ports;
  the test does not lock the host itself.
- The denial window is timed from reaching cruise, not from a mission
  waypoint, so a change to the mission's climb moves the denied stretch.
- The validator starts after `flight_ready` and is not ordered with the
  denial; in a slow stack it can still be running when GPS 1 is denied, and
  then judges the degraded state (exit 3 when it fails).
- Results depend on the images and the working tree: DemNav, WildNav and
  the fusion run from the demnav/wildnav images, while the bridge,
  validator, denier, recorder and ArduPilot's parameters
  (`systems/aerodrone/config`) are the working tree's (mounted), so
  uncommitted changes there are part of what is tested.
- The limits rest on few runs (five Gazebo, two Genesis, two Isaac on the
  committed code); revisit them as runs accumulate.
