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
  at the previous waypoint, because a GoTo can end before the vehicle gets
  there (in Isaac every other one did, until the tree sent each GoTo twice:
  "The GoTo race" below);
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
stays at 15 %: before the fused covariance followed the autopilot's lag,
its fused sigma stayed at the fixes' 5-6 m while the autopilot drifted up
to 22 m, 8.4 % beyond 3 sigma in one run. With it, nine Gazebo runs
(below) were 0, 0, 0, 0, 0, 0.1, 3.7, 7.6 and 7.9 % beyond 3 sigma (0-36 %
beyond 2): when DemNav is 14-21 m off for 20-30 s the fused sigma still
falls behind, and 10 % would leave the worst runs a margin of 1.3 where
the error limits keep 1.5 to 4. Mission progress (waypoints reached while denied) is
reported, not checked: see "Mission progress and the GoTo threshold"
below. Every limit is a scorer option (`--max-fused-p95-m`, ...),
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
  0 / 0 %. All pass. The runs of "Mission progress and the GoTo threshold"
  below have it too.
- **Mission progress.** These runs used as2's 0.2 m GoTo threshold, at
  which the mission hovered at the ridge entry for most of the denial (in
  Isaac 114 s of the 150 s): see "Mission progress and the GoTo threshold"
  below. In Isaac every other GoTo of the tree also succeeded at once (the
  vehicle flew straight from 0, 0 to 200, 100 and from 460, 140 to
  1060, 110): see "The GoTo race" below.
- **Seeding.** One run never started: a satellite tile failed to download
  (120 of 121), `terrain_assets` refused the holed orthophoto and the
  wrapper exited 2. `GPS_DENIED_KEEP_VOLUMES=true` reuses a seeded cache.
- Recordings are 220-300 kB compressed, stack logs 25-210 kB; a run holds
  the host for 8 (Gazebo), 14 (Genesis) or 35 (Isaac) minutes.

### Mission progress and the GoTo threshold

A GoTo completes when Aerostack2's state, ArduPilot's own position
(`/navigation/odometry`), is within `go_to_threshold` of its target, in 3D.
At as2's 0.2 m default the denied EKF stalled the mission at the ridge
entry: in the nine committed-code runs above, ArduPilot's distance from a
target it was holding (once within 1 m of it) was, while denied, 0.34 m
median, 1.2 m p95 and 2.1 m at most (Gazebo; Genesis and Isaac under 1 m),
and the mission held the ridge entry (within 5 m of it) for 114 s of the
150 s denial in Isaac, 77-78 s in Genesis and 7-77 s in Gazebo. The three
compositions now pass `go_to_threshold:=3.0` to `blueos_autonomy.launch.py`
(whose default stays 0.2 m, for the drone's map goals): above every such
hold distance, inside the smallest acceptance radius of a waypoint the tree
flies (4 m; the mission's are 4-8 m) and a seventeenth of the shortest leg
between two of its GoTos (50 m, 42 m horizontally;
`tests/test_aerial_mission_start.py` checks all three).

Runs with it, one GoTo per waypoint (the next section's runs send two):
`daba3ca` with this change and the rebuilt aerostack2 image,
`ROS_DOMAIN_ID=93`, 2026-10-11; columns as above, plus the waypoints
reached (setpoint changes) and the distance flown while denied. The control
is the same tree at 0.2 m (a compose override), the committed behaviour.
Every validation passed 31-36 s (Gazebo), 87 s (Genesis) or 112 s (Isaac)
before the denial. Two more Gazebo runs never started: a satellite tile
failed to download (120 of 121; see "Seeding").

| Run | Real time | Fused before | Fused | Autopilot after | DemNav fixes (n) | WildNav fixes (n) | Corrections | Fused beyond 2 / 3 sigma (sigma) | Route distance | Waypoints / flown | Result |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Gazebo 1 | 0.97x | 0.1 / 0.4 / 0.4 | 13.0 / 28.1 / 32.2 | 0.1 / 2.9 / 30.0 | 9.7 / 18.3 / 25.7 (414) | 3.4 / 6.2 / 6.4 (32) | 6.2 / 16.2 / 19.0 | 36.0 / 7.6 % (9.0 m) | 4.0 / 9.8 / 16.8 | 3 / 674 m | pass |
| Gazebo 2 | 0.97x | 0.1 / 0.3 / 0.4 | 7.0 / 14.8 / 17.6 | 0.2 / 0.7 / 3.8 | 6.0 / 11.7 / 15.5 (451) | 3.7 / 5.5 / 5.7 (31) | 4.8 / 7.5 / 10.0 | 5.2 / 0.0 % (7.1 m) | 1.3 / 6.0 / 9.0 | 3 / 694 m | pass |
| Gazebo 3 | 0.97x | 0.1 / 0.4 / 0.5 | 4.6 / 17.8 / 19.2 | 0.3 / 2.4 / 18.0 | 8.3 / 12.2 / 15.7 (419) | 1.7 / 6.5 / 7.5 (31) | 3.5 / 12.0 / 15.8 | 15.5 / 0.0 % (6.1 m) | 2.1 / 15.0 / 17.6 | 3 / 693 m | pass |
| Gazebo 4 | 0.96x | 0.1 / 0.7 / 0.8 | 6.4 / 12.7 / 14.4 | 0.6 / 0.8 / 3.4 | 10.5 / 15.8 / 22.5 (370) | 4.2 / 5.8 / 6.9 (31) | 4.8 / 8.4 / 10.9 | 4.7 / 0.0 % (8.2 m) | 3.1 / 7.6 / 11.6 | 3 / 705 m | pass |
| Genesis | 0.57x | 0.1 / 0.1 / 0.2 | 2.1 / 3.0 / 3.3 | 0.1 / 0.4 / 1.6 | 2.6 / 5.7 / 7.0 (1492) | 0.8 / 1.9 / 2.2 (53) | 0.9 / 2.3 / 3.2 | 0.0 / 0.0 % (3.8 m) | 1.2 / 2.5 / 2.7 | 3 / 711 m | pass |
| Isaac | 0.19x | 0.1 / 0.1 / 0.2 | 3.3 / 5.6 / 5.9 | 0.1 / 1.4 / 3.9 | 4.5 / 8.0 / 9.7 (888) | 1.6 / 2.8 / 3.7 (157) | 2.0 / 3.7 / 5.8 | 0.0 / 0.0 % (3.7 m) | 2.3 / 5.9 / 6.2 | 2 / 743 m | pass |
| Gazebo control (0.2 m) | 0.97x | 0.1 / 0.1 / 0.2 | 11.0 / 23.5 / 26.4 | 0.6 / 1.0 / 7.6 | 8.2 / 16.8 / 23.4 (430) | 3.5 / 5.3 / 5.9 (30) | 6.6 / 12.1 / 15.2 | 18.0 / 3.7 % (7.5 m) | 6.4 / 19.4 / 21.6 | 2 / 696 m | pass |

- **No stall.** In Gazebo and Genesis the mission reached the cruise
  entry, the ridge entry and the ridge mid while denied, moving on to the
  next waypoint 13-16, 68-72 and 140-146 s into the denial; each GoTo
  completed on arrival, and the vehicle left 1.8-3.6 s of simulation time
  after coming within 5 m (the tree's 2 s delay included). It flew
  674-711 m while denied, against 342 m in Genesis and 408 m in Gazebo in
  the fused-covariance runs, which hovered at the ridge entry. In Isaac it
  reached the ridge mid 99.6 s into the denial and left 0.4 s later; it
  flew 743 m while denied (189 m at 0.2 m). The control hovered 18 s at the
  ridge mid.
- **Accuracy.** Isaac 3.3 / 5.6 / 5.9 m, as committed (3.2 / 5.4 / 5.9).
  Genesis 2.1 / 3.0 / 3.3 m (2.0 / 2.5 / 2.7): its committed runs hovered at
  the ridge entry for the last 80 s of the denial at a steady 1.7-2.1 m;
  now it flies on to the ridge turn, and over the first 70 s both stay
  within 2.5 m. Gazebo spread further than in the committed runs (p95
  11-20 m) with either threshold: p95 13-28 m at 3 m, 23.5 m at 0.2 m. In
  Gazebo 1 and the control DemNav was 14-21 m off (median over 10 s)
  10-30 s after the loss, over the same ground, and the EKF followed it;
  the other runs' DemNav stayed within 6-10 m there. Every run passes, but
  the limits are only 1.25 and 1.4 times Gazebo 1's p95 and max, where
  `tests/test_gps_denied_sim_scorer.py` keeps 1.5 against the committed
  runs (these are not among them).
- **Full mission.** One full Gazebo mission with the validator
  (`NAVSIM_REQUIRE_VALID="demnav wildnav"`, `NAVSIM_REQUIRE_CORRECTION=true`)
  and the recorder, no denial: the validation passed after 104 s, all 20
  GoTos succeeded (only the first, at the take-off point, at once), the
  vehicle came within 0.3 m of every waypoint in truth, and the tree ended
  in SUCCESS after 18 minutes (the mission container exited 0).
- **Isaac skipped every other waypoint** with one GoTo per waypoint: the
  mission flew the cruise entry, the ridge mid and the ridge exit and cut
  past the climb waypoint, the ridge entry and the ridge turn (next
  section).

### The GoTo race

as2 1.1.3's GoTo (`go_to_plugin_position`, `GoToBase`) first checks its
distance to the target 100 ms after accepting a goal (its 10 Hz run timer
starts then), but computes that distance only when the next state arrives,
against the stored goal: until then it is the previous goal's. The state
follows ArduPilot's local position, 20 Hz of simulation time: every 50 ms
of wall time in Gazebo and 90 ms in Genesis, mostly before the check, but
every 260 ms in Isaac (0.19x), mostly after it. A goal then succeeds at
once whenever the vehicle is still within the threshold of the previous
waypoint (its setpoint is still sent when it ends, so the vehicle turns
towards it and is redirected to the next waypoint 0.4 s of simulation time
later). At 3 m it always is, so one GoTo per waypoint skipped every other
one in Isaac; at 0.2 m the vehicle had sometimes drifted past 0.2 m by
then, which kept the ridge entry, and its stall. No threshold fixes it.

The generated tree (`applications/demnav/scripts/build_georeferenced_terrain.py`)
therefore sends each waypoint's GoTo twice. The second's previous goal is
the same waypoint, so its first check, stale or not, is against that
waypoint: after a first GoTo that succeeded at once on the previous
waypoint's distance the second flies there, and after one that flew there
the second succeeds at once. A 2 s delay follows each: the GoTo rejects a
goal ("there is no localization") until a state has arrived since the last
goal ended, and the tree's GoTo node then fails, and the mission with it;
2 s is 8 states in Isaac. It costs 2 s of wall time per waypoint, a 2 s
longer hover at each in Gazebo (the full mission took 18 min 44 s, against
18 min 9 s), 0.4 s in Isaac. `tests/test_aerodrone_mission_frame.py` checks
the pairs.

Runs with it (3 m, otherwise as the previous section). Isaac, Gazebo 1
and the full mission ran on this change alone. Gazebo 2 and 3 also had
the object-tracking work, then in progress, in the tree and the aerostack2
image (moving cars and people in the Gazebo world, an idle FollowReference
behaviour), so Gazebo 4 and Genesis ran from a `git
archive` of HEAD with this change only and an aerostack2 image built from
it (Genesis with a genesis image rebuilt from HEAD's Dockerfile). Other work
loaded the host during Gazebo 1 and 2 (in Gazebo 2, three ONNX
evaluations on 18 of the 20 cores, load average 45-50): Gazebo ran at
0.80-0.81x real time, against 0.96-0.97x in every other Gazebo run here.
Validations passed 36-41 s (Gazebo), 88 s (Genesis) and 169 s (Isaac)
before the denial. One more Gazebo run never started (a tile, as above).

| Run | Real time | Fused before | Fused | Autopilot after | DemNav fixes (n) | WildNav fixes (n) | Corrections | Fused beyond 2 / 3 sigma (sigma) | Route distance | Waypoints / flown | Result |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Isaac | 0.19x | 0.1 / 0.1 / 0.2 | 3.0 / 5.5 / 6.1 | 0.1 / 0.6 / 2.7 | 4.5 / 8.9 / 13.9 (1085) | 1.7 / 2.5 / 3.1 (154) | 1.9 / 4.2 / 5.3 | 0.0 / 0.0 % (4.0 m) | 2.2 / 5.3 / 5.5 | 3 / 726 m | pass |
| Gazebo 1 (host loaded) | 0.81x | 0.1 / 0.3 / 1.1 | 13.0 / 40.5 / 49.7 | 0.2 / 0.5 / 28.8 | 8.0 / 19.0 / 28.8 (406) | 4.7 / 9.3 / 9.8 (37) | 8.6 / 23.4 / 29.1 | 19.9 / 7.9 % (10.8 m) | 10.0 / 38.4 / 41.6 | 2 / 698 m | fail: fused and autopilot p95, max |
| Gazebo 2 (host loaded, tracking work) | 0.80x | 0.2 / 0.6 / 0.6 | 7.6 / 24.0 / 29.7 | 0.1 / 0.2 / 7.0 | 10.8 / 14.8 / 20.7 (401) | 3.4 / 5.0 / 6.1 (37) | 5.5 / 10.7 / 16.9 | 14.0 / 0.1 % (7.3 m) | 3.1 / 13.1 / 14.9 | 2 / 689 m | pass |
| Gazebo 3 (tracking work) | 0.96x | 0.1 / 0.4 / 0.4 | 8.0 / 14.6 / 16.0 | 0.2 / 0.5 / 3.5 | 6.1 / 11.5 / 13.2 (463) | 2.9 / 6.5 / 7.5 (31) | 5.4 / 10.2 / 12.6 | 0.0 / 0.0 % (6.9 m) | 2.5 / 8.6 / 15.6 | 2 / 691 m | pass |
| Gazebo 4 (`git archive`) | 0.97x | 0.1 / 0.2 / 0.2 | 8.1 / 13.2 / 13.9 | 0.2 / 0.3 / 5.5 | 4.4 / 10.6 / 14.4 (468) | 4.0 / 6.8 / 7.6 (31) | 6.0 / 9.6 / 11.8 | 0.5 / 0.0 % (6.5 m) | 4.6 / 8.4 / 14.5 | 2 / 698 m | pass |
| Genesis (`git archive`) | 0.56x | 0.1 / 0.2 / 0.3 | 2.2 / 2.9 / 3.2 | 0.1 / 0.4 / 1.7 | 2.5 / 5.8 / 6.9 (1493) | 0.9 / 1.8 / 1.9 (54) | 1.0 / 2.3 / 3.0 | 0.0 / 0.0 % (3.7 m) | 1.2 / 2.6 / 2.7 | 3 / 692 m | pass |

- **Isaac flies every waypoint.** The first GoTo of the pair succeeded at
  once on the previous waypoint's distance at the cruise entry and the
  ridge mid, and the second flew there each time; where the first flew, the
  second succeeded at once. Waypoints while denied
  (simulation seconds from the denial, ArduPilot within 3 m, then the next
  waypoint; closest true horizontal distance): the cruise entry at 11.3 s,
  left at 11.6 s, 2.6 m; the ridge entry at 65.3 s, left at 66.1 s, 2.3 m;
  the ridge mid at 136.1 s, left at 136.4 s, 4.7 m, while ArduPilot was
  3.2 m off the truth (the waypoints' acceptance radius is 8 m). Then the
  ridge turn after the restore, 0.5 m. No stall, and the vehicle leaves
  each waypoint 0.3-1.0 s after coming within 3 m of it in ArduPilot's
  frame (the two 2 s delays are 0.4 s each at 0.19x), so in truth it comes
  within about the threshold plus the navigation error. With one GoTo per
  waypoint it reached only the ridge mid while denied.
- **Gazebo.** The race showed in Gazebo too on the loaded host: in Gazebo 1
  the cruise entry's first GoTo succeeded at once, and the second flew it.
  Gazebo 1 failed its error limits: 80-110 s into the denial DemNav was
  13-19 m off and the corrections 14-23 m, and the corrections 11-17 m
  again at 130-150 s; the EKF followed them to 31-40 m. That is the
  previous section's failure (its Gazebo 1 and 0.2 m control), larger; the
  doubled GoTo only adds 2 s of hover per waypoint, which puts the vehicle
  a few seconds behind on the same route while denied. Gazebo 4,
  this change alone on a quiet host, passed at 8.1 / 13.2 / 13.9 m with
  0.5 % of the fused fixes beyond 2 sigma, inside the committed runs'
  spread (median 6-8 m, p95 11-20 m, max 12-22 m); Gazebo 2 and 3
  passed at 7.6 / 24.0 / 29.7 and 8.0 / 14.6 / 16.0 m. All four reached the
  cruise and ridge entries while denied (true 0.7-9.8 m at ArduPilot's 3 m)
  and the ridge mid at 147-156 s, around the restore (with one GoTo per
  waypoint, 140-146 s: the pairs' extra hover).
- **Genesis** as with one GoTo per waypoint (2.1 / 3.0 / 3.3 m): the
  cruise entry, the ridge entry and the ridge mid while denied, ArduPilot
  within 3 m of them 12.9, 69.3 and 142.2 s in and the vehicle on to the
  next 2.3-2.6 s later, 1.5-2.3 m from them in truth. No first GoTo of a
  pair succeeded at once.
- **Full mission** (Gazebo, the validator with DemNav, WildNav and a
  correction required, the recorder, no denial): the validation passed
  after 111 s, all 40 GoTos succeeded (the first of a pair at once only at
  the take-off point; every second one at once, the vehicle already there),
  the vehicle came within 0.1 m horizontally and 0.7 m in 3D of every one of
  the 20 waypoints in truth, and the tree ended in SUCCESS (the mission
  container exited 0). No goal was rejected.

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
  committed code, and the runs of "Mission progress and the GoTo
  threshold"); revisit them as runs accumulate, Gazebo's error limits
  first.
