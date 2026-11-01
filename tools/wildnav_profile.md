# WildNav lighter matching profile

WildNav matches each downward still against the cached satellite tiles
about the navigation prior (`applications/wildnav/README.md`). On the HIL
bench Pi 5 (`tools/aerodrone_hil_test.md`, "2. GPS-denied") the match took
8.0 s (median) against 1.2 s on the workstation. The Pi was throttled at
85 °C, and WildNav's bursts took up to 2.1 of its 4 cores beside MAVROS,
DemNav, Aerostack2 and ArduPilot. The corrections came 8-11 s apart from
stills 8-11 s old, and the GPS-denied navigation failed.

The lighter profile runs on the companion computers: every BlueOS stack
through `compositions/blueos_nav_core.yaml` and the HIL Pi through
`tools/aerodrone_hil_pi.yaml`. The workstation simulations keep matching
every candidate tile unless the variables are set.

| Parameter | Profile | Node default | Variable |
|---|---|---|---|
| `match_stop_confidence` | 0.5 | 0 (every tile) | `WILDNAV_STOP_CONFIDENCE` |
| `opencv_threads` | 2 | -1 (every core) | `WILDNAV_THREADS` |
| `memory_cache_tiles` | 256 | 256 | (launch argument) |

- **Stop early.** The candidate tiles are tried nearest the prior first.
  A tile matched with at least `match_stop_confidence`, past the
  match-count, inlier, inlier-ratio and footprint gates, ends the search.
  The jump gate still applies to the result. Confidence is
  `min(1, inliers / 40) * inlier ratio * exp(-median reprojection / 8 px)`,
  so 0.5 needs at least 20 inliers, half of the ratio-tested matches as
  inliers, and a few pixels of reprojection error.
- **Two threads.** OpenCV's pool is capped at two of the Pi's four cores,
  which leaves the others to the autopilot and the rest of the stack.
- **Features in memory.** Each match used to decode every candidate
  tile's JPEG and decompress its descriptor `.npz`, which cost as much as
  the matching itself. Now they stay in memory, least recently used out
  first: 256 tiles, about 0.8 MB each for 1500 SIFT descriptors. This is
  the node's default everywhere and gives the same fixes.

The search radius, the zoom and the tile count stay per site and per
vehicle (the site and vehicle env files).

## Benchmark

`tools/wildnav_benchmark.py` runs the node's own matcher
(`WildNavNode._run_match`, with real ROS messages and parameters) on
stills made the way the Gazebo stack's synthetic downward camera makes them.
The camera crops 128 x 96 px of the cached tile at the vehicle, turns it to
the heading and scales it to 640 x 480 (`tests/test_wildnav_benchmark.py`
holds the stills equal to the simulator's, pixel for pixel). The stills
are at random positions about the site, and the prior is off the truth by
its stated sigma. Per profile it reports:
- the time from still to fix;
- the CPU time of the match across all threads;
- the stages (`describe` is the still's SIFT, `tile_features` the tiles'
  features, `knn_match` the descriptor matching);
- the fix error against the truth.

To run it on the Pi, take the arm64 image and the HIL tile cache, with
nothing else running (the HIL test's resource sampler shows the load):

```bash
docker run --rm --network none -e OPENCV_LOG_LEVEL=ERROR \
  -v /home/pi/cyberphysics/generated/aerodrone_hil/pi/wildnav_cache:/data/wildnav_cache:ro \
  -v /home/pi/cyberphysics/tools:/tools:ro salsicha/cyberphysics:wildnav-arm64 \
  python3 /tools/wildnav_benchmark.py --frames 20 --profile site --profile site_lite
```

The aircraft profiles use the image's seeded zoom-18 tiles:
`--site 37.7694,-122.4862 --area-m 250 --profile aircraft --profile aircraft_lite`.
A profile can also be `name=base:site,param:value,...`.

### Results (2026-11-01, bench Pi 5)

20 stills per profile, seed 1, prior sigma 10 m. "Before" ran the image
built before this change. The Pi ran nothing else and was not throttled:
ARM clock 2.4 GHz, 70-82 °C. Median / p95 / max.

| Profile | Tiles | Still to fix (s) | CPU (core-s) | Stages, median (s) | Fixes | Error (m) |
|---|---|---|---|---|---|---|
| Simulated site, before (`site`) | 121 at zoom 17 | 1.35 / 1.82 / 1.83 | 3.65 / 4.90 / 5.20 | describe 0.10, tile features 0.37, matching 0.83 | 18 / 20 | 0.32 / 0.41 / 0.43 |
| Simulated site, lighter (`site_lite`) | same | 0.14 / 0.84 / 0.97 | 0.21 / 1.57 / 1.80 | describe 0.11, tile features 0.00, matching 0.01 | 18 / 20 | 0.32 / 0.41 / 0.43 |
| Aircraft, before (`aircraft`) | 49 at zoom 18 | 0.49 / 0.70 / 0.73 | 1.13 / 1.86 / 2.05 | describe 0.09, tile features 0.15, matching 0.21 | 18 / 20 | 0.16 / 0.21 / 0.21 |
| Aircraft, lighter (`aircraft_lite`) | same | 0.13 / 0.20 / 0.22 | 0.19 / 0.33 / 0.36 | describe 0.10, tile features 0.00, matching 0.01 | 18 / 20 | 0.16 / 0.21 / 0.21 |

- **Every profile gave the same fixes.** The tile nearest the prior that
  matches is the one the full search picked.
- **The p95 and max are the stills with no fix.** Those still try every
  candidate tile, on two threads. A smaller base search radius bounds that
  too: with 250 m at the simulated site (25 tiles) the worst case was
  0.24 s and 0.42 core-s. It stays per site, since the radius also widens
  with the prior's sigma.
- **Two threads against four** cost a little latency on the stills that
  stop early (0.14 against 0.12 s) and save CPU (0.22 against 0.27 core-s).
- **The benchmark alone** took the idle Pi from 67 to 83 °C under the old
  matcher.

On the workstation (i9-13900H, 24 stills, seed 7) the full search and the
early stop gave identical fixes (19 / 24, 0.31 / 0.37 / 0.42 m), with a
median CPU of 2.7 against 0.2 core-s, for each of these priors:
- 50 m off, stating 50 m;
- 150 m off, stating 150 m;
- 100 m off, stating 10 m: an overconfident prior, whose search was only
  280 m.

No wrong tile passed the gates first in any of them.

The stills with no fix are the simulator's camera, not WildNav. Its crop
is clamped inside the vehicle's tile, so near a tile edge part of the
still is the border pixel smeared (`cv2.BORDER_REPLICATE`), and the
neighbouring tile never appears. All the misses had the vehicle within
about 50 px (47 m) of a tile edge, and the full search missed the same
stills.

## Limits

- **Best case.** The simulated stills are the cached tiles themselves, so
  the matches are near-perfect (inliers in the hundreds, confidence about
  1).
- **Real imagery** (an OAK-1 still against Esri imagery of another season
  and sun) matches with fewer inliers. The early stop then ends the search
  later, or not at all, and the cost falls back towards the full search's.
  That search is still on two threads and without the disk reads.
- **Check on the aircraft:** the fix log now ends with
  `<tiles tried>/<candidates> tiles in <s> s`. Check the threshold on the
  first flights' logs before lowering it.

## Flight checks (2026-11-01)

`tools/aerodrone_hil_test.md`, section "7. GPS-denied with the lighter
WildNav profile", has the full tables.

- **HIL, WildNav on the Pi:**
  - the match took 0.2-0.5 s on the loaded Pi, usually on the first of 121
    tiles; it took 8 s before;
  - camera to fix was 1.9 / 3.8 / 6.3 s, mostly the still's age over
    Wi-Fi;
  - the Pi's CPU fell from 73 to 45 % (median) and WildNav's container from
    130 to 51 % of a core;
  - the throttled share of samples fell from 87 to 57 %.

  The GPS-denied navigation still fails there, for reasons other than
  WildNav's compute.
- **All on the workstation** (`tools/aerodrone_hil_test.sh control
  gps_denied` with `WILDNAV_STOP_CONFIDENCE=0.5 WILDNAV_THREADS=2`): pass.
  - fused 5.3 / 8.3 / 9.6 m while denied, against 7.1 / 11.5 / 12.7 m in
    the earlier control without the profile;
  - WildNav's fixes 2.0 / 3.1 / 3.6 m;
  - camera to fix 0.30 s median, against 1.19 s.
