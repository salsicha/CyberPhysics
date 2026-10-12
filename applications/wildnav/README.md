# WildNav ROS 2

WildNav supplies low-rate absolute horizontal corrections by matching a
downward RGB image against cached, georeferenced satellite tiles. It is an
optional complement to DemNav:

- DemNav matches relative depth against terrain elevation.
- WildNav matches visible surface features against satellite imagery.
- `navigation_fusion_node` applies accepted corrections to high-rate MAVROS
  odometry and publishes `/navigation/odometry` for Aerostack.

Fusion weights accepted corrections by their inverse variance (confidence
only decides acceptance). On every accepted fix it hands the autopilot
(`/navigation/correction_fix`) the inverse-variance combination of each
source's latest fix, propagated to the present with the odometry velocity:
ArduPilot's EKF3 gives every GPS_INPUT sample the largest accuracy of the
last few seconds and fuses each as independent, so it cannot weight
interleaved sources itself, and it needs the dense stream (position-only
aiding every few seconds lets its tilt drift). Each sample's variance is
multiplied by n, the number sent in the last `correction_correlation_s`
(60 s): the fixes' errors are correlated and the EKF fuses every sample as
independent, so n (rate x correlation time) samples carry one independent
sample's information, whatever the rate. 60 s is the fixes' measured
integral correlation time against simulator truth: DemNav's errors are
terrain-fixed biases that fade over about a footprint flown (44-47 s in the
lite SITL stack, longer in Gazebo, shorter in Isaac;
tools/gps_denied_sitl_test.md has the measurements), and a faster EKF
response also closes a loop through its own attitude, which body-fixed
cameras use. A correction whose inflated sigma would exceed
`max_correction_sigma_m` (100 m, where EKF3 clips GPS accuracy) is held
back and not counted, so the stream thins out rather than being fused as
more accurate than it is. Sources whose fixes disagree by more than
`correction_consistency_sigma` (5) combined sigma are not combined: the one
farther from the fused position is left out, since a false match diluted
by the combination and sent with that variance would pass the EKF's
innovation gate.

`/navigation/fix` and `/navigation/odometry` carry the fused estimate's
uncertainty: per axis, the exponentially weighted (`fix_error_time_constant_s`,
5 s) mean square of the distance between each accepted fix and the estimate
at its time, plus `fix_variance_weight` (0.5) times the fixes' variance,
plus `dead_reckoning_drift_mps` growth since the last one. While the
autopilot absorbs the corrections the estimate is its odometry, which can
lag the fixes by tens of metres: the fixes' sigma alone was 3-4 times too
small in the lite SITL test (tools/gps_denied_sitl_test.md, "Fused fix
uncertainty"). DemNav and WildNav widen their search by three times this
sigma. Fusion rejects non-finite odometry and corrections,
invalid covariance, and confidence values outside 0–1. An invalid correction cannot update the fused
offset. If an existing offset becomes non-finite, the next valid raw sample
resets it and allows valid corrections to resume without restarting the node.

The original [TIERS WildNav](https://github.com/TIERS/wildnav) demonstration is
an offline SuperPoint/SuperGlue batch processor. This package retains its
image-to-satellite homography approach, but uses cached SIFT descriptors and a
bounded local tile search so it can run on a Raspberry Pi without CUDA.

## Topics

Inputs:

- `/oak1/image_highres`
- `/oak1/image_highres/camera_info`
- `/navigation/fix` as the visual-search prior (fresh within
  `prior_timeout_s`), falling back to a fresh `/drone0/sensor_measurements/gps`
  fix; the search radius widens with the prior's horizontal sigma
  (`search_radius_sigma_scale`, up to `max_search_radius_m`) and so does
  the `max_position_jump_m` gate
- `/navigation/origin` (EKF origin, transient-local): `/wildnav/odometry`
  is relative to it and waits for it; `/wildnav/fix` does not
- `/navigation/odometry` for the camera attitude (interpolated to each
  image stamp; `/clock` maps simulator-time image stamps onto MAVROS wall
  time) and the odometry template
- `/drone0/sensor_measurements/odom` as the fusion dead-reckoning input
- `/demnav/odometry`, `/demnav/confidence`, `/demnav/valid`

Outputs:

- `/wildnav/fix`
- `/wildnav/odometry`
- `/wildnav/confidence`
- `/wildnav/match_count`
- `/wildnav/valid`
- `/navigation/odometry`
- `/navigation/fix`
- `/navigation/correction_source`
- `/navigation/correction_confidence`

## Fix point and covariance

The fix is the vehicle's horizontal position, not the image centre's: the
camera looks down the body -z axis (`camera_mount_yaw_deg`: image top
heading from the nose, counter-clockwise; `camera_stabilized: true` for a
roll/pitch-stabilised camera), so the node finds the pixel looking straight
down from the full attitude at the image stamp and maps it through the
matched image-to-tile homography (`camera_geometry.py`). For a planar
ground patch this is exact without the yaw or the height. The
`max_position_jump_m` gate therefore compares vehicle positions.

The horizontal standard deviation combines the matching error
(`max(match_sigma_floor_m, median reprojection error * tile resolution)`;
the 3 m default floor is for real imagery, georeferenced to a few metres,
and simulators that render the tiles themselves set 1 m), the attitude
uncertainty `attitude_sigma_deg` (EKF and camera mount) times the height
above ground, which the matched scale at the nadir pixel gives, and the
attitude and position change over the stamp uncertainty `stamp_sigma_s`
(a `camera_stabilized` camera stays level, so the body's rates do not
count). `fov_deg` is the fallback field of view without `camera_info`.

## Map cache

Satellite tiles and their feature descriptors are stored under
`/data/wildnav_cache`. The default source is Esri World Imagery and can be
replaced with any XYZ tile endpoint:

```text
https://server.example/tiles/{z}/{x}/{y}.jpg
```

Confirm that the selected imagery provider permits the intended operational
and caching use. Once populated, the cache can be used without internet
access.

Set `WILDNAV_TILE_URL` before starting the Compose stack to select another XYZ
tile source.

WildNav is deliberately low rate. On Raspberry Pi, start with 0.2 Hz, zoom
level 18, a 250 m search radius, and no more than 49 candidate tiles.

## Build-time cache seeding

The Docker image seeds default Golden Gate Park satellite tiles and SIFT descriptors at build time (`37.7694, -122.4862`) into `/data/wildnav_cache`. Override these build args for another test area:

- `SEED_NAV_ASSETS=0` disables build-time seeding.
- `NAV_ASSET_LAT` and `NAV_ASSET_LON` set the cache center.
- `WILDNAV_ZOOM`, `WILDNAV_SEARCH_RADIUS_M`, `WILDNAV_MAX_TILES`, and `WILDNAV_FEATURE_BACKEND` control the seeded tile set.
- `WILDNAV_TILE_URL` selects the XYZ imagery endpoint.

Generated satellite tiles and descriptor files belong in the image or a runtime volume, not in the repository. Confirm the selected imagery provider permits the intended use.
