
# DemNav

DemNav caches a height map (SRTM-class DEM) for an area and runs a ROS 2
node that matches the downward depth image against it. From the depth image,
the vehicle's attitude and a position prior, it produces an absolute position
fix with an honest covariance for the navigation fusion and, through it,
ArduPilot's EKF (see `systems/aerodrone/README.md`).

DemNav also fits an affine metric scale to the relative depth values using the
matched DEM window. It publishes:

- `/demnav/fix`: corrected GPS with covariance
- `/demnav/odometry`: metric local correction input for navigation fusion
- `/demnav/metric_depth`: terrain-scaled `32FC1` depth
- `/demnav/points`: metric camera-frame point cloud
- `/demnav/confidence`, `/demnav/depth_scale`, and `/demnav/valid`

Metric output is rejected when terrain correlation or affine-fit support is
insufficient. ArduPilot GPS/EKF remains the fallback localization source.

## Frames, search prior and gating

- **Origin.** `/demnav/odometry` and `/demnav/pose` are local ENU positions
  about the autopilot's EKF origin, received on `origin_topic`
  (`/navigation/origin`, `geographic_msgs/GeoPointStamped`, reliable +
  transient-local). Until it arrives no local output is published and
  `/demnav/valid` stays false; `/demnav/fix` (lat/lon) is published
  regardless. `initial_lat`/`initial_lon` only pre-load the map (and anchor
  the synthetic world).
- **Search prior.** Each frame is searched about the fused fix on
  `prior_fix_topic` (`/navigation/fix`) while it is newer than
  `prior_timeout_s`, else a fresh raw GPS fix (`status >= STATUS_FIX`),
  else the frame is skipped. The radius is
  `clamp(search_radius_m + search_radius_sigma_scale * sigma,
  search_radius_m, max_search_radius_m)`, sigma being the prior's larger
  horizontal standard deviation.
- **Camera and fix point.** The depth camera looks down the body -z axis;
  with `camera_mount_yaw_deg: 0` the top of the image points at the nose
  (positive values turn it counter-clockwise seen from above). It is
  body-fixed, so banked or pitched it looks h * tan(tilt) away from the
  vehicle; `camera_stabilized: true` declares a roll/pitch-stabilised
  camera that only turns with the heading (the Genesis and
  `satellite_camera_sim` cameras). Each frame is resampled north-up onto
  the DEM grid through its ray geometry (`terrain_match.py`,
  `camera_geometry.py`), using the full odometry attitude interpolated to
  the frame stamp (simulator-time stamps are mapped onto MAVROS wall time
  through `/clock`; a frame without odometry within
  `max_attitude_offset_s` is skipped). The template's anchor is the point
  below the vehicle, so the fix is the vehicle's horizontal position, not
  where the optical axis meets the ground. With `depth_is_metric: true`
  (simulator depth cameras) every pixel is placed at its ground point in
  3D and the template holds the height below the camera there, so neither
  the view's tilt nor the relief displaces it. Relative depth (the OAK-1
  network output) has no scale and goes onto flat ground at the height
  above ground; the ramp (and, beyond 5 deg, the curvature) a tilted view
  of flat ground adds along the view's heading is left out of the template
  and of every DEM window, so a hillside sloping that way can no longer
  match the tilt. `max_attitude_deg` bounds the attitude matched. The
  ground sampling distance comes from `camera_info` (scaled to the depth
  size); `fov_deg` is only the fallback when no calibration is received.
- **Covariance.** The horizontal standard deviation (per axis) combines
  the matching error, the attitude uncertainty `attitude_sigma_deg` (EKF
  and camera mount) times the height above ground, and the attitude and
  position change over the stamp uncertainty `stamp_sigma_s` (and over the
  gap to the odometry sample used, when it could not be interpolated; a
  `camera_stabilized` camera stays level, so the body's rates do not
  count). The
  matching error is the spread of the correlation peak: (1 - score^2) /
  (template cells x the peak's curvature along its flattest direction)
  cells^2, scaled by `match_sigma_scale`, with a floor of
  `match_sigma_floor_cells` DEM cells; plus how far the match moves when
  the template is turned about the point below the vehicle, per
  `heading_sigma_deg` of heading error (metres per degree over a ridge,
  little over a compact feature); at most the look-alike exclusion radius.
  Relative depth adds the relief of the matched DEM window times the
  tangent of the view's edge (half the field of view plus the tilt):
  flat-ground projection displaces relief by that much. The defaults (2.0,
  0.1 cells, 1 deg) are calibrated on Isaac and Gazebo frames with ground
  truth. `attitude_sigma_deg` is the EKF's attitude error: the Isaac and
  Genesis stacks set 0.5 (their EKF's tilt error while GPS-denied, heading
  error under 1 deg); Gazebo's EKF is worse (tilt 0.7-0.9 deg while
  GPS-denied, heading 1-2 deg) and keeps the default 1. The aircraft needs
  its own flight-log calibration.
- **Height above ground.** A fresh, in-range downward rangefinder reading
  (`rangefinder_topic`, `sensor_msgs/Range`) is preferred. Otherwise SRTM
  heights are EGM96 mean sea level, so altitudes are converted first:
  `gps_altitude_datum: ellipsoid` (MAVROS `global_position/global`)
  subtracts the geoid height read from `geoid_file` (GeographicLib
  `egm96-5.pgm`); `msl` uses the altitude as is. Without the geoid file the
  node warns once and the height above ground is off by the local geoid
  height (about -32 m at Mount Tamalpais).
- **Ambiguity.** Besides `min_correlation`, a fix needs
  `min_peak_uniqueness`: the best terrain look-alike more than a footprint
  radius from the peak must leave clearly more depth variance unexplained
  than the peak does. Repetitive or featureless terrain is rejected rather
  than producing a confident fix hundreds of metres off.
- **Map edge.** Search windows that leave the height map are rejected, and
  the map is reloaded around the prior before the widest search window
  would cross its edge (`area_km` should comfortably exceed
  `max_search_radius_m`).

The cache currently uses SRTM-class elevation data. DemNav rejects matches when
the camera footprint spans fewer than `min_dem_footprint_pixels` DEM cells. At
low altitude this generally requires a higher-resolution terrain source;
changing `dem_resolution_m` cannot create source detail. Relative depth is
projected onto flat ground at the height above ground below the vehicle, so
steep relief across an oblique footprint still blurs its template (the
covariance grows with it); metric depth is placed in 3D.

## Where it runs

DemNav is a container image built on the aerostack2 image (ROS 2 Humble):
`salsicha/cyberphysics:demnav-latest` (amd64) in the simulation stacks, and
`demnav-arm64` on the vehicle. The node is CPU-bound Python (NumPy/OpenCV
correlation); it needs no GPU.

- **Today: BlueOS companion computer.** `compositions/blueos_nav_core.yaml`
  runs DemNav with MAVROS, the navigation bridge, WildNav and the fusion on
  the aerodrone's Raspberry Pi 5 with a Blue Robotics Navigator flight
  controller (`hardware/aerodrone.md`), from the arm64 images. The OAK-1
  computes the relative depth on the camera (MegaDepth, `megadepth_ros.py`).
- **Possible later target: Arduino VENTUNO Q** (not tested here). It is an
  edge-AI board announced for robotics; preorders opened in August 2026. It
  pairs a Qualcomm Dragonwing IQ-8275 with an STM32H5 MCU:
  - Linux side: octa-core CPU (2.35 GHz Gold Prime cores) and 16 GB
    LPDDR5. It runs Ubuntu (preloaded; Debian announced) and Arduino App
    Lab, whose "bricks" are containers.
  - AI: a Hexagon NPU (up to 40 dense TOPS) and an Adreno 623 GPU.
  - MCU: STM32H5F5 Cortex-M33 at 250 MHz, running the Arduino core on
    Zephyr, reached from Linux over an RPC bridge.
  - I/O: three MIPI-CSI camera connectors, 2.5 Gb Ethernet, Wi-Fi 6, CAN-FD.

  Why it fits DemNav:
  - 16 GB holds the DEM and tile caches and the whole navigation core
    (MAVROS, the bridge, DemNav, WildNav, the fusion) on one board.
  - The NPU could later take work DemNav and WildNav now do elsewhere or on
    the CPU: the monocular depth network that runs on the OAK-1, or learned
    features for WildNav. Neither exists here yet.
  - MIPI-CSI would allow a camera without the OAK-1.

  It would run as a companion computer next to the flight controller, not
  instead of it: ArduPilot stays on the flight controller, reached by MAVROS
  over serial or UDP (`BLUEOS_FCU_URL`). The MCU plays no part in DemNav.

  The likely path is the same arm64 containers as BlueOS
  (`CYBERPHYSICS_TAG=arm64` with `blueos_nav_core.yaml`), on the board's
  Ubuntu or as an App Lab brick. The image's userland
  (`ros:humble-ros-base`, Ubuntu 22.04) does not depend on the host's Ubuntu
  release. Still to verify on the board:
  - that Docker (or the App Lab brick format) runs these images;
  - per-frame match time at `max_search_radius_m` on its CPU (the Pi 5 is
    the reference);
  - the OAK-1 over USB 3, or a MIPI camera with an on-board depth model;
  - the MAVROS link to the flight controller.

  Packaging DemNav as an App Lab brick is not done.
- **Not the Arduino UNO Q.** Its quad Cortex-A53 with 2–4 GB RAM is too small
  for the navigation core. The UNO Q is used for the turret instead
  (`hardware/turret.md`).

## Build-time cache seeding

The Docker image seeds a default Golden Gate Park DEM cache at build time (`37.7694, -122.4862`) into `/data/demnav_cache`. Override these build args for another test area:

- `SEED_NAV_ASSETS=0` disables build-time seeding.
- `NAV_ASSET_LAT` and `NAV_ASSET_LON` set the cache center.
- `DEMNAV_AREA_KM` and `DEMNAV_DEM_RESOLUTION_M` set the generated DEM window.

Generated DEM cache files belong in the image or a runtime volume, not in the repository. The simulator-neutral georeferenced terrain builder and collision generator live in `scripts/`; simulator-specific loading remains with each simulator application.
