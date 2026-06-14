
# DemNav

This app cached height maps for a specified area. It then runs a ROS2 node in one of two environments:

1. Arduino App Lab brick 
2. BlueOS extension

The node receives position information, an unscaled depth image, and the latest kinematic information (velocity, acceleration, etc...), it matches the depth image to the geographic height map and produces a new position estimate.

DemNav also fits an affine metric scale to the relative depth values using the
matched DEM window. It publishes:

- `/demnav/fix`: corrected GPS with covariance
- `/demnav/odometry`: metric local correction input for navigation fusion
- `/demnav/metric_depth`: terrain-scaled `32FC1` depth
- `/demnav/points`: metric camera-frame point cloud
- `/demnav/confidence`, `/demnav/depth_scale`, and `/demnav/valid`

Metric output is rejected when terrain correlation or affine-fit support is
insufficient. ArduPilot GPS/EKF remains the fallback localization source.

The cache currently uses SRTM-class elevation data. DemNav rejects matches when
the camera footprint spans fewer than `min_dem_footprint_pixels` DEM cells. At
low altitude this generally requires a higher-resolution terrain source;
changing `dem_resolution_m` cannot create source detail. The current projection
also assumes a near-nadir camera and modest roll/pitch.

## Build-time cache seeding

The Docker image seeds a default Golden Gate Park DEM cache at build time (`37.7694, -122.4862`) into `/data/demnav_cache`. Override these build args for another test area:

- `SEED_NAV_ASSETS=0` disables build-time seeding.
- `NAV_ASSET_LAT` and `NAV_ASSET_LON` set the cache center.
- `DEMNAV_AREA_KM` and `DEMNAV_DEM_RESOLUTION_M` set the generated DEM window.

Generated DEM cache files belong in the image or a runtime volume, not in the repository.
