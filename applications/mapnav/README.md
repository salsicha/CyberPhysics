
# MapNav

This app cached height maps for a specified area. It then runs a ROS2 node in one of two environments:

1. Arduino App Lab brick 
2. BlueOS extension

The node receives position information, an unscaled depth image, and the latest kinematic information (velocity, acceleration, etc...), it matches the depth image to the geographic height map and produces a new position estimate.

MapNav also fits an affine metric scale to the relative depth values using the
matched DEM window. It publishes:

- `/mapnav/fix`: corrected GPS with covariance
- `/mapnav/odometry`: metric local odometry for the Aerostack2 estimator
- `/mapnav/metric_depth`: terrain-scaled `32FC1` depth
- `/mapnav/points`: metric camera-frame point cloud
- `/mapnav/confidence`, `/mapnav/depth_scale`, and `/mapnav/valid`

Metric output is rejected when terrain correlation or affine-fit support is
insufficient. ArduPilot GPS/EKF remains the fallback localization source.

The cache currently uses SRTM-class elevation data. MapNav rejects matches when
the camera footprint spans fewer than `min_dem_footprint_pixels` DEM cells. At
low altitude this generally requires a higher-resolution terrain source;
changing `dem_resolution_m` cannot create source detail. The current projection
also assumes a near-nadir camera and modest roll/pitch.
