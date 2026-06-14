# WildNav ROS 2

WildNav supplies low-rate absolute horizontal corrections by matching a
downward RGB image against cached, georeferenced satellite tiles. It is an
optional complement to DemNav:

- DemNav matches relative depth against terrain elevation.
- WildNav matches visible surface features against satellite imagery.
- `navigation_fusion_node` applies accepted corrections to high-rate MAVROS
  odometry and publishes `/navigation/odometry` for Aerostack.

The original [TIERS WildNav](https://github.com/TIERS/wildnav) demonstration is
an offline SuperPoint/SuperGlue batch processor. This package retains its
image-to-satellite homography approach, but uses cached SIFT descriptors and a
bounded local tile search so it can run on a Raspberry Pi without CUDA.

## Topics

Inputs:

- `/oak1/image_highres`
- `/oak1/image_highres/camera_info`
- `/navigation/odometry` as the visual-search position prediction
- `/drone0/sensor_measurements/odom` as the fusion dead-reckoning input
- `/drone0/sensor_measurements/gps`
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
