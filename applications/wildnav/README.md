# WildNav ROS 2

WildNav supplies low-rate absolute horizontal corrections by matching a
downward RGB image against cached, georeferenced satellite tiles. It is an
optional complement to MapNav:

- MapNav matches relative depth against terrain elevation.
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
- `/mapnav/odometry`, `/mapnav/confidence`, `/mapnav/valid`

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
