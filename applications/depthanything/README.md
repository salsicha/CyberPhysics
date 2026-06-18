# Depth Anything

This application builds a ROS 2 Jazzy node for Depth Anything V2 metric monocular depth. It is used by the RACECAR Neo stack to turn the simulator RGB camera stream into a metric depth image and optional point cloud.

The Docker image downloads the upstream Depth-Anything-V2 repository and the metric VKITTI small checkpoint during image build. Model weights are not stored in this repository.

## Build

```bash
make -C applications build_depthanything
```

## Topics

Default subscriptions:

- `/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/color/camera_info` (`sensor_msgs/CameraInfo`)

Default publications:

- `/depth_anything/depth/image` (`sensor_msgs/Image`, `32FC1`, meters)
- `/depth_anything/points` (`sensor_msgs/PointCloud2`)

## Launch

```bash
docker run --rm --network=host cyberphysics/depthanything:latest
```

Or as part of the RACECAR Neo stack:

```bash
docker compose -f compositions/racecarneo.yaml up racecarneo_depthanything
```

Useful launch parameters:

- `image_topic`
- `camera_info_topic`
- `depth_topic`
- `pointcloud_topic`
- `encoder` (`vits` by default)
- `metric_dataset` (`vkitti` by default)
- `max_depth`
- `publish_pointcloud`
- `pointcloud_stride`

## Notes

The VKITTI metric model is a better default for outdoor/vehicle-style scenes than indoor-only metric checkpoints. For hardware deployment, validate scale and failure modes against real camera data before using the output for safety-critical planning.
