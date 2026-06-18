# nvblox

This application builds an Isaac ROS nvblox image for ROS 2 Jazzy. It provides TSDF/ESDF reconstruction and the Nav2 costmap plugin used by RACECAR Neo.

Primary references:

- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox
- https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html

## Build

```bash
make -C applications build_nvblox
```

The nvblox image is large because NVIDIA's Debian packages pull CUDA, TensorRT, and Isaac ROS dependencies. The RACECAR Neo composition bind-mounts `applications/nvblox/racecarneo_nvblox.launch.py`, so local launch changes do not require rebuilding the image every time.

## RACECAR Neo Launch

`racecarneo_nvblox.launch.py` starts `nvblox::NvbloxNode` in a component container and remaps RACECAR topics into the camera names expected by nvblox:

- `camera_0/depth/image` -> `/depth_anything/depth/image`
- `camera_0/depth/camera_info` -> `/camera/color/camera_info`
- `camera_0/color/image` -> `/camera/color/image_raw`
- `camera_0/color/camera_info` -> `/camera/color/camera_info`

The RACECAR-specific mapper parameters live in `systems/racecarneo/config/nvblox.yaml`.

Run through the full stack:

```bash
docker compose -f compositions/racecarneo.yaml up racecarneo_nvblox
```

## Other Examples

Run the bundled quickstart bag headlessly:

```bash
docker compose --profile quickstart -f compositions/nvblox.yaml run --rm nvblox_quickstart
```

Run against a RealSense camera:

```bash
docker compose -f compositions/nvblox.yaml up realsense nvblox
```

Both examples launch without RViz. Use Foxglove or a separate RViz container for visualization.
