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

For Jetson Orin, use `make -C applications PLATFORM=arm64 build_nvblox`.
This selects Isaac ROS 4.6's `noble-jetpack` ARM64 repository, the SBSA
CUDA repository and Jetson Linux 39.2's VPI packages. It requires JetPack
7.2; desktop builds retain Isaac ROS 4.4. See [Jetson GPU images](../jetson.md)
for the complete build and runtime checks. ARM64 builds omit the recorded
quickstart bags by default (`DOWNLOAD_EXAMPLE_ASSETS=1` includes them).

## RACECAR Neo Launch

`racecarneo_nvblox.launch.py` starts `nvblox::NvbloxNode` in a component container and remaps RACECAR topics into the camera names expected by nvblox (the launch file's defaults):

- `camera_0/depth/image` -> `/depth_anything/depth/image` (`depth_image_topic`)
- `camera_0/depth/camera_info` -> `/camera/color/camera_info` (`depth_camera_info_topic`)
- `camera_0/color/image` -> `/camera/color/image_raw` (`color_image_topic`)
- `camera_0/color/camera_info` -> `/camera/color/camera_info` (`color_camera_info_topic`)

Each topic is a launch argument. The simulator compositions map the
simulated camera's depth (`/camera/depth/image_rect_raw`);
`compositions/racecarneo_hardware.yaml` maps the RealSense's depth aligned
to its colour camera (`/camera/aligned_depth_to_color/image_raw`, 16UC1
millimetres, with `/camera/aligned_depth_to_color/camera_info`), which
nvblox_ros reads as it is (16UC1 millimetres or 32FC1 metres).

The RACECAR-specific mapper parameters live in `systems/racecarneo/config/nvblox.yaml`;
`overrides_file` loads a second file over it: `nvblox_hardware.yaml` on the
car and `systems/racecar_turret/config/nvblox_isaac.yaml` in Isaac (dynamic
mode).

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

Both examples launch without RViz. Use Foxglove (`foxglove_bridge` at `ws://localhost:8765`) or a separate RViz container for visualization.
