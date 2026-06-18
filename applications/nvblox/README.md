# Nvblox

This app builds the current Isaac ROS nvblox Debian packages for ROS 2 Jazzy and downloads the matching NVIDIA quickstart assets during image build. Assets are stored in the image under `/workspace/isaac_ros_assets`; generated maps are not committed to the repo.

Primary references:
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox
- https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html

Build:

```bash
make -C applications build_nvblox
```

Run the bundled quickstart rosbag headlessly:

```bash
docker compose --profile quickstart -f compositions/nvblox.yaml run --rm nvblox_quickstart
```

Run against a RealSense camera:

```bash
docker compose -f compositions/nvblox.yaml up realsense nvblox
```

Both compose paths launch nvblox with `run_rviz:=False`. Use Foxglove or a separate RViz container for visualization.
