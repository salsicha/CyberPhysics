<a href="">
  <img src="https://media.githubusercontent.com/media/salsicha/CyberPhysics/main/icon.png"
    height="70" align="right" alt="CyberPhysics" />
</a>

# CyberPhysics

CyberPhysics is a container-first workspace for robotics, simulation, and ML applications. It keeps reusable application images in `applications/`, robot-specific calibration and configuration in `systems/`, and runnable multi-container stacks in `compositions/`.

The repository is designed for Ubuntu 24.04 and ROS 2 Jazzy unless an application README says otherwise.

## Repository Layout

- `applications/`: reusable Dockerized tools such as ROS 2, Gazebo, Isaac Sim integrations, nvblox, Depth Anything, NiceGUI, DemNav, WildNav, and robot-specific bridges.
- `systems/`: hardware or simulation setups, including URDF/Xacro files, calibration, controller YAML, topic conventions, and robot-specific navigation config.
- `compositions/`: Docker Compose files that combine applications into working systems.

Keep large generated assets out of git. Docker images may download model weights, simulator assets, or example datasets during build; runtime maps should be downloaded or generated outside the repository.

## Install

```bash
./install.sh
```

## Build Images

Build from the `applications/` directory with `make build_<app_name>`:

```bash
make -C applications build_ros2
make -C applications build_racecarneo
make -C applications build_depthanything
```

For ARM64 targets such as Raspberry Pi or Jetson:

```bash
cd applications
make target_arm64 build_ros2
```

## Run A Composition

```bash
docker compose -f compositions/marimo.yaml up
```

The RACECAR Neo simulation and navigation stack is a representative full pipeline:

```bash
docker compose -f compositions/racecarneo.yaml up
```

That stack runs the headless MIT RACECAR Neo simulator, publishes ROS 2 camera/lidar/IMU/odometry topics, estimates metric monocular depth with Depth Anything, builds nvblox TSDF/ESDF slices, runs Nav2, and serves a NiceGUI waypoint interface at `http://localhost:8080`.

## Observe ROS Data

Foxglove can be launched through a compose stack or on the host:

```bash
docker compose -f compositions/foxglove.yaml up
foxglove-studio
```

For nvblox visualization in Foxglove, install NVIDIA's Foxglove extension on the host:

```bash
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
cd isaac_ros_nvblox/nvblox_foxglove
npm install
npm run local-install
```

The extension expects Node/npm compatible with NVIDIA's instructions; Node 20.10.0 has worked for this setup.

## Development Conventions

- Put reusable software in `applications/<name>`.
- Put robot-specific configuration in `systems/<system>`.
- Put runnable stacks in `compositions/<stack>.yaml`.
- Prefer headless simulation containers that publish ROS 2 topics.
- Do not commit downloaded models, simulator assets, maps, logs, or bags.

## Real-Time Notes

Pin latency-sensitive containers to dedicated CPU cores when needed:

```bash
docker run --cpuset-cpus="0,1" -it your_image your_command
```

```yaml
services:
  your_service:
    image: your_image
    cpuset: "0,1"
```

Reserve the cores on the host before launching containers:

```bash
cset shield --cpu 0,1 --kthread on
```

Restore the host when finished:

```bash
cset shield --reset
```
