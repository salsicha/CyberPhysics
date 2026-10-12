# SpaceROS

[Space ROS](https://github.com/space-ros/space-ros) with the
[Curiosity rover demo](https://github.com/space-ros/demos/tree/main/curiosity_rover)
built in, plus the CyberPhysics lunar rover simulator (`scripts/`).

## What is in the image

- Base: `osrf/space-ros:jazzy-2026.07.0` (`SPACEROS_RELEASE` in the
  Dockerfile). The space-ros/demos checkout uses the same tag, sparse-checked
  out to `/demos/curiosity_rover`.
- Upstream's `build.sh`/`run.sh` only run `docker compose`, so the Dockerfile
  repeats the steps of the demo's two images itself:
  - `Dockerfile` (Space ROS side): `control_msgs` and Cyclone DDS from the ROS
    apt repository.
  - `Dockerfile.GUI` (Gazebo side): Gazebo Harmonic through `ros-jazzy-ros-gz`
    (the Jazzy `gz_*_vendor` packages, not a second copy from
    packages.osrfoundation.org), `xacro`, `sdformat_urdf`, `rviz2`.
  - `rosdep install` for the rest of the package.xml dependencies
    (`ros2_control`, `gz_ros2_control`, `ros2_controllers`).
  - One `colcon build` of `curiosity_description`, `curiosity_gazebo` and
    `curiosity_rover_demo` into `/demos/curiosity_rover/install`.
- `/entrypoint.sh` sources ROS 2 Jazzy (`/opt/ros/jazzy`, apt), then Space ROS
  (`/opt/ros/spaceros`), then the demo workspace. Interactive shells source
  the same environment through `~/.bashrc`.
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp`. The base originally hard-wired Cyclone.
  `enable_zenoh.sh` rebuilds its RMW dispatcher, C/C++/Python client libraries
  and generated message dispatchers from the image's exact source manifest.
  Both introspection and Fast CDR serialization support remain available.
  The Compose graphs use the [shared middleware settings](../../systems/ros2/README.md),
  with explicit DDS fallback. This is a local source rebuild, not an upstream
  qualification claim.
- `/launch/curiosity_sim.launch.py` (from `launch/`) is the default command.

Build:

```bash
cd applications
REPOSITORY=salsicha/cyberphysics PLATFORM=amd64 APP=spaceros ./_build_it.sh
```

## Curiosity rover demo

```bash
docker compose -f compositions/spaceros.yaml up                      # Gazebo GUI on $DISPLAY
SPACEROS_GUI=false docker compose -f compositions/spaceros.yaml up   # headless
```

If the GUI can't open the display, run `xhost +local:` first.

`curiosity_sim.launch.py` does what upstream's `curiosity_gazebo.launch.py`
and `mars_rover.launch.py` do, in one launch:

- the Mars world in Gazebo, the rover spawned from xacro, and
  `robot_state_publisher`;
- the bridges for `/clock`, `/scan`, `/image_raw` and
  `/model/curiosity_mars_rover/odometry`, plus the odometry TF publisher;
- the ros2_control controllers: `joint_state_broadcaster`, arm, mast, wheel
  velocity, steering and suspension;
- the Space ROS demo nodes `move_arm`, `move_mast`, `move_wheel` and
  `run_demo`.

| Argument | Default | |
| --- | --- | --- |
| `gui` | `true` | `false` runs only the Gazebo server (`-s --headless-rendering`). The camera and gpu_lidar still render, through EGL, so no X server is needed. |
| `demo` | `true` | `false` leaves out the `curiosity_rover_demo` nodes |
| `gz_verbosity` | `3` | `gz sim -v` level (upstream uses 4) |

Differences from upstream:

- The controllers start with `controller_manager spawner` and a 60 s switch
  timeout. Upstream uses `ros2 control load_controller` behind a
  `set_hardware_component_state` workaround. The 5 s default timeout of
  `load_controller` failed intermittently here: activating
  `joint_state_broadcaster` takes 6–8 s while the sensors start. Jazzy's
  ros2_control activates the `GazeboSystem` hardware without the workaround.
- The robot is spawned from the `robot_description` topic.
- RViz is installed, but not upstream's MoveIt and Nav2 RViz plugins. The
  Curiosity demo doesn't use them.

Drive the rover with the demo's services (all `std_srvs/srv/Empty`):

```bash
docker exec spaceros /entrypoint.sh ros2 service call /move_forward std_srvs/srv/Empty
#   /turn_left /turn_right /move_stop /open_arm /close_arm /mast_open /mast_close /mast_rotate
docker exec -it spaceros bash        # shell with the environment sourced
```

Upstream's README lists `/stop`, but the service is actually `/move_stop`.

Without a display, run the image directly:

```bash
docker run --rm --network host -e ROS_DOMAIN_ID=227 salsicha/cyberphysics:spaceros-latest \
  ros2 launch /launch/curiosity_sim.launch.py gui:=false
```

Performance: the terrain's collision mesh is a 20 MB STL, so physics runs
well below real time. Measured real-time factor was about 0.05 on a 20-core
CPU, with or without a GPU. Rendering falls back to Mesa (llvmpipe) on the
CPU. To render on an NVIDIA GPU instead (EGL through the NVIDIA container
toolkit; the "libEGL warning: egl: failed to create dri2 screen" lines come
from Mesa probing first and are harmless), add a compose override:

```yaml
services:
  spaceros:
    gpus: all
    environment:
      - NVIDIA_DRIVER_CAPABILITIES=all
```

Stopping takes about 5 s: the Gazebo server ignores SIGINT until
`ros2 launch` escalates to SIGTERM. The image's `STOPSIGNAL` is `SIGINT`.

## Lunar Rover Simulation

The image also includes a lightweight ROS 2 lunar rover simulator and waypoint
autonomy node for the `systems/lunar_rover` south-pole traverse scenario:

```bash
docker compose -f compositions/lunar_rover_sim.yaml up
```

The scenario models crater-rim slope, loose regolith slip, low sun/shadow
illumination, thermal swing, battery draw, and solar recovery so the autonomy
stack can complete a realistic prospecting traverse without manual commands.
`compositions/lunar_rover_hardware.yaml` is the runtime skeleton for drivers
on a physical rover.
