# Unreal Engine

Runs **your own** packaged Unreal Engine (UE) project headless on an NVIDIA GPU
and connects it to ROS 2 Jazzy. Unreal Engine itself is not in this repository
or the image: the engine, and any project built with it, needs your Epic Games
account, so you bring the packaged build and the container runs it.

The image (`salsicha/cyberphysics:unrealengine-latest`, built on the `ros2`
image) contains:

- `/server/run_server.sh`: finds the packaged project in `/server/build` and
  runs it off-screen on the GPU as the non-root user `unrealuser` (UE refuses
  to run as root)
- rosbridge_server for projects that use the ROSIntegration plugin
- `/client/example.py`: a minimal ROS 2 client for the project's topics
- the Vulkan ICD for the NVIDIA driver, and `NVIDIA_DRIVER_CAPABILITIES` with
  `graphics`

## What you provide

1. **A packaged Linux build of your UE project with a ROS plugin.** In the
   editor: *Platforms > Linux > Package Project* (on Windows, install the
   Linux cross-compile toolchain first). The output folder (`Linux/`, or
   `LinuxNoEditor/` in UE4) contains `<Project>.sh`, `<Project>/Binaries/Linux/`,
   `<Project>/Content/` and `Engine/`. The project needs one of:
   - [rclUE](https://github.com/rapyuta-robotics/rclUE): native ROS 2 inside
     UE. Its topics appear directly on the ROS 2 graph: nothing else is
     needed beyond the same `ROS_DOMAIN_ID` (the compose files set it on every
     service, default 0). The launcher removes the image's `/opt/ros` paths
     from the engine's environment so rclUE uses the ROS 2 libraries it ships.
   - [ROSIntegration](https://github.com/code-iai/ROSIntegration): talks to
     rosbridge. In the project's ROSIntegration game instance set
     `ROSVersion` = 2, `ROSBridgeServerProtocol` = `ws`,
     `ROSBridgeServerHosts` = `127.0.0.1` and `ROSBridgeServerPorts` = 9090.
     The `ue_rosbridge` service listens there in BSON mode, which
     ROSIntegration always uses. The image patches rosbridge_server so BSON
     arrives over websocket: stock ROS 2 rosbridge decodes binary frames as
     text ([rosbridge_suite#718](https://github.com/RobotWebTools/rosbridge_suite/issues/718)).
2. **An NVIDIA GPU, its driver and the
   [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)**
   on the host. UE renders through Vulkan even when off-screen.

## Where to put the build

Copy the packaged folder into `applications/unrealengine/server/build/`
(git ignores everything there except `.gitkeep`, and the Docker build leaves it
out of the image):

```
applications/unrealengine/server/build/
└── Linux/
    ├── MyProject.sh
    ├── MyProject/Binaries/Linux/...
    └── Engine/...
```

or leave it where it is and pass its absolute path in `UE_BUILD_DIR`. It is
bind-mounted at `/server/build`. The launcher looks up to two folders down for
a `<Project>.sh` that has a `<Project>/Binaries/Linux/` folder next to it. If
the folder holds more than one project, or the layout is different, set
`UE_PROJECT_SCRIPT` (absolute, or relative to the build folder).

The container user is uid 1000. UE writes logs and config under
`<Project>/Saved/`, so the build folder must be readable and writable by that
uid (it is if your host user is uid 1000; otherwise `chown -R 1000` the folder).

## Start

```bash
make -C applications build_unrealengine     # or pull the image
./start.sh unrealengine
UE_BUILD_DIR=/abs/path/to/Linux UE_MAP=/Game/Maps/Warehouse ./start.sh unrealengine
```

This starts `compositions/unrealengine.yaml`: `ue_server` (one GPU reserved,
host networking), `ue_rosbridge` (ws://localhost:9090) and `ue_client`. With no
build mounted, `ue_server` exits with a message that says what to put where.
`docker compose -f compositions/unrealengine.yaml down` stops the stack: the
launcher passes the stop on to the engine as SIGTERM, and the client sends one
zero command.

Server settings (environment variables, all optional; set them on the host
before `./start.sh`):

| Variable | Default | UE argument |
| --- | --- | --- |
| `UE_BUILD_DIR` | `applications/unrealengine/server/build` | host folder mounted at `/server/build` |
| `UE_PROJECT_SCRIPT` | the single project found | which `<Project>.sh` to run |
| `UE_MAP` | the project's default map | map, e.g. `/Game/Maps/Warehouse` |
| `UE_RES_X`, `UE_RES_Y` | 1280, 720 | `-ResX=`, `-ResY=` |
| `UE_PORT` | UE default (7777) | `-port=`: UE's own network (game) port, not the ROS one |
| `UE_GRAPHICS_ADAPTER` | UE's choice | `-graphicsadapter=` |
| `UE_RENDER_OFFSCREEN` | 1 | `-RenderOffScreen`; 0 opens a window (`-windowed`) on `$DISPLAY`, after `xhost +local:` |
| `UE_NOSOUND` | 1 | `-nosound` |
| `UE_EXTRA_ARGS` | | anything else, split on spaces, e.g. `-ForceRes -benchmark` |

The launcher always adds `-unattended -stdout -FullStdOutLogOutput` (no dialogs,
full log in `docker logs`) and reports the GPU it sees, warning if there is
none. Without compose:

```bash
docker run --rm --gpus all --network host -v /abs/path/to/Linux:/server/build \
    -e UE_MAP=/Game/Maps/Warehouse salsicha/cyberphysics:unrealengine-latest
```

rosbridge: `ROSBRIDGE_PORT` (9090) and `ROSBRIDGE_BSON_ONLY` (true; set it to
false for JSON clients such as roslibjs or other UE websocket plugins).
If you only use rclUE, leave rosbridge out:
`./start.sh unrealengine --scale ue_rosbridge=0`.

## Point the client at your topics

`client/example.py` subscribes to a `sensor_msgs/Image` topic and a pose topic
(`nav_msgs/Odometry` or `geometry_msgs/PoseStamped`), logs their rate, header
stamp latency, image size and last position every 2 s, and publishes a
`geometry_msgs/Twist` at 10 Hz (zero by default). The compose files fill its
parameters from:

| Variable | Default | Parameter |
| --- | --- | --- |
| `UE_IMAGE_TOPIC` | `/ue/camera/image_raw` | `image_topic` |
| `UE_POSE_TOPIC` | `/ue/odom` | `pose_topic` |
| `UE_POSE_TYPE` | `odometry` | `pose_type` (`odometry` or `pose`) |
| `UE_CMD_TOPIC` | `/ue/cmd_vel` | `cmd_topic` |
| `UE_CMD_LINEAR_X` | 0.0 | `linear_x` (m/s) |
| `UE_CMD_ANGULAR_Z` | 0.0 | `angular_z` (rad/s) |

```bash
UE_IMAGE_TOPIC=/robot/camera/image_raw UE_POSE_TOPIC=/robot/odom \
UE_CMD_TOPIC=/robot/cmd_vel UE_CMD_LINEAR_X=0.2 ./start.sh unrealengine
```

The client is mounted from `applications/unrealengine/client`, so edits apply
on the next start. It also takes `cmd_rate_hz` (0 disables commands),
`report_period_s` and `run_duration_s` (`python3 example.py --ros-args -p ...`).
Latency is only meaningful when the project stamps messages with the same
clock as the client: wall time, or simulation time on `/clock` with
`-p use_sim_time:=true`. When the stamps are clearly on another clock (more
than 10 s apart) the client says so instead of printing a latency.

## k3s

`compositions/ue_k3s.yaml` is the same stack without the X11 plumbing, with
the client baked into the image instead of mounted, and with
[kompose](https://kompose.io) labels (`IfNotPresent` pulls, `hostPath`
volumes) for a single-node k3s that uses the local Docker images
(`./install.sh --with-k3s` installs k3s with `--docker`):

```bash
UE_BUILD_DIR=/abs/path/to/Linux kompose convert -f compositions/ue_k3s.yaml -o ue_k3s-manifests.yaml
```

Check the generated manifests before `kubectl apply`, because kompose does not
carry everything over from compose:

- every pod needs `hostNetwork: true` (ROS 2 DDS discovery and the rosbridge
  port), and the same `ROS_DOMAIN_ID` as the rest of your ROS 2 system;
- `ue_server` needs the GPU: install the
  [NVIDIA device plugin](https://github.com/NVIDIA/k8s-device-plugin) (with
  k3s on Docker it needs `nvidia` as Docker's default runtime) and add
  `resources.limits: {nvidia.com/gpu: 1}`;
- the build `hostPath` must exist on the node that runs `ue_server`;
- `ue_server` must not run as root (the image's user is uid 1000; the launcher
  drops to it if started as root).

This k3s path has not been tested end to end here; the compose stack has.

## Tests

`tests/test_unrealengine_client.py` covers the client's rate and latency
bookkeeping and callbacks with ROS stubbed out, and
`tests/test_unrealengine_launcher.py` runs `run_server.sh` against fake builds
(missing build, layout detection, argument passing, stop forwarding).
