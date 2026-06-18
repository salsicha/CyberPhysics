# MIT RACECAR Neo

This application packages the official MIT RACECAR Neo Linux simulator and Python library, then exposes the simulator through a ROS 2 bridge.

The Docker image downloads upstream assets at build time:

- `MITRacecarNeo/RacecarNeo-Simulator`, `linux` branch
- `MITRacecarNeo/racecar-neo-library`, `main` branch

Simulator assets are not stored in this repository.

## Build

```bash
make -C applications build_racecarneo
```

## Run The Official Simulator Headlessly And Publish ROS 2 Topics

```bash
docker compose -f compositions/racecarneo.yaml up racecarneo_sim racecarneo_bridge
```

The Unity simulator runs headlessly under `xvfb-run` in `racecarneo_sim`, so it does not need a host X11 display mount. The Python bridge connects to the simulator over UDP ports `5064` and `5065` and publishes ROS 2 messages.

The bridge publishes:

- `camera/color/image_raw` (`sensor_msgs/Image`, `bgr8`)
- `camera/depth/image_rect_raw` (`sensor_msgs/Image`, `32FC1`, meters)
- `camera/color/camera_info` (`sensor_msgs/CameraInfo`)
- `scan` (`sensor_msgs/LaserScan`)
- `imu/data_raw` (`sensor_msgs/Imu`)

The bridge subscribes to:

- `ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)

The bridge sends the MIT protocol `racecar_go` autostart packet for the first 20 seconds of startup. Simulator builds that honor this packet can begin publishing without UI interaction. If a particular Unity build still waits for an in-window click, that is an upstream simulator limitation rather than a ROS transport issue; the ROS bridge is already ready to publish as soon as RacecarSim emits update frames.
