# NiceGUI

This application provides a browser UI container for ROS 2 systems. The current default UI is a RACECAR Neo operator view.

## Build

```bash
make -C applications build_nicegui
```

## Run

As part of RACECAR Neo:

```bash
docker compose -f compositions/racecarneo.yaml up racecarneo_nicegui
```

Open:

```text
http://localhost:8080
```

## RACECAR Neo UI

The UI subscribes to:

- `/odom`
- `/camera/color/image_raw`
- `/depth_anything/depth/image`

It sends goals to the Nav2 `NavigateToPose` action server and displays basic pipeline status, vehicle pose, a camera preview, and a simple 3D vehicle marker.

## Notes

NiceGUI runs inside a Python virtual environment with ROS 2 Python packages from the system environment. The Dockerfile pins `numpy<2` so OpenCV and `cv_bridge` stay ABI-compatible.

Project reference: https://github.com/zauberzeug/nicegui
