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

- TF `map -> base_link` (the displayed vehicle pose)
- `/odom` (pipeline status only)
- `/camera/color/image_raw`
- `/depth_anything/depth/image`

It sends goals in the `map` frame to the Nav2 `NavigateToPose` action server, the same frame as the displayed pose, so coordinates read off the display stay valid after SLAM corrects odometry. It also shows basic pipeline status, a camera preview, and a simple 3D vehicle marker.

## Notes

NiceGUI runs inside a Python virtual environment with ROS 2 Python packages from the system environment. The Dockerfile pins `numpy<2` so OpenCV and `cv_bridge` stay ABI-compatible, and pins `nicegui==3.13.0` because `scripts/main.py` sets the internal `ui_run.APP_IMPORT_STRING`.

Project reference: https://github.com/zauberzeug/nicegui
