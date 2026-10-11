# NiceGUI

This application provides a browser UI container for ROS 2 systems. The current default UI is the RACECAR Neo operator page, which also manages the turret of the RACECAR Turret (`UI_TURRET=1`).

## Build

```bash
make -C applications build_nicegui
```

## Run

As part of RACECAR Neo:

```bash
docker compose -f compositions/racecarneo.yaml up racecarneo_nicegui
```

As part of the RACECAR Turret: `compositions/racecar_turret_hardware.yaml` (`systems/racecar_turret/README.md`).

Open:

```text
http://localhost:8080
```

One page, for a phone, a tablet or a laptop, with touch or mouse. A sticky bar at the top holds **Stop all**; the two joysticks are docked at the bottom so both thumbs reach them on a phone.

## RACECAR Neo UI

- **Map**: the `/map` occupancy grid with the robot (TF `map -> base_link`) and the Nav2 path (`/plan`). Tap or click the map to send a `NavigateToPose` goal there, heading away from the robot; the x/y/yaw form under "Waypoint" sends a typed one. Goals are in the `map` frame, the same frame as the displayed pose, so coordinates read off the display stay valid after SLAM corrects odometry. **Cancel goal** cancels every Nav2 navigation goal (`navigate_to_pose` and `navigate_through_poses`, whoever sent it).
- **Drive joystick**: while held it publishes `geometry_msgs/Twist` on `/cmd_vel_teleop` at 10 Hz (up to `UI_MAX_SPEED`, default 0.3 m/s; left and right set the front-wheel angle up to `UI_MAX_STEERING`, default 0.42 rad, forward and in reverse), and a final zero on release. Starting to drive cancels the active Nav2 goal. `cmd_vel_to_ackermann` gives `/cmd_vel_teleop` priority over Nav2's `/cmd_vel` while it keeps arriving.
- **Stop / Resume**: `/racecarneo/stop_request` (`std_msgs/Bool`, transient-local): true holds zero speed whatever Nav2 or the joystick say, until Resume. The drive status line and the STOPPED badge come from `/racecarneo/control_status`.
- **Front camera** (`/camera/color/image_raw`) and freshness indicators (TF pose, `/odom`, `/map`, camera, `/depth_anything/depth/image`, drive status).

## RACECAR Turret UI (`UI_TURRET=1`)

Adds, per `systems/racecar_turret/interfaces.md`:

- **Turret camera**: `/turret/camera/image_raw`, JPEG-encoded at 8 Hz (at most 640 px wide), with the tracker's boxes from `/turret/yolo/segments`; the tracked target is drawn thicker (amber, green once locked). Tap a box to lock it (`/turret/command` `{"action": "lock", "target_id": ...}`), or tap elsewhere to lock whatever the tracker finds at that point (`{"action": "lock", "point": [u, v]}`).
- **Release**, **Center** and an **aim joystick** (`{"action": "manual", "pan_rate", "tilt_rate"}` at 10 Hz while held, up to `UI_TURRET_MAX_RATE`, default 1 rad/s; right pans right, up tilts up; a zero-rate command on release).
- **Arm / Disarm**: `/turret/emergency_stop` (`std_msgs/Bool`, transient-local), false arms. The badge shows the e-stop state and `/turret/actuator_ready`.
- The tracker's `/turret/status`: mode, target, locked, aim error, base compensation, detections and message.

**Stop all** publishes `/racecarneo/stop_request` true, cancels Nav2 and, on the turret page, engages `/turret/emergency_stop`.

## Joysticks and the deadman

The browser reports a held joystick every 100 ms (`scripts/joystick.js`); the server repeats the command at 10 Hz only while those reports keep arriving (0.35 s), then sends one zero command and stops. A tab that freezes, goes to the background or loses its connection therefore stops sending within 0.35 s, and `cmd_vel_to_ackermann` (and the tracker) treat a stream that stops for 0.5 s as released. Reports held up in a reconnecting socket are recognised by their send time and ignored. Two browsers driving at once both publish: drive from one.

## Code

- `scripts/main.py`: the page and the ROS node (rclpy spins in a daemon thread; page handlers publish from NiceGUI's event loop). Images are encoded in the ROS thread at a modest rate and served at `/frames/<name>`.
- `scripts/racecar_ui_logic.py`: the decisions, without ROS or NiceGUI: joysticks to commands and the deadman, map image and taps to goals, boxes and taps to lock commands, status parsing. Tested by `tests/test_racecar_turret_ui.py`; the node by `tests/test_racecarneo_nicegui.py`.
- `scripts/mock_racecar_turret.py`: a fake RACECAR Turret graph (map, TF, Nav2 action server, cameras, segments, tracker status, control status) that acts on the page's commands, so the page can be exercised without a robot:

```bash
docker run --rm -it --network host -e ROS_DOMAIN_ID=83 \
  -v "$PWD/applications/nicegui/scripts:/workspace/scripts:ro" \
  salsicha/cyberphysics:nicegui-latest python3 /workspace/scripts/mock_racecar_turret.py
docker run --rm -it --network host -e ROS_DOMAIN_ID=83 -e UI_TURRET=1 \
  salsicha/cyberphysics:nicegui-latest python3 /workspace/scripts/main.py
```

`--command-log FILE` writes every command it receives as a JSON line; `--no-control-status` leaves `/racecarneo/control_status` to a real `cmd_vel_to_ackermann`.

## Notes

NiceGUI runs inside a Python virtual environment with ROS 2 Python packages from the system environment. The Dockerfile pins `numpy<2` so OpenCV and `cv_bridge` stay ABI-compatible, and pins `nicegui==3.13.0` because `scripts/main.py` sets the internal `ui_run.APP_IMPORT_STRING`. Set `UI_PORT` to serve on another port.

The page has no login: anyone who can reach port 8080 can drive the car. Keep the car's network private.

Project reference: https://github.com/zauberzeug/nicegui
