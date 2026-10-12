# NiceGUI

This application provides a browser UI container for ROS 2 systems. The current default UI is the RACECAR Neo operator page, which also manages the turret of the RACECAR Turret (`UI_TURRET=1`), and, in a turret-only layout (`UI_TURRET=1 UI_CAR=0`), the standalone PanTiltROS turret.

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

As part of the standalone PanTiltROS turret (turret-only layout): the `turret_ui` service of `compositions/turret_hardware.yaml` (and of `turret_uno_q.yaml`'s `host` profile).

Open:

```text
http://localhost:8080
```

One page, for a phone, a tablet or a laptop, with touch or mouse. A sticky bar at the top holds **Stop all**; the two joysticks are docked at the bottom so both thumbs reach them on a phone.

## RACECAR Neo UI

- **Map**: the `/map` occupancy grid with the robot (TF `map -> base_link`) and the Nav2 path (`/plan`). Tap or click the map to send a `NavigateToPose` goal there, heading away from the robot; the x/y/yaw form under "Waypoint" sends a typed one. Goals are in the `map` frame, the same frame as the displayed pose, so coordinates read off the display stay valid after SLAM corrects odometry. **Cancel goal** cancels every Nav2 navigation goal (`navigate_to_pose` and `navigate_through_poses`, whoever sent it).
- **Drive joystick**: while held it publishes `geometry_msgs/Twist` on `/cmd_vel_teleop` at 10 Hz (up to `UI_MAX_SPEED`, default 0.3 m/s; left and right set the front-wheel angle up to `UI_MAX_STEERING`, default 0.42 rad, forward and in reverse), and a final zero on release. Starting to drive cancels the active Nav2 goal. `cmd_vel_to_ackermann` gives `/cmd_vel_teleop` priority over Nav2's `/cmd_vel` while it keeps arriving.
- **Stop / Resume**: `/racecarneo/stop_request` (`std_msgs/Bool`, transient-local): true holds zero speed whatever Nav2 or the joystick say, until Resume. The drive status line and the STOPPED badge come from `/racecarneo/control_status`.
- **Front camera** (`/camera/color/image_raw`), JPEG-encoded at 5 Hz (at most 480 px wide, quality 75) by default and sent only while its panel is open, and freshness indicators (TF pose, `/odom`, `/map`, camera, `/depth_anything/depth/image`, drive status).

## RACECAR Turret UI (`UI_TURRET=1`)

Adds, per `systems/racecar_turret/interfaces.md`:

- **Turret camera**: `/turret/camera/image_raw`, JPEG-encoded at 8 Hz (at most 640 px wide, quality 75) by default, with the tracker's boxes from `/turret/yolo/segments`; the tracked target is drawn thicker (amber, green once locked). Tap a box to lock it (`/turret/command` `{"action": "lock", "target_id": ...}`), or tap elsewhere to lock whatever the tracker finds at that point (`{"action": "lock", "point": [u, v]}`).
- **Release**, **Center** and an **aim joystick** (`{"action": "manual", "pan_rate", "tilt_rate"}` at 10 Hz while held, up to `UI_TURRET_MAX_RATE`, default 1 rad/s; right pans right, up tilts up; a zero-rate command on release).
- **Arm / Disarm**: `/turret/emergency_stop` (`std_msgs/Bool`, transient-local), false arms. The badge shows the e-stop state and `/turret/actuator_ready`.
- The tracker's `/turret/status`: mode, target, locked, aim error, base compensation, detections and message.
- **Foam dart launcher** (`systems/racecar_turret/interfaces.md`, "Foam dart launcher"):
  - **Fire** sends `{"action": "tag", "target_id": <the locked id>}`, one fire cycle (prime, chamber, trigger) per tap. It is enabled only while a fresh `/turret/status` (2 s) has an empty `fire_block` (locked within the aim error at the aim point, a person not too close, the launcher ready with darts, the line of fire clear of the vehicle, past the minimum interval) and the page agrees: armed, `/turret/actuator_ready` true, mode `auto` with a target, locked, darts left, launcher `ready`. The reason it is disabled is shown under it, and a tap is checked again before it is sent. The tracker checks the gates once more when the command arrives; the button is a convenience, not the safety.
  - **Reload** sends `{"action": "reload", "darts": N}` with the number next to it (a whole number, 0 to 1000; the tracker refuses it during a cycle).
  - **Next** (`{"action": "next"}`: aim at the nearest untagged detection), **Clear tags** (`{"action": "clear_tags"}`) and an **Auto next** toggle (`{"action": "auto_next", "enabled": ...}`, which follows the tracker's `auto_next`). Auto next only aims; every shot needs a tap.
  - The launcher line: its state (`none`, `ready`, `priming`, `chambering`, `firing`, `empty`, `fault`), the darts left, the tag count and the last shot's result. Tagged detections are drawn violet and dashed.

**Stop all** publishes `/racecarneo/stop_request` true, cancels Nav2 and, on the turret page, engages `/turret/emergency_stop`.

## Turret-only layout (`UI_TURRET=1 UI_CAR=0`)

The standalone turret's page, titled PanTiltROS Turret: the turret card above alone (one column, at most tablet width) and the aim joystick; no map, car camera, drive joystick or Nav2. Stop all engages `/turret/emergency_stop` only. `UI_TURRET_CAMERA_TOPIC` names the camera to show (default `/turret/camera/image_raw`; `turret_hardware.yaml` passes the tracker's `TURRET_CAMERA_TOPIC`, a RealSense's by default).

## Joysticks and the deadman

The browser reports a held joystick every 100 ms (`scripts/joystick.js`); the server repeats the command at 10 Hz only while those reports keep arriving (0.35 s), then sends one zero command and stops. A tab that freezes, goes to the background or loses its connection therefore stops sending within 0.35 s, and `cmd_vel_to_ackermann` (and the tracker) treat a stream that stops for 0.5 s as released. Reports held up in a reconnecting socket are recognised by their send time and ignored. Two browsers driving at once both publish: drive from one.

## Code

- `scripts/main.py`: the page and the ROS node (rclpy spins in a daemon thread; page handlers publish from NiceGUI's event loop). Images are encoded in the ROS thread at a modest rate and served at `/frames/<name>`.
- `scripts/racecar_ui_logic.py`: the decisions, without ROS or NiceGUI: joysticks to commands and the deadman, map image and taps to goals, boxes and taps to lock commands, status parsing. Tested by `tests/test_racecar_turret_ui.py`; the node by `tests/test_racecarneo_nicegui.py`.
- `scripts/mock_racecar_turret.py`: a fake RACECAR Turret graph (map, TF, Nav2 action server, cameras, segments, tracker status, control status) that acts on the page's commands, so the page can be exercised without a robot. It also plays the foam dart launcher: the tracker's gates (armed, locked within 2 deg at the aim point, a person aimed at the torso and too close above 0.6 of the image, the launcher ready, darts left, the car's no-fire zone when aiming low ahead, 2 s between shots; a refused shot says why in `last_tag` and `fire_block`) and its fire cycle (the rack drawn and returned at the ST3215's speed, the trigger to 0.6 rad and back, a dart counted off; `actuator_joint` and `trigger_joint` in `/turret/joint_states`), aborted by the e-stop, a lost lock or a command that moves the aim into a fault that disarming and arming clears, and Reload, Next, Clear tags and auto next:

```bash
docker run --rm -it --network host -e ROS_DOMAIN_ID=83 \
  -v "$PWD/applications/nicegui/scripts:/workspace/scripts:ro" \
  salsicha/cyberphysics:nicegui-latest python3 /workspace/scripts/mock_racecar_turret.py
docker run --rm -it --network host -e ROS_DOMAIN_ID=83 -e UI_TURRET=1 \
  salsicha/cyberphysics:nicegui-latest python3 /workspace/scripts/main.py
```

`--command-log FILE` writes every command it receives as a JSON line; `--no-control-status` leaves `/racecarneo/control_status` to a real `cmd_vel_to_ackermann`; `--static` keeps the people still; `--no-actuator` plays a turret without a launcher. Add `-e UI_CAR=0` to the page's container for the turret-only layout.

## Notes

NiceGUI runs inside a Python virtual environment with ROS 2 Python packages from the system environment. The Dockerfile pins `numpy<2` so OpenCV and `cv_bridge` stay ABI-compatible, and pins `nicegui==3.13.0` because `scripts/main.py` sets the internal `ui_run.APP_IMPORT_STRING`. Set `UI_PORT` to serve on another port.

Environment: `UI_TURRET` (1: the turret card), `UI_CAR` (0 with `UI_TURRET=1`: turret-only), `UI_TURRET_CAMERA_TOPIC`, `UI_PORT`, `UI_MAX_SPEED`, `UI_MAX_STEERING`, `UI_WHEELBASE`, `UI_TURRET_MAX_RATE`, and the camera streams below.

## Image streams and bandwidth

Each camera is sent as JPEG frames, set by three variables per stream (`racecar_ui_logic.image_stream`):

| Stream | Rate (Hz) | Width (px, at most) | JPEG quality | Defaults |
|---|---|---|---|---|
| Front camera | `UI_CAMERA_HZ` (0 turns it off) | `UI_CAMERA_MAX_WIDTH` | `UI_CAMERA_JPEG_QUALITY` | 5, 480, 75 |
| Turret camera | `UI_TURRET_CAMERA_HZ` | `UI_TURRET_CAMERA_MAX_WIDTH` | `UI_TURRET_CAMERA_JPEG_QUALITY` | 8, 640, 75 |

Values are clamped to 0.5-30 Hz, 160-1920 px and quality 10-95; an unreadable value keeps the default. Frames are scaled down keeping their aspect ratio, never up. The page refreshes at 5 Hz, so a browser shows at most 5 frames per second of a stream; a faster rate only shortens the delay to the newest frame. The front camera is sent only while its panel is open. The map is a PNG sent on each `/map` (SLAM Toolbox: every 2 s).

The defaults suit a local link: one browser takes about 1.5-3 Mbit/s with both cameras (`systems/racecarneo/README.md`, "Long-range link", has the estimate). Over the car's long-range Wi-Fi HaLow link `compositions/racecarneo_hardware.yaml` and `racecar_turret_hardware.yaml` lower them to 2 Hz, 320 px, quality 60 (front) and 4 Hz, 480 px, quality 60 (turret): about 0.5-0.9 Mbit/s. The tracker works on the camera's own frames; only the page's copy is smaller.

The page has no login: anyone who can reach port 8080 can drive the car. Keep the car's network private.

Project reference: https://github.com/zauberzeug/nicegui
