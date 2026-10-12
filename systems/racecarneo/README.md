# RACECAR Neo System

This folder contains configuration for the MIT RACECAR Neo hardware/simulation setup. The reusable application code lives in `applications/`; this directory holds the files that are specific to this vehicle.

## Contents

- `config/bridge.yaml`: ROS parameters for the official Unity simulator bridge.
- `config/cmd_vel_to_ackermann.yaml`: conversion from Nav2 velocity commands to Ackermann commands in SI units (m/s, rad). Manual driving on `/cmd_vel_teleop` (the web UI's joystick) takes priority while it keeps arriving (0.5 s timeout, then zero speed), a true on `/racecarneo/stop_request` (transient-local) latches zero speed until a false, and `/racecarneo/control_status` reports the source, the stop and the command as JSON (`systems/racecar_turret/interfaces.md`).
- `config/depthanything.yaml`: metric Depth Anything topic/model settings.
- `config/nav2.yaml`: Nav2 planner, controller, behavior, and nvblox costmap-layer settings.
- `config/nvblox.yaml`: nvblox TSDF/ESDF mapper overrides for the RACECAR camera setup.
- `config/nvblox_hardware.yaml`: over `nvblox.yaml` on the car: dynamic mode (people decay instead of staying in the map) with the RealSense's 5 m range.
- `config/slam_toolbox.yaml`: online SLAM Toolbox settings for `map -> odom`.
- `config/topics.yaml`: canonical topics and frames for this system.
- `config/sensor_models.yaml`: hardware-to-simulation sensor, actuator, safety, and noise-model contract.
- `calibration/camera.yaml`: simulator camera intrinsics (replace for hardware).
- `urdf/racecarneo.urdf.xacro`: frame model and the single source of sensor extrinsics for simulator and hardware. Frame names match `config/topics.yaml`.
- `launch/racecarneo_description.launch.py`: runs `robot_state_publisher` on the URDF; pass `camera_optical_frames:=false` when the RealSense driver publishes the optical frames. The RACECAR Turret passes its own URDF (`urdf:=`) and `joint_states_topic:=/turret/joint_states`.
- `scenarios/campus_closed_course.json`: tracked source-of-truth contract for the realistic outdoor/indoor closed-course environment, semantic labels, collision geometry, dynamic actors, spawn poses, and scenario variants.
- `missions/nav2_waypoints.json`: Nav2 waypoint missions for loop, slalom, lane-change, obstacle-avoidance, recovery, and multi-lap validation.
- `validation/sensor_behavior_tests.json`: sensor-driven behavior test manifest for nominal, degraded-camera, lidar-dropout, IMU-bias, and odometry-slip runs.
- `validation/acceptance_thresholds.json`: default Racecar Neo mission acceptance thresholds.

## Runtime Pipeline

The full simulator composition is `compositions/racecarneo.yaml`:

```bash
docker compose -f compositions/racecarneo.yaml up
```

Data flow:

```text
Unity simulator -> racecarneo_bridge -> /camera/color/image_raw, /camera/depth/image_rect_raw, /scan, /imu/data_raw, /odom
/camera/depth/image_rect_raw + /camera/color/camera_info -> nvblox (static mode, nvblox.yaml) -> map slices
/camera/color/image_raw -> Depth Anything -> /depth_anything/depth/image (published, not mapped)
nvblox static map slice + /scan + SLAM map frame -> Nav2 -> /cmd_vel
/cmd_vel (or /cmd_vel_teleop) -> cmd_vel_to_ackermann -> /ackermann_cmd (m/s, rad) -> simulator bridge
URDF -> robot_state_publisher -> base_link -> racecarneo/* sensor frames
```

On the car, `compositions/racecarneo_hardware.yaml` (and the RACECAR
Turret's `racecar_turret_hardware.yaml`, which extends it):

```text
RealSense (colour 640x480, depth 848x480, 15 Hz) -> /camera/color/image_raw,
    /camera/aligned_depth_to_color/image_raw (16UC1 mm, 640x480) + its camera_info
aligned depth + its camera_info -> nvblox (dynamic mode, nvblox.yaml + nvblox_hardware.yaml) -> map slices
nvblox static map slice + /scan + SLAM map frame -> Nav2 -> /cmd_vel
/cmd_vel (or /cmd_vel_teleop) -> cmd_vel_to_ackermann -> /ackermann_cmd (m/s, rad) -> motor bridge (outside the composition)
```

Depth Anything's monocular depth reads about 9 times the true depth from
the car's 0.22 m camera height (`applications/depthanything/README.md`,
"Accuracy"), so nvblox maps the depth camera's own depth in both, and on
the car Depth Anything runs only under the `depthanything` profile
(`docker compose -f compositions/racecarneo_hardware.yaml --profile
depthanything up -d`), for comparison. In dynamic mode the people walking
by go to nvblox's decaying dynamic layer and Nav2 reads the static slice,
so the lidar gives it the people, as in the RACECAR Turret's Isaac
evaluation (`applications/isaac/RACECAR_TURRET.md`, "People in nvblox's
map"). The web page's `depth` indicator follows `/depth_anything/depth/image`,
so without the profile it stays red on the car.

The NiceGUI frontend (`applications/nicegui`) shows the SLAM map with the `map -> base_link` pose and the Nav2 path, the camera, the drive status and pipeline freshness; it sends Nav2 waypoints in the `map` frame through `NavigateToPose` (tap the map or type one), cancels them, drives manually with an on-screen joystick on `/cmd_vel_teleop`, and stops the car with `/racecarneo/stop_request`. The closed-course scenario contract in `systems/racecarneo/scenarios/campus_closed_course.json` defines the environment geometry that Unity/Gazebo/Isaac assets should instantiate for validation.

## Long-range link

The physical car's operator link is Wi-Fi HaLow (802.11ah, sub-GHz), not
the Jetson's own Wi-Fi: a Morse Micro HaLowLink 2 on the car, in Extender
mode and cabled to the Jetson's Ethernet, bridges the car onto the ground
station's network, where a second HaLowLink 2 is the access point (the
user guide's "Virtual Wire"). Parts and mounting:
`hardware/racecarneo.md` ("Long-range link"); the ground side, pairing,
addressing plan and range test: `hardware/ground_station.md`. HaLow is not
a safety link: the RC transmitter and the RC relay in the e-stop loop stay
the wireless stop, and the car does not go beyond their range.

- **Addressing.** One bridged network, 192.168.2.0/24, static addresses
  (the ground station's plan, shared with the aerodrone, whose BlueOS
  keeps 192.168.2.2 and sends MAVLink to the laptop at 192.168.2.1). The
  Jetson's `eth0` is **192.168.2.10/24**, without a gateway, so its own
  Wi-Fi keeps the internet:

  ```bash
  sudo nmcli connection add type ethernet ifname eth0 con-name halow \
    ipv4.method manual ipv4.addresses 192.168.2.10/24 ipv4.never-default yes ipv6.method disabled
  sudo nmcli connection up halow
  ```

  The operator opens **`http://192.168.2.10:8080`** from the laptop
  (192.168.2.1) or from a phone or tablet on the access point's 2.4 GHz
  Wi-Fi (DHCP from the access point). The services run with host
  networking and listen on every interface, so nothing in the compositions
  changes. DHCP from the access point works too, but then the address can
  change. Give the car its own `ROS_DOMAIN_ID` (for example
  `export ROS_DOMAIN_ID=10` before `docker compose up`): the bridge carries
  DDS discovery, and on separate domains the car's and the aerodrone's ROS
  graphs never meet.
- **Pairing.** On the bench: hold the car unit's Mode button until its
  status LED flashes quickly aqua (Extender), then press Mode once on the
  access point and once on the car's unit; the car unit's HaLow LED turns
  solid purple when it has joined. Then give it its own management address
  (192.168.2.251) and turn its 2.4 GHz access point off, so that it does not
  share 2.4 GHz with the RC link (`hardware/ground_station.md`, "Pairing a
  vehicle's unit").
- **Region and band.** US: 902-928 MHz, an FCC Part 15 device (up to
  26 dBm); EU: 863-868 MHz (Morse Micro limits it to 14 dBm). The
  HaLowLink 2's region is fixed by the version bought and cannot be changed
  later: buy every unit in the version for the country of use and check
  its status page, and fit the antenna for that band
  (`hardware/racecarneo.md`).
- **Range.** Morse Micro gives about 1 km line of sight. The car's antenna
  stands only about 0.2 m above the ground, where the ground reflection
  cuts the range: it then grows with the square root of the product of the
  two antenna heights, so the ground antenna goes on a 2.5-3 m mast. Expect
  a few hundred metres in the open and less through trees; measure it with
  the ground station's range test, the RC stop included, before driving
  out.
- **Security.** The page has no login: anyone who reaches port 8080 can
  drive the car (and, on the RACECAR Turret, aim and fire it). Keep the
  HaLow network encrypted (**WPA3-SAE**, with your own passphrase set before
  pairing) and private: no internet uplink, no shared LAN, the car unit's
  2.4 GHz access point off, nothing on it but the ground station's devices.
- **Bandwidth.** HaLow carries tens of Mbit/s at short range and a few
  Mbit/s or less at range (802.11ah's lowest rates are a few hundred
  kbit/s), shared by everything on it. What the page sends to one open
  browser (JPEG sizes measured on 640 x 480 photographs; busy outdoor
  scenes are at the top of each range):

  | Stream | Page defaults (local link) | Hardware compositions (HaLow) |
  |---|---|---|
  | Turret camera (RACECAR Turret) | 640 px, JPEG 75, encoded at 8 Hz, shown at up to 5 Hz (the page refreshes at 5 Hz): 22-46 kB a frame, 0.9-1.8 Mbit/s | 480 px, JPEG 60, 4 Hz: 9-20 kB, 0.3-0.6 Mbit/s |
  | Front camera | 480 px, JPEG 75, 5 Hz: 12-27 kB, 0.5-1.1 Mbit/s | 320 px, JPEG 60, 2 Hz: 5-10 kB, under 0.2 Mbit/s |
  | Map (PNG on each `/map`, every 2 s) | 10-22 kB, under 0.1 Mbit/s | the same |
  | Boxes, status, labels (5 Hz) | about 0.05 Mbit/s | the same |
  | Joysticks and taps (to the car) | under 0.02 Mbit/s | the same |
  | **One browser** | **about 1.5-3 Mbit/s** | **about 0.5-0.9 Mbit/s** |

  The front camera is sent only while its panel on the page is open, and
  `UI_CAMERA_HZ=0` turns it off. `compositions/racecarneo_hardware.yaml`
  and `racecar_turret_hardware.yaml` therefore default to the HaLow column
  (`UI_CAMERA_*`, `UI_TURRET_CAMERA_*`; `applications/nicegui/README.md`);
  over Ethernet or Wi-Fi set the page's defaults in the shell, for example
  `UI_TURRET_CAMERA_HZ=8 UI_TURRET_CAMERA_MAX_WIDTH=640
  UI_TURRET_CAMERA_JPEG_QUALITY=75`, and at the edge of the range go lower.
  Open the page in one browser at a time over HaLow. The RACECAR Neo stack
  runs the nicegui image's own scripts: rebuild it
  (`make -C applications build_nicegui`) for these settings (the RACECAR
  Turret stack mounts the checkout's scripts).
- **Foxglove** (`ws://<car>:8765`, the `racecarneo_foxglove_bridge`
  service) is a bench tool, over Ethernet or the Jetson's Wi-Fi. It sends
  whatever the client subscribes to, uncompressed camera images and point
  clouds included, which fills the HaLow link and starves the page; over
  HaLow use the page, or subscribe Foxglove to small status topics only.

## Asset Policy

The official MIT Unity simulator and Python library are downloaded into the `salsicha/cyberphysics:racecarneo-latest` image during build. Generated maps, bags, logs, and simulator assets should not be committed here.

A more detailed Gazebo or Isaac model can be added here later by extending the URDF with wheel joints, Ackermann steering controllers, sensors, and simulator-specific tags.
