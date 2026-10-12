# RACECAR Turret

The RACECAR Neo (`systems/racecarneo`) with the PanTiltROS turret
(`applications/pantiltros`) on a raised plate behind the lidar. The car
drives with SLAM Toolbox and Nav2 exactly as the RACECAR does, while the
turret keeps a selected object centred in its camera, compensating the car's
own rotation. One web page on the car manages both, from a phone, a tablet
or a laptop.

- Interfaces (topics, commands, frames): [interfaces.md](interfaces.md).
- Bill of materials, power sizing, compute budget: [hardware/racecar_turret.md](../../hardware/racecar_turret.md).
- Hardware stack: `compositions/racecar_turret_hardware.yaml`.
- Simulation and evaluation in Isaac Sim: [applications/isaac/RACECAR_TURRET.md](../../applications/isaac/RACECAR_TURRET.md).

## Contents

- `interfaces.md`: the contract between the car, the turret, the UI and the
  simulation.
- `urdf/racecar_turret.urdf.xacro`: the frame model: the RACECAR's URDF
  (included unchanged) plus `turret/base_link`, the `pan_joint` and
  `tilt_joint` and the turret camera frames. It includes the RACECAR's file
  relatively, so mount `systems/` whole.
- `config/turret_tracker.yaml`: the tracker on the car (USB camera, the car's
  IMU for base compensation); `config/turret_tracker_isaac.yaml`: in Isaac.
- `config/scan_filter.yaml`: masks the plate's posts out of the lidar scan.
- `calibration/turret_camera.yaml`: the turret camera's intrinsics
  (nominal: replace with a calibration).
- `missions/`, `scenarios/`, `validation/`: the Isaac evaluation.

## Hardware

The car is the RACECAR Neo build of `hardware/racecarneo.md`. On it:

- a plate 0.24 m above the ground, 0.10 m behind the chassis centre, on four
  thin posts from the top deck, above the lidar so that only the posts cross
  its scan plane;
- the pan servo on the plate, the tilt servo on the pan stage and the USB
  turret camera on the tilt stage (pan +-90 deg, tilt -37 to +43 deg), with
  the printed foam dart launcher above the camera (toy power, foam darts
  only; [interfaces.md](interfaces.md), "Foam dart launcher");
- four ST3215 serial bus servos (pan, tilt, and the launcher's priming
  actuator and trigger) driven by the Jetson itself: its 40-pin header UART
  goes through a Waveshare Bus Servo Adapter (A) to the servo bus; no
  microcontroller;
- a 12 V servo rail from the drive battery through a fuse, a buck-boost
  regulator and a relay opened by the second pole of the emergency stop.

## Wiring and power

```text
drive battery (3S or more) --+-- contactor (coil via e-stop pole 1) -- VESC -- drive motor   (as the RACECAR)
                             +-- 19 V regulator -- Jetson                                 (as the RACECAR)
                             +-- 20 A fuse -- 12 V buck-boost, >= 12 A -- 5 A fuse
                                   -- relay, >= 20 A contacts (coil via e-stop pole 2) -- Bus Servo Adapter (A) DC in
                                      -- servo bus (daisy chain) -- pan ST3215 (ID 12)
                                                                 -- tilt ST3215 (ID 13)
                                                                 -- launcher actuator ST3215 (ID 14, tilt stage)
                                                                 -- launcher trigger ST3215 (ID 15, tilt stage)
Jetson 40-pin header           Bus Servo Adapter (A), jumper on A (UART)
  pin  8  UART1 TX  ---------->  TX    (Waveshare: "RX-RX, TX-TX"; the labels
  pin 10  UART1 RX  <----------  RX     are the host's side)
  pin  6  GND       -----------  GND
Turret camera  -- USB 2 -- Jetson (or the powered hub)
```

- Three jumper wires, nothing else, between the Jetson and the adapter. If
  the servos do not answer, first swap TX and RX.
- Waveshare does not document the adapter's UART logic level, and the
  Jetson header is 3.3 V and not 5 V tolerant: measure first (bring-up step
  3). Only if the adapter's TX idles above 3.3 V, add a bidirectional level
  shifter that handles 1 Mbps (BSS138 module or TXS0104E), or use the USB
  fallback.
- Never feed the servo rail from the Jetson or a USB port; the adapter's
  ground is common with the Jetson's through the GND wire.
- Four servos at locked rotor draw 10.8 A at 12 V (130 W), more than the
  adapter's servo port (6 A): the 12 V regulator is rated at least 12 A so
  the rail does not sag when they accelerate together, the 5 A fuse
  protects the adapter's port against a sustained stall, the 20 A fuse the
  regulator's input (about 15 A from a sagging 3S pack), and the relay that
  the e-stop's second pole drops breaks the 12 V line (contacts rated 20 A
  or more at 14 V DC, flyback diode across its coil). 14 AWG silicone wire
  before the regulator, 18 AWG after it. The software limits the launcher
  servos' torque (actuator about 55 %, trigger about 30 %), which brings
  the rail's worst case near 7.7 A (hardware/racecar_turret.md).
- Route the camera cable and the servo bus with a half-turn service loop
  around the pan axis (bill of materials), and the launcher servos' cable
  with a loop across the tilt axis, and check the full pan and tilt range by
  hand, launcher cocked and primed, before powering the servos.

## Bring-up

Car first, then the turret with its stop engaged. Each step assumes the
previous one works.

1. **Car.** Build and check the RACECAR Neo as `hardware/racecarneo.md`
   says: manual driving with the RC transmitter, then its ROS topics
   (`/scan`, `/imu/data_raw`, `/camera/color/*`, `/ackermann_cmd`), then
   `compositions/racecarneo_hardware.yaml` with the wheels off the ground.
   Test the emergency stop's drive pole.
2. **Lidar mask.** With the plate and posts mounted, run the lidar driver
   on `/scan_raw`; the composition's `racecar_turret_scan_filter` service
   runs the filter chain of `config/scan_filter.yaml` and publishes `/scan`.
   Check in RViz or Foxglove that no return remains at the posts. Unmasked, the posts are
   obstacles inside the footprint and Nav2's collision monitor stops the car.
3. **Jetson UART.** Enable pins 8 and 10 as UART1 (`sudo
   /opt/nvidia/jetson-io/jetson-io.py`, Configure the 40-pin header, select
   `uarta`, save and reboot). Make sure nothing else uses the port
   (`sudo lsof /dev/ttyTHS1`). Loopback test, with a jumper between pins 8
   and 10 and nothing else on them:

   ```bash
   sudo stty -F /dev/ttyTHS1 1000000 raw -echo
   sudo timeout 2 cat /dev/ttyTHS1 > /tmp/uart_loopback.txt &
   sleep 0.5; echo "uart loopback ok" | sudo tee /dev/ttyTHS1 > /dev/null; wait
   cat /tmp/uart_loopback.txt   # must print: uart loopback ok
   ```

   Nothing printed means the header UART does not transmit (reported on some
   JetPack releases, see the bill of materials): use the USB fallback below.
   Then power the adapter alone (servo supply on, not connected to the
   Jetson) and measure its TX pin's idle voltage to GND: 3.3 V or less.
4. **Servo IDs and zero.** ST3215s ship with ID 1. One servo at a time on
   the bus, set the pan servo to ID 12 and the tilt servo to ID 13 (the
   adapter on USB with jumper B and Waveshare's or Feetech's servo tool), and
   centre both (step 2048 is 0 rad) before fitting the horns, the tilt
   bracket and the camera mount with the turret looking straight ahead and
   level. Directions and offsets: `applications/pantiltros/config/
   arduino_bridge.yaml` (`pan_direction`, `pan_zero_steps`, ...).
5. **Turret, stopped.** Wire the adapter to the header (jumper back on A)
   and press the emergency stop (servo power off). Start the stack (next
   section) and open the page: the turret shows DISARMED. Check
   `/turret/camera/image_raw` in the page, then release the physical stop
   with the turret still disarmed: `/turret/joint_states` must report the
   real pan and tilt (`ros2 topic echo /turret/joint_states`), the turret
   must not move, and the page's badge shows the actuator ready once armed.
6. **Turret, armed.** With the car's wheels off the ground or the car
   stopped (Stop), press Arm and move the turret with the aim joystick,
   then Center; check the directions (stick right pans right, up tilts up)
   and that the physical stop drops the turret's torque. Then tap a person's
   box to lock it, and walk.
7. **Together.** Drive slowly with the drive joystick while the turret
   tracks, then send Nav2 goals by tapping the map.

USB fallback: put the adapter's jumper on B and plug its USB-C into the
Jetson; leave pins 8 and 10 unconnected. Start the stack with its port:
`TURRET_ARDUINO_PORT=/dev/serial/by-id/usb-...` (`ls /dev/serial/by-id/`).
The software is the same (`applications/pantiltros/config/jetson_uart.env`).

## Running the hardware composition

```bash
# The turret camera's stable path; with the RealSense plugged in,
# /dev/video0 is usually one of the RealSense's nodes.
ls /dev/v4l/by-id/
export TURRET_CAMERA_DEVICE=/dev/v4l/by-id/usb-<your camera>-video-index0
docker compose -f compositions/racecar_turret_hardware.yaml up -d
```

- The car services are those of `racecarneo_hardware.yaml` (extended, same
  service and container names, so the two stacks cannot run at once);
  `robot_state_publisher` runs the combined URDF on `/turret/joint_states`;
  `cmd_vel_to_ackermann` and the web UI run this checkout's scripts.
- The turret services use the `pantiltros` image, which compose builds
  (`pull_policy: build`; it contains `usb_cam` and the servo bridge). The
  YOLO model is downloaded on the first start into `cache/turret/models/`.
- `TURRET_ARDUINO_PORT` (default `/dev/ttyTHS1`) is the servo port, passed
  into the bridge's container as a device; `ROS_DOMAIN_ID` as usual.
- Every ROS container uses FastDDS over UDP with 4 MiB socket buffers
  (the turret camera's frames) and the open-file limit of the hardware
  compositions.
- The tracker's base-rotation compensation reads the car's IMU on
  `/imu/data_raw` (from the car's IMU driver, as the RACECAR expects);
  without it the tracker aims as on a fixed base and says so in its status.

## Using the web page

Open `http://<car>:8080` (`applications/nicegui/README.md` has the details):

- **Stop all**, top right on every screen: car stop, Nav2 canceled, turret
  disarmed.
- **Car**: the map with the robot and the Nav2 path; tap it to send a goal
  there (or type one under Waypoint), Cancel goal, Stop and Resume. The drive
  joystick (bottom left) drives at up to 0.3 m/s and takes over from Nav2,
  canceling its goal.
- **Turret**: the turret camera with the detected objects; tap one to track
  it, or Release, Center, Arm, Disarm. The aim joystick (bottom right) aims
  by hand. The status line shows the mode, the target, whether it is
  locked, the aim error and whether base compensation is active.
- **Foam dart launcher** ([interfaces.md](interfaces.md), "Foam dart
  launcher"): **Fire** shoots one dart at the locked target (prime the rack,
  chamber, trigger, trigger back). It is enabled only while the tracker's
  status says every condition holds (armed, actuator ready, launcher ready
  with darts left, target locked within 2 deg at its aim point, a person
  aimed at the torso and not too close, the line of fire clear of the car,
  2 s since the last shot), and the line under it says why not otherwise. **Reload** sets the dart count to
  the number next to it. **Next** aims at the nearest untagged detection,
  **Clear tags** empties the tagged set, **Auto next** aims at the next one
  after each shot. The launcher line shows its state, the darts left, the
  tag count and the last shot's result; tagged objects are drawn violet and
  dashed.

## Long-range link

The car's long-range link is the RACECAR Neo's: a HaLowLink 2 on the car,
bridged to the Jetson's Ethernet, paired with the ground station's access
point over Wi-Fi HaLow (`systems/racecarneo/README.md`, "Long-range link":
addressing, pairing, region and band, range, security, bandwidth;
`hardware/ground_station.md`: the ground side and the range test). The
2.4 GHz RC link and its relay in the e-stop loop stay the wireless stop;
HaLow is not a safety link.

- **Addressing.** The page is at `http://192.168.2.10:8080` (the Jetson's
  `eth0`, static, no gateway) from the ground laptop (192.168.2.1) or a
  tablet or phone on the access point's 2.4 GHz Wi-Fi. Give the car its own
  `ROS_DOMAIN_ID`, apart from the aerodrone's.
- **Region and range.** 902-928 MHz in the US (FCC Part 15), 863-868 MHz in
  the EU, set by the unit's region version, which cannot be changed; about
  1 km line of sight by Morse Micro's figure, much less with the car's
  antenna 0.2 m above the ground: put the ground antenna on a mast and run
  the ground station's range test, the RC stop included.
- **The antenna and the turret.** The car's antenna is a 120 mm half-wave
  dipole at the rear-right corner of the deck, its tip under the launcher's
  tail at full nose-up tilt; the HaLowLink 2 stands behind the camera arm,
  in front of the plate. Neither is in the turret's sweep, its line of fire
  or its camera's level view (`hardware/parts/racecar_turret/README.md`,
  assembly check). Do not fit the HaLowLink's own, longer antenna on this
  car: the launcher's tail would reach it.
- **The turret camera over HaLow.** `compositions/racecar_turret_hardware.yaml`
  sends it at 4 Hz, 480 px, JPEG quality 60, about 0.3-0.6 Mbit/s; with
  the map, the boxes and the status, one browser takes about 0.5-0.9 Mbit/s
  (the front camera's panel closed: it is sent only while open). The
  tracker runs on the car at the camera's full rate and size whatever the
  page shows; only what the operator sees is reduced. Over Ethernet or the
  Jetson's Wi-Fi set `UI_TURRET_CAMERA_HZ=8 UI_TURRET_CAMERA_MAX_WIDTH=640
  UI_TURRET_CAMERA_JPEG_QUALITY=75` for the page's full view.
- **Fire over HaLow.** Every gate is checked on the car by the tracker when
  the command arrives, not by the page; a link that drops or lags makes the
  page's status stale (2 s) and disables Fire, and nothing fires without a
  tap. The page has no login: on an open network anyone could arm the
  turret and fire it. Keep the HaLow network encrypted (WPA3-SAE, your own
  passphrase) and private.
- **Foxglove** is a bench tool here too (Ethernet or Wi-Fi): the turret
  camera's raw frames through it would fill the HaLow link.

## Safety

- The physical emergency stop cuts drive power and servo power. Test both
  poles before each session. It is the turret's only hardware safety: there
  is no microcontroller watchdog, and the ST3215s hold their last goal with
  torque on if the Jetson or the bridge stops.
- Software stops: Stop (car, latched until Resume, also across a restart of
  `cmd_vel_to_ackermann`), Disarm (turret, latched), Stop all. They need the
  software running; they are not a substitute for the physical stop.
- Manual driving and aiming stop by themselves: the page sends nothing once
  the joystick is released or the tab stops reporting (0.35 s), and
  `cmd_vel_to_ackermann` commands zero speed when `/cmd_vel_teleop` stops
  for 0.5 s; the turret holds when manual commands stop.
- 30 kg.cm servos: keep fingers out of the turret while it is armed.
- The dart launcher primes and fires only on a Fire tap, once, at the
  target the page shows locked, and only while every gate holds: no
  automatic, timed or on-lock firing, and Auto next only aims. A person is
  aimed at the torso, never the head, and is not fired at when too close
  (taller than 0.6 of the image). It does not fire into the car: aimed low
  ahead (pan -0.17 to 0.26 rad with the tilt below -0.42 rad, plus a 0.05
  rad margin; `no_fire_zones_rad` in `config/turret_tracker.yaml`) a dart
  would hit the RealSense or the camera arm, so Fire is refused there, and
  a shot whose line of fire becomes blocked is stopped before the sear
  releases. Disarm, Stop all, Release, any other aim command, the target
  leaving the lock, the bridge's 500 ms watchdog and the tracker shutting
  down return the trigger to safe and stop the rack. A step not measured in
  time is a fault: Fire stays disabled until Disarm and Arm. After a fault
  the launcher may still be primed: put its safety catch on and clear it by
  hand before arming again. Toy-power darts only, and eye protection for
  everyone in range.
- The page has no login: anyone on the car's network can drive it. Keep that
  network private.

## Status and limitations

- Not yet run on the physical car. The UI, `cmd_vel_to_ackermann`'s
  teleop/stop mux, the combined URDF with `robot_state_publisher` and the
  lidar mask were checked on a desktop host with a mock graph and synthetic
  scans; the turret camera driver was checked to start in the image, not
  with a camera.
- arm64/Jetson images: not supported as is. The RACECAR's nvblox and Depth
  Anything images and the PanTiltROS image install x86_64 GPU packages
  (`compositions/racecarneo_hardware.yaml`); running the GPU parts on the
  Orin Nano needs JetPack builds of them, and the compute budget of the bill
  of materials.
