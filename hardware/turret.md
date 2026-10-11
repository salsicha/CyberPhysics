# Turret Hardware Shopping List

Goal: build a pan-tilt perception turret that can be simulated and validated
against the `applications/pantiltros` ROS 2 control stack. The turret should
segment a realistic camera scene, select a target object, and drive pan/tilt
servos to keep that object centered while it moves through the scene.

Baseline hardware

- 1 x two-axis pan-tilt mechanism compatible with `pan_tilt_ros`.
- 2 x Waveshare ST3215, Waveshare ST3215-HS, or Feetech STS3215 serial bus
  servos for pan and tilt.
- 1 x turret controller, either:
  - Arduino UNO R4 WiFi, Arduino SKU ABX00087, plugged into the compute host
    over USB (5 V logic). Buy the real Arduino board, not a clone, so the
    USB-C interface, Wi-Fi/Bluetooth, and board package match the supported
    Arduino tooling; or
  - Arduino UNO Q, SKU ABX00162 (2 GB RAM, 16 GB eMMC) or ABX00173 (4 GB,
    32 GB): its STM32U585 MCU drives the servos and its Debian Linux side
    runs the ROS bridge, so the board joins the ROS graph over Wi-Fi or
    Ethernet instead of USB (3.3 V logic). See "Arduino UNO Q" below.
- 1 x USB-C data cable (UNO R4 WiFi: to the host; UNO Q: for flashing and
  setup) and, for the UNO Q, a 5 V / 3 A USB-C power supply or a 7-24 V supply
  on VIN.
- 1 x half-duplex TTL serial interface, level shifter, or ST3215/STS3215
  Arduino shield/cape between the Arduino hardware UART and the serial bus
  servos, with its MCU side at the board's logic level (5 V for the UNO R4,
  3.3 V for the UNO Q). Do not power the servos from the Arduino board.
- 1 x optional USB serial bus servo adapter for one-time servo ID, midpoint,
  and limit setup before the Arduino firmware takes over runtime control.
- 1 x rigid base or tripod mount with enough stiffness to avoid camera shake.
- 1 x RGB camera mounted on the tilt payload link. Use a global-shutter camera
  if fast moving targets are required; otherwise a UVC camera or OAK RGB stream
  is acceptable for simulation parity.
- 1 x compute host capable of running ROS 2, PanTiltROS, and Ultralytics YOLO
  segmentation. Use GPU acceleration for high-resolution YOLO26 segmentation
  tests.
- 1 x regulated servo power supply sized for both servos at peak load.
- 1 x physical emergency-stop switch that removes servo power, plus a way to
  publish the software stop on `/turret/emergency_stop` (see below).
- Cabling, strain relief, mounting hardware, and optional cable chain so the
  pan/tilt range cannot pull camera or servo cables tight.

Arduino integration

Status: the firmware, `applications/arduino/scripts/turret_firmware`, builds
for the UNO R4 WiFi (`arduino:renesas_uno:unor4wifi`) and the UNO Q MCU
(`arduino:zephyr:unoq`) in the `arduino` image, and its protocol, limit,
watchdog and Feetech packet logic is unit tested on the host
(`tests/test_turret_firmware.py`, packets byte-identical to Feetech's
`SCServo_Linux`). The UNO Q build and its bridge have run on a real UNO Q
without servos (see "Verified on a real UNO Q" below); nothing has run on an
UNO R4 WiFi or on real servos yet: check the servo IDs, zero offsets,
directions, speed and limits in `src/turret_config.h` with the servos free to
move before mounting the payload. The ST3215 node `pan_tilt_ctrl_node`
(package `pan_tilt_ros` in `applications/pantiltros/pantiltros`) is an
alternative without an Arduino: built in the PanTiltROS image with the
Feetech `SCServo_Linux` library, it drives the servos directly from the host
over a USB bus adapter, but has not been run against real ST3215 servos here
either.

Firmware (`turret_firmware.ino`, shared core in `src/`):

- Servos: two ST3215/STS3215 on a half-duplex bus at 1 Mbps on the board's
  D0/D1 UART (`Serial1`), Feetech STS protocol implemented in `src/sts_servo.*`
  (no external servo library): sync write of goal position, speed and
  acceleration (registers 41-47), present position read (register 56), torque
  enable (register 40). Defaults in `src/turret_config.h`: pan ID 12, tilt ID
  13 (as `pantiltros/config/ctrl_config.yaml`), step 2048 at 0 rad with steps
  decreasing as the angle grows (the `pan_tilt_ros` convention), limits pan
  +-1.5708 rad and tilt -0.65..0.75 rad, 2000 steps/s, acceleration 50.
- Behaviour, identical on both boards: joint limits clamp every command; the
  first `SET` enables torque; no `SET` for 500 ms (watchdog) or a `STOP`
  holds the measured position (goal = present position, torque stays on; set
  `stop_disables_torque` for torque off instead); position feedback at 25 Hz,
  reported only when both servos answered; a servo that does not answer (or
  reports a fault bit) produces an `ERR`, repeated at most once per second;
  nothing is written to the servos before the first `SET` or `STOP` (the
  position reads start at power-up, so missing servos are reported at once).
- Half-duplex adapters that echo transmitted bytes back to RX are handled. If
  the adapter needs a direction (transmit-enable) input, wire it to a free pin
  and set `TURRET_SERVO_DIR_PIN` (HIGH while transmitting).
- Flash the UNO R4 WiFi from the Arduino IDE 2 of the `arduino` profile
  (`/home/ubuntu/scripts/turret_firmware` in the container) or with
  `arduino-cli compile --upload -b arduino:renesas_uno:unor4wifi -p /dev/ttyACM0 turret_firmware`
  in that container. For the UNO Q see below.

Wiring (both boards): board D1 (TX) and D0 (RX) to the adapter's MCU side,
adapter data line to the servos' signal pin, a common ground between board,
adapter and servo supply, and the servo supply only to the servos and the
adapter's bus side. Keep the physical emergency-stop switch in the servo
supply.

- `compositions/turret_hardware.yaml` runs the YOLO tracker with
  `applications/pantiltros/config/yolo_tracker_hardware.yaml` and the serial
  bridge `applications/pantiltros/scripts/turret_arduino_bridge.py` for an
  UNO R4 WiFi on the host's USB; the `arduino` profile adds the Arduino IDE
  container for flashing firmware.
- Serial protocol (ASCII lines, `\n` terminated, radians, 115200 baud by
  default, see `config/arduino.env`):
  - host to firmware `SET pan_joint <pan> tilt_joint <tilt>`: move to the
    position (already clamped to the joint limits by the bridge);
  - host to firmware `STOP`: stop moving and hold (or disable torque). Sent
    when the software e-stop engages, on bridge shutdown, and after
    (re)connecting while the e-stop is engaged;
  - firmware to host `POS pan_joint <pan> tilt_joint <tilt>`: measured joint
    positions, e.g. at 20-50 Hz. These are the only source of
    `/turret/joint_states`; the bridge never echoes commands;
  - firmware to host `ERR <text>`: shown in the bridge diagnostics.
- The firmware also stops on its own when no `SET` arrives for 0.5 s, applies
  its own servo limits, and drives the ST3215/STS3215 bus through the
  half-duplex interface.
- The bridge waits 2 s after opening the port (Arduino reset) and drops
  commands while disconnected, resetting or e-stopped, so stale commands are
  never replayed. Without `POS` feedback the tracker drives open loop and only
  while the bridge reports `/turret/actuator_ready`.
- Keep servo power on a separate regulated supply with a shared signal ground
  to the Arduino and a physical emergency-stop path that removes actuator
  power.

Arduino UNO Q

The UNO Q has two processors: a Qualcomm QRB2210 running Debian (quad
Cortex-A53, 2 or 4 GB RAM) and an STM32U585 MCU (Cortex-M33, 160 MHz) running
the Arduino core on Zephyr. Its USB-C port belongs to the Linux side: a host
PC sees `2341:0078 Arduino Uno Q` with an ADB interface and a CDC-ACM serial
port (`/dev/ttyACM0`), not the MCU's UART. The serial port is the board's
`arduino-router-serial` service (`socat` between the USB gadget port
`/dev/ttyGS0` and the router's monitor port, TCP 7500): it carries only the
MCU's `Monitor` text (on the UNO Q `Serial` is the same object), which the
sketch sends as `mon/write` RPCs, and what the PC types reaches the sketch's
`Monitor.read()`. The baud rate is irrelevant. So the UNO R4 setup (bridge on
the host, firmware on `/dev/ttyACM0`) does not apply, and `turret_firmware`
prints nothing there.
Instead the bridge runs on the board and reaches the MCU through
`arduino-router`, the msgpack-RPC router on `/var/run/arduino-router.sock`
that owns the internal MCU link (`/dev/ttyHS1`, 115200 baud, the MCU's
LPUART1; the sketch's `Serial1` is USART1 on D0/D1):

- MCU: the same `turret_firmware` sketch, built for `arduino:zephyr:unoq`
  (Arduino Q Boards core 1.0.0, Arduino_RouterBridge 0.4.3). It provides the
  RPC methods `turret/set(pan, tilt) -> bool`, `turret/stop() -> bool` and
  `turret/get_position() -> [pan, tilt]` (`[]` without a fresh reading), and
  sends the notifications `turret/pos [pan, tilt]` (25 Hz) and
  `turret/err [text]`: the `SET`, `STOP`, `POS` and `ERR` messages above.
- Linux: `turret_arduino_bridge.py` with `transport: uno_q_bridge`
  (`config/arduino_uno_q.env`) in the small `unoq` image
  (`applications/unoq`, ROS 2 Jazzy ros-core, about 0.5 GB). It registers for
  the notifications, probes the MCU with `turret/get_position` every second,
  and reports `/turret/actuator_ready` only while the MCU answers; an RPC
  unanswered for 0.5 s or a sketch that is not running (`method ... not
  available`) makes it false. Topics, e-stop handling and diagnostics are
  those of the serial transport.
- The YOLO tracker and camera stay on the GPU host. Both machines need the
  same `ROS_DOMAIN_ID` and a network that passes DDS discovery (multicast)
  between them.

Wiring: the servo adapter connects to D1 (PB6, USART1 TX) and D0 (PB7,
USART1 RX), `Serial1` in the sketch. UNO Q GPIO is 3.3 V: power the adapter's
MCU side from the board's 3.3 V pin, or put a level shifter between a 5 V
adapter and D0/D1, and never pull D0/D1 up to the servo supply. USART1 is also
the Zephyr console, so boot messages (ASCII text, never a valid Feetech
packet) appear on D1 before the sketch starts. Power the board from a
5 V / 3 A USB-C supply (it negotiates only 5 V) or 7-24 V on VIN, and the
servos from their own supply as above.

Deploy:

1. Flash `turret_firmware` to the MCU from a PC with the board on USB, with
   the Arduino IDE 2 or arduino-cli, for example in the `arduino` image, which
   has the core and libraries; the container needs the board's USB device
   (ADB) and its serial port (from which arduino-cli takes the serial number):

   ```bash
   docker run --rm --device "/dev/bus/usb/$(lsusb -d 2341:0078 | awk '{print $2 "/" substr($4, 1, 3)}')" \
     --device /dev/ttyACM0 salsicha/cyberphysics:arduino-latest \
     arduino-cli compile --upload -b arduino:zephyr:unoq -p /dev/ttyACM0 turret_firmware
   ```

   The core's `remoteocd` pushes the files over ADB to `/tmp/remoteocd` on
   the board and runs the board's own OpenOCD (`/opt/openocd`, SWD through
   the Linux side's GPIOs), as the `arduino` user without a password. It also
   writes the core's Zephyr loader when the MCU has a different one (the
   first upload here did, 29 s; later ones took 11 s); expect Arduino App
   Lab, whose core on the board is 0.51.0, to write its own back when it
   next uploads. Run no other ADB
   server on the PC at the same time. The compile message "Bootloader file
   specified but missing: .../bootloaders/zephyr-arduino_uno_q_stm32u585xx.elf"
   is harmless: arduino-cli looks there to merge a bootloader into an
   exported hex, while the upload takes the loader from the core's
   `firmwares/` folder. Arduino App Lab on the board works too.
2. On the UNO Q, with this repository checked out:
   `ROS_DOMAIN_ID=<id> docker compose -f compositions/turret_uno_q.yaml --profile board up -d`
   (builds the `unoq` image on the board the first time; a board without
   internet: `applications/unoq/README.md`). After an `arduino-router`
   restart, which recreates its socket, the bridge exits within
   `router_lost_exit_s` (30 s) and Docker restarts it on the new socket.
3. On the GPU host:
   `ROS_DOMAIN_ID=<id> docker compose -f compositions/turret_uno_q.yaml --profile host up`
   (`--profile realsense` adds the RealSense camera). Do not run
   `turret_hardware.yaml` at the same time.
4. Check `/turret/diagnostics` ("UNO Q MCU connected") and
   `/turret/joint_states`, then release the emergency stop.

Only one bridge may own the `turret/*` notifications: a second bridge on the
same board reports "another arduino-router client already receives
turret/pos" (or `turret/err`) and never becomes ready.

Verified on a real UNO Q (2 GB model, October 2026), without servos and
without a network, over ADB from a PC:

- Board: Debian 13 (trixie), kernel 6.16.7, Docker 26.1.5 with Compose 2.26.1
  and buildx 0.13.1, arduino-router 0.5.3, 1.7 GB RAM. The router socket is
  `srw-rw-rw- root root`, so any local user or container can use it.
  `sudo` and `systemctl restart arduino-router` need a password, so the
  router restart was reproduced by recreating the socket file of a proxy in
  front of the router instead.
- Upload: the command in step 1 (arduino-cli 1.5.1, `arduino:zephyr` 1.0.0),
  for `turret_firmware` and `unoq_sub`.
- Firmware over the router: `turret/get_position` returns `[]` without
  servos; `turret/set` returns `true` (float64, float32 and integer
  arguments are accepted) and `false` for NaN or infinity; wrong argument
  counts or types get router errors. With no servos the MCU sends
  `turret/err` "servo 12 (pan_joint) timeout" and "servo 13 (tilt_joint)
  timeout" about once a second from power-up on (position reads), "torque
  enable timeout" after a `SET`, and "cannot hold: no position reply" 0.51 s
  after the last `SET` (watchdog; SETs every 100 ms keep it quiet) and about
  20 ms after a `STOP`. RPC round trips take about 7 ms. `turret/pos` needs
  servos.
- Bridge (`compositions/turret_uno_q.yaml --profile board`): registers both
  notifications, sends `turret/stop` once the MCU answers while the e-stop is
  engaged, publishes `/turret/actuator_ready` false until the e-stop is
  released and true after; `/turret/diagnostics` is ERROR with the last
  firmware message ("firmware: servo 13 (tilt_joint) timeout") without
  servos. Joint commands become `turret/set` RPCs only after the release,
  clamped to the joint limits (pan 3.0 and tilt -2.0 arrive as 1.5708 and
  -0.65), are dropped and counted while e-stopped, and NaN is rejected in the
  bridge. Engaging the e-stop and stopping the container each send
  `turret/stop` (the one at shutdown still reaches the MCU although the bridge
  closes the socket at once). Reflashing or resetting the MCU makes the bridge
  not ready ("no answer" while the MCU is halted, "turret/get_position not
  available" while another sketch runs) and ready again, with its
  notifications still registered, when `turret_firmware` is back. A
  recreated router socket made the bridge exit with status 1 after 30 s and
  Docker restart it on the new socket. A second bridge never became ready.
- Resources: the bridge uses about 66 MB of RAM and, idle, 35 % of one
  Cortex-A53 core, nearly all of it rclpy's executor waking for the 50 Hz
  `poll_rate_hz` timer (18 % at 25 Hz, 7 % at 10 Hz, at the cost of up to one
  period of extra joint-state latency). The compositions cap the containers'
  open-file limit: with Docker's default of about 2^30 on the board, LTTng-UST,
  loaded by every ROS 2 process, takes 128 MiB more per process.
- `ros2 topic pub` on `/turret/emergency_stop` with default QoS works but
  logs an "incompatible QoS ... DURABILITY" warning in both processes: the
  bridge also subscribes transient-local, for latched publishers.

Still to check, with servos and a network: `turret/pos` at 25 Hz and
`/turret/joint_states`, torque enable, motion, holding, limits, directions
and zero offsets, the half-duplex adapter at 1 Mbps and 3.3 V on D0/D1, the
servo fault bits, `stop_disables_torque`, a real `arduino-router` restart,
and DDS between the board and the GPU host (tracker, `ROS_DOMAIN_ID`,
multicast over Wi-Fi or Ethernet).

ROS contract

- Camera stream: the hardware composition defaults to the RealSense topics
  `/camera/camera/color/image_raw` and `/camera/camera/color/camera_info`
  (`TURRET_CAMERA_TOPIC`, `TURRET_CAMERA_INFO_TOPIC`).
- Segmentation output: `/turret/yolo/segments`, `/turret/yolo/target_mask`,
  `/turret/yolo/target_bbox`, `/turret/yolo/target_confidence`, and
  `/turret/yolo/annotated_image`.
- Tracker output: `/turret/target/pixel_error`,
  `/turret/target/angular_error`, `/turret/target/selected_id`, and
  `/turret/target/locked`.
- Joint commands: `sensor_msgs/msg/JointState` on `/turret/joint_commands`
  with `pan_joint` and `tilt_joint`.
- Measured joint state: `sensor_msgs/msg/JointState` on `/turret/joint_states`,
  published by the bridge from firmware `POS` lines (UNO Q: `turret/pos`
  notifications) only.
- Emergency stop: `std_msgs/msg/Bool` on `/turret/emergency_stop`. `true`
  stops the tracker from commanding and makes the bridge send `STOP` and drop
  commands. On hardware both start engaged and need an explicit `false`
  (`require_estop_release`), e.g.
  `ros2 topic pub --times 3 /turret/emergency_stop std_msgs/msg/Bool "{data: false}"`.
- Actuator readiness: `std_msgs/msg/Bool` on `/turret/actuator_ready`
  (transient-local), true only while the bridge is connected, settled, not
  e-stopped and writing commands.
- Diagnostics: `/turret/diagnostics` from the tracker and the bridge.

Simulation compatibility notes

- Start from `applications/pantiltros/pantiltros/urdf/pan_tilt_urdf.xacro`
  for geometry, joint names, limits, and the payload link.
- Preserve the current PanTiltROS default joint names: `pan_joint` and
  `tilt_joint`.
- Simulate servo position limits, speed, acceleration, quantization, backlash,
  current/temperature diagnostics, command latency, and emergency-stop torque
  disable behavior.
- The segmentation model should track the current Ultralytics YOLO
  instance-segmentation checkpoint at implementation time. As of June 2026,
  public Ultralytics YOLO26 material describes instance segmentation support.
  Use the latest available YOLO26 `-seg` checkpoint in the installed
  Ultralytics release for GPU validation, the smallest available YOLO26
  segmentation checkpoint for CPU smoke tests, and re-verify the exact
  checkpoint name before implementation.

Useful hardware and software references

- Arduino UNO R4 WiFi:
  https://store.arduino.cc/products/uno-r4-wifi
- Arduino UNO Q: https://docs.arduino.cc/hardware/uno-q, the Bridge
  library https://github.com/arduino-libraries/Arduino_RouterBridge and the
  router https://github.com/arduino/arduino-router
- micro-ROS Arduino:
  https://github.com/micro-ROS/micro_ros_arduino
- PanTiltROS:
  https://github.com/adityakamath/pan_tilt_ros
