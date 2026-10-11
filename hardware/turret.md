# Turret Hardware Shopping List

Goal: build a pan-tilt perception turret that can be simulated and validated
against the `applications/pantiltros` ROS 2 control stack. The turret should
segment a realistic camera scene, select a target object, and drive pan/tilt
servos to keep that object centered while it moves through the scene.

The pan/tilt mechanism, its base (tripod adapter) and the camera mounts are
printed: CadQuery sources, STLs and printing and assembly instructions in
[hardware/parts/turret/README.md](parts/turret/README.md); the generated
part and fastener tables are under "3D-printed parts" below.

## Purchased parts

Baseline hardware

- 1 x two-axis pan-tilt mechanism compatible with `pan_tilt_ros`: printed
  (see "3D-printed parts").
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
  - or no microcontroller: the compute host drives the servos itself over its
    own UART or USB through a bus servo adapter such as the Waveshare Bus
    Servo Adapter (A) (the RACECAR Turret's Jetson). See "Direct from a Linux
    host" below.
- 1 x USB-C data cable (UNO R4 WiFi: to the host; UNO Q: for flashing and
  setup) and, for the UNO Q, a 5 V / 3 A USB-C power supply or a 7-24 V supply
  on VIN.
- 1 x half-duplex TTL serial interface, level shifter, or ST3215/STS3215
  Arduino shield/cape between the Arduino hardware UART and the serial bus
  servos, with its MCU side at the board's logic level (5 V for the UNO R4,
  3.3 V for the UNO Q). Do not power the servos from the Arduino board.
- 1 x optional USB serial bus servo adapter for one-time servo ID, midpoint,
  and limit setup before the Arduino firmware takes over runtime control.
- 1 x rigid base or tripod mount with enough stiffness to avoid camera shake:
  printed (the base plate, with a captive 1/4"-20 tripod nut).
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

For the printed mechanism:

- The servos are the **12 V** versions (ST3215 30 kg.cm; the 7.4 V
  ST3215-C001 is 19.5 kg.cm): the Waveshare Bus Servo Adapter (A) takes
  9-12.6 V and passes it to the servos. Two servos at locked rotor draw
  2 x 2.7 A at 12 V; the adapter's servo port is marked 6 A maximum. Each
  servo needs an output and an idler horn with 4 x M3 on a 14 mm circle
  (Waveshare's pack shows two aluminium horns per servo; Feetech sells the
  STS3215 without accessories: check yours).
- The camera plates take the **Arducam B0385** board (38 mm, M2 holes on a
  34 mm square, the RACECAR Turret's camera) or any camera with a 1/4"-20
  thread in its base (RealSense D435i, cased UVC cameras) up to about 150 g.
- The base plate has standoffs for the Arduino UNO R4 WiFi or UNO Q and for
  the Waveshare Bus Servo Adapter (A), the half-duplex interface (also
  needed for the direct-from-host option). Its UART side appears to run at
  3.3 V (its schematic powers the half-duplex buffers from 3.3 V): direct for
  the UNO Q and a Linux host; with the 5 V UNO R4 WiFi check that the board
  reads the adapter's 3.3 V TX reliably, or add a level shifter.
- The emergency stop must switch the servo supply's current (5.4 A at 12 V
  with both servos stalled). A 22 mm control switch is rated for much less
  DC current: use an enclosed e-stop rated for the load, or let its contact
  open a relay in the servo supply.
- Screws, inserts, the tripod nut and rubber feet: the generated fastener
  table below.

Why these parts are bought, not printed:

| Part | Why it stays purchased |
|---|---|
| Servos and their horns | Motors and gearboxes; the aluminium horns carry the joint torque on a 25-tooth spline that printed plastic would strip. |
| Camera, controller, adapter, supply | Electronics. |
| Emergency stop, relay, fuses | Safety: rated parts that must open the servo supply every time. |
| Screws, inserts, nuts | Metal threads; printed threads strip and creep. The tripod thread is a steel 1/4"-20 nut held captive in the base plate. |
| Cables, spiral wrap | Flex with the pan and tilt motion. |

## 3D-printed parts

Designed in CadQuery (`hardware/parts/turret/`), validated and exported by
`tools/build_printable_parts.py`; STLs in [`parts/turret/stl/`](parts/turret/stl/).
Printer: Bambu Lab P1S. Plates, settings, post-processing and assembly:
[parts/turret/README.md](parts/turret/README.md). Print the "fit test
first" parts before the rest; print one of the two camera plates, and the
base plate only for the standalone turret (the RACECAR Turret stands on its
own plate). Times and masses are estimates.

<!-- BEGIN GENERATED: printed-parts (tools/build_printable_parts.py) -->
| Part | Qty | Material | Bambu Studio settings (P1S, 0.4 mm) | Est. time (each) | Est. mass (each) | File |
|---|---|---|---|---|---|---|
| Base plate (standalone turret, tripod adapter) | 1 | PETG | 0.20 mm, 3 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 2 h 26 min | 88 g | [base_plate.stl](parts/turret/stl/base_plate.stl) |
| Camera plate, Arducam B0385 | 1 | ASA | 0.20 mm, 3 walls, 40% gyroid, no supports | 11 min | 5.5 g | [camera_plate_b0385.stl](parts/turret/stl/camera_plate_b0385.stl) |
| Camera plate, 1/4"-20 shelf | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 26 min | 12 g | [camera_plate_tripod.stl](parts/turret/stl/camera_plate_tripod.stl) |
| Camera yoke (tilt stage) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 21 min | 9.2 g | [camera_yoke.stl](parts/turret/stl/camera_yoke.stl) |
| Pan base (pan servo) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 1 h 09 min | 36 g | [pan_base.stl](parts/turret/stl/pan_base.stl) |
| Pan hub (on the pan horn) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 50% gyroid, no supports | 22 min | 11 g | [pan_hub.stl](parts/turret/stl/pan_hub.stl) |
| Tilt bracket (on the pan horn) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 34 min | 16 g | [tilt_bracket.stl](parts/turret/stl/tilt_bracket.stl) |
| **Total, all pieces** | | | | **5 h 29 min** | **177 g** | |
<!-- END GENERATED: printed-parts -->

### Fasteners

Totals for one turret with both camera plates, from the parts' metadata
(screws are listed with the part their head bears on). Buy 10-20 % spares.

<!-- BEGIN GENERATED: fasteners (tools/build_printable_parts.py) -->
| Item | Qty | Where |
|---|---|---|
| 1/4"-20 UNC hex nut | 1 | tripod thread, captive under the pan base (base_plate, 1) |
| 1/4"-20 UNC x 3/8" button head screw | 1 | camera to the tripod plate shelf (camera_plate_tripod, 1) |
| M2 x 10 pan head screw (or PA2.0 x 10 self-tapping) | 2 | pan servo to the pan base, from below into the idler-face holes (pan_base, 2) |
| M2 x 3 mm brass heat-set insert (CNC Kitchen type, 3.2 mm hole) | 8 | Bus Servo Adapter standoffs, from the top (base_plate, 4); standoffs, from the top (camera_plate_b0385, 4) |
| M2 x 5 socket head cap screw | 8 | Bus Servo Adapter (A) to the base plate standoffs (base_plate, 4); Arducam B0385 board to the camera plate standoffs (camera_plate_b0385, 4) |
| M2 x 6 socket head cap screw (or PA2.0 x 6 self-tapping) | 2 | tilt servo into the tilt bracket's -y wall (heads in counterbores, 4 mm into the case) (tilt_bracket, 2) |
| M2 x 8 socket head cap screw (or PA2.0 x 8 self-tapping) | 2 | tilt servo into the tilt bracket's +y wall (heads in counterbores, 4 mm into the case) (tilt_bracket, 2) |
| M3 x 10 socket head cap screw | 4 | pan base to the base plate or turret plate (pan_base, 4) |
| M3 x 3 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 8 | Arduino standoffs, from the top (base_plate, 4); crossbar, from the front, through holes pressed flush (camera_yoke, 4) |
| M3 x 5 socket head cap screw | 16 | Arduino UNO to the base plate standoffs (base_plate, 4); camera yoke to the tilt servo's output and idler horns (no longer: the tip must stay inside the horn flange) (camera_yoke, 8); pan hub to the pan servo's horn (no longer: the tip must stay inside the horn flange) (pan_hub, 4) |
| M3 x 5.7 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 8 | pan base screws, from the top (base_plate, 4); column top face (tilt bracket screws) (pan_hub, 4) |
| M3 x 6 socket head cap screw | 8 | camera plate to the camera yoke's crossbar inserts (camera_plate_b0385, 4); tripod plate to the camera yoke's crossbar inserts (camera_plate_tripod, 4) |
| M3 x 8 socket head cap screw | 4 | tilt bracket to the pan hub (heads recessed in the floor) (tilt_bracket, 4) |
| adhesive rubber foot, 10-12 mm | 4 | base plate corners, for bench use (base_plate, 4) |
| zip tie 2.5 mm | 6 | servo bus and USB cables on the base plate (base_plate, 3); camera USB cable at the camera plate's bottom slot (camera_plate_b0385, 1); servo cables at the pan base's rear hook (pan_base, 1); cable loop at the tilt bracket's rear hook (tilt_bracket, 1) |
<!-- END GENERATED: fasteners -->

## Integration, firmware and ROS contract

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

Direct from a Linux host (no Arduino)

The bridge's `sts` transport (`TURRET_ARDUINO_TRANSPORT=sts`) drives the two
servos from any Linux host with a UART or a USB bus servo adapter, without a
microcontroller: Feetech STS packets at 1 Mbps on the host's serial port,
through a half-duplex adapter that also feeds the servos from the servo
supply. The RACECAR Turret (`systems/racecar_turret`) uses it on the car's
Jetson: `compositions/racecar_turret_hardware.yaml` with
`applications/pantiltros/config/jetson_uart.env` (`/dev/ttyTHS1`, the 40-pin
header UART on pins 8 TX and 10 RX, 1000000 baud).

- Same behaviour as the firmware: `turret_arduino_bridge.py` carries a Python
  port of `src/sts_servo.*` and of the `Controller` in `src/turret_core.*`:
  the packets above (sync write of registers 41-47, present position read of
  register 56, torque enable in register 40, checksums, status packets and
  their fault bits), the limits clamping every command, torque on at the
  first `SET`, the 500 ms watchdog and `STOP` holding the measured position,
  25 Hz feedback reported only when both servos answered, servo timeouts and
  faults reported at most once per second, nothing written before the first
  `SET` or `STOP`, and adapters that echo TX back to RX. The settings of
  `src/turret_config.h` are bridge parameters in
  `applications/pantiltros/config/arduino_bridge.yaml` with the same defaults
  (`pan_servo_id` 12, `tilt_servo_id` 13, `*_zero_steps` 2048,
  `*_direction` -1, `servo_speed` 2000, `servo_acceleration` 50,
  `stop_disables_torque` false), plus `servo_reply_timeout_s` (10 ms; the
  firmware waits 3 ms). `/turret/joint_states`, `/turret/actuator_ready`,
  `/turret/emergency_stop` and the diagnostics are those of the other
  transports; the diagnostics add `servo_ids`, `servo_state` (idle, active,
  holding) and `bad_packets`. `tests/test_turret_bridge_sts.py` checks the
  packets against the reference bytes of the firmware's host test (from
  `SCServo_Linux`) and against the compiled firmware code on random input,
  and runs the firmware test's scenarios against a simulated servo bus
  (echoing adapter, missing servo, fault bits, watchdog, `STOP`, limits).
- Safety difference: there is no microcontroller watchdog. The watchdog and
  `STOP` run in the bridge process, so if the host, the container or the
  bridge dies, or the UART link breaks, nothing holds the servos: the ST3215s
  keep their last goal (already clamped to the joint limits), finish the move
  in progress and stay there with torque on. The physical emergency-stop
  switch that cuts the servo supply is the hardware safety, and with this
  transport the only stop that does not depend on the host.
- The other host-side option is the upstream C++ node `pan_tilt_ctrl_node`
  (package `pan_tilt_ros`, built in the PanTiltROS image with Feetech's
  `SCServo_Linux`; `usb_port` and `baud_rate` in
  `pantiltros/config/ctrl_config.yaml`). It is not used here because its
  interface and behaviour differ from the turret contract: `/joint_commands`
  and `/joint_states` instead of the `/turret/*` topics, no
  `/turret/emergency_stop` or `/turret/actuator_ready`, a joystick button
  that turns torque off as its stop, limits in servo steps, and a joint state
  with whichever servos answered. The bridge's `sts` transport keeps one ROS
  contract and one safety behaviour for the UNO R4, the UNO Q and the host
  itself, so the tracker, the UI and the compositions do not change with the
  hardware.

Wiring to a Jetson's 40-pin header, Waveshare Bus Servo Adapter (A)
(https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)) with its jumper cap
on A (UART mode; B is USB mode):

| Jetson 40-pin header | Bus Servo Adapter (A) |
|---|---|
| pin 8, UART TX (3.3 V) | TX |
| pin 10, UART RX (3.3 V) | RX |
| pin 6, GND | GND |

- Waveshare's wiki, for UART control from a Raspberry Pi Zero, ESP32,
  Arduino or STM32 board: "the connection must be RX-RX, TX-TX", with the
  jumper cap on A. The adapter's labels are thus from the host's side: host
  TX to the pin labelled TX. If the servos do not answer, swap TX and RX
  first.
- Logic level: Waveshare does not document the logic level of the adapter's
  host UART pins (its UART example is a Raspberry Pi Zero, whose GPIO is
  3.3 V, but that is no specification), and the Jetson header is 3.3 V and
  not 5 V tolerant. Before connecting the Jetson, power the adapter and
  measure its TX pin's idle voltage (and RX's) to GND. If it is above 3.3 V,
  put a bidirectional level shifter rated for 1 Mbps in between (e.g. a
  BSS138-based module or a TXS0104E). The USB connection (jumper on B)
  avoids the question entirely.
- Servos on the adapter's servo connectors; the servo supply into its DC jack
  (5.5 x 2.1 mm) or screw terminal through the emergency-stop switch
  (Waveshare: "Input voltage: 9~12.6V (should match servo voltage)"). Do not
  power the servos from the Jetson.

Bring-up on the Jetson:

1. Enable pins 8 and 10 as UART: `sudo /opt/nvidia/jetson-io/jetson-io.py`,
   "Configure Jetson 40pin Header", "Configure header pins manually", select
   `uarta`, save and reboot. Check that no serial console or other program
   holds the port: `sudo fuser -v /dev/ttyTHS1`. The Tegra `ttyTHS` UART
   supports 1 Mbps (`stty` accepts up to 4 Mbps). Which `ttyTHS` device is
   the header UART differs between Jetson modules and JetPack releases; the
   loopback below confirms it.
2. Loopback test, adapter disconnected and a jumper wire between pins 8 and
   10 (as root or a member of `dialout`):

   ```bash
   stty -F /dev/ttyTHS1 1000000 raw -echo
   timeout 2 cat /dev/ttyTHS1 & sleep 0.5; echo hello > /dev/ttyTHS1; wait   # prints hello
   ```

   `-echo` matters: with the loopback, the tty's own echo would send every
   received byte out again.
3. Wire the adapter as above, switch the servo supply on and start the
   composition with the emergency stop engaged. `/turret/diagnostics` should
   say "servo bus connected" (`servo_state` idle) and `/turret/joint_states`
   should arrive at 25 Hz before the release. "servo 12 (pan_joint) timeout"
   and "servo 13 (tilt_joint) timeout" together mean no servo answers:
   TX and RX swapped, the jumper on B, the wrong port, no servo supply, or a
   header UART that does not transmit (below).

Header UART TX on some JetPack releases: an NVIDIA forum thread
(https://forums.developer.nvidia.com/t/ttyths1-uart1-40-pin-header-pins-8-10-tx-pad-does-not-drive-on-jetpack-7-l4t-r39-2-orin-nano-super-devkit/373290)
reports that on a Jetson Orin Nano Super developer kit with JetPack 7
(L4T R39.2), `/dev/ttyTHS1`'s TX pad (pin 8) does not drive: it sits near
0.3 V instead of idling at 3.3 V, RX works, and the loopback returns nothing.
Enabling `uarta` with jetson-io did not change it (the pads stay unclaimed in
the pinmux), a second user reproduced it, and the thread has no confirmed
fix (NVIDIA suggested a pinmux generated with its pinmux spreadsheet and
flashed). If the loopback fails, measure pin 8 at idle. The fallback is the
same adapter over USB with its jumper on B (Waveshare lists the Jetson Orin
Nano among the USB hosts): only the device path changes, same transport and
1000000 baud. Set `TURRET_ARDUINO_PORT=/dev/serial/by-id/usb-...` in the
shell (`racecar_turret_hardware.yaml` passes that device in and gives it to
the bridge) or in `jetson_uart.env`. On other Linux hosts, likewise, put
their UART's or USB adapter's device in `TURRET_ARDUINO_PORT`.

Status: written and tested against the firmware's reference packets and a
simulated servo bus only. No bus servo adapter or ST3215 was attached to the
development machine, so nothing of it has run on hardware. Still to check:
the adapter's UART logic level and TX/RX labels, `/dev/ttyTHS1` at 1 Mbps
on the car's JetPack, reply times against `servo_reply_timeout_s` (native
UART and USB), whether the adapter echoes, torque enable, motion, holding,
limits, directions and zero offsets, the servo fault bits, and the CPU load
of the 50 Hz poll timer on the Jetson.

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
  notifications; `sts`: the servos' position replies) only.
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
