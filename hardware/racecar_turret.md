# RACECAR Turret bill of materials

Goal: the RACECAR Neo (`hardware/racecarneo.md`) with the PanTiltROS
turret on a raised plate behind the lidar, so that it drives (SLAM Toolbox +
Nav2) and keeps a selected object centred in the turret camera at the same
time, managed from one web page on the car. Interfaces:
`systems/racecar_turret/interfaces.md`; frames:
`systems/racecar_turret/urdf/racecar_turret.urdf.xacro`; bring-up and
safety: `systems/racecar_turret/README.md`; software stack:
`compositions/racecar_turret_hardware.yaml`.

The turret has no microcontroller: the Jetson drives the four ST3215 serial
bus servos (pan, tilt, and the foam dart launcher's priming actuator and
trigger) itself, from its 40-pin header UART through a Waveshare Bus Servo Adapter (A), with three jumper
wires (TX, RX, GND).

The page reaches the operator over the RACECAR Neo's long-range Wi-Fi HaLow
link, from the ground station of [hardware/ground_station.md](ground_station.md)
(shared with the aerodrone); the 2.4 GHz RC link and the physical
emergency stop stay the stops.

## Compatibility notes

- **No watchdog between the Jetson and the servos.** The ST3215s run in
  position mode: if the Jetson, the container or the bridge dies, they hold
  their last goal with torque on. The software stop (`/turret/emergency_stop`,
  the UI's Disarm and Stop all) needs that software to be running; the
  hardware safety is the physical emergency stop that cuts servo power. It is
  required, not optional.
- **Jetson header UART.** Pins 8 (UART1 TX) and 10 (UART1 RX) are
  `/dev/ttyTHS1` once enabled as a UART with jetson-io (`sudo
  /opt/nvidia/jetson-io/jetson-io.py`, Configure the 40-pin header, pins 8/10
  as `uarta`). The Tegra high-speed UART runs the servos' 1 Mbps (`stty`
  accepts up to 4 Mbps). There are reports of that UART not transmitting on
  some JetPack releases, for example
  [JetPack 7 / L4T R39.2 on the Orin Nano Super developer kit](https://forums.developer.nvidia.com/t/ttyths1-uart1-40-pin-header-pins-8-10-tx-pad-does-not-drive-on-jetpack-7-l4t-r39-2-orin-nano-super-devkit/373290)
  (the pin mux is never applied; jetson-io fails on that board): run the
  loopback test of the README before wiring the servos.
- **USB fallback.** If the header UART does not work, move the adapter's
  jumper to B and plug its USB-C into the Jetson: the same software drives
  it on its `/dev/serial/by-id/...` port (README). That also avoids the logic
  level question below. No extra part is needed.
- **Logic level.** Waveshare does not document the logic level of the
  adapter's UART pins, and the Jetson header is 3.3 V and not 5 V tolerant.
  Before connecting it to the Jetson, power the adapter and measure its TX
  pin's idle voltage to GND: it must not exceed 3.3 V. Only if it does,
  put a bidirectional 3.3 V / 5 V level shifter that handles 1 Mbps
  (a BSS138 four-channel module or a TXS0104E) between the two, or use the
  USB fallback.
- **Wiring labels.** Waveshare's
  [wiki for this board](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A))
  says the connection "must be RX-RX, TX-TX": its UART pin labels name the
  host's signals. Connect Jetson pin 8 (TX) to the adapter pin labelled TX,
  pin 10 (RX) to RX, and a Jetson GND pin (6) to GND. If the servos do not
  answer, the first thing to try is swapping TX and RX.
- **ST3215 at 12 V.** The servos take 6-12.6 V and are specified at 12 V
  (30 kg.cm, 0.222 s/60 deg, 2.7 A locked rotor, 200 mA no load); the
  adapter takes 9-12.6 V and passes it to the servos. The car runs on 3S or
  more (the VESC 6 needs at least 9 V, `hardware/racecarneo.md`); a 3S pack
  (9.0-12.6 V) sits inside the adapter's range but sags with charge (speed
  and torque with it) and is at the 12.6 V ceiling when full. So the servo
  rail is a regulated 12 V from the drive battery. The URDF's servo speed and the tracker's rate limits
  assume 12 V.
- **Jetson software.** Native ARM64 builds for nvblox and PanTiltROS target
  JetPack 7.2 on Orin, with Isaac ROS 4.6 and CUDA-enabled PyTorch. Build
  and run the acceptance checks in [Jetson GPU images](../applications/jetson.md)
  before bring-up. An x86 GPU host is not required by the build design;
  performance on the physical Orin Nano still needs measurement.
- **Compute budget (Orin Nano 8 GB).** YOLO and nvblox share one small GPU
  and 8 GB of memory with the CPU. As configured for a desktop GPU they
  may not both keep up; reduce, roughly in this order:
  1. Depth: done. nvblox maps the RealSense's depth aligned to its colour
     camera (`/camera/aligned_depth_to_color/image_raw`, 15 Hz), and Depth
     Anything (48 ms per frame on a laptop RTX 4000 Ada, and 9 times the
     true depth from this camera height) runs only under the composition's
     `depthanything` profile, for comparison
     (`applications/depthanything/README.md`, "On the car"). The RealSense's
     alignment and colour conversion run on the CPU.
  2. YOLO: export the tracker's model to TensorRT FP16 (`yolo export
     format=engine half=True`), at `image_size` 416-480 instead of 640, with
     the class prompts fixed at export so the text encoder does not run; a
     detection model instead of segmentation if box centres are good
     enough for aiming. 10-15 Hz is enough for the tracker, which
     compensates latency.
  3. nvblox: 0.10 m voxels instead of 0.05 m, depth integration at 5 Hz or
     less, no mesh or colour integration (only the 2D ESDF slice Nav2 uses).
     Its dynamic mode (`systems/racecarneo/config/nvblox_hardware.yaml`)
     took 0.32-0.48 CPU cores in Isaac against 0.17-0.19 in static mode,
     and 0.89 GB of GPU memory; its dynamic layer was evaluated with 15 Hz
     depth, so check people still decay at a lower rate.
  4. The rest: `nvpmodel` at the highest power mode and `jetson_clocks`; an
     8 GB swap file on the NVMe so a load spike does not kill a container;
     watch `tegrastats` with the whole stack running before driving; keep
     Foxglove off camera topics.
- **Lidar scan plane.** The plate is above the RPLIDAR (scan plane 0.18 m,
  top about 0.20 m, plate 0.24 m) so only its four posts cross the scan
  plane, behind the lidar at bearings of about +-162 and +-169 deg, 0.14 and
  0.23 m away. They are inside Nav2's 0.22 m robot radius: unmasked, they
  become lethal cost inside the footprint and the collision monitor
  (`min_points` 4) stops the car. Mask them with laser_filters before every
  `/scan` consumer: `systems/racecar_turret/config/scan_filter.yaml`, a
  `LaserScanBoxFilter` that blanks only the returns inside a box around the
  posts in `base_link` (checked against a synthetic scan: the posts' beams
  removed, the rest kept). The alternative is
  `LaserScanAngularBoundsFilterInPlace` over the rear +-25 deg in the
  lidar's own angles (check which way the driver's 0 rad points), which also
  hides whatever is behind the posts. Thin posts mask fewer beams. On the
  printed RACECAR deck the posts (x = -57 and -143 mm, y = +-48 mm) also stand
  inside the rear sector of about +-23 deg that the camera arm's fin hides
  from the lidar (the fin is inside the C1's 50 mm blind range and returns
  nothing), so they return nothing even before the filter; the assembly
  check of `hardware/parts/racecar_turret` verifies both.

- **Long-range link.** The page goes over the car's Wi-Fi HaLow link
  (`hardware/racecarneo.md`, "Long-range link"), a few Mbit/s at range:
  `compositions/racecar_turret_hardware.yaml` sends the turret camera to
  the page at 4 Hz, 480 px, JPEG quality 60 (about 0.5 Mbit/s); the tracker
  still works on the camera's full frames. The tracker checks every Fire
  gate on the car, and a status that goes stale over a poor link disables
  Fire. HaLow is not a safety link: the physical stop and the RC relay are.

- **Line of fire.** The foam dart launcher's bore points at the car's own
  RealSense and its arm below about -28 deg of tilt within about -6 to +12
  deg of pan (the RACECAR Turret's assembly check maps it). The fire cycle
  must refuse a shot there; nothing else of the car is in the line of fire
  over the turret's range.

## Purchased parts

### Car

Everything in `hardware/racecarneo.md`, with these changes:

- **Emergency stop**: a two-pole latching emergency-stop switch (two
  normally-closed contacts, at least 15 A DC each at 16 V) instead of the
  car's single-pole one: one pole opens the drive motor's contactor as before
  (`hardware/racecarneo.md`: a 22 mm switch cannot break the motor current
  itself), the other the turret's 12 V servo rail (10.8 A with the four
  servos stalled; a contact that is not rated to break that DC current opens
  a relay in the rail instead, see "Servo power" under Turret). Mount it on
  top where it can be hit while the car moves (the RACECAR deck's printed
  e-stop tower). Not printed, not replaced by software: it is the only stop
  that works when the Jetson does not.
- **USB**: the turret camera is one more USB device next to the RealSense,
  the lidar and the drive electronics: the developer kit's fourth USB-A port
  takes it (the car uses three); add a powered USB hub only for more devices
  (for example the servo adapter's USB fallback), and give the RealSense its
  own port on the Jetson.
- **Top deck**: the turret plate's four posts need two M3 x 3 inserts each
  in the top deck behind the lidar; the printed rear deck has them, for posts
  at the corners of an 86 x 96 mm rectangle centred 0.10 m behind
  `base_link` (centre of the chassis), i.e. 0.18 m behind the lidar.
- **Long-range link**: the RACECAR Neo's (`hardware/racecarneo.md`,
  "Long-range link"; the ground side in `hardware/ground_station.md`): the
  HaLowLink 2 upright behind the camera arm, in front of the turret plate,
  and its antenna at the rear-right corner of the deck. Neither is in the
  turret's sweep, its line of fire or its camera's view at level tilt (the
  assembly check below: the launcher passes about 19 mm over the antenna's
  tip at full nose-up tilt). Only with the **120 mm** antenna of that list:
  the HaLowLink's own antenna (about 200 mm) would be hit by the launcher,
  and the same series' 868 MHz version for the EU (135.5 mm) would come
  within about 3 mm of it; in the EU fit an 863-870 MHz dipole no longer
  than 120 mm.

### Turret

- 4 x **Waveshare ST3215** serial bus servo, 12 V, 30 kg.cm (or Feetech
  STS3215, 12 V version), with their metal horns. IDs 12 (pan), 13 (tilt),
  14 (the foam dart launcher's priming actuator) and 15 (its trigger), set
  once before assembly (README). The launcher's two need their output
  horns only. (No smaller STS servo runs on the 12 V rail: the STS3032 is
  4.8-6 V.)
- **Foam dart launcher**, the purchased parts of `hardware/turret.md`
  ("Foam dart launcher" and its power and safety section): the K&S 8140
  brass barrel (cut to 45 mm), the 1.5 N/mm plunger spring (**no stiffer
  spring**), the clip and sear springs, two O-rings, a 3 mm dowel pin,
  **Elite-size foam darts only**, eye protection for everyone in range and a
  chronograph for the bring-up (**at most about 30 m/s**).
- 1 x **Waveshare Bus Servo Adapter (A)**, jumper on A (UART).
- 3 x female-female jumper wires, about 40 cm (Dupont, or JST to suit the
  adapter): Jetson pins 8, 10 and 6 to the adapter's TX, RX and GND. The
  header is at the front of the car and the adapter under the turret plate:
  the run along the deck and up a post is about 0.38 m.
- 1 x servo bus cable, about 20 cm, adapter to pan servo, 1 x about
  15 cm, pan servo to tilt servo, 1 x about 25 cm, tilt servo to the
  launcher's actuator servo across the tilt joint, and 1 x about 25 cm,
  actuator to trigger servo (the servos' 3-pin daisy-chain cables; one
  usually comes with each servo).
- 1 x **turret camera: Arducam B0385**, 1 MP OV9782 **global shutter colour**
  USB camera board (UVC, USB 2.0, MJPEG 640x480 at up to 100 fps, 5 V
  200 mA), with its low-distortion M12 lens of **70 deg horizontal field of
  view**. Global shutter keeps targets sharp while the car and the turret
  move; colour, because YOLO is trained on colour images. Alternatives:
  - ELP USBGS720P02-L36, colour version (AR0144 global shutter, 1280x720 at
    60 fps MJPEG, M12): with its 3.6 mm lens about 56 deg horizontal on that
    1/4" sensor; a 2.8 mm M12 lens gives about 69 deg.
  - e-con Systems See3CAM_24CUG (AR0234 colour global shutter, USB 3,
    1920x1200): its stock lens is 104.6 deg horizontal, too wide for small
    targets; fit a longer M12 lens (about 4.5 mm for 65 deg across the full
    sensor width) and check the field of view at 640x480.
  Whichever camera: calibrate it at 640x480 and replace
  `systems/racecar_turret/calibration/turret_camera.yaml` (nominal values).
- 1 x thin, flexible USB 2.0 cable for the camera, about 1 m (the B0385's
  supplied cable is 1 m; the route over the pan loop, down a post and along
  the deck to the Jetson is about 0.8 m), with a right-angle plug at the
  camera (silicone-jacketed "soft" cable; a stiff USB 3 cable fights the pan
  and tilt servos).
- Servo power, from the drive battery (four servos):
  - 1 x **12 V buck-boost regulator**, at least **12 A** continuous
    (150 W), input range covering at least 9-13 V (3S; wider if the car runs
    4S). Sizing: four ST3215 at locked rotor draw 4 x 2.7 A = **10.8 A** at
    12 V (130 W); tracking a person draws well under 1 A per servo, priming
    about 1 A on the actuator for under a second, but the servos
    accelerating together, a jammed turret or a hard stop must not brown
    out the rail. At 90 % efficiency the input draws up to about 15 A from a
    sagging 3S pack (9.6 V).
  - 1 x inline blade fuse holder with a **20 A** fuse on the regulator
    input, at the battery tap, and 1 x **5 A** fuse between the regulator
    output and the adapter: the adapter's servo port is marked 6 A maximum,
    less than the servos stalled, so this fuse carries normal operation and
    short peaks and opens on a sustained stall. Set the launcher servos'
    torque limits in the software: the actuator about 55 % (priming needs
    about 0.9 N m: the spring's 65 N on the 13.5 mm pinion; at full torque
    the pinion could push about 220 N), the trigger about 30 %; that brings
    the rail's worst case near 7.7 A.
  - 14 AWG silicone wire on the input side (20 A fuse), 18 AWG on the 12 V
    side; an XT60 Y-tap or a free output of the car's power distribution
    board, and a 5.5 x 2.1 mm DC plug (or the adapter's screw terminal) for
    the adapter.
  - The emergency stop's second pole in the 12 V line, through a relay if
    its contact is not rated to break 10.8 A DC (see Car): an automotive 12 V
    relay with contacts rated 20 A or more at 14 V DC (a 30 A ISO mini relay
    and socket), normally-open contact in the rail, the stop's
    normally-closed pole in series with its coil (about 150 mA) and a
    flyback diode (1N4007) across the coil: pressing the stop, a broken
    wire or a dead regulator all open the rail.
- Turret plate supports: 4 x **thin posts**, printed (`plate_post`, 12 mm,
  8 mm where they cross the scan plane; see "3D-printed parts"), or M3
  aluminium hex standoffs (5 mm across flats) stacked to length, or 4 mm
  carbon-fibre rods in printed sockets. Length: from the top deck to 0.24 m
  above the ground (the plate's underside, `turret/base_link`); measure your
  deck. Metal standoffs are stiffer than printed posts and thinner in the
  scan plane; on the printed deck the posts stand in the camera arm fin's
  shadow, so their thickness hides no extra scan.
- Cable management for +-90 deg of pan: 1 x 10 mm spiral wrap (about 0.5 m),
  small cable ties, adhesive-backed tie mounts. The camera cable and the
  tilt servo's bus cable cross the pan joint: give them a service loop of
  about half a turn around the pan axis, clamped on the pan stage and on the
  plate, so that at +-90 deg nothing pulls tight or reaches the lidar; check
  by hand through the full range before powering the servos. The printed pan
  base and tilt bracket have the tie hooks for that loop (about 120 mm of
  slack between them), the plate has the pass-throughs, and printed clips
  hold the cables on the posts.
- Fasteners: M3 x 8-12 mm screws and M3 heat-set inserts for the printed
  parts and the plate, M2 or M2.5 screws for the camera board (check its
  holes), the screws that come with the servos for their horns (with
  medium thread locker), Nylock nuts where printed parts clamp the deck. The
  generated fastener tables (below, and those of `hardware/racecarneo.md`
  and `hardware/turret.md`) list every one.

## 3D-printed parts

Printed on the reference Bambu Lab P1S (`hardware/parts/README.md`). The car
and the turret mechanism are the RACECAR Neo's and the turret's printed
parts; this platform adds three: CAD, STLs and the printing and assembly
instructions in [`parts/racecar_turret/`](parts/racecar_turret/README.md).

- **Turret plate** (`turret_plate`, `hardware/parts/racecar_turret`): 110 x
  116 mm, 5 mm thick, with the four post holes, the pan base's four inserts,
  a pass-through for the servo bus beside the pan base and one for the camera
  cable behind it. ASA (the car runs in the sun).
- **Four posts** (`plate_post`): 12 mm, narrowing to 8 mm at the scan plane,
  an M3 insert in the top end and a flange with two M3 screws into the deck's
  inserts at the bottom (the "deck feet").
- **Pan base**, **tilt bracket** (with the **pan hub** on the pan horn) and
  **camera mount** (`camera_yoke` and `camera_plate_b0385`): the turret's
  parts, `hardware/parts/turret` (pan axis on the plate's centre, tilt axis
  0.055 m above the pan joint origin, B0385 lens on the tilt axis about
  0.035 m in front of it).
- **Foam dart launcher** (the `launcher_*` parts): the turret's too. A
  spring-plunger launcher above the camera, its bore 48 mm above the
  camera's axis and parallel to it: plunger tube, plunger, sear and safety,
  a sliding breech block under a clip of six darts, the brass barrel in a
  printed clamp, a muzzle cap; the third servo primes it through an
  18-tooth, module 1.5 pinion (13.5 mm pitch radius) and a 38 mm rack
  stroke, the fourth lifts the sear. About 425 g with a full clip and both
  servos. Toy power only (at most about 30 m/s): `hardware/turret.md`.
- **Servo adapter mount**: four standoffs under the turret plate hold the Bus
  Servo Adapter (A), components down, its jumper and DC jack reachable from
  the side.
- **Cable clips**: the service-loop hooks on the turret's pan base and tilt
  bracket, and clips on the posts (`post_clip`).

Not printed: the emergency stop, the fuses and holders, the regulator and
all power wiring (safety-critical: bought, rated parts), and the servo horns
(metal, from the servos).

<!-- BEGIN GENERATED: printed-parts (tools/build_printable_parts.py) -->
| Part | Qty | Material | Bambu Studio settings (P1S, 0.4 mm) | Est. time (each) | Est. mass (each) | File |
|---|---|---|---|---|---|---|
| Plate post | 4 | ASA | 0.20 mm, 5 walls, 60% gyroid, no supports | 38 min | 12 g | [plate_post.stl](parts/racecar_turret/stl/plate_post.stl) |
| Post cable clip | 4 | PETG | 0.20 mm, 4 walls, 100% gyroid, no supports | 3 min | 1.1 g | [post_clip.stl](parts/racecar_turret/stl/post_clip.stl) |
| Turret plate (with the servo adapter mount) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 1 h 19 min | 45 g | [turret_plate.stl](parts/racecar_turret/stl/turret_plate.stl) |
| **Total, all pieces** | | | | **4 h 04 min** | **95 g** | |
<!-- END GENERATED: printed-parts -->

### Fasteners

Totals for this platform's three parts (screws are listed with the part
their head bears on); the car's and the turret's are in their own BOMs. Buy
10-20 % spares.

<!-- BEGIN GENERATED: fasteners (tools/build_printable_parts.py) -->
| Item | Qty | Where |
|---|---|---|
| M2 x 3 mm brass heat-set insert (CNC Kitchen type, 3.2 mm hole) | 4 | adapter standoffs under the plate (turret_plate, 4) |
| M2 x 5 socket head cap screw | 4 | Bus Servo Adapter (A) to the plate's standoffs (turret_plate, 4) |
| M3 washer | 4 | turret plate screws (turret_plate, 4) |
| M3 x 10 socket head cap screw | 4 | turret plate onto the posts' top inserts (turret_plate, 4) |
| M3 x 3 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 4 | pan base screws, from the top, through holes pressed flush (turret_plate, 4) |
| M3 x 5.7 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 4 | top end (plate screw) (plate_post, 4) |
| M3 x 8 socket head cap screw | 8 | plate post flange to the deck inserts (plate_post, 8) |
| zip tie 2.5 mm | 2 | servo bus and camera cable at the plate's pass-throughs (turret_plate, 2) |
<!-- END GENERATED: fasteners -->
