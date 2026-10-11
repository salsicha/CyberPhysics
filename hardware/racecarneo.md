# RACECAR Neo bill of materials

Goal: a physical 1/10-scale RACECAR Neo that matches the simulated
CyberPhysics pipeline (`systems/racecarneo`): forward RGB camera -> Depth
Anything -> nvblox/Nav2, with a 2D lidar for SLAM Toolbox, an IMU,
odometry, and Ackermann command output, run by
`compositions/racecarneo_hardware.yaml`.

The chassis is a Traxxas Slash 4x4 class 1/10 short-course truck. Everything
the car adds sits on a printed deck on four TPU-isolated post adapters: the
RPLIDAR C1 on a short mast, the RealSense on an arm over the lidar, the
Jetson in front of the lidar, the IMU at the car's centre, the VESC, the
power converters and the emergency stop at the rear. Printing and assembly
instructions: [hardware/parts/racecarneo/README.md](parts/racecarneo/README.md).

- Sensor positions are the URDF's (`systems/racecarneo/urdf/racecarneo.urdf.xacro`,
  `base_link` on the ground under the chassis centre): lidar scan plane
  (0.08, 0, 0.18) m, RealSense `camera_link` (0.18, 0, 0.22) m; the printed
  mounts put them there to within 0.5 mm (assembly check). The IMU ends up
  at (0, 0, 0.142) m instead of the URDF's nominal 0.08 m (see the notes).
- The deck adds about 1.35 kg (printed parts, electronics, cables; computed
  by the assembly check) to the 2.4-2.6 kg truck.

## Compatibility notes

- **Chassis and wheelbase.** The Traxxas Slash 4x4 (68154-4, 68086-4,
  68386-4) is 568 x 296 x 193 mm with a **324 mm** (12.75 in) wheelbase,
  296 mm track and 72 mm ground clearance (traxxas.com). The 335 mm figure
  quoted before belongs to the two-wheel-drive Slash.
  `cmd_vel_to_ackermann.yaml`'s 0.33 m is within 2 %; measure your steering
  lock as that file says.
- **Body posts.** The deck clamps the chassis' round body posts with printed
  adapters. Traxxas does not publish the posts' positions or diameter, and
  the current Slash 4x4 kits use "clipless" latching body mounts instead of
  posts and clips: measure your chassis and set `POSTS` and `POST_D` in
  `hardware/parts/racecarneo/_frame.py` (the adapters are fit-test parts),
  or adapt the adapter sleeve to the latching mounts.
- **Battery voltage.** The VESC 6 MkVI needs at least 3S (Trampa: 9-48 V,
  3S-12S). Use **3S** packs (11.1 V nominal, what
  `systems/racecarneo/config/sensor_models.yaml` models), not 2S. The stock
  battery hold-down takes packs up to 25 mm tall (165 x 50 x 23-25 mm
  compartment); most 3S packs are taller and need Traxxas' extended
  hold-down 5827X (owner's manual). Not every VESC runs on 3S: the Flipsky
  Mini FSESC 6.7, for example, needs 4S or more.
- **Motor.** `cmd_vel_to_ackermann.yaml` limits the car to 0.35 m/s. The
  stock Velineon motor is sensorless; a sensorless brushless motor starts
  and runs roughly at such low speeds, and its odometry from the VESC is
  coarse there. A sensored 540-size brushless motor (hall cable to the
  VESC) gives smooth low-speed torque and better wheel odometry; keep the
  stock motor for first tests.
- **Jetson software.** `compositions/racecarneo_hardware.yaml` states that
  arm64 is not supported as is: the nvblox image installs x86_64 Isaac ROS
  packages and the Depth Anything image CPU-only PyTorch on arm64. The
  Jetson in this list cannot run the composition until those images have
  Jetson builds; until then run the GPU services on an x86 GPU computer on
  the same network, or feed nvblox the RealSense depth instead of Depth
  Anything.
- **Drivers the composition expects but does not start.** It needs
  `/scan` (frame `racecarneo/lidar_link`), `/imu/data_raw`
  (`racecarneo/imu_link`), odometry and an `/ackermann_cmd` consumer from
  "hardware drivers outside this file": an RPLIDAR driver for the C1
  (Slamtec's `sllidar_ros2`), the VESC driver with its Ackermann and
  odometry nodes (`vesc_driver`, `vesc_ackermann`), and a BNO08x driver.
  The RealSense's own IMU (`/camera/imu`, in the camera's frame) is not the
  `/imu/data_raw` the stack expects.
- **IMU height.** The URDF's `imu_link` is at 0.08 m, inside the chassis;
  the printed mount puts the IMU on the deck at (0, 0, 0.142) m. An IMU's
  readings do not depend on its height; set `imu_joint` z to 0.142 if a
  consumer uses the lever arm.
- **Emergency stop.** A 22 mm mushroom switch is rated for control
  circuits (Schneider XB5AS8442: 10 A thermal, 0.55 A DC-13 at 125 V), not
  for 30-80 A of motor current: its contact opens the coil of a contactor
  (or the control loop of a MOSFET battery switch) in the battery-to-VESC
  line. The compute branch stays on, so logs survive a stop.
- **Steering servo power.** The VESC 6 MkVI drives the steering servo's
  signal from its servo port, but its 5 V output gives 1 A; a 20 kg.cm
  servo draws 2-3 A at stall. Power the servo from a separate 6 V BEC.
- **USB.** The developer kit has four USB 3.2 Type-A ports and a USB-C:
  RealSense (USB 3, a blue port of its own), lidar (USB adapter), VESC
  (USB) and one spare. No powered hub is needed for the car alone.

## Purchased parts

### Chassis

- 1 x **Traxxas Slash 4x4** (VXL or BL-2s), or an equivalent 1/10 4WD
  short-course or rally chassis with round body posts and spare parts
  available. Keep its steering servo if it is reliable; otherwise a 20 kg.cm
  or stronger metal-gear waterproof servo.
- 2 or more x **3S LiPo**, about 5000 mAh, at most 165 x 50 mm footprint,
  with the extended battery hold-down if taller than 25 mm; Traxxas iD or
  XT60/XT90 plugs to match the VESC (adapter leads as needed).
- 1 x LiPo balance charger for 3S, LiPo bag, voltage checker.
- Optional: 1 x sensored 540-size brushless motor, about 3000-4000 kV, with
  a hall sensor cable (see Compatibility notes).

### Compute and control

- 1 x **NVIDIA Jetson Orin Nano Super Developer Kit**, 8 GB (Wi-Fi, Gigabit
  Ethernet, two M.2 Key M slots on the carrier; 9-20 V on a 5.5/2.5 mm jack,
  3.5 A max; 7-25 W).
- 1 x **NVMe SSD**, M.2 2280, 256 GB or larger, in the carrier's M.2 slot
  (the developer kit needs no extra M.2 carrier).
- 1 x **VESC 6 MkVI** (Trampa, 75 x 70 x 18 mm, 80 A continuous, 3S-12S) or
  another VESC 6-class controller that runs on 3S, with USB to the Jetson.
  Set it up for the motor with VESC Tool (motor detection, current limits,
  servo output enabled) before the first drive.
- 1 x **RC transmitter and receiver** (2.4 GHz, PWM or SBUS) for first
  drives on the stock electronics, and, with an RC relay switch in the
  e-stop loop, as a wireless stop once the car drives itself. Manual driving
  with the VESC uses the web UI's joystick (`/cmd_vel_teleop`).

### Sensors

- 1 x **Intel RealSense D435i** (indoor, 90 x 25 x 25 mm, 75 g, built-in
  IMU) or **D455** (longer range outdoors, 124 x 29 x 26 mm, 116 g); the
  printed cradle takes either. The D455's USB-C is on its bottom face: use a
  right-angle cable.
- 1 x **USB 3 Type-C to Type-A cable**, about 0.5 m, locking or right-angle
  at the camera, plus strain relief (the camera arm has tie slots).
- 1 x **Slamtec RPLIDAR C1** (360 deg, 0.05-12 m, 10 Hz, 5 kS/s, 55.6 mm
  square, 110 g, 5 V 260 mA; UART with the supplied USB adapter). The printed
  mast is for the C1. Other lidars need their own mast: the A2M12 (76 mm
  round, M3 holes) or S2 (77 mm square), or a Hokuyo UST-10LX (270 deg,
  10-30 V, Ethernet), whose rear 90 deg are blind anyway.
- 1 x **IMU: Adafruit BNO085 breakout** (4754, 25.4 x 22.9 mm, I2C/UART) on
  the Jetson's 40-pin header (3.3 V), or a USB IMU such as the WitMotion
  WT901C (51 x 36 x 15 mm; then fix it with foam tape on the IMU mount).

### Power and safety

- 1 x **19 V buck-boost converter**, 9-13 V in, at least 3 A out (57 W;
  the Jetson's jack takes 3.5 A at most), with a 5.5/2.5 mm barrel lead to
  the Jetson. A 3S pack sags below the carrier's 9 V minimum under motor
  load, so the Jetson is never fed from the pack directly.
- 1 x **6 V BEC**, at least 3 A continuous, for the steering servo.
- 1 x **emergency stop: Schneider XB5AS8442** (22 mm, 40 mm mushroom, turn
  to release, 1 NC, positive opening) or an equivalent 22 mm latching
  e-stop, in the printed tower (22.3-22.5 mm cut-out, 4 mm panel).
- 1 x **motor-power switch the e-stop controls**: a DC contactor or relay
  rated at least 60 A at 12.6 V (coil from the battery through the e-stop's
  NC contact and the RC relay), with a precharge resistor or a contactor
  rated for capacitive loads; or a MOSFET anti-spark battery switch with a
  low-current control loop through the e-stop (check that opening the loop
  cuts the power).
- 1 x **main fuse**, 60 A (ANL or MAXI blade) in the motor branch, and a
  10 A blade fuse in the electronics branch; fuse holders.
- 1 x main power switch or loop key for the electronics branch.
- Optional: a second contact block (NO) on the e-stop to a Jetson GPIO, so
  the software can publish `/racecarneo/estop`.
- Silicone wire (12 AWG motor branch, 18-20 AWG electronics), XT60/XT90,
  heat-shrink tubing, cable sleeving.

### Mounting

- Screws, nuts, washers, heat-set inserts, straps and zip ties: the
  generated [fastener table](#fasteners) lists every one.
- Medium (blue) thread locker, for screws into metal only.

### Why these parts are bought, not printed

| Part | Why it stays purchased |
|---|---|
| Chassis, suspension, steering, drivetrain, wheels | Impact and fatigue loads on every bump; the truck is designed and tuned as a whole. |
| Battery hold-down | Holds about 400 g of LiPo through crashes; the stock (or extended) hold-down is part of the chassis' battery compartment, whose geometry Traxxas does not publish. A printed retainer that lets go drops the pack onto the drivetrain. |
| Body posts | Part of the chassis; the printed adapters clamp them. |
| Electronics, sensors, motor, servo | Electronics and motors. |
| Emergency stop, contactor, fuses, switches | Safety: rated, tested parts that must open under fault current every time. |
| Screws, inserts, nuts | Metal threads; printed threads strip and creep. |
| Wiring, connectors | Carry up to 80 A. |
| Straps | Hook-and-loop holds the VESC without published mounting holes. |

### Power and wiring

Budget (estimates where marked; the developer kit powers the USB devices):

| Load | Supply | Typical | Peak | Source |
|---|---|---|---|---|
| Jetson Orin Nano Super with USB devices | 19 V converter | 15-25 W | about 35 W | NVIDIA: 7-25 W modes, MAXN SUPER uncapped |
| RealSense D435i/D455 (via the Jetson) | USB 3 | 1.5 W | 3.5 W | estimate |
| RPLIDAR C1 (via the Jetson) | USB 5 V | 1.2 W | 4 W (start) | Slamtec: 230-260 mA, 800 mA at start |
| BNO085 | 3.3 V header | 0.05 W | 0.05 W | estimate |
| Steering servo | 6 V BEC | 1 W | 15-18 W (stall) | estimate |
| Drive motor | VESC | 50-150 W | 600-900 W | estimate (3S, 50-80 A peaks) |

- Battery -> main fuse (60 A) -> contactor (opened by the e-stop) -> VESC.
- Battery -> 10 A fuse -> main switch -> 19 V converter -> Jetson jack;
  6 V BEC -> steering servo power (signal from the VESC's servo port).
- Common ground; signal leads away from the motor leads.

The wiring and connector details are in
[the printed-parts README, "Wiring"](parts/racecarneo/README.md#wiring).

## 3D-printed parts

Designed in CadQuery (`hardware/parts/racecarneo/`), validated and exported
by `tools/build_printable_parts.py`; STLs in
[`parts/racecarneo/stl/`](parts/racecarneo/stl/). Printer: Bambu Lab P1S.
Plates, settings, post-processing and assembly:
[parts/racecarneo/README.md](parts/racecarneo/README.md). Print the "fit
test first" parts before the rest. Times and masses are estimates.

<!-- BEGIN GENERATED: printed-parts (tools/build_printable_parts.py) -->
| Part | Qty | Material | Bambu Studio settings (P1S, 0.4 mm) | Est. time (each) | Est. mass (each) | File |
|---|---|---|---|---|---|---|
| Camera arm (RealSense over the lidar) | 1 | ASA | 0.20 mm, 5 walls, 40% gyroid, no supports | 2 h 39 min | 90 g | [camera_arm.stl](parts/racecarneo/stl/camera_arm.stl) |
| Camera cradle (RealSense D435i / D455, pitch adjust) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 19 min | 9.1 g | [camera_cradle.stl](parts/racecarneo/stl/camera_cradle.stl) |
| Deck, front half | 1 | ASA | 0.20 mm, 3 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 1 h 57 min | 66 g | [deck_front.stl](parts/racecarneo/stl/deck_front.stl) |
| Deck grommet (TPU top hat) | 8 | TPU95A | 0.20 mm, 3 walls, 100% gyroid, no supports | 2 min | 0.2 g | [deck_grommet.stl](parts/racecarneo/stl/deck_grommet.stl) |
| Deck, rear half | 1 | ASA | 0.20 mm, 3 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 2 h 59 min | 102 g | [deck_rear.stl](parts/racecarneo/stl/deck_rear.stl) |
| E-stop tower (22 mm mushroom) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 1 h 06 min | 32 g | [estop_tower.stl](parts/racecarneo/stl/estop_tower.stl) |
| IMU mount (BNO085 breakout) (fit test first) | 1 | ASA | 0.20 mm, 3 walls, 40% gyroid, no supports | 9 min | 4.4 g | [imu_mount.stl](parts/racecarneo/stl/imu_mount.stl) |
| Jetson tray (Orin Nano developer kit carrier) (fit test first) | 1 | ASA | 0.20 mm, 3 walls, 30% gyroid, no supports | 30 min | 17 g | [jetson_tray.stl](parts/racecarneo/stl/jetson_tray.stl) |
| Lidar mast (RPLIDAR C1) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 1 h 03 min | 35 g | [lidar_mast.stl](parts/racecarneo/stl/lidar_mast.stl) |
| Post adapter, front (chassis body post to deck) (fit test first) | 2 | ASA | 0.20 mm, 5 walls, 50% gyroid, no supports | 16 min | 7.0 g | [post_adapter_front.stl](parts/racecarneo/stl/post_adapter_front.stl) |
| Post adapter, rear (chassis body post to deck) (fit test first) | 2 | ASA | 0.20 mm, 5 walls, 50% gyroid, no supports | 14 min | 5.7 g | [post_adapter_rear.stl](parts/racecarneo/stl/post_adapter_rear.stl) |
| Power tray (converters, fuse, contactor) | 2 | ASA | 0.20 mm, 3 walls, 25% gyroid, no supports | 15 min | 7.9 g | [power_tray.stl](parts/racecarneo/stl/power_tray.stl) |
| Rim cable clip | 10 | PETG | 0.20 mm, 4 walls, 100% gyroid, no supports | 4 min | 1.1 g | [rim_clip.stl](parts/racecarneo/stl/rim_clip.stl) |
| VESC tray | 1 | ASA | 0.20 mm, 3 walls, 30% gyroid, no supports | 31 min | 17 g | [vesc_tray.stl](parts/racecarneo/stl/vesc_tray.stl) |
| **Total, all pieces** | | | | **13 h 32 min** | **427 g** | |
<!-- END GENERATED: printed-parts -->

### Fasteners

Totals for one car, from the parts' metadata (screws are listed with the
part their head bears on). Buy 10-20 % spares.

<!-- BEGIN GENERATED: fasteners (tools/build_printable_parts.py) -->
| Item | Qty | Where |
|---|---|---|
| 1/4"-20 UNC x 3/8" button head screw | 1 | RealSense to the cradle (6 mm of thread in the camera; the D455 allows 9 mm) (camera_cradle, 1) |
| M2.5 x 4 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 8 | IMU bosses, from the top (imu_mount, 4); carrier bosses, from the top (jetson_tray, 4) |
| M2.5 x 6 socket head cap screw | 8 | BNO085 breakout to the IMU mount (imu_mount, 4); Jetson carrier to the tray bosses (jetson_tray, 4) |
| M2.5 x 8 socket head cap screw | 4 | RPLIDAR C1 to the mast seat, from below (enters the lidar 3.5 mm; 4 mm is the limit) (lidar_mast, 4) |
| M3 nyloc nut | 16 | under the deck, camera arm foot screws (camera_arm, 2); camera cradle pivot and pitch lock (camera_cradle, 2); lap joint rim screws (deck_rear, 2); under the deck, lidar mast flange screws (lidar_mast, 6); post adapter pinch clamp (front) (post_adapter_front, 2); post adapter pinch clamp (rear) (post_adapter_rear, 2) |
| M3 washer | 18 | camera arm foot screws, both ends (camera_arm, 4); camera cradle pivot and pitch lock, both sides (camera_cradle, 4); on the upper deck grommets, front (deck_front, 2); on the upper deck grommets, rear (deck_rear, 2); under the nyloc nuts of the lidar mast flange (lidar_mast, 6) |
| M3 x 12 socket head cap screw | 2 | lap joint rim screws at x = 43 mm (deck_rear, 2) |
| M3 x 16 socket head cap screw | 10 | deck to the front post adapters, through a pair of deck grommets (deck_front, 2); deck to the rear post adapters, through a pair of deck grommets (deck_rear, 2); lidar mast flange through the deck (and the lap joint) (lidar_mast, 6) |
| M3 x 20 socket head cap screw | 4 | post adapter pinch clamp (front) (post_adapter_front, 2); post adapter pinch clamp (rear) (post_adapter_rear, 2) |
| M3 x 25 socket head cap screw | 2 | camera arm foot through the lidar mast flange and the deck (camera_arm, 2) |
| M3 x 3 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 26 | top face, through holes, pressed flush: Jetson tray (deck_front, 4); top face, through holes, pressed flush: IMU, power tray, VESC tray, e-stop tower, and 8 for the RACECAR Turret's plate posts (deck_rear, 22) |
| M3 x 30 socket head cap screw | 2 | camera cradle pivot and pitch lock, through the arm's tongue (camera_cradle, 2) |
| M3 x 5.7 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 4 | seat, from the top (deck screw) (post_adapter_front, 2); seat, from the top (deck screw) (post_adapter_rear, 2) |
| M3 x 6 socket head cap screw | 14 | IMU mount to the rear deck inserts (imu_mount, 2); Jetson tray to the front deck inserts (jetson_tray, 4); power tray to the rear deck inserts (power_tray, 4); VESC tray to the rear deck inserts (vesc_tray, 4) |
| M3 x 8 socket head cap screw | 4 | e-stop tower ears to the rear deck inserts (estop_tower, 4) |
| hook-and-loop strap, 20 mm, 200 mm | 2 | VESC to its tray (vesc_tray, 2) |
| silicone thermal pad or rubber pad, 70 x 60 x 1 mm | 1 | between the VESC and its tray (vesc_tray, 1) |
| zip tie 2.5 mm | 16 | RealSense USB cable along the camera arm (camera_arm, 4); USB cables along the Jetson tray's sides (jetson_tray, 4); modules and leads on the power trays (power_tray, 8) |
<!-- END GENERATED: fasteners -->

### Assembly notes

- Build order, torques and checks:
  [parts/racecarneo/README.md](parts/racecarneo/README.md#steps).
- Deck height: the deck's top face 132 mm above the ground with the car on
  its wheels puts the scan plane at 180 mm and the camera at 220 mm (the
  URDF); set it with the post adapters before tightening them.
- The camera arm's fin is the only printed part in the lidar's scan plane.
  It stands 31-46 mm behind the lidar axis, inside the C1's 50 mm blind
  range, so it returns nothing; it hides a rear sector of about +-23 deg.
  Everything else stays under the scan plane, or inside that sector (the
  e-stop, and the RACECAR Turret's posts). Do not add parts that cross the
  plane elsewhere: inside Nav2's 0.22 m robot radius a return stops the car
  through the collision monitor.

## Recommended assembly order

1. Assemble and test the RC car in manual mode with the stock radio and
   electronics.
2. Print the parts ([parts/racecarneo/README.md](parts/racecarneo/README.md)),
   fit the post adapters and the deck, and check the deck height.
3. Install the VESC, the contactor, the e-stop and the converters; verify
   throttle and steering with the wheels off the ground and the e-stop
   cutting the motor.
4. Add the Jetson, the RealSense, the lidar and the IMU on the deck.
5. Verify the ROS topics before autonomous driving:
   - `/camera/color/image_raw`
   - `/camera/color/camera_info`
   - `/scan`
   - `/imu/data_raw`
   - `/ackermann_cmd`
6. Keep the emergency stop (and the RC relay) in the loop before enabling
   Nav2 or autonomous throttle.

## Useful references

- Printed parts, printing and assembly: hardware/parts/racecarneo/README.md
- Project sensor topics: systems/racecarneo/config/topics.yaml
- Project runtime pipeline: systems/racecarneo/README.md
- Frames and sensor extrinsics: systems/racecarneo/urdf/racecarneo.urdf.xacro
- NVIDIA Jetson Orin Nano Super Developer Kit:
  https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/
- Jetson Orin Nano Developer Kit Carrier Board Specification (SP-11324-001):
  https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/jetson_orin_nano_devkit_carrier_board_specification_sp.pdf
- RealSense D400 series datasheet (337029):
  https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf
- RealSense D435i: https://www.realsenseai.com/products/depth-camera-d435i/
- Slamtec RPLIDAR C1: https://www.slamtec.com/en/c1/spec
- Traxxas Slash 4x4: https://traxxas.com/slash-4x4-vxl-68386-4
- Trampa VESC 6 MkVI: https://trampa.co.uk/product/vesc6-mk6/
- Adafruit BNO085 breakout: https://www.adafruit.com/product/4754
- Schneider XB5AS8442: https://www.se.com/us/en/product/download-pdf/XB5AS8442
