# Lunar rover bill of materials

Goal: a six-wheel lunar rover development platform around Space ROS and the
existing `applications/spaceros` image, built to the CyberPhysics lunar
rover simulator (`systems/lunar_rover/config/simulation.yaml`) so the
south-pole traverse autonomy can be validated in simulation and then on a
terrestrial regolith testbed.

**Scope.** This is a terrestrial prototype and software validation
platform, not a flight-ready lunar vehicle. Lunar hardware needs
mission-specific radiation, vacuum, thermal, launch-load,
contamination-control, reliability and planetary-protection engineering
that a COTS and 3D-printed build cannot provide.

The rover: six 360 mm wheels (0.18 m rolling radius at the grouser tips,
the simulator's), skid steer (the simulator has no steering joints), a
rocker-bogie suspension with a differential, a 600 x 300 x 200 mm body
of 2040 aluminium extrusion, a 24 V LiFePO4 pack (640 Wh, the simulator's
650 Wh), about 1.16 x 0.69 m and 38 kg without payload (computed by the
assembly check). The wheels, actuator mounts, electronics trays, sensor
mounts and brackets are printed; the structure, drive, power, electronics
and sensors are bought. Printing and assembly instructions:
[hardware/parts/lunar_rover/README.md](parts/lunar_rover/README.md).

## Compatibility notes

- **Why not the JPL Open Source Rover or Sawppy.** Both are open
  rocker-bogie designs (OSR: Apache-2.0; Sawppy: MIT) and were considered.
  OSR is a 12.7 kg, corner-steered rover with 144 mm wheels on a goBILDA
  aluminium structure; Sawppy a light, servo-driven rover with printed 124 mm
  wheels. Both have wheels about a third of the simulator's 0.18 m radius,
  far less payload than the old list's 20 kg, and corner steering where the
  simulator is skid-steer; reusing either would mean changing the simulator's
  wheel radius, speed, power and payload. This rover keeps the simulator's
  numbers instead; it takes OSR's rocker-bogie proportions (pivots placed
  for equal wheel loads, a differential bar) and Sawppy's idea of printed
  wheels.
- **Payload.** The old list asked for at least 20 kg. With the AK70-10's
  8.3 N m rated torque the rover climbs the simulator's steepest slope
  (21 deg) in loose regolith with up to 18.6 kg of payload (rolling
  resistance 0.15 assumed); 20 kg needs 8.5 N m per wheel, inside the
  actuators' 24.8 N m peak but above their rated torque, so only for short
  climbs. On level ground the payload is limited by the structure, not the
  drive (keep it under 20 kg).
- **Wheel drive: CubeMars AK70-10.** A brushed planetary gearmotor (the old
  list's option) on 360 mm wheels overloads its output bearing: the IG52
  class allows 196 N radial at 10 mm from its flange, the wheel loads it at
  60 mm. The AK70-10 is a 24/48 V actuator with a cross-roller bearing
  (2000 N dynamic), a 10:1 planetary, an encoder, a temperature sensor and
  its own driver on CAN, so it mounts the wheel directly and covers the old
  list's "wheel encoders or integrated smart motor controllers" and "motor
  controller banks with current sensing and ROS 2-compatible interfaces".
  At the simulator's top speed and worst slip (0.38 m/s, 0.65) a wheel
  turns at 58 rpm; the AK70-10 is rated for 148 rpm at 24 V.
- **Drivers are not in the repository.** `compositions/lunar_rover_hardware.yaml`
  is a runtime skeleton. The topic contract
  (`systems/lunar_rover/config/topics.yaml`) needs these nodes, all on Cyclone
  DDS (`rmw_cyclonedds_cpp`, the only RMW in Space ROS):
  - drive: `/lunar_rover/cmd_vel` to left and right wheel speeds (skid steer,
    0.18 m wheels), commands to the six AK70-10 on one CAN bus, and
    `/lunar_rover/wheel_states` (`front_left_wheel` ... `rear_right_wheel`,
    the simulator's names) from their encoders;
  - odometry: `/lunar_rover/odom` from wheel odometry fused with the IMU;
  - `/lunar_rover/imu` from the 9-axis IMU, and `/lunar_rover/terrain/slope_deg`
    from its roll and pitch;
  - `/lunar_rover/terrain/slip_ratio` from wheel speed against visual
    odometry (the OAK-1's feature tracker or the D455);
  - `/lunar_rover/battery` (voltage, current, percentage) from the battery
    monitor;
  - `/lunar_rover/solar/illumination` from the lux sensor (the simulator
    publishes up to 110,000 lux; the VEML7700 reads to about 120,000);
  - `/lunar_rover/thermal/body` from the body temperature probe.
- **Space ROS runs on x86-64 here.** The repository builds the spaceros image
  for amd64 (`applications/spaceros/README.md`), so the onboard computer is
  an x86-64 mini PC, not a Jetson.
- **Voltages.** The simulator's battery reads 20-28 V; an 8S LiFePO4 pack is
  29.2 V full, about 26 V on its plateau and cut off near 20-22 V. The AK70-10
  takes the pack directly (24/48 V); the Livox Mid-360 does not (9-27 V): it
  runs from the 12 V converter.
- **Temperatures.** The simulator's body temperature (-135 to +50 C) is a
  lunar model; on Earth the limits are the hardware's: AK70-10 -20 to 50 C,
  Mid-360 -20 to 55 C, LiFePO4 charging 0 to 45 C. Watch the actuators'
  temperature in a hot test pit.
- **Hazard cameras.** The D455's stereo pair is the front hazard camera
  pair (the old list had a forward depth camera and a separate hazard pair).
  Its lower corners see the tops of the front wheels: mask them.
- **GNSS** is for Earth testing only; disable it or replace it with the
  localization inputs for lunar validation, as the old list said.

## Purchased parts

### Drive

- 6 x **CubeMars AK70-10 KV100** actuator (24/48 V, 8.3 N m rated, 24.8 N m
  peak, 621 g, CAN), plus 1 spare. Set the CAN IDs 1-6: front left, front
  right, middle left, middle right, rear left, rear right.
- 1 x **USB-CAN adapter** with a Linux SocketCAN driver (CANable 2.0 or PEAK
  PCAN-USB), 2 x 120 ohm CAN terminators, CAN and power cables with the
  actuators' XT30 (2+2) connectors (CubeMars sells them), 6 x 10 A blade
  fuses and holders (one per actuator).

### Structure (aluminium, purchased)

- **2040 extrusion**, slot 6 (Misumi HFS5-2040 or any B-type 20-series):
  body 4 x 600 mm (top and bottom side rails), 5 x 260 mm (four cross rails
  and the differential cross rail), 4 x 120 mm (corner posts); rockers
  2 x 640 mm; bogies 2 x 440 mm; legs 2 x 160 mm (front) and 4 x 70 mm
  (middle and rear). Cut square.
- **2020 extrusion**, slot 6: front mast 400 mm, rear mast 200 mm,
  differential bar 410 mm.
- 2040 cast corner brackets (body corners, legs to beams: about 24), drop-in
  M5 T-nuts and M5 x 10 button heads (about 120 of each, in addition to the
  printed parts' fastener table).
- **Pivots** (12 mm, steel): rocker pivots: 2 x 12 mm h6 shaft, 80 mm, in an
  SHF12 shaft support on each body top side rail, turning in 2 x KFL001
  flanged bearings on each rocker beam; bogie pivots: an aluminium pivot
  plate (4 mm 6061, 40 x 130 mm) bolted to the rear end of each rocker beam,
  with an SHF12 support and a 12 mm shaft, turning in 2 x KFL001 on each
  bogie beam; differential: 1 x KP001 pillow block under the differential
  cross rail, a 12 mm shaft through the bar, 2 links of M6 rod ends (one
  male, one female per link, PTFE-lined) with M6 x 30 bolts.
- **Panels**: 3 mm aluminium composite (ACP) for the floor and lid
  (560 x 260 mm each), sides and ends, cut to the frame; foam gasket tape
  on the joints against regolith dust.

### Power

- 1 x **24 V LiFePO4 pack**, 8S 25 Ah (640 Wh), with BMS, at most
  260 x 180 x 140 mm and 7 kg (the printed end blocks fit any pack in that
  envelope), and a LiFePO4 charger (29.2 V, 5-10 A).
- 1 x battery disconnect switch (service disconnect), 1 x 40 A main fuse
  (MIDI or DIN fuse holder), 1 x **contactor**, 24 V coil, 50 A (the
  actuators' supply, opened by the e-stop loop).
- 1 x **Victron SmartShunt** (VE.Direct to USB): battery voltage, current
  and state of charge for `/lunar_rover/battery`.
- 1 x DC-DC 24 to 12 V, 10 A (DIN): lidar, switch, radio, safety controller;
  1 x DC-DC 24 to 19 V (or a 12-24 V input mini PC).
- 1 x **solar panel**, 40-50 W, at most 520 x 400 mm with mounting holes in
  its frame's bottom flange (the simulator charges at 38 W in full sun), and
  1 x Victron SmartSolar MPPT 75/10 (12/24 V). The panel is a payload input
  for power-management tests, not the field safety power.
- Silicone wire (12 AWG for the 24 V bus, 18 AWG for actuator feeds), ferrules,
  DIN terminal blocks, heat-shrink, 2 x TS35 DIN rail cut to 240 mm.

### Compute, communications, safety

- 1 x **x86-64 mini PC**, 16 GB RAM, 512 GB SSD, VESA 75/100 mounting
  (Ubuntu 24.04, Docker, the spaceros image).
- 1 x 5-port Ethernet switch (DIN, 12-24 V), lidar and radio.
- 1 x Wi-Fi 6, private LTE or long-range Ethernet radio for supervised field
  operation; operator laptop with Docker, RViz/Foxglove; gamepad for manual
  recovery.
- 1 x **Schneider Harmony XB5AS8442** emergency stop (40 mm, 1 NC; a 2 NC
  variant for a dual-channel loop), 1 x M16 cable gland.
- 1 x **remote e-stop** (wireless e-stop set with a normally-closed relay
  output), wired in series with the e-stop.
- 1 x **safety controller**: a microcontroller watchdog (Arduino UNO R4 class)
  that holds the contactor closed only while it receives ROS heartbeats and
  opens it on a timeout, through a relay; it also reads the lux sensor (I2C)
  and the temperature probes (1-Wire) if no USB adapters are used.

### Sensors

- 1 x **Intel RealSense D455** (front hazard camera, 124 x 29 x 26 mm, 116 g),
  USB 3 cable.
- 1 x **Luxonis OAK-1**, fixed-focus (downward terrain camera: slip and visual
  odometry), USB 3 cable with a right-angle C plug.
- 1 x **Livox Mid-360** (360 x -7..52 deg lidar, IP67), its M12 splitter cable,
  and 1 x aluminium plate 100 x 100 x 3 mm (Livox's heat-sink requirement).
- 1 x 9-axis IMU with temperature compensation (for example a VectorNav
  VN-100 or a MicroStrain 3DM-CV7-AHRS), screwed to the floor at the body
  centre, x forward.
- 1 x u-blox ZED-F9P GNSS receiver, 1 x multi-band magnetic-base antenna
  (u-blox ANN-MB class), 1 x 100 x 1.5 mm steel disc (ground plane).
- 1 x **Adafruit VEML7700** lux sensor (product 4162, 0-120 klux) and a clear
  acrylic cover, 32 x 25 x 2 mm.
- 4 x DS18B20 waterproof temperature probes (battery, power tray, an
  actuator, body exterior) and a USB-1-Wire adapter (or the safety controller).

### Field and safety kit

- Spares: 1 actuator, 2 wheel segments and 4 grousers per wheel, 1 actuator
  mount, fuses, CAN cable, a charged spare pack.
- Crimp tools, multimeter, thread locker (medium), battery-safe storage bag,
  PPE for regolith simulant (dust mask, eye protection), a terrain testbed or
  field area with sand or regolith simulant, slopes, shadows, rocks and marked
  safety boundaries.

### Why these parts are bought, not printed

| Part | Why it stays purchased |
|---|---|
| Rockers, bogies, legs, body frame (2040/2020 aluminium) | A 640 mm rocker carries half the rover over rocks: printed beams that long would not fit the bed, flex several times more than aluminium and creep under a constant load. The printed parts are the joints' brackets where the loads are short and compressive. |
| Pivot shafts, flanged bearings, rod ends | Wear surfaces under cyclic load: printed bushings wear oval and creep; steel shafts and ball bearings keep the suspension free and true. |
| Actuators, CAN adapter, motor controllers | Electromagnetic and power electronics. |
| Battery, BMS, fuses, contactor, disconnect | Energy storage and electrical protection: they must break fault currents and contain a cell failure. |
| E-stop, remote e-stop, safety relay | Safety-rated components; a printed button or latch is not. |
| Lidar heat-sink plate | Livox asks for a metal plate at least 3 mm thick and 10,000 mm2 under the Mid-360; plastic would insulate it. |
| GNSS steel disc | The antenna's magnet needs steel, and the disc is its ground plane. |
| Body panels | Larger than the bed, and must stay flat and dust-tight. |
| Screws, T-nuts, inserts | Metal threads; printed threads strip and creep. |

### Power budget

At the 24 V bus (the simulator: 42 W idle, 115 W driving at full speed):

| Load | Typical | Peak | Source |
|---|---|---|---|
| Mini PC (Space ROS, drivers, perception) | 20 W | 35 W | estimate |
| Livox Mid-360 | 6.5 W | 14 W (cold start) | Livox manual |
| RealSense D455, OAK-1 | 6 W | 8 W | estimates from the datasheets |
| Switch, radio, safety controller, sensors | 8 W | 12 W | estimate |
| Six actuators at standstill | 9 W | 9 W | estimate |
| **Idle total** | **about 50 W** | | |
| Driving on the level in loose regolith, 0.38 m/s | +50 W | | estimate: 0.15 rolling resistance, 60 % drive efficiency |
| Climbing 21 deg with 10 kg, 0.38 m/s | +165 W | | estimate |

Endurance with 640 Wh: about 6.5 h on the level, about 3 h climbing
(estimates). Converter losses are inside the estimates.

## 3D-printed parts

Designed in CadQuery (`hardware/parts/lunar_rover/`), validated and
exported by `tools/build_printable_parts.py`; STLs in
[`parts/lunar_rover/stl/`](parts/lunar_rover/stl/). Printer: Bambu Lab P1S.
Plates, settings, post-processing and assembly:
[parts/lunar_rover/README.md](parts/lunar_rover/README.md). Print the
"fit test first" parts before the rest. Times and masses are estimates;
"flight-critical" in this table means a part whose failure disables the
rover. The wheels are most of the printing: 48 segments, 96 grousers and 6
hubs, about 210 of the rover's 290 estimated print hours and 6.6 kg of its
8.9 kg of filament.

<!-- BEGIN GENERATED: printed-parts (tools/build_printable_parts.py) -->
| Part | Qty | Material | Bambu Studio settings (P1S, 0.4 mm) | Est. time (each) | Est. mass (each) | File |
|---|---|---|---|---|---|---|
| Actuator mount (AK70-10 cup on the leg) (flight-critical, fit test first) | 6 | ASA | 0.20 mm, 6 walls, 50% gyroid, no supports | 5 h 22 min | 171 g | [actuator_mount.stl](parts/lunar_rover/stl/actuator_mount.stl) |
| Actuator shroud (TPU dust cap) | 6 | TPU95A | 0.20 mm, 3 walls, 30% gyroid, no supports | 1 h 45 min | 26 g | [actuator_shroud.stl](parts/lunar_rover/stl/actuator_shroud.stl) |
| Battery end block (flight-critical) | 2 | PETG | 0.20 mm, 4 walls, 40% gyroid, no supports | 3 h 15 min | 112 g | [battery_end.stl](parts/lunar_rover/stl/battery_end.stl) |
| Extrusion cable clip | 24 | ASA | 0.20 mm, 4 walls, 100% gyroid, no supports | 5 min | 1.5 g | [cable_clip.stl](parts/lunar_rover/stl/cable_clip.stl) |
| Compute tray (VESA 75/100) | 1 | PETG | 0.20 mm, 3 walls, 25% gyroid, no supports | 3 h 10 min | 114 g | [compute_tray.stl](parts/lunar_rover/stl/compute_tray.stl) |
| Differential clevis (M6 rod end) (flight-critical, fit test first) | 4 | ASA | 0.20 mm, 5 walls, 50% gyroid, no supports | 28 min | 13 g | [diff_clevis.stl](parts/lunar_rover/stl/diff_clevis.stl) |
| E-stop housing (flight-critical, fit test first) | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports | 1 h 47 min | 56 g | [estop_housing.stl](parts/lunar_rover/stl/estop_housing.stl) |
| GNSS and lux head (rear mast) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports | 1 h 42 min | 55 g | [gnss_head.stl](parts/lunar_rover/stl/gnss_head.stl) |
| Grouser (tread bar) | 96 | ASA | 0.20 mm, 4 walls, 60% gyroid, no supports | 22 min | 12 g | [grouser.stl](parts/lunar_rover/stl/grouser.stl) |
| Hazard camera mount (RealSense D455) (fit test first) | 1 | ASA | 0.20 mm, 5 walls, 40% gyroid, no supports | 1 h 54 min | 56 g | [hazcam_mount.stl](parts/lunar_rover/stl/hazcam_mount.stl) |
| Lidar mount (Livox Mid-360 on the mast) (fit test first) | 1 | ASA | 0.20 mm, 5 walls, 40% gyroid, no supports | 1 h 58 min | 64 g | [lidar_mount.stl](parts/lunar_rover/stl/lidar_mount.stl) |
| Mast foot (2020 on a 2040 side rail) | 2 | ASA | 0.20 mm, 5 walls, 40% gyroid, no supports | 1 h 08 min | 36 g | [mast_foot.stl](parts/lunar_rover/stl/mast_foot.stl) |
| Solar panel post (fit test first) | 4 | ASA | 0.20 mm, 5 walls, 40% gyroid, no supports | 1 h 57 min | 60 g | [panel_clamp.stl](parts/lunar_rover/stl/panel_clamp.stl) |
| Power tray (two DIN rails) | 1 | PETG | 0.20 mm, 3 walls, 25% gyroid, no supports | 3 h 30 min | 127 g | [power_tray.stl](parts/lunar_rover/stl/power_tray.stl) |
| Terrain camera cradle (OAK-1, downward) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 36 min | 19 g | [terrain_cam_mount.stl](parts/lunar_rover/stl/terrain_cam_mount.stl) |
| Wheel hub (flight-critical, fit test first) | 6 | ASA | 0.20 mm, 5 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 6 h 07 min | 210 g | [wheel_hub.stl](parts/lunar_rover/stl/wheel_hub.stl) |
| Wheel segment (1/8 of a wheel) (flight-critical) | 48 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports | 2 h 54 min | 89 g | [wheel_segment.stl](parts/lunar_rover/stl/wheel_segment.stl) |
| **Total, all pieces** | | | | **288 h 19 min** | **8939 g** | |
<!-- END GENERATED: printed-parts -->

### Fasteners

Totals for one rover, from the parts' metadata (screws are listed with the
part their head bears on). Buy 10-20 % spares; the structure's own corner
brackets and T-nuts are listed under Structure above.

<!-- BEGIN GENERATED: fasteners (tools/build_printable_parts.py) -->
| Item | Qty | Where |
|---|---|---|
| 1/4"-20 UNC x 3/8" button head screw | 1 | OAK-1 to the cradle (engages about 4.5 mm of the 10 mm thread) (terrain_cam_mount, 1) |
| M16 x 1.5 cable gland (4-8 mm cable) | 1 | e-stop cable, right side (estop_housing, 1) |
| M3 x 10 socket head cap screw | 4 | lidar through the printed and aluminium plates, heads recessed (4.5 mm into its 5 mm threads) (lidar_mount, 4) |
| M3 x 10 socket head cap screw, class 12.9 | 48 | actuator to the cup's front ring (7 mm into the 10 mm threads), medium thread locker (actuator_mount, 48) |
| M3 x 16 socket head cap screw, class 12.9 | 36 | hub to the actuator's output flange (5.5 mm into the 7 mm threads), medium thread locker (wheel_hub, 36) |
| M3 x 5.7 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 192 | base, from below (grouser, 192) |
| M3 x 8 socket head cap screw | 192 | from inside the rim into the grouser (3 mm rim, 5 mm into the 5.7 mm insert) (grouser, 192) |
| M4 x 10 socket head cap screw | 4 | mini PC to the tray (VESA holes, from below) (compute_tray, 4) |
| M4 x 12 socket head cap screw + M4 nyloc nut | 4 | aluminium plate to the mount (lidar_mount, 4) |
| M4 x 14 socket head cap screw + M4 washer | 96 | segment web to the hub inserts (7 mm into the 8.1 mm insert) (wheel_segment, 96) |
| M4 x 16 socket head cap screw + M4 nyloc nut | 96 | end wall to the next segment's end wall (one segment of each pair carries the screws) (wheel_segment, 96) |
| M4 x 25 socket head cap screw + M4 nyloc nut + M4 washer | 8 | tray to the floor panel (compute_tray, 4); tray to the floor panel (power_tray, 4) |
| M4 x 8 pan head screw + M4 washer | 10 | DIN rails to the tray (power_tray, 10) |
| M4 x 8 socket head cap screw | 2 | D455 to the plate (3.5 mm into the camera; Intel's max 4 mm, 0.4 N m) (hazcam_mount, 2) |
| M4 x 8.1 mm brass heat-set insert (CNC Kitchen type, 5.7 mm hole) | 106 | top face, DIN rail screws (power_tray, 10); outboard face, segment web bolts (wheel_hub, 96) |
| M5 x 10 button head screw + M5 drop-in T-nut (slot 6) | 74 | cup to the leg (actuator_mount, 24); cable clip to an extrusion (cable_clip, 24); head to the mast (gnss_head, 2); mount to the mast (lidar_mount, 2); foot to the rail (4) and mast to the foot (2) (mast_foot, 12); post to the top side rail (panel_clamp, 8); cradle to the front bottom cross rail (terrain_cam_mount, 2) |
| M5 x 12 button head screw + M5 drop-in T-nut (slot 6) | 6 | housing to the rear top cross rail (estop_housing, 2); mount to the front top cross rail (hazcam_mount, 4) |
| M5 x 12 socket head cap screw + M5 drop-in T-nut (slot 6) | 8 | clevis to the beam (diff_clevis, 8) |
| M5 x 16 socket head cap screw + 2 M5 washers + M5 nyloc nut | 4 | panel flange to the post (panel_clamp, 4) |
| M5 x 20 socket head cap screw + M5 nyloc nut + M5 washer | 6 | end block to the floor panel (battery_end, 6) |
| M6 x 30 socket head cap screw + M6 nyloc nut + 2 M6 washers | 4 | rod end in the clevis (diff_clevis, 4) |
| battery strap, 20 mm, 700 mm, cam buckle | 2 | over the top of the pack, through a slot in each end block (two straps in all) (battery_end, 2) |
| clear acrylic cover, 32 x 25 x 2 mm, and clear silicone | 1 | over the lux sensor pocket (gnss_head, 1) |
| closed-cell foam pad, 3 mm | 2 | between the pack and the end block (battery_end, 2) |
| closed-cell foam pad, 6 mm, 130 x 180 mm | 2 | under the pack (two pads side by side) (battery_end, 2) |
| double-sided foam tape (VHB), 100 mm disc | 1 | steel disc on the head (gnss_head, 1) |
| hook-and-loop strap, 20 mm, 400 mm | 1 | mini PC over the tray, through the strap slots (compute_tray, 1) |
| zip tie 3.6 mm | 6 | actuator cable to the leg, below the shroud slot (actuator_shroud, 6) |
<!-- END GENERATED: fasteners -->

### Assembly notes

- Build order, torques and checks: [parts/lunar_rover/README.md](parts/lunar_rover/README.md#steps).
- Pivots: the rocker pivots sit over the middle wheels and the bogie pivots
  half way between the middle and rear wheels, so the six wheels carry equal
  loads on level ground.
- The wheel radius at the grouser tips is the simulator's 0.18 m; worn
  grousers reduce it: replace them before the radius drops by 2 mm or update
  the driver's wheel radius.

## Recommended assembly order

1. Print the fit-test parts and check them against the actuator, cameras,
   lidar, e-stop and panel; then print the rest.
2. Build the body frame, rockers, bogies, legs and differential; check the
   suspension moves freely with the wheels off.
3. Mount the actuators and wheels, bench-test each actuator on CAN with the
   rover on stands (wheels free).
4. Wire power, the e-stop loop and the safety controller; test the e-stop and
   the watchdog before any driving.
5. Mount the sensors, bring up the drivers, and check every topic of the
   contract below against the simulator's.
6. Drive manually on flat ground, then on the regolith testbed, before
   running `lunar_waypoint_nav`.

## Software stack

- Space ROS / ROS 2 Jazzy in `applications/spaceros`.
- Space ROS Curiosity rover demo for baseline rover workflows.
- CyberPhysics `lunar_rover_sim.py` for deterministic south-pole terrain,
  slip, illumination, thermal, battery, IMU, wheel-state and odometry topics.
- CyberPhysics `lunar_waypoint_nav.py` for autonomous waypoint traverse and
  terrain-aware speed limiting.
- Optional RViz or Foxglove for telemetry inspection.

ROS 2 topic contract (`systems/lunar_rover/config/topics.yaml`):

- `/lunar_rover/odom`
- `/lunar_rover/imu`
- `/lunar_rover/battery`
- `/lunar_rover/wheel_states`
- `/lunar_rover/cmd_vel`
- `/lunar_rover/terrain/slope_deg`
- `/lunar_rover/terrain/slip_ratio`
- `/lunar_rover/solar/illumination`
- `/lunar_rover/thermal/body`
- `/lunar_rover/mission_status`

## Useful references

- Printed parts, printing and assembly: hardware/parts/lunar_rover/README.md
- Space ROS: https://github.com/space-ros/space-ros
- Space ROS Curiosity rover demo:
  https://github.com/space-ros/demos/blob/main/curiosity_rover/README.md
- CubeMars AK70-10: https://www.cubemars.com/product/ak70-10-kv100-robotic-actuator.html
- Livox Mid-360 user manual:
  https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Mid-360_User_Manual_EN.pdf
- RealSense D400 series datasheet (D455):
  https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf
- JPL Open Source Rover (Apache-2.0): https://github.com/nasa-jpl/open-source-rover
- Sawppy the Rover (MIT): https://github.com/Roger-random/Sawppy_Rover
- NASA Lunar Terrain Vehicle Services:
  https://www.nasa.gov/news-release/nasa-selects-companies-to-advance-moon-mobility-for-artemis-missions/
- NASA VIPER lunar rover: https://www.nasa.gov/viper/
