# Aerodrone bill of materials

Goal: a quadrotor that mirrors the CyberPhysics Aerostack2 navigation
simulation and runs the BlueOS stack of `compositions/aerodrone_hardware.yaml`:
ArduCopter on a Blue Robotics Navigator with a Raspberry Pi 5, GPS and
compass, a downward OAK-1 (WildNav stills, MegaDepth relative depth for
DemNav, and the YOLO object tracker), and a downward Benewake rangefinder
for height above ground.

The frame is printed: a 500 mm quad X (X500 class) with two printed hub
plates clamping four 16 mm carbon arm tubes, printed motor mounts, landing
gear and payload mounts. Printing and assembly instructions:
[hardware/parts/aerodrone/README.md](parts/aerodrone/README.md).

- Frame: 500 mm motor to motor, square quad X, ArduPilot `FRAME_CLASS 1`,
  `FRAME_TYPE 1` (`systems/aerodrone/config/arducopter.params`).
- Motor class: 2216, about 920 KV, 10 x 4.5 in propellers, 4S.
- Estimated take-off mass about 2.1 kg with a 4S 5000 mAh pack (computed by
  the assembly check, see the printed-parts README).

## Compatibility notes

- **Raspberry Pi 5 + Navigator.** Blue Robotics' store page still lists the
  Navigator for the Raspberry Pi 4, but its August 2025 hardware revision
  moved the magnetometer "to improve compatibility with Raspberry Pi 5",
  BlueOS 1.4 added Pi 5 support, and users report the pair working with
  BlueOS 1.4.3. Buy the current Navigator revision and run BlueOS 1.4 or
  newer. If in doubt, a Raspberry Pi 4 fits the same printed tray (same
  58 x 49 mm holes).
- **Pi 5 cooling.** The Navigator kit's aluminium heatsink sits between the
  Pi and the Navigator and is the Pi's cooler; the Raspberry Pi Active
  Cooler does not fit under it. DemNav, WildNav and MAVROS keep the CPU
  busy, so check `vcgencmd get_throttled` with the whole stack running on
  the bench, in the sun, before flying.
- **Pi 5 USB current.** The Pi is powered through the Navigator's 40-pin
  header, so it cannot negotiate a 5 A USB-C supply and limits its USB ports
  to 600 mA in total by default. The OAK-1 alone draws up to about 4 W
  (0.8 A): set `usb_max_current_enable=1` in the Pi's `config.txt`
  (`/boot/firmware/config.txt` on Bookworm-based images) or the camera
  browns out under load.
- **OAK-1 variant.** The software assumes a 69 deg horizontal field of view
  (`FOV_DEG` in `compositions/blueos_nav_core.yaml`), which is the
  **fixed-focus** IMX378 OAK-1 (82/69/55 deg). The auto-focus version is
  78/66/54 deg and its lens can hunt under vibration: buy the fixed-focus
  one, or change `FOV_DEG`. The OAK-1 PoE has a different body and does not
  fit the printed cradle.
- **USB 3 and GNSS.** USB 3 radiates in the GNSS L1 band. The GPS sits on a
  130 mm mast at the rear, away from the camera cable; if satellite counts
  drop when the OAK-1 streams, reroute the cable or force the camera to
  USB 2.

## Purchased parts

### Airframe and propulsion

- 4 x carbon fibre tube, **16 mm OD** (16 x 14 mm, roll-wrapped), cut to
  **180.0 mm** (buy 2 x 400 mm or 1 x 1 m; spares for crashes).
- 4 x **Holybro 2216 KV920** motor (the X500 V2 motor: 16 x 16 mm M3 mount),
  plus 1 spare. Any 2216-2312, 880-960 KV motor with a 16 x 16, 16 x 19 or
  19 x 19 mm mount fits the printed mount.
- 4 x **ESC, 20-30 A, 4S, PWM input** (BLHeli_S, BLHeli_32 or AM32), plus 1
  spare. The Navigator drives standard PWM only, so no DShot-only ESC;
  prefer ESCs without a BEC (or remove the BEC's red wire).
- **Propellers 1045** (10 x 4.5 in): at least three sets of 2 CW + 2 CCW.
- 1 x power distribution board or harness: four ESC outputs and one for the
  5 V supply, XT60 battery connector.
- No prop guards: the aircraft flies outdoors at 15-120 m, full guards for
  10 in props add a few hundred grams, cost thrust and (264 mm across) do
  not fit the 250 mm print bed in one piece. Do first hovers tethered or
  over a net instead.

### Power

- 2-3 x **4S LiPo, 4000-5000 mAh**, 25C or more, XT60, at most
  160 x 50 x 40 mm (the tray and straps take any pack up to that size);
  about 550 g for 5000 mAh.
- 1 x LiPo balance charger, LiPo bag, LiPo voltage checker.
- 1 x **Blue Robotics Power Sense Module** (6S, 100 A non-continuous; 5.5 mm
  bullet input, #6 spade output, JST-GH cable to the Navigator **POWER**
  port). It senses only: it does not supply 5 V. Add an XT60 pigtail for its
  input.
- 1 x **Blue Robotics 5 V 6 A Power Supply** (spade input, 0.1 in header
  output) for the Navigator's main 5 V input; the Navigator powers the Pi.
  Blue Robotics asks for a supply of at least 4.5 A.
- Silicone wire (12-14 AWG for power, 20-22 AWG for leads), XT60 and
  5.5 mm bullet connectors, heat-shrink tubing.

### Flight controller and compute

- 1 x **Raspberry Pi 5**, 8 GB (16 GB optional).
  Possible later alternative, untested: an Arduino VENTUNO Q (16 GB, NPU)
  as the companion computer; see "Where it runs" in
  applications/demnav/README.md.
- 1 x **Blue Robotics Navigator Flight Controller** kit (current revision):
  includes the heatsink, a 128 GB microSD with BlueOS, thermal pads,
  JST-GH cables and M2.5 screws. Run BlueOS 1.4 or newer and ArduCopter
  (4.6, the version the parameters were validated on in SITL).
- No separate microSD, Active Cooler or NVMe HAT: the kit's card and
  heatsink are used, and a PCIe HAT does not stack with the Navigator.

### Sensors

- 1 x **Luxonis OAK-1, fixed-focus** (USB, IMX378, 69 deg HFOV), nadir
  facing, USB-C end toward the tail (image top toward the nose,
  `CAMERA_MOUNT_YAW_DEG = 0.0`). It runs MegaDepth and the YOLO tracker on
  its Myriad X and publishes `/oak1/image_highres`, `/oak1/relative_depth`
  and the `/tracking/...` topics.
- 1 x **USB 3 Type-C to Type-A cable, about 0.3 m, right-angle C end**
  (exits sideways from the camera).
- 1 x **Benewake TFmini-S** (UART version), downward. Benewake TF-Luna is
  the budget alternative (same mount slots, `RANGEFINDER_MODEL=tf_luna`); a
  LightWare SF11/C needs its own mount. Keep it even with the OAK-1:
  MegaDepth depth is relative, not metric.
- 1 x **3.3 V USB-UART adapter** (CP2102 or FT232, with a 5 V pin) for the
  rangefinder: `compositions/aerodrone_hardware.yaml` reads it on the Pi
  (`RANGEFINDER_DEVICE`), not through ArduPilot.
- 1 x **Holybro M10** (or M9N) standard GPS with IST8310 compass. Its
  standard cable is a 10-pin JST-GH for a Pixhawk GPS1 port; the Navigator's
  GPS port (SERIAL 5 + I2C 6) is a 6-pin JST-GH: get or make a 6-pin
  cable and check the pin order before plugging in.

### Radio

- 1 x RC transmitter and **SBUS-output receiver** (or ExpressLRS set to SBUS
  output) on the Navigator's 3-pin 0.1 in RC header (inverted SBUS by
  default).
- 1 x telemetry radio pair (915 or 868 MHz SiK) on a Navigator SERIAL port
  (JST-GH 6); Wi-Fi or Ethernet to BlueOS on the bench.

### Vibration isolation and mounting

- 4 x **silicone gimbal damper balls** with 6.5 mm necks (for 1.5-2 mm
  plates), plus spares: they isolate the OAK-1 stage. (The stack tray sits
  on printed TPU grommets.)
- Screws, nuts, washers, heat-set inserts, straps, foam tape and zip ties:
  the generated [fastener table](#fasteners) below lists every one.

### Field and safety kit

- Spare propellers, spare arm tube, spare ESC and motor, spare printed
  motor mount and landing leg.
- Smoke stopper, multimeter, LiPo checker, landing pad, fire-safe bag.

### Why these parts are bought, not printed

| Part | Why it stays purchased |
|---|---|
| Propellers | Spin at several thousand rpm with centrifugal and fatigue loads printed layers cannot carry; they must be balanced and true. A failed prop is a crash. |
| Motors, ESCs | Electromagnetic and power electronics. |
| Arm tubes | A 180 mm cantilever carrying a motor: carbon fibre's modulus is roughly 30 times that of printed PETG or ASA, so a printed arm of the same stiffness is several times heavier, and a lighter one flexes and resonates in the 80-170 Hz rotor band the IMU must not see. Printed layers also creep and fatigue. |
| Battery, straps | Energy storage; a strap or a printed buckle that lets go drops the pack. Two straps, each in its own slot pair. |
| Flight controller, Pi, sensors | Electronics. |
| Screws, inserts, nuts | Metal threads; printed threads strip and creep. |
| Damper balls | Soft silicone is about an order of magnitude softer than TPU 95A; a printable TPU isolator for the 0.13 kg camera stage would resonate near or above 100 Hz, inside the rotor band, and amplify vibration instead of removing it. |
| Wiring, connectors | Carry up to about 70 A. |

### Power and wiring

5 V budget (Navigator rail, from the 5 V 6 A supply):

| Load | Typical | Peak | Source |
|---|---|---|---|
| Raspberry Pi 5 under the navigation load | 1.5 A | 2.0 A | estimate |
| OAK-1 (via Pi USB 3) | 0.6 A | 0.8 A | Luxonis: 2.5-3 W + up to 1 W AI |
| Navigator | 0.15 A | 0.2 A | Blue Robotics |
| GPS (M10) | 0.1 A | 0.2 A | Holybro: < 200 mA |
| TFmini-S (via USB-UART) | 0.14 A | 0.14 A | Benewake: <= 140 mA average |
| RC receiver | 0.1 A | 0.1 A | estimate |
| Telemetry radio | 0.1 A | 0.5 A | estimate (transmit peaks) |
| **Total** | **about 2.8 A** | **about 3.9 A** | |

The supply (6 A) has margin, but Blue Robotics rates the Navigator's
power to the Pi and peripherals at 4 A continuous, 4.5 A peak: the peak
budget is close to it. If `vcgencmd get_throttled` reports under-voltage,
power the telemetry radio from a separate 5 V BEC on the power
distribution. Motors: about 15-20 A in hover and 65-75 A at full throttle
(estimates for this motor and prop at 4S), within the Power Sense Module's
100 A; hover time with a 5000 mAh pack roughly 12 min (estimate).

Signal and power connections are listed in
[the printed-parts README, "Wiring"](parts/aerodrone/README.md#wiring).
ArduPilot battery monitor for the Power Sense Module: `BATT_MONITOR 4`,
`BATT_VOLT_MULT 11.0`, `BATT_AMP_PERVLT 37.8788`, `BATT_AMP_OFFSET 0.330`
(Blue Robotics' scale factors; BlueOS normally sets them).

## 3D-printed parts

Designed in CadQuery (`hardware/parts/aerodrone/`), validated and exported
by `tools/build_printable_parts.py`; STLs in
[`parts/aerodrone/stl/`](parts/aerodrone/stl/). Printer: Bambu Lab P1S.
Plates, settings, post-processing and assembly:
[parts/aerodrone/README.md](parts/aerodrone/README.md). Print the
"fit test first" parts before the rest. Times and masses are estimates.

<!-- BEGIN GENERATED: printed-parts (tools/build_printable_parts.py) -->
| Part | Qty | Material | Bambu Studio settings (P1S, 0.4 mm) | Est. time (each) | Est. mass (each) | File |
|---|---|---|---|---|---|---|
| Battery tray (flight-critical) | 1 | PETG | 0.20 mm, 3 walls, 30% gyroid, no supports | 55 min | 32 g | [battery_tray.stl](parts/aerodrone/stl/battery_tray.stl) |
| Arm cable clip | 8 | PETG | 0.20 mm, 4 walls, 100% gyroid, no supports | 5 min | 2.0 g | [cable_clip.stl](parts/aerodrone/stl/cable_clip.stl) |
| GPS mast | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 1 h 20 min | 34 g | [gps_mast.stl](parts/aerodrone/stl/gps_mast.stl) |
| Hub bottom plate (flight-critical) | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 2 h 59 min | 102 g | [hub_bottom_plate.stl](parts/aerodrone/stl/hub_bottom_plate.stl) |
| Hub top plate (flight-critical) | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 3 h 01 min | 103 g | [hub_top_plate.stl](parts/aerodrone/stl/hub_top_plate.stl) |
| Landing leg | 4 | ASA | 0.20 mm, 5 walls, 40% gyroid, no supports | 46 min | 26 g | [landing_leg.stl](parts/aerodrone/stl/landing_leg.stl) |
| Landing foot | 4 | TPU95A | 0.20 mm, 3 walls, 30% gyroid, no supports | 23 min | 5.0 g | [leg_foot.stl](parts/aerodrone/stl/leg_foot.stl) |
| Motor mount (flight-critical, fit test first) | 4 | ASA | 0.20 mm, 5 walls, 50% gyroid, no supports | 46 min | 24 g | [motor_mount.stl](parts/aerodrone/stl/motor_mount.stl) |
| OAK-1 cradle (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 31 min | 15 g | [oak1_cradle.stl](parts/aerodrone/stl/oak1_cradle.stl) |
| OAK-1 camera stage (isolated, pitch pivot) (fit test first) | 1 | PETG | 0.20 mm, 4 walls, 40% gyroid, no supports | 48 min | 25 g | [oak1_stage.stl](parts/aerodrone/stl/oak1_stage.stl) |
| Sensor nose (OAK-1 damper deck and rangefinder mount) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports | 58 min | 32 g | [sensor_nose.stl](parts/aerodrone/stl/sensor_nose.stl) |
| Stack grommet (TPU top hat) | 8 | TPU95A | 0.20 mm, 3 walls, 100% gyroid, no supports | 1 min | 0.2 g | [stack_grommet.stl](parts/aerodrone/stl/stack_grommet.stl) |
| Stack tray (Pi 5 + Navigator) (flight-critical, fit test first) | 1 | PETG | 0.20 mm, 3 walls, 30% gyroid, no supports | 25 min | 15 g | [stack_tray.stl](parts/aerodrone/stl/stack_tray.stl) |
| **Total, all pieces** | | | | **19 h 31 min** | **593 g** | |
<!-- END GENERATED: printed-parts -->

### Fasteners

Totals for one aircraft, from the parts' metadata (screws are listed with
the part their head bears on). Buy 10-20 % spares.

<!-- BEGIN GENERATED: fasteners (tools/build_printable_parts.py) -->
| Item | Qty | Where |
|---|---|---|
| 1/4"-20 UNC x 1/2" button or socket head screw | 1 | OAK-1 to cradle (engages about 8 mm of the 10 mm thread) (oak1_cradle, 1) |
| M2.5 x 12 socket head cap screw | 2 | from under the tray through the Pi into the Navigator heatsink (the two holes the Navigator guide's standoff uses) (stack_tray, 2) |
| M3 washer | 22 | under the arm clamp screw heads (hub_top_plate, 16); under the pitch-lock screws (oak1_stage, 2); on the upper stack grommets (stack_tray, 4) |
| M3 x 10 socket head cap screw | 7 | GPS mast to top plate (gps_mast, 3); sensor nose to bottom plate (sensor_nose, 4) |
| M3 x 12 socket head cap screw | 4 | stack tray to top plate, through a pair of stack grommets (stack_tray, 4) |
| M3 x 14 socket head cap screw | 4 | battery tray to bottom plate (heads recessed in the posts) (battery_tray, 4) |
| M3 x 16 socket head cap screw | 8 | landing leg to bottom plate (landing_leg, 8) |
| M3 x 25 socket head cap screw | 24 | arm clamps, through the top plate into the bottom plate inserts (hub_top_plate, 16); pinch clamp on the arm tube (motor_mount, 8) |
| M3 x 3 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 4 | side bosses: pitch pivot (2) and pitch lock (2), pressed flush from outside (through holes) (oak1_cradle, 4) |
| M3 x 5.7 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 47 | clamp blocks, from the top face (arm clamp bolts) (hub_bottom_plate, 16); underside of the ears, two per arm (landing legs) (hub_bottom_plate, 8); underside, rear and middle bosses (battery tray) (hub_bottom_plate, 4); underside, front bosses (sensor nose) (hub_bottom_plate, 4); top face, stack tray grommet bolts (hub_top_plate, 4); top face, GPS mast flange (hub_top_plate, 3); far side of the pinch clamp (motor_mount, 8) |
| M3 x 8 socket head cap screw | 4 | pitch pivot (2) and pitch lock (2), into the cradle inserts (oak1_stage, 4) |
| M3 x 8 socket head cap screw (or the motor's own screws) | 16 | motor to mount (motor_mount, 16) |
| ST2.9 x 9.5 pan head self-tapping screw (TFmini-S) or M2 x 12 + nut (TF-Luna) | 2 | rangefinder ears, from above (sensor_nose, 2) |
| anti-slip foam pad, 150 x 50 mm | 1 | between the battery and the tray (battery_tray, 1) |
| battery strap, 20 mm hook-and-loop, 250-300 mm, with rubberised face | 2 | battery to tray (battery_tray, 2) |
| double-sided foam tape (VHB), 50 mm disc | 1 | GPS puck on the mast seat (gps_mast, 1) |
| silicone gimbal damper ball, 6.5 mm neck | 4 | camera stage isolation (sensor_nose, 4) |
| zip tie 2.5 mm | 12 | motor leads to the arm cable clip (cable_clip, 8); power sense module and 5 V BEC to the four centre slots (hub_bottom_plate, 4) |
<!-- END GENERATED: fasteners -->

### Assembly notes

- Build order, torques and checks: [parts/aerodrone/README.md](parts/aerodrone/README.md#steps).
- Arm tubes: cut to 180.0 mm, square; they set the 500 mm motor spacing.
- Battery position sets the centre of gravity: about 24 mm behind the hub
  centre with the parts above (the assembly check computes it); trim by
  balancing.
- Motor seats must be parallel to the hub before the pinch screws are
  tightened.
- Nothing hangs in the OAK-1's view: the legs splay outward and the battery
  sits above the lens; the assembly check verifies it with a 2 deg margin.

### Simulator geometry

The simulators model a different airframe (not changed here):

- Gazebo (`applications/gazebo`) flies Aerostack2's `quadrotor_base` model
  (its geometry lives in the `as2_gazebo_assets` package, not in this
  repository) with the downward cameras on the body centre, 0.05-0.06 m
  below it, and a 0.34 x 0.34 m landing-gear collision box 0.13 m down
  (`models/quadrotor_ardupilot.sdf.jinja`); in flight it learns
  `MOT_THST_HOVER 0.67`.
- Isaac (`applications/isaac/scripts/aerodrone_navsim.py`): a 1.5 kg,
  0.47 x 0.47 x 0.11 m body with rotors at (+-0.13, +-0.20-0.22) m, a
  rectangle wider than it is long rather than a square X, and the camera
  0.08 m below the centre.
- Genesis (`applications/genesis/scripts/aerodrone_navsim.py`) flies the
  27 g Crazyflie `cf2x` with 28 mm arms and compensates in
  `applications/genesis/config/aerodrone_sitl.params`.
- The printed aircraft: about 2.1 kg, square quad X with rotors at
  (+-177, +-177) mm, OAK-1 lens about 117 mm ahead of and 80 mm below the
  arm-tube plane, rangefinder 152 mm ahead, GPS 57 mm behind and about
  150 mm above, feet at (+-106, +-106) mm, 206 mm below. Its hover throttle
  is unknown until flown and will differ from the `MOT_THST_HOVER 0.67` in
  `arducopter.params`, which the Gazebo model learned; ArduPilot relearns it
  in the first hovers (`MOT_HOVER_LEARN`). Measured camera and
  rangefinder extrinsics belong in `systems/aerodrone/config/sensors.yaml`
  once calibrated.

## Recommended assembly order

1. Print, then assemble the frame, motors, ESCs and power distribution with
   the propellers off ([parts/aerodrone/README.md](parts/aerodrone/README.md)).
2. Mount the Raspberry Pi/Navigator stack and wire the PWM outputs, Power
   Sense Module, GPS/compass, RC receiver and telemetry.
3. Verify ArduPilot motor order, RC failsafe, battery monitor, compass, and
   IMU calibration before installing propellers.
4. Mount the OAK-1 nadir and verify the MegaDepth pipeline:
   - WildNav should consume /oak1/image_highres.
   - DemNav should consume /oak1/relative_depth.
   - Camera calibration should come from /oak1/camera_info or
     /oak1/image_highres/camera_info, depending on the consumer.
5. Run tethered or prop-off bench tests until GPS, odom, OAK-1 RGB,
   MegaDepth relative depth, rangefinder altitude, and camera calibration
   topics are stable.
6. Perform manual hover and failsafe tests before enabling any autonomous
   waypoint, DemNav, WildNav or tracking behaviour; GPS-denied flight as in
   systems/aerodrone/README.md.

## Useful references

- Printed parts, printing and assembly: hardware/parts/aerodrone/README.md
- Project Aerostack2 nav topics: systems/aerodrone/config/topics.env
- Project Aerostack2 simulation notes: applications/gazebo/AERODRONE.md
- Project BlueOS/OAK-1 Aerostack composition:
  compositions/blueos_aerostack.yaml
- Project OAK-1 MegaDepth node: applications/oak1/scripts/megadepth_ros.py
- Project rangefinder driver: applications/rangefinder/README.md
- Raspberry Pi 5 mechanical drawing:
  https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf
- Blue Robotics Navigator Flight Controller:
  https://bluerobotics.com/store/comm-control-power/control/navigator/
- Navigator Hardware Setup:
  https://bluerobotics.com/learn/navigator-hardware-setup/
- Blue Robotics Power Sense Module:
  https://bluerobotics.com/store/comm-control-power/elec-packages/psm-asm-r2-rp/
- Luxonis OAK-1: https://shop.luxonis.com/products/oak-1 and
  https://docs.luxonis.com/hardware/products/OAK-1
- Benewake TFmini-S: https://en.benewake.com/TFminiS/index.html
- Benewake TF-Luna datasheet:
  https://en.benewake.com/uploadfiles/2024/04/20240426135921367.pdf
- Holybro X500 V2 (motors, props, frame class):
  https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2
- Holybro M10/M9N GPS:
  https://docs.holybro.com/gps-and-rtk-system/m8n-m9n-m10-gps
