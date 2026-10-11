# Airplane bill of materials

Goal: a fixed-wing mapping airplane that runs the BlueOS stack of
`compositions/airplane_hardware.yaml` (`systems/airplane/`): ArduPlane on
a Blue Robotics Navigator with a Raspberry Pi 5, GPS and compass, an
airspeed sensor, and a downward OAK-1 for WildNav stills and MegaDepth
relative depth for DemNav, publishing the `/plane0/sensor_measurements/*`
and `/airplane/downward/*` topics.

The airframe is a purchased EPO foam flying wing (reference: **Skywalker
X8**, 2120 mm); everything that holds the avionics inside and on it is
printed: avionics deck and Pi 5 + Navigator tray, the nadir camera frame
and damped cradle, battery tray, GPS, antenna and pitot mounts, cable
clips. Printing and assembly instructions:
[hardware/parts/airplane/README.md](parts/airplane/README.md).

- Airframe: Skywalker X8, 2120 mm span, 80 dm2, flying weight 2.5-3.0 kg,
  payload 1-2 kg, CG 430-440 mm behind the nose, 65-70 km/h (18-19 m/s:
  `AIRSPEED_CRUISE 19` in `systems/airplane/config/ardupilot_plane.params`),
  catapult or bungee launch, belly landing.
- Power: 6S 5000-6000 mAh, a 4120-class ~550 KV pusher motor, 12 x 6 in
  prop, 60-100 A ESC (the X8 listing's recommendation).
- Printed payload: about 250 g; the whole payload (avionics, printed parts,
  wiring, 6S pack) about 1.6 kg (assembly check, see the printed-parts
  README), inside the X8's 1-2 kg.

## Compatibility notes

- **Airframe choice.** The previous list offered the Heewing T1 Ranger,
  but that is a 730 mm, 350-500 g aircraft whose 245 x 51 x 48 mm cabin
  cannot take the 56 mm wide Pi 5 + Navigator stack, let alone the OAK-1
  and a battery. Any EPO pusher of the X8's class works if its bay meets
  the printed parts' envelope (a flat floor about 110 mm wide, 60-65 mm of
  height under the hatch, a belly to cut the camera hole in) and it carries
  about 1.6 kg of payload; a conventional pusher (for example a Volantex
  Ranger EX class 2 m airframe) flies closer to the SITL model but most
  have narrower bays: hold the printed parts in it before buying.
- **SITL model.** `compositions/airplane_sim.yaml` runs ArduPilot's
  built-in conventional `plane` model (ailerons, elevator, rudder); the X8
  is a flying wing with two elevons (`SERVO1_FUNCTION 77` elevon left,
  `SERVO2_FUNCTION 78` elevon right, `SERVO3_FUNCTION 70` throttle). For
  closer SITL behaviour set `AIRPLANE_SITL_MODEL=plane-elevon` in
  `systems/airplane/config/sitl.env`; the attitude and navigation tuning
  comes from AUTOTUNE on the real aircraft either way.
- **Raspberry Pi 5 + Navigator.** As for the aerodrone
  ([hardware/aerodrone.md](aerodrone.md#compatibility-notes)): buy the
  current Navigator revision (August 2025: magnetometer moved for Pi 5
  compatibility) and run BlueOS 1.4 or newer. The Navigator kit's heatsink
  is the Pi's cooler: no Active Cooler (it does not fit under the heatsink)
  and no NVMe HAT (it does not stack with the Navigator).
- **Pi 5 USB current.** The Pi is powered through the Navigator's header,
  so it limits its USB ports to 600 mA unless `usb_max_current_enable=1` is
  set in `/boot/firmware/config.txt`; the OAK-1 alone draws up to 0.8 A.
- **OAK-1 variant.** `FOV_DEG=69.0` in `systems/airplane/config/blueos.env`
  and `horizontal_fov_deg: 69.0` in the SITL camera are the **fixed-focus**
  OAK-1 (82/69/55 deg). The OAK-1 PoE (offered before) has a different
  body that does not fit the printed cradle and needs a PoE injector on the
  Pi's only Ethernet port: use the USB OAK-1.
- **No second camera.** The previous list suggested an optional nadir
  global-shutter camera; nothing in the software reads one. At the
  scenario's 180 m and 24 m/s the OAK-1 sees a ground pixel of about
  19 cm, and a 1 ms exposure smears 2.4 cm: add a second camera only
  together with the software that uses it.
- **Airspeed.** The SITL parameters set `ARSPD_TYPE 0` and `ARSPD_USE 0`
  (SITL only). On the aircraft set `ARSPD_TYPE 1` (MS4525), `ARSPD_USE 1`
  and the bus of the Navigator's I2C 6 port, and calibrate it in BlueOS
  before the first flight: ArduPlane's stall protection and TECS depend on
  it in wind.
- **Servo power.** The Navigator does not power its servo rail from its
  main 5 V input: the elevon servos need a separate BEC on its **AUX 5V**
  input (Blue Robotics, Navigator Hardware Setup).
- **USB 3 and GNSS.** USB 3 radiates in the GNSS L1 band: the GPS goes on
  the left wing, 250 mm out, at least 150 mm from the USB 3 cable, the
  battery leads and the ESC (the assembly check measures it).
- **GPS_INPUT corrections** stay off on the airplane
  (`systems/airplane/README.md`); nothing in this list depends on them.

## Purchased parts

### Airframe and propulsion

- 1 x **Skywalker X8** EPO flying wing kit (2120 mm), with its carbon
  spars and control horns, plus a field-repair kit (foam-safe CA with
  kicker, packing tape, spare control horns and clevises).
- 2 x **17 g metal-gear digital servo** for the elevons (the X8 listing's
  size), with servo extension leads to the bay.
- 1 x **4120-class motor, about 550 KV** (for example SunnySky X4120
  550 KV, the X8 listing's suggestion), with its prop adapter.
- 1 x **ESC, 60-80 A, 6S, PWM input**, opto (no BEC) or with its BEC's red
  wire removed: the Navigator drives standard PWM and its servo rail is fed
  by the separate servo BEC below.
- **12 x 6 in folding propellers, pusher** (a reverse-pitch "P" prop, or
  a standard prop with the motor direction reversed in the ESC so it pushes
  air aft), at least three, and a spinner/folding hub for the motor shaft.

### Power

- 2-3 x **6S LiPo, 5000-6000 mAh**, 35C, XT90 (or XT60 if the ESC uses
  it), at most 170 x 52 x 62 mm: the battery tray takes any pack up to
  that size; about 900 g.
- 1 x LiPo balance charger for 6S, LiPo bag, LiPo checker.
- 1 x **Blue Robotics Power Sense Module** (25.2 V / 6S max, 100 A
  non-continuous; 5.5 mm bullet input, #6 spade output, JST-GH to the
  Navigator **POWER** port). It senses only; it does not supply 5 V. Fit
  an XT90 pigtail on its input.
- 1 x **Blue Robotics 5 V 6 A Power Supply** (7-26 V input) for the
  Navigator's main 5 V input; the Navigator powers the Pi.
- 1 x **servo BEC, 5 V, 3 A or more** (input 6S) on the Navigator's
  **AUX 5V** input (the servo rail).
- Silicone wire (12 AWG for power, 20-22 AWG for leads), XT90 and 5.5 mm
  bullet connectors, heat-shrink.

### Flight controller and compute

- 1 x **Raspberry Pi 5**, 8 GB (16 GB optional).
- 1 x **Blue Robotics Navigator Flight Controller** kit (current revision):
  heatsink, 128 GB microSD with BlueOS, thermal pads, JST-GH cables and
  M2.5 screws. BlueOS 1.4 or newer and ArduPlane installed through
  BlueOS, 4.6 when it offers it (the version SITL runs:
  `ARDUPILOT_REF=Plane-4.6.3`).

### Sensors

- 1 x **Luxonis OAK-1, fixed-focus** (USB, IMX378, 69 deg HFOV), nadir,
  USB-C end toward the tail (image top toward the nose,
  `CAMERA_MOUNT_YAW_DEG=0.0`). It runs MegaDepth on its Myriad X and
  publishes `/airplane/downward/image_raw`, `/image_highres` and
  `/relative_depth` (`OAK1_TRACKING=false` on the airplane).
- 1 x **USB 3 Type-C to Type-A cable, about 0.3 m, right-angle C end**.
- 1 x **Holybro Digital Air Speed Sensor MS4525DO with the PT60 pitot**
  (I2C, 5 V, GH 4-pin cable to the Navigator's **I2C 6** port; 1 m of
  silicone tubing).
- 1 x **Holybro M10** (or M9N) standard GPS with IST8310 compass, and a
  6-pin JST-GH cable about 400 mm long for the Navigator's
  **SERIAL 5 + I2C 6** port (the puck's own cable is a 10-pin Pixhawk GPS1
  cable: make or buy a 6-pin one and check the pin order).
- No rangefinder: the airplane's DemNav runs without one
  (`RANGEFINDER_TOPIC` unset in `systems/airplane/config/blueos.env`).

### Radio

- 1 x RC transmitter and **SBUS-output receiver** (or ExpressLRS set to
  SBUS) on the Navigator's RC header, with two antennas at 90 deg.
- 1 x **telemetry radio pair** (915 or 868 MHz SiK) on a Navigator
  SERIAL port, and an **RP-SMA bulkhead extension cable** (150-200 mm) to
  take its antenna out through the antenna mount. Wi-Fi or Ethernet to
  BlueOS on the bench.

### Vibration isolation and mounting

- 4 x **silicone gimbal damper balls** with 6.5 mm necks for the OAK-1
  cradle (the stack tray sits on printed TPU grommets).
- Foam-safe adhesives: 5-minute epoxy and UHU Por (or hot glue) to bond
  the printed parts to the EPO; plain cyanoacrylate dissolves EPO.
- Screws, inserts, straps, zip ties and foam tape: the generated
  [fastener table](#fasteners) lists every one.

### Launch, recovery and field kit

- 1 x **bungee launcher** (or catapult) for a 3 kg wing (the X8 listing:
  catapult or bungee launch); landing is on the belly in grass.
- Spare propellers, spare servo, spare printed camera frame and cradle,
  foam repair kit, landing markers, smoke stopper, multimeter.
- Operator laptop with QGroundControl, Docker and the ROS 2 tools.

### Why these parts are bought, not printed

| Part | Why it stays purchased |
|---|---|
| Airframe (wing, fuselage, spars) | Printed airframes exist (LW-PLA designs from Eclipson, 3DLabPrint, Planeprint), but none is a 2 m, 1-2 kg payload mapping wing; their STLs are paid and cannot be committed under this repository's MIT licence; foamed PLA softens around 55-60 C (a car or a dark wing in the sun; Bambu's foaming ASA Aero is rated to 78-85 C but is not in this toolchain's profiles); and an X8-size airframe would be tens of bed-size sections glued along spars, 40-60 h of printing, and heavier than the moulded EPO. EPO also takes a belly landing every flight and is glued in the field where printed skins crack. A new airframe designed here would have no aerodynamic or flight validation behind it. |
| Propeller | Spins at several thousand rpm with fatigue loads printed layers cannot carry; must be balanced. |
| Motor, ESC, servos | Electromechanical and power electronics. |
| Carbon spars and control horns | Come with the kit; carry the wing bending loads. |
| Battery, straps | Energy storage; a strap that lets go moves the CG and ends the flight. |
| Flight controller, Pi, sensors, radios | Electronics. |
| Damper balls | Soft silicone is far softer than TPU 95A; a printable TPU isolator for the 0.1 kg camera would resonate too high to isolate anything. |
| Screws, inserts | Metal threads; printed threads strip and creep. |

### Power and wiring

5 V budget (Navigator main rail, from the 5 V 6 A supply):

| Load | Typical | Peak | Source |
|---|---|---|---|
| Raspberry Pi 5 under the navigation load | 1.5 A | 2.0 A | estimate |
| OAK-1 (via Pi USB 3) | 0.6 A | 0.8 A | Luxonis: 2.5-3 W + up to 1 W AI |
| Navigator | 0.15 A | 0.2 A | Blue Robotics |
| GPS (M10) | 0.1 A | 0.2 A | Holybro: < 200 mA |
| Airspeed sensor | 0.01 A | 0.01 A | estimate |
| RC receiver | 0.1 A | 0.1 A | estimate |
| Telemetry radio | 0.1 A | 0.5 A | estimate (transmit peaks) |
| **Total** | **about 2.6 A** | **about 3.8 A** | |

Within the supply's 6 A, but close to the Navigator's 4.5 A peak to the
Pi and peripherals: if `vcgencmd get_throttled` reports under-voltage,
feed the telemetry radio from the servo BEC instead. Servo rail (servo
BEC on AUX 5V): two 17 g servos, about 0.3 A cruising and 1-1.5 A each at
stall (estimate): 3 A or more. Motor: up to about 50-60 A at full
throttle on 6S (estimate for this motor and prop), within the Power Sense
Module's 100 A. Endurance: up to 25 min on LiPo (the X8 listing).

Signal and power connections:
[the printed-parts README, "Wiring"](parts/airplane/README.md#wiring).
ArduPilot battery monitor for the Power Sense Module: `BATT_MONITOR 4`,
`BATT_VOLT_MULT 11.0`, `BATT_AMP_PERVLT 37.8788`, `BATT_AMP_OFFSET 0.330`
(Blue Robotics' scale factors; BlueOS normally sets them).

## 3D-printed parts

Designed in CadQuery (`hardware/parts/airplane/`), validated and exported
by `tools/build_printable_parts.py`; STLs in
[`parts/airplane/stl/`](parts/airplane/stl/). Printer: Bambu Lab P1S.
Plates, settings, post-processing and assembly:
[parts/airplane/README.md](parts/airplane/README.md). Print the "fit test
first" parts before the rest: the X8's bay is not dimensioned by its
maker, so every part that is bonded into it is checked in the real bay
first. Times and masses are estimates.

<!-- BEGIN GENERATED: printed-parts (tools/build_printable_parts.py) -->
| Part | Qty | Material | Bambu Studio settings (P1S, 0.4 mm) | Est. time (each) | Est. mass (each) | File |
|---|---|---|---|---|---|---|
| Antenna mount (telemetry RP-SMA and RC receiver antennas) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports | 27 min | 13 g | [antenna_mount.stl](parts/airplane/stl/antenna_mount.stl) |
| Avionics deck (flight-critical, fit test first) | 1 | ASA | 0.20 mm, 3 walls, 20% gyroid, no supports, brim outer brim only, 5 mm | 1 h 21 min | 46 g | [avionics_deck.stl](parts/airplane/stl/avionics_deck.stl) |
| Battery tray (flight-critical, fit test first) | 1 | PETG | 0.20 mm, 3 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 1 h 38 min | 58 g | [battery_tray.stl](parts/airplane/stl/battery_tray.stl) |
| Cable clip (bonded) | 8 | PETG | 0.20 mm, 3 walls, 100% gyroid, no supports | 3 min | 1.2 g | [cable_clip.stl](parts/airplane/stl/cable_clip.stl) |
| Camera deck (OAK-1 damper deck) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports | 39 min | 22 g | [camera_deck.stl](parts/airplane/stl/camera_deck.stl) |
| Camera frame (belly cut-out template and pillars) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports | 1 h 00 min | 30 g | [camera_frame.stl](parts/airplane/stl/camera_frame.stl) |
| GPS mount (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports | 45 min | 24 g | [gps_mount.stl](parts/airplane/stl/gps_mount.stl) |
| OAK-1 cradle (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 40 min | 21 g | [oak1_cradle.stl](parts/airplane/stl/oak1_cradle.stl) |
| Pitot mount (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports | 29 min | 14 g | [pitot_mount.stl](parts/airplane/stl/pitot_mount.stl) |
| Stack grommet (TPU top hat) | 8 | TPU95A | 0.20 mm, 3 walls, 100% gyroid, no supports | 1 min | 0.2 g | [stack_grommet.stl](parts/airplane/stl/stack_grommet.stl) |
| Stack tray (Pi 5 + Navigator) (flight-critical, fit test first) | 1 | ASA | 0.20 mm, 3 walls, 30% gyroid, no supports | 23 min | 12 g | [stack_tray.stl](parts/airplane/stl/stack_tray.stl) |
| **Total, all pieces** | | | | **7 h 57 min** | **252 g** | |
<!-- END GENERATED: printed-parts -->

### Fasteners

Totals for one aircraft, from the parts' metadata (screws are listed with
the part their head bears on). Buy 10-20 % spares.

<!-- BEGIN GENERATED: fasteners (tools/build_printable_parts.py) -->
| Item | Qty | Where |
|---|---|---|
| 1/4"-20 UNC x 1/2" button or socket head screw | 1 | OAK-1 to cradle (engages about 8 mm of the 10 mm thread) (oak1_cradle, 1) |
| M2.5 x 12 socket head cap screw | 2 | from under the tray through the Pi into the Navigator heatsink (the two holes the Navigator guide's standoff uses) (stack_tray, 2) |
| M3 washer | 4 | on the upper stack grommets (stack_tray, 4) |
| M3 x 10 socket head cap screw | 4 | camera deck to the frame pillars (camera_deck, 4) |
| M3 x 12 socket head cap screw | 4 | stack tray to the avionics deck, through a pair of stack grommets (stack_tray, 4) |
| M3 x 5.7 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 8 | stack grommet bosses, from the top (avionics_deck, 4); pillar tops (camera deck screws) (camera_frame, 4) |
| RP-SMA bulkhead extension cable, 150-200 mm (to suit the telemetry radio) | 1 | telemetry antenna through the antenna mount (antenna_mount, 1) |
| anti-slip foam pad, 160 x 50 mm | 1 | between the battery and the tray (battery_tray, 1) |
| battery strap, 20 mm hook-and-loop, 300-350 mm, with rubberised face | 2 | battery to tray (battery_tray, 2) |
| double-sided foam tape (VHB), 50 mm disc | 1 | GPS puck on the mount seat (gps_mount, 1) |
| silicone gimbal damper ball, 6.5 mm neck | 4 | camera cradle isolation (camera_deck, 4) |
| zip tie 2.5 mm, 150 mm | 6 | power modules and radios to the deck's tie bridges (avionics_deck, 6) |
| zip tie 3.6 mm, 150 mm | 8 | cables to the cable clip (cable_clip, 8) |
| zip tie 3.6 mm, 200 mm | 1 | pitot base in the pitot mount, through the floor tunnel (pitot_mount, 1) |
<!-- END GENERATED: fasteners -->

### Assembly notes

- Build order, bonding and checks:
  [parts/airplane/README.md](parts/airplane/README.md#steps).
- The battery position sets the centre of gravity: the assembly check
  puts the pack's centre about 170 mm ahead of the CG mark, assuming the
  bare airframe balances on it (Skywalker publishes no empty-airframe CG);
  balance the real aircraft at 430-440 mm behind the nose and mark the
  pack's place on the tray.
- The lens sits recessed above the belly (belly thickness + 5 mm) and
  nothing hangs in the OAK-1's view; the assembly check verifies the view
  through a 14 mm belly with a 2 deg margin.

### Simulator geometry

The SITL stack flies ArduPilot's built-in `plane` model (a conventional
aircraft) and renders the nadir camera at 180 m AGL and 24 m/s with a 69 deg
horizontal FOV (`systems/airplane/config/satellite_camera_sim.yaml`). The
printed aircraft: an X8 flying wing (elevons), OAK-1 lens about 5 mm ahead
of the CG mark on the centre line, GPS 250 mm out on the left wing, pitot at
the nose. Measured camera extrinsics belong in a calibration file once
flown.

## Recommended assembly order

1. Build the X8 kit (wings, spars, elevon servos and linkages, motor and
   ESC), then print the parts and check the fit-test parts in the bay.
2. Bond and fit the avionics as in
   [parts/airplane/README.md](parts/airplane/README.md), wire the power
   path, Navigator, servos, receiver, telemetry, GPS and airspeed sensor.
3. Props off: in BlueOS/QGroundControl set the frame outputs (elevons,
   throttle), check the elevon directions in MANUAL and FBWA, calibrate
   accelerometers, compass and airspeed, set the RC and GCS failsafes and
   the battery monitor.
4. Mount the OAK-1 and verify the camera topics on the bench: WildNav
   consumes `/airplane/downward/image_highres`, DemNav
   `/airplane/downward/relative_depth` and the `camera_info` topics.
5. Balance at 430-440 mm behind the nose with the flight battery fitted.
6. Manual flights (FBWA), autotune, then AUTO missions
   (`systems/airplane/missions/`); WildNav/DemNav corrections only after
   the navigation outputs agree with GPS in flight.

## Useful references

- Printed parts, printing and assembly: hardware/parts/airplane/README.md
- System, topics and SITL: systems/airplane/README.md,
  systems/airplane/config/topics.yaml, compositions/airplane_hardware.yaml,
  compositions/airplane_sim.yaml
- Skywalker X8: https://www.readymaderc.com/products/details/skywalker-model-x8-wing,
  https://uavgarage.com/shop/skywalker-x8-2122mm-uav-fixed-wing-drone/
- ArduPlane: https://ardupilot.org/plane/
- Blue Robotics Navigator: https://bluerobotics.com/store/comm-control-power/control/navigator/
  and https://bluerobotics.com/learn/navigator-hardware-setup/
- Blue Robotics Power Sense Module:
  https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
- Blue Robotics 5 V 6 A Power Supply:
  https://bluerobotics.com/store/comm-control-power/control/bec-5v6a-r1/
- Holybro MS4525DO airspeed sensor:
  https://holybro.com/products/digital-air-speed-sensor-ms4525do
- Holybro M10/M9N GPS: https://docs.holybro.com/gps-and-rtk-system/m8n-m9n-m10-gps
- Luxonis OAK-1: https://docs.luxonis.com/hardware/products/OAK-1
- BlueOS: https://blueos.cloud/
