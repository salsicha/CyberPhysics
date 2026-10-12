# Submarine bill of materials

Goal: an ROV that runs the stack of `compositions/submarine_hardware.yaml`
(`systems/submarine/`): ArduSub on a Blue Robotics Navigator with a
Raspberry Pi 5 and BlueOS, MAVROS and the `applications/submarine` bridge
publishing `/sub0/sensor_measurements/{gps,odom,imu,pressure,battery}` and,
once enabled, following `/sub0/cmd_vel` (the pier-side pipeline inspection
at 1.5-4.5 m in `systems/submarine/config/pipeline_inspection.yaml`).

The vehicle is a purchased **Blue Robotics BlueROV2** in its standard
six-thruster configuration. Nothing printed is part of a pressure housing
or a seal: enclosures, end caps, domes, penetrators and O-rings are Blue
Robotics parts. The printed parts are a collar that holds the Doppler
velocity log under the frame, two skids that protect it, trim weight
holders and a topside tether spool. Printing and assembly instructions:
[hardware/parts/submarine/README.md](parts/submarine/README.md).

- BlueROV2: 457 x 338 x 254 mm, 11-12 kg in air with ballast and battery,
  6 x T200 (4 vectored, 2 vertical), 4 in electronics and 3 in battery
  enclosures, Raspberry Pi 4 (2 GB, replaced here by a Pi 5) + Navigator,
  Bar30 depth sensor, leak probes, low-light HD USB camera (110 deg) on a
  tilt, 1.2-1.4 kg payload.
- **Depth rating: 100 m** with the acrylic enclosures this list specifies
  (Blue Robotics' rating for the BlueROV2; aluminium enclosures raise it to
  300 m). Every other wet part listed is rated deeper: T200 300 m (tested),
  Bar30 0-296 m, DVL A50 300 m, Lumen 500 m. The printed parts are solid
  (100 % infill) and carry no pressure. The mission runs at 1.5-4.5 m.

## Compatibility notes

- **Already in the BlueROV2.** The kit includes the Navigator, Bar30, leak
  detection, camera and tilt, thrusters, ESCs and the tether interface:
  the previous list bought a Navigator, a Bar30 and a leak sensor on top
  of it. Buy them only as spares.
- **Standard frame, not Heavy.** `compositions/submarine_sim.yaml` runs
  ArduSub SITL with `ARDUPILOT_FRAME: vectored`, the standard six-thruster
  BlueROV2. The Heavy Configuration (eight thrusters, `vectored_6dof`) is
  an option only together with that SITL frame change.
- **Raspberry Pi 5.** The BlueROV2 ships with a Raspberry Pi 4 (2 GB); the
  CyberPhysics stack in Docker beside BlueOS is specified for a Pi 5
  (8 GB). It takes the same 58 x 49 mm holes on Blue Robotics' electronics
  tray. Check the Navigator's revision (the August 2025 one is the Pi 5
  revision) and run BlueOS 1.4 or newer. Keep BlueOS's kernel: after an OS
  upgrade to Raspberry Pi OS's 6.12 kernel ArduPilot (4.5.3 to 4.7.1) does
  not start on the Pi 5 (`applications/navigator/README.md`). No Active
  Cooler (it does not fit under the Navigator's heatsink) and no NVMe HAT:
  inside the sealed 4 in tube the heatsink is the cooler; watch `vcgencmd
  get_throttled` on the bench with the tube closed.
- **Underwater position (gap in the previous list).** Below the surface
  there is no GPS: without a positioning sensor ArduSub holds depth and
  heading but not position, `/sub0/sensor_measurements/odom` has no
  usable horizontal position, and the pipeline mission (waypoints with a
  0.9 m acceptance radius, velocity setpoints in GUIDED) cannot run. A
  **Water Linked DVL A50** gives ArduSub velocity and dead-reckoned
  position through Water Linked's BlueOS extension. `/sub0/sensor_measurements/gps`
  stays empty under water unless an acoustic positioning system (USBL,
  for example Water Linked's Underwater GPS) is added; at the surface the
  ROV has none either, so the GPS topic is only for a USBL.
- **Velocity setpoints** stay off (`enable_velocity_setpoints: false` in
  `systems/submarine/config/bridge.yaml`) until leak detection, depth
  failsafes and manual override are tested in a controlled tank; on loss
  of the ground station the ROV disarms (`FS_GCS_ENABLE 2` in
  `systems/submarine/config/ardusub.params`).

## Purchased parts

### Vehicle

- 1 x **Blue Robotics BlueROV2**, standard configuration with **acrylic**
  4 in electronics and 3 in battery enclosures (100 m), 2 or 4 Lumen
  lights (2 is enough at 1.5-4.5 m in clear water; 4 in turbid harbours).
- 1 x spare **T200 thruster** and a spare propeller set (clockwise and
  counter-clockwise).
- 1 x enclosure service kit: spare O-rings for the flanges and end caps,
  silicone grease, penetrator blanks, the vacuum test pump (in the kit).

### Compute

- 1 x **Raspberry Pi 5**, 8 GB, replacing the BlueROV2's Pi 4 under its
  Navigator; microSD with BlueOS 1.4 or newer; ArduSub through BlueOS.

### Navigation payload

- 1 x **Water Linked DVL A50** (300 m), Blue Robotics' BlueROV2 version:
  66 mm diameter, 25 mm tall, 170 g in air and 105 g in water, 10-30 V,
  4 W, Ethernet; it needs:
  - 1 x **Blue Robotics Ethernet Switch** in the electronics enclosure
    (stacks on the tether interface board);
  - 1 x **WetLink penetrator M10-7.5mm-HC** for its cable in a free
    end-cap hole (fit and vacuum-test it like every penetrator);
  - power from the battery bus (spade terminals), and the **Water Linked
    DVL BlueOS extension**.
  The printed collar (below) replaces the optional mounting bracket.
- Optional: an acoustic positioning system (USBL) for absolute position
  and the GPS topic; a Ping360 or Ping2 for inspection or altitude (the
  DVL already measures altitude from 5 cm to 50 m).

### Tether and topside

- 1 x **Fathom tether**, 25-50 m for the pier-side mission (standard
  7.6 mm, or slim 4 mm), and the **Fathom-X tether interface (FXTI)**
  topside.
- The **printed tether spool** holds about 57 m of standard or 200 m of
  slim tether (assembly check); for longer tethers buy Blue Robotics'
  spool with a slip ring.
- 1 x laptop with Cockpit or QGroundControl, Docker and the ROS 2 tools;
  1 x **gamepad**.

### Power

- 2 x **Blue Robotics 14.8 V 18 Ah Li-ion battery** (XT90; about 2 h each)
  and its charger.

### Field kit

- Dry bag, hand tools, 316 spare fasteners and M16 trim washers,
  silicone grease, dielectric grease, cable ties, labels, multimeter,
  towels, fresh water for rinsing.

### Why these parts are bought, not printed

| Part | Why it stays purchased |
|---|---|
| Enclosure tubes, end caps, domes, flanges, O-rings, penetrators, vents | The pressure boundary and its seals: at 100 m every square centimetre carries about 10 kgf. Printed parts are porous between layers and creep under load; nothing printed may be part of it. |
| Frame (HDPE panels, enclosure cradles and clamps) | HDPE is tough (it dents instead of cracking when the ROV is dropped on a deck), floats slightly (0.95 g/cm3), and comes cut in the kit with every thruster and enclosure hole. Printed panels would be bed-size pieces bolted together, heavier in water (ASA and PETG sink) and weaker at the joints. |
| Thrusters, ESCs, lights, camera, sensors, DVL | Sealed electromechanics and electronics. |
| Buoyancy foam | R-3318 foam holds its volume under pressure; a printed float traps air that floods or collapses with depth. |
| Ballast weights | Stainless steel: about seven times denser than any printable plastic. |
| Tether, battery | Cable and energy storage. |
| Screws, nuts, hose clamp | 316 stainless; brass heat-set inserts and printed threads do not belong under salt water. |

### Printed parts in water

- **Solid prints.** Every part that goes under water is printed with 100 %
  infill: sparse infill traps air that floods slowly (the trim drifts
  through a dive) or collapses at depth.
- **Material and buoyancy.** ASA (1.05-1.07 g/cm3) is close to sea water
  (1.025): the printed collar, skids and trim holders weigh about 15 g in
  sea water, against about 80 g if printed in PETG (1.25-1.27 g/cm3); the
  assembly check computes both. The DVL (105 g in water) and the stainless
  screws add about 200 g: remove one or two of the BlueROV2's 200 g
  ballast weights and fine-tune with M16 washers on the printed holders.
- **Water uptake.** Bambu lists 0.45 % saturated moisture uptake for ASA
  and PETG Basic (25 C, 55 % RH); soaked parts take up more, of the order
  of a few grams for these parts: retrim after the first long dive.
- **Hardware.** 316 (A4) screws, nuts and hose clamp only under water;
  rinse in fresh water after every salt-water dive.

### Power and wiring

The BlueROV2's power and signal wiring stays as built; the additions are
the DVL (battery voltage through the fused bus, Ethernet through the
switch) and the Pi 5 swap. 5 V on the Navigator must stay at
5.0-5.35 V with the Pi 5 under load. Details:
[the printed-parts README, "Wiring"](parts/submarine/README.md#wiring).

## 3D-printed parts

Designed in CadQuery (`hardware/parts/submarine/`), validated and exported
by `tools/build_printable_parts.py`; STLs in
[`parts/submarine/stl/`](parts/submarine/stl/). Printer: Bambu Lab P1S.
Plates, settings, post-processing and assembly:
[parts/submarine/README.md](parts/submarine/README.md). Times and masses
are estimates.

<!-- BEGIN GENERATED: printed-parts (tools/build_printable_parts.py) -->
| Part | Qty | Material | Bambu Studio settings (P1S, 0.4 mm) | Est. time (each) | Est. mass (each) | File |
|---|---|---|---|---|---|---|
| DVL mount (Water Linked A50 collar) (fit test first) | 1 | ASA | 0.20 mm, 5 walls, 100% gyroid, no supports | 2 h 07 min | 71 g | [dvl_mount.stl](parts/submarine/stl/dvl_mount.stl) |
| Skid (DVL guard) (fit test first) | 2 | ASA | 0.20 mm, 5 walls, 100% gyroid, no supports | 3 h 37 min | 124 g | [skid.stl](parts/submarine/stl/skid.stl) |
| Tether spool drum | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports | 5 h 16 min | 172 g | [spool_drum.stl](parts/submarine/stl/spool_drum.stl) |
| Tether spool flange | 2 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports | 3 h 45 min | 129 g | [spool_flange.stl](parts/submarine/stl/spool_flange.stl) |
| Tether spool crank handle | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports | 34 min | 15 g | [spool_handle.stl](parts/submarine/stl/spool_handle.stl) |
| Trim weight holder | 4 | ASA | 0.20 mm, 5 walls, 100% gyroid, no supports | 22 min | 9.7 g | [trim_weight_holder.stl](parts/submarine/stl/trim_weight_holder.stl) |
| **Total, all pieces** | | | | **24 h 11 min** | **802 g** | |
<!-- END GENERATED: printed-parts -->

### Fasteners

Totals for one ROV, from the parts' metadata. Everything that goes under
water is A4 (316) stainless; the brass inserts are in the topside spool
only. Buy 10-20 % spares.

<!-- BEGIN GENERATED: fasteners (tools/build_printable_parts.py) -->
| Item | Qty | Where |
|---|---|---|
| M16 washer (ISO 7089), A4 (316) stainless | 20 | trim weight, as needed (0-9 per holder; buy 20 to start) (trim_weight_holder, 20) |
| M4 x 10 socket head cap screw, A4 (316) stainless | 12 | spool flange to the drum inserts (spool_flange, 12) |
| M4 x 8.1 mm brass heat-set insert (CNC Kitchen type, 5.7 mm hole) | 12 | both end faces, six each (spool_drum, 12) |
| M5 nyloc nut, A4 (316) stainless | 15 | DVL mount, on top of the bottom panel (dvl_mount, 4); skid, on top of the bottom panel (skid, 6); crank handle, in the hex pocket on the flange's inner face (spool_handle, 1); trim weight holder, on top of the bottom panel (trim_weight_holder, 4) |
| M5 washer, A4 (316) stainless | 26 | under the DVL mount screw heads and nuts (dvl_mount, 8); under the skid screw heads and nuts (skid, 12); crank handle: under the head, and between handle and boss (spool_handle, 2); trim weight holder nut, on top of the bottom panel (trim_weight_holder, 4) |
| M5 x 20 mm fender (penny) washer, A4 (316) stainless | 4 | retains the trim washers (trim_weight_holder, 4) |
| M5 x 30 socket head cap screw, A4 (316) stainless | 4 | DVL mount through the bottom panel (dvl_mount, 4) |
| M5 x 35 socket head cap screw, A4 (316) stainless | 6 | skid through the bottom panel (heads recessed) (skid, 6) |
| M5 x 55 socket head cap screw, A4 (316) stainless | 4 | trim weight holder and its washers through the bottom panel (trim_weight_holder, 4) |
| M5 x 70 socket head cap screw, A4 (316) stainless | 1 | crank handle axle, through the flange boss (spool_handle, 1) |
| hose clamp, 9 mm band, 60-80 mm range, W5 (316) stainless | 1 | DVL collar (dvl_mount, 1) |
<!-- END GENERATED: fasteners -->

### Assembly notes

- Build order and checks:
  [parts/submarine/README.md](parts/submarine/README.md#steps).
- The bottom panel's outline and the ballast weights under it are not
  dimensioned by Blue Robotics: lay the printed parts on the panel and
  check the clear area before drilling.
- With the skids fitted the DVL sits 9 mm above the bottom: it cannot lock
  while the ROV rests on the seabed (it needs 5 cm).

### Simulator geometry

`compositions/submarine_sim.yaml` runs ArduSub SITL's `vectored` frame (the
standard BlueROV2) and a bridge with velocity setpoints enabled; SITL has
perfect position, which the real ROV gets only from the DVL.

## Recommended assembly order

1. Build or unpack the BlueROV2 as in Blue Robotics' assembly guide, and
   vacuum-test the enclosures before any dive.
2. Swap in the Pi 5, flash BlueOS 1.4 or newer, restore ArduSub's
   parameters, load `systems/submarine/config/ardusub.params`.
3. Fit the DVL: Ethernet switch, M10 penetrator (vacuum test again),
   printed collar and skids, trim (parts/submarine/README.md).
4. Bench: start `compositions/submarine_hardware.yaml`; check
   `/sub0/sensor_measurements/*` and the DVL extension's status.
5. Tank: leak test, trim and buoyancy (just positive), manual flying,
   depth hold, DVL position hold; then the pipeline mission with velocity
   setpoints enabled.

## Useful references

- Printed parts, printing and assembly: hardware/parts/submarine/README.md
- System and topics: systems/submarine/README.md,
  systems/submarine/config/topics.yaml
- BlueROV2: https://bluerobotics.com/store/rov/bluerov2/
- BlueROV2 frame: https://bluerobotics.com/store/rov/bluerov2-components-spares/brov2-asm-frame/
- BlueROV2 assembly: https://bluerobotics.com/learn/bluerov2-assembly/
- Water Linked DVL A50 datasheet: https://www.waterlinked.com/datasheets/dvl-a50
  and https://docs.waterlinked.com/dvl/dvl-a50/
- Water Linked DVL for the BlueROV2: https://bluerobotics.com/store/the-reef/dvl-a50/
- Ethernet Switch: https://bluerobotics.com/store/comm-control-power/tether-interface/ethswitch/
- T200: https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/
- Bar30: https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1/
- Lumen: https://bluerobotics.com/store/thrusters/lights/lumen-r2-rp/
- Bambu ASA and PETG Basic data sheets (density, water uptake):
  https://store.bblcdn.com/s1/default/cb94589bf7994fdcbfa833badefae9cd/Bambu_PETG_Basic_Technical_Data_Sheet.pdf
- ArduSub: https://www.ardusub.com/
