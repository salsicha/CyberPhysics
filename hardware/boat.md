# Boat bill of materials

Goal: a surface vessel that runs the stack of
`compositions/boat_hardware.yaml` (`systems/boat/`): ArduRover on a Blue
Robotics Navigator with a Raspberry Pi 5 and BlueOS, MAVROS and the
`applications/boat` bridge publishing `/boat0/sensor_measurements/*` and,
once enabled, following `/boat0/cmd_vel` (the harbor survey in
`systems/boat/config/harbor_survey.yaml`).

The vessel is a purchased **Blue Robotics BlueBoat**: hulls, hatches,
motors and electronics come as a sealed, tested system, and the hull is not
printed (see "Why these parts are bought"). The printed parts are top-side
accessories: an accessory deck on a crossbar and a camera/light mast.
Printing and assembly instructions:
[hardware/parts/boat/README.md](parts/boat/README.md).

- BlueBoat: 1200 x 930 x 460 mm deployed, 14.5 kg without batteries,
  15 kg of batteries and payload, 320 mm maximum draft, two M200 motors
  (8.2 kgf static thrust), 3 m/s, 4S-6S Li-ion in the hulls.
- Electronics in the hatch lids: Navigator, Raspberry Pi 4 (2 GB, replaced
  here by a Pi 5), mRobotics M9N GPS with compass, Wi-Fi router, two ESCs,
  fuse board with payload power (battery voltage and 5 V).

## Compatibility notes

- **Already in the BlueBoat.** The base package includes the Navigator,
  GPS/compass, router, ESCs and motors: the previous list bought a
  Navigator and a GPS on top of it. Buy them only as spares.
- **Raspberry Pi 5.** The BlueBoat ships with a Raspberry Pi 4 (2 GB);
  the CyberPhysics stack (MAVROS, the bridge and rosbridge in Docker beside
  BlueOS) wants a Pi 5 with 8 GB. The Pi 5 fits the same 58 x 49 mm holes
  under the Navigator. Check the Navigator's revision: the August 2025
  revision moved its magnetometer for Pi 5 compatibility; with an older
  board use the GPS puck's compass as the primary one. Run BlueOS 1.4 or
  newer. Keep BlueOS's kernel: after an OS upgrade to Raspberry Pi OS's
  6.12 kernel ArduPilot (4.5.3 to 4.7.1) does not start on the Pi 5
  (`applications/navigator/README.md`). No Active Cooler (it does not fit
  under the Navigator's heatsink, which is the Pi's cooler), no NVMe HAT.
- **Pi 5 USB current.** Powered through the Navigator's header, the Pi
  limits its USB ports to 600 mA unless `usb_max_current_enable=1` is set
  in `/boot/firmware/config.txt`: set it before plugging in a USB camera.
- **Parameters.** `systems/boat/config/ardurover.params` is the hardware
  baseline (pre-arm checks, RC and GCS failsafes, `WP_SPEED 1.2`), loaded
  on top of the BlueBoat's own BlueOS defaults (skid steering, motor
  outputs); `ardurover_sitl.params` is SITL only.
- **Velocity setpoints** stay off (`enable_velocity_setpoints: false` in
  `systems/boat/config/bridge.yaml`) until RC override and failsafes are
  tested on the water.
- **Range.** The standard Wi-Fi link reaches about 250 m; the BaseStation
  with a directional antenna more than 800 m (Blue Robotics). The harbor
  survey stays within 50 m of the launch point.

## Purchased parts

### Vessel

- 1 x **Blue Robotics BlueBoat**, base package (hulls, frame and crossbars,
  M200 motors with weedless propellers, hatch lids with the electronics
  installed, battery cables, tools, spare fuses). The ready-to-run package
  adds batteries, charger and BaseStation.
- 1 x spare **weedless propeller set** (left- and right-hand) for the M200
  motors.

### Compute

- 1 x **Raspberry Pi 5**, 8 GB, replacing the BlueBoat's Pi 4 under its
  Navigator (keep the Pi 4 as a spare).
- 1 x microSD with BlueOS 1.4 or newer for the Pi 5 (or re-flash the
  BlueBoat's card); ArduRover through BlueOS.

### Power

- 2-4 x **Blue Robotics 14.8 V 18 Ah Li-ion battery** (4S, XT90); the
  BlueBoat takes up to 8 in its hulls, each on a 60 A fuse.
- 1 x **charger** for them, and a fire-safe bag for transport.

### Communications and control

- 1 x **BlueBoat BaseStation** (or an equivalent 2.4 GHz long-range
  access point) on the shore.
- 1 x laptop with Cockpit or QGroundControl, Docker and the ROS 2 tools.
- 1 x **gamepad** for manual control and recovery.

### Optional payloads

- **Ping2 echosounder** with Blue Robotics' **Ping Integration Kit for
  BlueBoat** (anodized aluminium weedless mount on the starboard hull's
  inner face; cable through the rear fairing; Navigator **SERIAL 3**) if
  bathymetry is a first mission.
- **Ethernet switch** for Ethernet sonars or several network payloads.
- **Camera or all-round light** with a 1/4"-20 tripod thread on the
  printed mast (USB camera through a starboard-lid M10 penetrator, or a
  self-contained action camera); nothing in the CyberPhysics stack reads a
  boat camera yet.

### Field kit

- Dry bag, hand tools (3, 4, 5 mm hex keys, 10 mm wrench), dielectric
  grease, medium thread locker, spare 316 fasteners, tape, multimeter,
  towels, fresh water for rinsing after salt water.

### Why these parts are bought, not printed

| Part | Why it stays purchased |
|---|---|
| Hulls | Each hull is a 1.2 m seamless LDPE moulding that flexes instead of cracking when it hits a dock or a rock, and carries the Li-ion batteries. On a 250 mm bed one printed hull is at least five sections and four joints below the waterline; FDM walls and bonded joints seep unless coated and tested, printed PETG/ASA cracks along layers in an impact, and a leak puts sea water on the batteries. Blue Robotics sells the hulls with hatches and frames without electronics for under 2000 USD (their staff, on the forum), which a printed hull would not beat on cost either. |
| Hatch lids, latches and O-rings | They are the seal over the electronics (neoprene O-ring, glass-filled polycarbonate lid): nothing printed may be part of it. |
| Motor mounts | Glass-filled polycarbonate parts that carry 8.2 kgf of thrust under water and take the hits from weeds and debris. |
| Electronics trays | Blue Robotics' ABS trays come installed in the hatch lids with the electronics; printing copies saves nothing. |
| Crossbars and brackets | 6061-T6 aluminium and glass-filled PC: they carry the hulls and the folding joint. |
| Ping2 mount (optional) | Below the waterline, where it meets weeds, logs and the bottom when beaching: the aluminium weedless mount is the safe choice. |
| Penetrators, motors, ESCs, batteries, electronics | Seals, electromechanics, energy storage. |
| Screws and nuts | 316 stainless in salt water; printed threads strip and creep. |

### Power and wiring

The BlueBoat's fuse board feeds the Navigator, Pi, router and ESCs. The
Pi 5 draws up to about 1 A more at 5 V than the Pi 4 under load
(estimate): after the swap check the 5 V rail at the Navigator
(5.0-5.35 V) and `vcgencmd get_throttled` = 0x0 with the whole stack
running. A USB camera on the mast draws its power
from the Pi's USB (set `usb_max_current_enable=1`); a light or a camera
with its own supply uses the tray's payload power (battery voltage or
5 V). Cables leave the hatch through the lid's M10 penetrator holes
(starboard: 2 x M10; port: 4 x M10 and 2 x M14), never through a gap in
the seal. Details: [the printed-parts README, "Wiring"](parts/boat/README.md#wiring).

## 3D-printed parts

Designed in CadQuery (`hardware/parts/boat/`), validated and exported by
`tools/build_printable_parts.py`; STLs in
[`parts/boat/stl/`](parts/boat/stl/). Printer: Bambu Lab P1S. Plates,
settings, post-processing and assembly:
[parts/boat/README.md](parts/boat/README.md). Times and masses are
estimates.

<!-- BEGIN GENERATED: printed-parts (tools/build_printable_parts.py) -->
| Part | Qty | Material | Bambu Studio settings (P1S, 0.4 mm) | Est. time (each) | Est. mass (each) | File |
|---|---|---|---|---|---|---|
| Camera mast | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports, brim outer brim only, 5 mm | 3 h 07 min | 89 g | [camera_mast.stl](parts/boat/stl/camera_mast.stl) |
| Crossbar plate (accessory deck) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 3 h 25 min | 117 g | [crossbar_plate.stl](parts/boat/stl/crossbar_plate.stl) |
| **Total, all pieces** | | | | **6 h 32 min** | **206 g** | |
<!-- END GENERATED: printed-parts -->

### Fasteners

Totals for one boat, from the parts' metadata. All exposed hardware is
A4 (316) stainless. Buy 10-20 % spares.

<!-- BEGIN GENERATED: fasteners (tools/build_printable_parts.py) -->
| Item | Qty | Where |
|---|---|---|
| 1/4"-20 UNC hex nut, stainless | 1 | captive in the mast head (camera_mast, 1) |
| 1/4"-20 UNC x 1" stud (set screw), stainless | 1 | mast head to the ball head (thread locker on the nut side) (camera_mast, 1) |
| M4 x 12 socket head cap screw, A4 (316) stainless | 4 | camera mast to the crossbar plate inserts (camera_mast, 4) |
| M4 x 8.1 mm brass heat-set insert (CNC Kitchen type, 5.7 mm hole) | 10 | top face, 50 mm accessory grid (crossbar_plate, 10) |
| M6 nyloc nut, A4 (316) stainless | 4 | under the crossbar (crossbar_plate, 4) |
| M6 washer, A4 (316) stainless | 8 | under the crossbar plate screw heads and nuts (crossbar_plate, 8) |
| M6 x 60 socket head cap screw, A4 (316) stainless | 4 | crossbar plate through the crossbar (crossbar_plate, 4) |
| mini ball head with a 1/4"-20 female base | 1 | camera or light on the mast (camera_mast, 1) |
<!-- END GENERATED: fasteners -->

### Assembly notes

- Build order and checks: [parts/boat/README.md](parts/boat/README.md#steps).
- The crossbar's height is not in Blue Robotics' documents: measure it
  before buying the crossbar plate's screws (bar height + 20 mm).
- Everything on the plate adds windage and raises the centre of gravity:
  keep mast payloads under about 0.5 kg.

### Simulator geometry

`compositions/boat_sim.yaml` runs ArduRover SITL's generic `rover` frame
with the hardware baseline parameters layered under the SITL relaxations;
the BlueBoat's own skid-steering setup and its 3 m/s top speed are not
modelled, and the survey runs at `WP_SPEED 1.2` and the waypoint follower's
1.1 m/s.

## Recommended assembly order

1. Assemble the BlueBoat as in Blue Robotics' BlueBoat Assembly Guide, and
   test it once with its own Pi 4 and BlueOS.
2. Swap in the Raspberry Pi 5 under the Navigator (Navigator Hardware
   Setup), flash BlueOS 1.4 or newer, restore the BlueBoat's parameters,
   then load `systems/boat/config/ardurover.params`.
3. Print and fit the crossbar plate and mast (parts/boat/README.md).
4. Bench: start `compositions/boat_hardware.yaml` on the Pi and check the
   `/boat0/sensor_measurements/*` topics; failsafes with the props in air.
5. On the water: manual driving, RC/GCS failsafe and return, then the
   survey in AUTO; enable velocity setpoints last.

## Useful references

- Printed parts, printing and assembly: hardware/parts/boat/README.md
- System and topics: systems/boat/README.md, systems/boat/config/topics.yaml
- BlueBoat: https://bluerobotics.com/store/boat/blueboat/blueboat/
- BlueBoat General Integration Guide:
  https://bluerobotics.com/learn/blueboat-general-integration-guide/
- BlueBoat Crossbars and Brackets:
  https://bluerobotics.com/store/boat/blueboat-components-spares/blueboat-frame-and-crossbars/
- BlueBoat Assembly Guide: https://bluerobotics.com/learn/blueboat-assembly/
- Ping Integration Kit for BlueBoat:
  https://bluerobotics.com/learn/ping2-integration-kit-for-blueboat-installation-guide/
- Blue Robotics forum, hull-only BlueBoat with printed parts:
  https://discuss.bluerobotics.com/t/blueboat-hull-3d-printed-parts/21060
- Navigator: https://bluerobotics.com/store/comm-control-power/control/navigator/
- BlueOS: https://blueos.cloud/docs/latest/usage/overview/
- ArduRover: https://ardupilot.org/rover/
