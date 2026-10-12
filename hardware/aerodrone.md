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
- Propulsion: T-Motor MN3510 KV700 motors, T-Motor P12*4 (12 x 4 in)
  propellers, T-Motor AIR 40A ESCs (PWM), 4S 5000 mAh LiPo; one set for both
  configurations (see ["Propulsion: why 12 in on 4S"](#propulsion-why-12-in-on-4s)).
- Estimated take-off mass about 2.5 kg with the pack and the HaLow link:
  thrust-to-weight 2.5, about 12.8 min of hover (computed by the assembly
  check, see the printed-parts README).
- Ground link: an IP link over Wi-Fi HaLow (802.11ah, sub-GHz) to the ground
  station carries MAVLink, BlueOS and the tracking page; the 2.4 GHz RC link
  stays the independent manual-control and failsafe link. See
  ["Radio"](#radio); the ground side is in
  [hardware/ground_station.md](ground_station.md).
- Optional payload: the turret's foam dart launcher, pointing straight down
  under the hub, its bore on the vertical axis through the centre of mass;
  the battery then rides on the top deck above the stack and taller legs
  carry the aircraft. About 3.2 kg, thrust-to-weight 2.0 at the pack's
  nominal 14.8 V (2.4 at 16 V), about 9 min: see
  ["Foam dart launcher (optional payload)"](#foam-dart-launcher-optional-payload).

## Compatibility notes

- **Raspberry Pi 5 + Navigator.** Blue Robotics' store page still lists
  the Navigator for the Raspberry Pi 4, but its August 2025 hardware
  revision moved the magnetometer "to improve compatibility with Raspberry
  Pi 5", BlueOS 1.4 added Pi 5 support, and users report the pair working
  with BlueOS 1.4.3. Buy the current Navigator revision and run BlueOS 1.4
  or newer. If in doubt, a Raspberry Pi 4 fits the same printed tray (same
  58 x 49 mm holes). Keep BlueOS's kernel: after an OS upgrade to
  Raspberry Pi OS's 6.12 kernel ArduPilot (4.5.3 to 4.7.1) does not start
  on the Pi 5 (`applications/navigator/README.md`).
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
  210 mm mast at the rear, away from the camera cable; if satellite counts
  drop when the OAK-1 streams, reroute the cable or force the camera to
  USB 2.

## Purchased parts

### Airframe and propulsion

- 4 x carbon fibre tube, **16 mm OD** (16 x 14 mm, roll-wrapped), cut to
  **180.0 mm** (buy 2 x 400 mm or 1 x 1 m; spares for crashes).
- 4 x **T-Motor MN3510 KV700** (Navigator series, 3-4S; 41.8 x 28.5 mm,
  97 g without its cables, 118 g with its 600 mm leads; four M3 on a 25 mm
  circle; max. 25 A for 180 s), plus 1 spare (about USD 80 each,
  https://store.tmotor.com/product/mn3510-motor-navigator-type.html). Cut
  the leads to about 200 mm. The printed mount's slots take a 19-25 mm
  circle; T-Motor's two M3 x 12 in the box are for the propeller, so buy the
  motor screws (the fastener table).
- 4 x **T-Motor AIR 40A 2-6S, no BEC** (about USD 40,
  https://store.tmotor.com/product/air-40a-6s-esc.html), plus 1 spare: the
  ESC T-Motor pairs with this motor class (its page names the U3 KV700,
  which has the MN3510 KV700's test table), 40 A, standard PWM input (signal
  up to 621 Hz; the Navigator's default 490 Hz is inside it), no BEC. The
  Navigator drives standard PWM only, so no DShot-only ESC. Resellers give
  26 g and 55.6 x 25.2 x 11.3 mm (or 68 x 25 x 8.7 mm); its throttle range is
  not on T-Motor's page: check it on the bench (ArduPilot's ESC calibration
  or `MOT_PWM_MIN`/`MOT_PWM_MAX` from the ESC's manual).
- **Propellers T-Motor P12*4** (12 x 4 in polished carbon, 14.5 g, rated
  4 kg thrust, about USD 42 a pair,
  https://store.tmotor.com/product/polish-carbon-fiber-12x4-prop.html): at
  least three pairs of each hand (CW + CCW), on the motor's 12 mm prop face
  with T-Motor's two M3 x 12 and the cover plate. The motor's test table
  names the propeller "T-MOTOR 12*4CF".
- 1 x power distribution board or harness rated for **90 A** bursts: four
  ESC outputs and one each for the 5 V supply, the HaLowLink's regulator
  and the launcher's 12 V regulator; **XT90** battery connector and 10 AWG
  silicone from the XT90 to the board (see "Power").
- No prop guards: the aircraft flies outdoors at 15-120 m, full guards for
  12 in props add a few hundred grams, cost thrust and (over 310 mm across)
  do not fit the 250 mm print bed in one piece. Do first hovers tethered or
  over a net instead.

### Propulsion: why 12 in on 4S

With the foam dart launcher (2.98 kg) the first propulsion, Holybro's
2216 KV920 with 10 x 4.5 in propellers (the X500 V2's), gave a
thrust-to-weight of 1.79 by T-Motor's AIR2216 II KV920 + T1045 table at
16 V and 1.61 at the T1045's 1.2 kg rating. The target: at least 2:1 with a
full clip, from one set that also flies the aircraft without the launcher,
and a reasonable flight time. The candidates, from the makers' test tables
at the voltage each states, in the launcher configuration with a full clip
(the chosen set's mass from the assembly check, the others' from it and
their part masses):

| Motor + propeller (table voltage) | Max. thrust per motor | Launcher config. mass | T/W at the table's voltage | T/W at 3.7 V/cell | Hover power | Flight time | Tip gap | Verdict |
|---|---|---|---|---|---|---|---|---|
| Old: AIR2216 II KV920 + T1045 10 in, 4S (16 V) | 1332 g; prop rated 1.2 kg | 2.98 kg | 1.79 (1.61 at the prop's rating) | 1.53 | 464 W | 7.7 min | 100 mm | under 2:1 |
| **T-Motor MN3510 KV700 + 12*4CF, 4S (14.8 V)** | 1600 g; P12*4 rated 4 kg | 3.17 kg | **2.02** | 2.02 | 391 W | 9.1 min | 49 mm | **chosen** |
| T-Motor MN3110 KV700 + 12*4CF, 4S (14.8 V) | 1290 g | 3.10 kg | 1.67 | 1.67 | 375 W | 9.5 min | 49 mm | too little thrust |
| KDE2814XF-775 + KDE-CF125-DP 12.5 in, 4S (16.8 V) | 1909 g | 3.21 kg | 2.38 | 1.85 | 524 W | 6.8 min | 36 mm | 2:1 only on a full pack; propellers discontinued |
| KDE2814XF-515 + KDE-CF125-DP 12.5 in, 6S (25.2 V) | 2328 g | 3.31 kg (6S 4500 mAh) | 2.82 | 2.19 | 539 W | 8.9 min | 36 mm | propellers discontinued; heavier |
| T-Motor MN3510 KV360 + 13*4.4CF, 6S (22.2 V) | 1300 g | 3.28 kg (6S 4500 mAh) | 1.59 | 1.59 | 383 W | 12.5 min | 23 mm | too little thrust; 13 in too close |

T/W at 3.7 V per cell scales the table's maximum with the square of the
voltage (rpm follows the voltage, thrust the square of the rpm). Hover
power is the four motors at the table's voltage plus 16 W of avionics; the
flight time runs to 20 % left on the pack's nominal energy (4S 5000 mAh:
74 Wh; 6S 4500 mAh: 100 Wh). On that basis the old set's 8.3 min (from its
current at 16 V) is 7.7 min. KDE measured at 3500 ft; its sheets estimate
about 10 % more thrust at sea level (the 6S set: 2.40 at 22.2 V).

Why the MN3510 KV700 on 12 in, 4S:

- **Thrust margin**: 2.02 at the pack's nominal 14.8 V, the table's own
  voltage, with a full clip (2.36 at 16 V, the voltage of the old table;
  about 2.6 on a full pack, 1.8 at 3.5 V per cell). The propeller is no
  longer the limit: the P12*4 is rated 4 kg, 5.1 times the hover thrust.
  Without the launcher: 2.54.
- **Hover efficiency and flight time**: 12 in propellers at about 0.8 kg each
  hover at about 8.4 g/W against 6.7 g/W for the 10 in set, so the flight
  time goes up (9.1 min with the launcher, 12.8 without; it was 8.3 and 11.2)
  although the aircraft is 190 g heavier.
- **Mass**: +190 g (motors 104 g with shortened leads against 63 g, the
  propellers, the ESCs, XT90 and 10 AWG leads). The pack stays: a 6000 mAh
  4S pack (about 110 g more) would take the launcher configuration under
  2:1 (1.95).
- **6S was not better here**: the only 6S set with enough thrust in a
  maker's table, KDE's 2814XF-515, was tested with KDE's 12.5 in
  propellers, which KDE has archived ("no longer available for purchase");
  without them its table does not apply, and 12.5 in leaves 36 mm between
  tips. T-Motor's 6S motors for 12-13 in (MN3510 KV360) are efficiency
  motors with 1.3 kg. A 6S pack of the same energy class is heavier and
  taller (Tattu 4500 mAh 6S: 650 g, 138 x 42.5 x 52 mm, over the deck's
  40 mm pack height). Electrically 6S would have worked: the Power Sense
  Module takes 6S, the 5 V 6 A supply 7-26 V, the Pololu regulators up to
  50 V, the AIR 40A 2-6S. Staying on 4S keeps the packs, the charger and
  every regulator as they are.
- **Availability**: all three are in T-Motor's current catalogue (motor
  about USD 80, ESC 40, propellers 42 a pair); the U3 KV700 is the same
  motor's industrial version with the same table if the MN3510 is out of
  stock (128 g, a heavier choice).
- **Clearance on the 500 mm frame**: adjacent motors are 354 mm apart, so
  12 in propellers leave 49 mm between tips (13 in would leave 23 mm). The
  assembly check finds every disc at least 12 mm from any other body (the
  top deck's rib); the deck's rear corners reach 17 mm in over the rear
  discs' tips, 16 mm above them, the outer ends of its strap slots (and the
  straps under it) about 5 mm; with the launcher the pack's rear corners sit
  on the tips' circle (1 mm outside it at the balance point, about 7 mm in
  if trimmed fully aft), 20 mm above it. Keep the straps tight and their
  tails on top of the pack. Nothing else is over a disc; see the
  printed-parts README, "Checks".
- **Mount**: four M3 on a 25 mm circle; the printed `motor_mount`'s slots
  now run from a 19 to a 25 mm circle, turned 45 deg off the arm so no screw
  head lands under the clamp, on a 42 mm seat for the 41.8 mm motor (it was
  a 16-19 mm cross on a 34 mm seat).
- **Legs and ground clearance**: no change. The propellers turn 252 mm above
  the ground (403 mm on the tall legs); a tip would touch the ground only at
  49 deg of roll or pitch about two feet (76 deg on the tall legs), past
  the angle at which the aircraft tips over anyway (27 and 33 deg).
- **Electrics**: 4S stays, so the Power Sense Module, the 5 V 6 A supply
  and both Pololu regulators stay (D36V28F5 5.3-50 V in for the HaLowLink;
  D36V50F12 13.3-50 V in for the launcher's servos, whose 12 V sags a
  little near the end of the pack, as before). The higher current needs the
  XT90 and 10 AWG battery leads ("Power").

Sources (test tables at the voltage given): T-Motor MN3510
(https://store.tmotor.com/product/mn3510-motor-navigator-type.html: KV700
at 14.8 V with the 12*4CF, KV360 at 22.2 V with the 13*4.4CF), U3 KV700
(https://store.tmotor.com/product/u3-motor-u-power.html, the same table),
MN3110 (https://store.tmotor.com/product/mn3110-motor-navigator-type.html:
KV700 at 14.8 V with the 12*4CF), P12*4 and AIR 40A (links above); KDE
Direct 2814XF-775 and 2814XF-515
(https://www.kdedirect.com/products/kde2814xf-775,
https://www.kdedirect.com/products/kde2814xf-515; test sheets
https://cdn.shopify.com/s/files/1/0496/8205/files/KDE2814-775.pdf at 16.8 V
and https://cdn.shopify.com/s/files/1/0496/8205/files/KDE2814-515.pdf at
25.2 V) and KDE-CF125-DP (https://www.kdedirect.com/products/kde-cf125-dp,
archived); Tattu 4500 mAh 6S
(https://genstattu.com/tattu-4500mah-22-2v-25c-6s1p-lipo-battery-pack-with-xt60-plug.html);
the old set: the Air Gear 450 II datasheet (below, "Flight impact").

### Power

- 2-3 x **4S LiPo, 5000 mAh**, 25C or more, **XT90**, at most
  160 x 50 x 40 mm (the tray and straps take any pack up to that size);
  about 550 g. Not bigger: a 6000 mAh pack takes the launcher configuration
  under 2:1. Re-terminate packs that come with an XT60.
- 1 x LiPo balance charger, LiPo bag, LiPo voltage checker.
- 1 x **Blue Robotics Power Sense Module** (max. 25.2 V (6S), current
  sensing to 100 A non-continuous; 5.5 mm bullet input, #6 spade output,
  JST-GH cable to the Navigator **POWER** port;
  https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/).
  It senses only: it does not supply 5 V. Add an XT90 pigtail (10 AWG) for
  its input. The motors draw about 19 A in hover (26 A with the launcher)
  and 67 A at full throttle at 14.8 V (T-Motor's table), about 86 A from a
  full 16.8 V pack: inside its 100 A, briefly.
- 1 x **Blue Robotics 5 V 6 A Power Supply** (7-26 V in, spade input, 0.1 in
  header output; https://bluerobotics.com/store/comm-control-power/control/bec-5v6a-r1/)
  for the Navigator's main 5 V input; the Navigator powers the Pi. Blue
  Robotics asks for a supply of at least 4.5 A. 4S (13.2-16.8 V) is well
  inside its input range.
- Silicone wire, XT90 and 5.5 mm bullet connectors, heat-shrink tubing:
  **10 AWG** from the XT90 through the Power Sense Module to the power
  distribution, 14 AWG to each ESC, the motors' 18 AWG
  leads, 20 AWG for the regulators' taps, 20-22 AWG for signal leads.
  Holybro's connector and wire rating
  (https://docs.holybro.com/power-module-and-pdb/power-module/connector-and-wire-rating)
  gives an XT60 on 12 AWG 30 A continuous and 60 A for under a minute, an
  XT90 on 10 AWG 45 A and 90 A: the launcher configuration's 26 A hover is
  near the XT60's continuous rating and full throttle is over its burst
  rating, hence the XT90.
- Fuses: none on the main battery path (it feeds the motors; a blown fuse
  there is a crash); the small taps keep theirs, sized for 4S: 2 A for the
  HaLowLink's regulator (at most about 0.85 A in at 13.2 V) and 3 A slow for
  the launcher's 12 V regulator (2.4 A worst case at 13.2 V, below).

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
- **No SiK telemetry radio**: it would share the sub-GHz band with the
  HaLow link, and MAVLink now goes over HaLow.
- 1 x **Morse Micro HaLowLink 2** (MM8108 Wi-Fi HaLow, OpenWrt 23.05, 2 x
  Gigabit Ethernet, USB-C for power and data, SMA antenna connector;
  88 x 68 x 24 mm, 102 g without its stand, 5 V 2 A, 0-40 C; about USD 129,
  sold by GL.iNet, https://www.gl-inet.com/en-us/products/halowlink2;
  product brief:
  https://www.morsemicro.com/resources/product_brief/HaLowLink-2_Product-Brief.pdf).
  It stays in its case (the brief gives no bare-board option) and stands
  on its side in the printed `halow_cradle` on the `top_deck`, connector
  side up. Its region-specific dipole antenna (2.34 dBi in the user guide)
  goes on an **SMA extension** (RG316, about 15 cm, SMA plug to SMA bulkhead
  jack) to the cradle's stay at the front of the deck, where it stands
  **vertically**, about 130 mm from the GPS puck and clear of the arms,
  propellers and launcher (the assembly check measures it).
  - Mode: **Extender** (aqua status LED), paired with the ground station's
    HaLowLink 2 Access Point: the two make the user guide's "Virtual Wire"
    (HaLowLink user guide 2.11, section 6.3, "Using HaLow to Extend an
    Existing Network - Virtual Wire", which makes the HaLow link work "just
    like an Ethernet cable"; Morse Micro APPNOTE-37). The Extender is a
    Client (WDS) that bridges its Ethernet ports to the HaLow link. Its
    **LAN** port goes to the Pi 5's Ethernet.
  - Addressing, the shared plan of
    [hardware/ground_station.md](ground_station.md) (one bridged
    192.168.2.0/24, no gateways): nothing to change on the Pi. BlueOS keeps
    its defaults: the Pi's `eth0` is **192.168.2.2** (BlueOS's `cable_guy`
    default, `core/services/cable_guy/config.py`) and its default **"GCS
    Client Link"** endpoint sends MAVLink as a UDP client to
    **192.168.2.1:14550** (`core/services/ardupilot_manager/autopilot_manager.py`,
    https://github.com/bluerobotics/BlueOS), the ground laptop's address.
    This HaLowLink 2 takes the static **192.168.2.252** (set in its own Quick
    Config after pairing, so its web interface stays at
    `https://192.168.2.252` without the access point's DHCP), the ground
    access point is .250, the car's unit .251 and the car's Jetson .10.
    Turn this unit's own 2.4 GHz access point **off** (Quick Config,
    Wireless): on the aircraft it would share 2.4 GHz with the RC link.
    BlueOS, Cockpit or QGroundControl and the tracking page then all run
    over HaLow. The ground side (Access Point, antenna, laptop, pairing,
    range test) is in [hardware/ground_station.md](ground_station.md).
  - Power: its own regulator, not the Navigator's 5 V rail (below): a
    **Pololu D36V28F5** (5 V, 3.2 A, 5.3-50 V in,
    https://www.pololu.com/product/3782) on the power distribution through
    an inline **2 A** fuse, into the unit's USB-C through a **USB-C plug
    wired as a power source** (the unit asks for 5 V 2 A): VBUS and GND from
    the regulator's output on 20 AWG, about 15 cm, and the plug's CC pin
    pulled up to VBUS through a **10 kOhm** resistor, the USB Type-C
    "3.0 A" current advertisement (22 kOhm advertises 1.5 A, 56 kOhm only
    default USB power: USB Type-C Cable and Connector Specification, the Rp
    values for a 5 V pull-up), with no 5.1 kOhm pull-down on CC (a plug
    breakout made for a sink, such as Adafruit's USB Type C Plug Breakout,
    product 5978, carries one on CC1: cut it off). Heat-shrink the joints
    and tie the lead to the cradle. Or: a 5 V step-down module with a native
    USB-C output that advertises at least 2 A (3 A preferred) and takes the
    pack's 13-17 V, with a short USB-C to USB-C cable rated 3 A. Not a USB-A
    socket with a USB-A to USB-C cable or adapter: the HaLowLink user guide
    2.11's troubleshooting table (section 14, "The Status LED is not
    Illuminated") says "Do not use a USB-A to USB-C adapter", and such a
    cable advertises only default USB power.
  - Bandwidth: HaLow gives a few Mbit/s at range. MAVLink and the tracking
    page's JPEG (at most 640 px, about 2-5 Hz) fit; full-rate video does
    not.
  - Regulatory: 902-928 MHz in the US (FCC Part 15 device), 863-868 MHz in
    the EU (where Morse Micro limits its transmit power to 14 dBm). The
    region is locked at the factory: buy the version for the country of
    operation, the same region as the ground station's unit
    (hardware/ground_station.md), and use its antenna. Keep 20 cm between the antenna and people while it transmits
    (the user guide's RF exposure statement).
  - **ArduPilot's GCS failsafe now follows the HaLow link**: the ground
    station's MAVLink heartbeat arrives over HaLow, so a lost HaLow link
    trips the GCS failsafe where it is enabled (`FS_GCS_ENABLE`); the
    2.4 GHz RC link keeps manual control and its own RC failsafe.

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

### Foam dart launcher (optional payload)

**Toy power, foam darts only.** The launcher is the turret's spring-plunger
launcher for Elite-size foam darts (core `hardware/parts/turret/_launcher.py`;
power, safety and bring-up in [hardware/turret.md](turret.md), "Foam dart
launcher: power and safety"; contract in
`systems/racecar_turret/interfaces.md`, "Foam dart launcher"), mounted
pointing straight down under the hub. Print its parts from the turret's
STLs ([parts/turret/stl/](parts/turret/stl/), plates and settings in
[parts/turret/README.md](parts/turret/README.md)), every `launcher_*` part
except `launcher_breech`: this aircraft prints its own breech
(`launcher_breech_down`, without the turret's pedestal) and the parts that
hold the launcher (the generated table below: `launcher_hanger`,
`launcher_cam_guide`, `launcher_dart_detent`, `tall_leg_socket`,
`tall_leg_foot`; the battery goes on the `top_deck`).

Purchased, as the turret's bill of materials specifies them:

- 2 x **Waveshare ST3215** (12 V) serial bus servos: the priming actuator
  (bus ID 14) and the trigger (ID 15), output horns only.
- 1 x **K&S Precision Metals 8140** brass tube, 17/32 in OD x 0.014 in wall
  (ID 12.78 mm): the barrel, cut to 45 mm.
- 1 x **plunger spring**, music wire, **1.5 N/mm (+-10 %)**, free length
  80 mm, OD 28 mm, wire 2.0 mm; the clip spring and the sear spring;
  NBR 70 O-rings 27 x 2.5 and 10 x 2 mm; a 3 x 10 mm dowel pin; silicone
  grease. **No stiffer spring.** The launcher's screws and inserts are in
  the turret's fastener table (the `launcher_*` rows; the breech's are
  in this aircraft's table).
- **Elite-size foam darts only**; safety glasses for everyone nearby; a
  chronograph (10-50 m/s) for the bring-up.

Purchased for the aircraft side (servo drive and power, legs):

- 1 x **Waveshare Bus Servo Adapter (A)** (42 x 33 mm), jumper on **B**
  (USB), on the Raspberry Pi 5's free USB 2 port; in the hub between the
  plates, ahead of the power module.
- 1 x **Pololu D36V50F12** 12 V step-down regulator (4.5 A typical maximum,
  13.3-50 V in, reverse-voltage, over-current and thermal protection;
  https://www.pololu.com/product/4095); in the hub, foam tape and a zip tie.
- 1 x inline mini blade (ATM) fuse holder with a **3 A** fuse, at the
  regulator's tap on the power distribution; it is also the launcher's
  power disconnect.
- 20 AWG silicone wire for the 12 V rail, a 5.5 x 2.1 mm DC plug (or the
  adapter's screw terminal), a USB-C to USB-A cable about 0.3 m, a servo
  bus cable about 30 cm from the adapter to the actuator servo (the
  actuator-to-trigger cable about 25 cm, as on the turret).
- 2 x **carbon fibre tube 12 x 10 mm, 1 m**: the four tall legs, cut to
  386 mm each.
- The battery, its two straps and the anti-slip pad move from the tray to
  the deck.

**Servo drive: the adapter on USB.** The two ST3215s need a half-duplex
serial bus at 1 Mbps and 12 V. The fewest parts is the Waveshare Bus Servo
Adapter (A) on the Pi 5's USB (jumper on B) with the turret bridge's `sts`
transport (`applications/pantiltros/scripts/turret_arduino_bridge.py`,
`TURRET_ARDUINO_PORT=/dev/serial/by-id/...`, as on the RACECAR Turret): no
level shifter, no microcontroller, and the Navigator's serial ports stay
with ArduPilot. The alternative, the adapter's UART mode (jumper on A) on a
Pi UART, has no free header pins: the Navigator HAT takes the 40-pin header
and brings the Pi's UARTs out as its SERIAL 1 (`/dev/ttyS0`), SERIAL 3
(`/dev/ttyAMA1`), SERIAL 4 (`/dev/ttyAMA2`) and SERIAL 5 + I2C 6
(`/dev/ttyAMA3`) JST-GH ports (Blue Robotics, Navigator Hardware Setup).
SERIAL 5 carries the GPS; with the SiK radio gone SERIAL 1, 3 and 4 are
free (3.3 V, up to 3 MBd), but BlueOS assigns those ports to ArduPilot: using one means taking it out of BlueOS's serial configuration,
and the adapter's UART logic level is unverified (hardware/turret.md).
Keep it as a fallback.

**12 V servo rail**, from the flight battery through the fuse and the
regulator, not from the Navigator's 5 V rail:

| Load (12 V) | Typical | Peak | Source |
|---|---|---|---|
| Actuator ST3215 (ID 14), holding / priming | 0.1 A | about 1 A for under 1 s; 1.5 A stalled at its 55 % torque limit | hardware/turret.md |
| Trigger ST3215 (ID 15) | 0.05 A | 0.8 A stalled at its 30 % torque limit | hardware/turret.md |
| Bus Servo Adapter (A) | 0.02 A | 0.02 A | estimate |
| **Total** | **about 0.2 A (2 W)** | **about 2.3 A (28 W)** | |

The regulator's 4.5 A covers the 2.3 A worst case with the turret's torque
limits; it does not cover both servos stalled without them (2 x 2.7 A), so
the limits are required, and the regulator's current limit and the fuse
stop a stall. From the pack the worst case is about 2.1 A (28 W at 90 % and
14.8 V): the 3 A slow fuse carries it and protects the 20 AWG lead from the
pack's short-circuit current. Below about 13.3 V in (a pack nearly empty)
the output sags under 12 V; the ST3215 runs from 6 V, with less torque. The
2 W at idle cost the flight time nothing measurable. The adapter's USB side
takes some 20-50 mA (estimate) from the Pi's USB, i.e. the Navigator's 5 V
rail: about 2.75 A typical and 3.45 A peak of its 4 A continuous, 4.5 A peak
(table below).

**Flight impact** (the assembly check, with T-Motor's MN3510 KV700 +
T-MOTOR 12*4CF test table at 14.8 V,
https://store.tmotor.com/product/mn3510-motor-navigator-type.html: 1600 g
per motor at full throttle; the P12*4 is rated 4 kg, so the motor, not the
propeller, sets the maximum. Hover throttle, current and rpm are the
table's, interpolated at a quarter of the take-off mass; the flight time
runs to 20 % left on the 4S 5000 mAh pack with 1.1 A of avionics;
`MOT_THST_HOVER` is the hover thrust as a fraction of the full-pack
thrust, the table's maximum scaled to 16.8 V.)

| | Take-off mass | Thrust-to-weight (14.8 V; at 16 V) | Hover throttle | Hover current (14.8 V) | Hover rpm | `MOT_THST_HOVER` | Flight time, 5000 mAh to 20 % |
|---|---|---|---|---|---|---|---|
| Without the launcher (with the HaLow link) | 2.52 kg | 2.54; 2.97 | 53 % | 18.7 A | 5200 (87 Hz) | about 0.31 | about 12.8 min |
| With the launcher, full clip | 3.17 kg | **2.02; 2.36** | 61 % | 26.4 A | 5900 (99 Hz) | about 0.38 | about 9.1 min (-29 %) |

(With the first propulsion, the 2216 KV920 + 10 in at 16 V: 2.33 kg, 2.29,
58 %, 21.4 A, about 11.2 min without the launcher; 2.98 kg, 1.79 (1.61 at
the T1045's 1.2 kg rating), 66 %, 29.0 A, about 8.3 min with it; Air Gear
450 II datasheet,
https://cdn.robotshop.com/media/T/Tmo/RB-Tmo-267/pdf/ligpower_airgear_450ii_combo_set_multi_rotor_uav_p_datasheet.pdf.
Before the HaLow link and the top deck: 2.09 kg.)

- **Thrust-to-weight is 2:1 with a full clip** at the pack's nominal
  14.8 V (2.02; 2.36 at 16 V, about 2.6 on a full pack, about 1.8 near the
  end at 3.5 V per cell). Why this set: ["Propulsion: why 12 in on
  4S"](#propulsion-why-12-in-on-4s). Do not add mass to the launcher
  configuration without checking it: about 35 g more takes it under 2.0 at
  14.8 V (still 2.3 at 16 V), and a bigger pack does too.
- **ArduPilot's `MOT_THST_HOVER` must be relearned** (`MOT_HOVER_LEARN 2`)
  in the first hovers with the launcher fitted, and again when it comes off:
  the two configurations hover at about 0.31 and 0.38. The hardware overlay
  `systems/aerodrone/config/arducopter_hardware.params` starts from 0.31
  (see "ArduPilot parameters" below); the `MOT_THST_HOVER 0.67` in
  `arducopter.params` is Gazebo's model's and stays for the simulators.
  The launcher hangs about 290 mm under the hub and raises the pitch and
  roll inertia: re-run AutoTune.
- The simulations' vehicle masses do not match the real airframe anyway,
  with or without the launcher: Isaac flies a 1.5 kg body with a rectangular
  rotor layout, Gazebo Aerostack2's generic quadrotor; neither models the
  launcher.

**Operating rules.**

- **Foam darts only, toy power**: Elite-size foam darts, nothing else in the
  clip or the barrel; the 1.5 N/mm spring, never a stiffer one. Measure the
  muzzle velocity with a chronograph on this aircraft before the first
  flight (the detent ring changes the shot): ten shots, every one under
  30 m/s.
- **Eye protection for everyone nearby**, on the ground and around the
  flying area.
- **The launcher fires only at objects, never at people.** The
  fire-control software must refuse to fire whenever a person is detected
  in the camera's view; nothing fires automatically (contract:
  `systems/racecar_turret/interfaces.md`).
- **Dropping or firing objects from a drone is regulated**: in the US, FAA
  Part 107.23(b) forbids dropping an object in a manner that creates an
  undue hazard to persons or property, the operations-over-people rules
  (Part 107, subpart D) apply, and Section 363 of the FAA Reauthorization
  Act of 2018 (49 U.S.C. 44802 note) forbids operating a drone equipped
  with a dangerous weapon. The operator must comply with these and with
  local rules; never fly the launcher over people.
- **The safety catch stays on until armed**: pushed in (SAFE) for
  transport, handling and maintenance, with the muzzle cap on and the
  launcher's fuse pulled; pulled back (FIRE) only at the take-off spot with
  nobody under or near the aircraft; safety on first after landing. A
  cocked launcher stays cocked when its power is cut.
- Before every flight: the hanger, cam guide and leg sockets free of
  cracks, their screws tight, the safety lanyard fitted, the battery straps
  through both slot pairs.

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
| Wiring, connectors | Carry up to about 90 A. |
| Tall leg tubes (launcher configuration) | 386 mm legs that take the landing: carbon is several times stiffer and lighter than a printed leg of that length, and a printed one would not fit the bed. |
| Launcher spring, barrel, O-rings | The spring sets the launcher's power: a rated, measured part, never printed (hardware/turret.md). |

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
| **Total** | **about 2.7 A** | **about 3.4 A** | |

The supply (6 A) has margin, and the peak is now 1.1 A under the 4.5 A
peak (4 A continuous) that Blue Robotics rates the Navigator's power to the
Pi and peripherals: the SiK radio is gone, and the HaLowLink 2 (5 V, up to
2 A) has its own regulator on the power distribution, not this rail. With
the foam dart launcher its servo adapter adds some 20-50 mA through the
Pi's USB; its servos run from their own 12 V rail (above). If `vcgencmd
get_throttled` still reports under-voltage, check the 5 V supply's leads.
Motors: about 19 A in hover (26 A with the launcher) and 67 A at full
throttle at 14.8 V (T-Motor's table: 16.8 A per motor), about 86 A from a
full pack, within the Power Sense Module's 100 A (non-continuous), the
XT90's 90 A burst and the AIR 40A's 40 A per motor; hover time with a
5000 mAh pack about 12.8 min (9.1 min with the launcher). The HaLowLink 2
draws up to about 0.7 A from the pack (10 W).

Signal and power connections are listed in
[the printed-parts README, "Wiring"](parts/aerodrone/README.md#wiring).

### ArduPilot parameters

`systems/aerodrone/config/arducopter.params` is shared: the Gazebo and
Isaac SITL simulations load it first
(`applications/ardupilot_sitl/scripts/run_arducopter_sitl.sh`). The
printed aircraft's propulsion and battery values are therefore in a
hardware-only overlay that no simulator loads,
`systems/aerodrone/config/arducopter_hardware.params`. On the aircraft load,
in this order, with MAVProxy from the ground laptop (`param load ...`, as in
systems/aerodrone/README.md):

1. `arducopter.params` (frame, failsafes, navigation speeds);
2. `arducopter_hardware.params`; reboot the autopilot and load it a second
   time (the harmonic notch and battery monitor parameters appear only
   after the reboot that enables them);
3. `arducopter_gps_input.params` (GPS-denied corrections).

The overlay sets, from the numbers above (and
`tests/test_ardupilot_param_validation.py` checks them against the
assembly check):

- `MOT_BAT_VOLT_MAX 16.8`, `MOT_BAT_VOLT_MIN 13.2` (4.2 and 3.3 V per
  cell, ArduPilot's tuning guide): the motor output is compensated for the
  pack's voltage.
- `MOT_THST_HOVER 0.31` (without the launcher; start from 0.38 with it),
  `MOT_HOVER_LEARN 2`: ArduPilot learns and saves the real value.
- The Power Sense Module (`BATT_MONITOR 4`, `BATT_VOLT_MULT 11.0`,
  `BATT_AMP_PERVLT 37.8788`, `BATT_AMP_OFFSET 0.330`, Blue Robotics' scale
  factors; BlueOS normally sets them and the ADC pins) and the 4S 5000 mAh
  pack: `BATT_CAPACITY 5000`, `BATT_ARM_VOLT 15.2`; low at 14.0 V or
  1000 mAh left (the 20 % the flight times stop at) -> RTL, critical at
  13.6 V or 500 mAh -> Land (`BATT_FS_LOW_ACT 2`, `BATT_FS_CRT_ACT 1`), on
  the sag-compensated voltage (`BATT_FS_VOLTSRC 1`).
- A throttle-based harmonic notch (the PWM ESCs report no rpm):
  `INS_HNTCH_MODE 1`, `INS_HNTCH_FREQ 85` Hz and `INS_HNTCH_REF 0.31` at the
  hover without the launcher (87 Hz by the table), `INS_HNTCH_BW 42`,
  first and second harmonics (`INS_HNTCH_HMNCS 3`); with the throttle it
  moves up to about 94 Hz in the launcher configuration's hover (99 Hz by
  the table). A starting point: confirm it with the first flight's FFT log
  and keep `INS_HNTCH_REF` equal to the learned `MOT_THST_HOVER`.

Not set, to check on the bench: the AIR 40A's throttle range
(`MOT_PWM_MIN`/`MOT_PWM_MAX`, or ArduPilot's ESC calibration) and
`MOT_SPIN_ARM`/`MOT_SPIN_MIN` (motor test, props off).

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
| GPS mast | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 1 h 58 min | 48 g | [gps_mast.stl](parts/aerodrone/stl/gps_mast.stl) |
| HaLowLink 2 cradle (on the top deck) (fit test first) | 1 | ASA | 0.20 mm, 3 walls, 30% gyroid, no supports | 1 h 04 min | 34 g | [halow_cradle.stl](parts/aerodrone/stl/halow_cradle.stl) |
| Hub bottom plate (flight-critical) | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 2 h 59 min | 102 g | [hub_bottom_plate.stl](parts/aerodrone/stl/hub_bottom_plate.stl) |
| Hub top plate (flight-critical) | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 3 h 03 min | 104 g | [hub_top_plate.stl](parts/aerodrone/stl/hub_top_plate.stl) |
| Landing leg | 4 | ASA | 0.20 mm, 5 walls, 40% gyroid, no supports | 46 min | 26 g | [landing_leg.stl](parts/aerodrone/stl/landing_leg.stl) |
| Launcher breech, downward (no pedestal; aerodrone) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports | 1 h 19 min | 42 g | [launcher_breech_down.stl](parts/aerodrone/stl/launcher_breech_down.stl) |
| Launcher cam guide (pad on the downward breech) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports | 1 h 04 min | 36 g | [launcher_cam_guide.stl](parts/aerodrone/stl/launcher_cam_guide.stl) |
| Launcher dart detent (TPU ring, aerodrone) (fit test first) | 1 | TPU95A | 0.20 mm, 2 walls, 100% gyroid, no supports | 1 min | 0.1 g | [launcher_dart_detent.stl](parts/aerodrone/stl/launcher_dart_detent.stl) |
| Launcher hanger (flange, keel, rib) (flight-critical) | 1 | ASA | 0.20 mm, 4 walls, 25% gyroid, no supports, brim outer brim only, 5 mm | 3 h 06 min | 87 g | [launcher_hanger.stl](parts/aerodrone/stl/launcher_hanger.stl) |
| Landing foot | 4 | TPU95A | 0.20 mm, 3 walls, 30% gyroid, no supports | 23 min | 5.0 g | [leg_foot.stl](parts/aerodrone/stl/leg_foot.stl) |
| Motor mount (flight-critical, fit test first) | 4 | ASA | 0.20 mm, 5 walls, 50% gyroid, no supports | 49 min | 25 g | [motor_mount.stl](parts/aerodrone/stl/motor_mount.stl) |
| OAK-1 cradle (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 40% gyroid, no supports | 31 min | 15 g | [oak1_cradle.stl](parts/aerodrone/stl/oak1_cradle.stl) |
| OAK-1 camera stage (isolated, pitch pivot) (fit test first) | 1 | PETG | 0.20 mm, 4 walls, 40% gyroid, no supports | 48 min | 25 g | [oak1_stage.stl](parts/aerodrone/stl/oak1_stage.stl) |
| Sensor nose (OAK-1 damper deck and rangefinder mount) (fit test first) | 1 | ASA | 0.20 mm, 4 walls, 30% gyroid, no supports | 1 h 12 min | 40 g | [sensor_nose.stl](parts/aerodrone/stl/sensor_nose.stl) |
| Stack grommet (TPU top hat) | 8 | TPU95A | 0.20 mm, 3 walls, 100% gyroid, no supports | 1 min | 0.2 g | [stack_grommet.stl](parts/aerodrone/stl/stack_grommet.stl) |
| Stack tray (Pi 5 + Navigator) (flight-critical, fit test first) | 1 | PETG | 0.20 mm, 3 walls, 30% gyroid, no supports | 25 min | 15 g | [stack_tray.stl](parts/aerodrone/stl/stack_tray.stl) |
| Tall leg foot (TPU, launcher configuration) | 4 | TPU95A | 0.20 mm, 3 walls, 40% gyroid, no supports | 30 min | 6.6 g | [tall_leg_foot.stl](parts/aerodrone/stl/tall_leg_foot.stl) |
| Tall leg socket (launcher configuration) | 4 | ASA | 0.20 mm, 5 walls, 40% gyroid, no supports | 46 min | 21 g | [tall_leg_socket.stl](parts/aerodrone/stl/tall_leg_socket.stl) |
| Top deck (HaLowLink 2; the battery with the launcher) (flight-critical, fit test first) | 1 | ASA | 0.20 mm, 3 walls, 20% gyroid, no supports | 1 h 59 min | 64 g | [top_deck.stl](parts/aerodrone/stl/top_deck.stl) |
| **Total, all pieces** | | | | **34 h 10 min** | **994 g** | |
<!-- END GENERATED: printed-parts -->

### Fasteners

Totals for one aircraft, from the parts' metadata (screws are listed with
the part their head bears on). Buy 10-20 % spares.

<!-- BEGIN GENERATED: fasteners (tools/build_printable_parts.py) -->
| Item | Qty | Where |
|---|---|---|
| 1/4"-20 UNC x 1/2" button or socket head screw | 1 | OAK-1 to cradle (engages about 8 mm of the 10 mm thread) (oak1_cradle, 1) |
| Dyneema or Kevlar cord, 1.5 mm, 300 mm | 1 | safety lanyard: launcher (round the plunger tube, above the saddle) to the hub, slack (launcher_hanger, 1) |
| M2.5 x 12 socket head cap screw | 2 | from under the tray through the Pi into the Navigator heatsink (the two holes the Navigator guide's standoff uses) (stack_tray, 2) |
| M3 nyloc nut | 8 | tall leg socket pinch clamp (tall_leg_socket, 8) |
| M3 washer | 22 | under the arm clamp screw heads (hub_top_plate, 16); under the pitch-lock screws (oak1_stage, 2); on the upper stack grommets (stack_tray, 4) |
| M3 x 10 socket head cap screw | 12 | cam guide to the breech's underside inserts (heads in counterbores) (launcher_cam_guide, 4); hanger flange to the bottom plate's battery-tray inserts (launcher_hanger, 4); sensor nose to bottom plate (sensor_nose, 4) |
| M3 x 12 socket head cap screw | 7 | GPS mast to top plate (gps_mast, 3); stack tray to top plate, through a pair of stack grommets (stack_tray, 4) |
| M3 x 14 socket head cap screw | 4 | battery tray to bottom plate (heads recessed in the posts) (battery_tray, 4) |
| M3 x 16 socket head cap screw | 16 | landing leg to bottom plate (landing_leg, 8); tall leg socket to the bottom plate (heads in counterbores) (tall_leg_socket, 8) |
| M3 x 20 socket head cap screw | 8 | tall leg socket pinch clamp on the carbon tube (tall_leg_socket, 8) |
| M3 x 25 socket head cap screw | 24 | arm clamps, through the top plate into the bottom plate inserts (hub_top_plate, 16); pinch clamp on the arm tube (motor_mount, 8) |
| M3 x 3 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 15 | rear plate, upper flange screws (launcher_breech_down, 2); front face, keel screws (launcher_cam_guide, 4); side bosses: pitch pivot (2) and pitch lock (2), pressed flush from outside (through holes) (oak1_cradle, 4); top face, HaLowLink cradle screws, pressed flush (through holes) (top_deck, 5) |
| M3 x 5 socket head cap screw | 5 | HaLowLink cradle to the top deck (heads recessed) (halow_cradle, 5) |
| M3 x 5.7 mm brass heat-set insert (CNC Kitchen type, 4 mm hole) | 63 | clamp blocks, from the top face (arm clamp bolts) (hub_bottom_plate, 16); underside of the ears, two per arm (landing legs) (hub_bottom_plate, 8); underside, rear and middle bosses (battery tray) (hub_bottom_plate, 4); underside, front bosses (sensor nose) (hub_bottom_plate, 4); top face, stack tray grommet bolts (hub_top_plate, 4); top face, GPS mast flange (hub_top_plate, 3); top face, top deck posts (hub_top_plate, 4); rear plate, lower flange screws (launcher_breech_down, 2); wall bosses, cover screws, from the top (launcher_breech_down, 4); front wall, barrel clamp screws (launcher_breech_down, 2); underside, cam guide screws (launcher_breech_down, 4); far side of the pinch clamp (motor_mount, 8) |
| M3 x 50 socket head cap screw | 4 | top deck posts to the top plate inserts (heads recessed in the deck) (top_deck, 4) |
| M3 x 8 socket head cap screw | 24 | hanger keel to the cam guide's inserts (launcher_hanger, 4); motor to mount (T-Motor's two M3 x 12 are for the propeller) (motor_mount, 16); pitch pivot (2) and pitch lock (2), into the cradle inserts (oak1_stage, 4) |
| ST2.9 x 9.5 pan head self-tapping screw (TFmini-S) or M2 x 12 + nut (TF-Luna) | 2 | rangefinder ears, from above (sensor_nose, 2) |
| anti-slip foam pad, 150 x 50 mm | 1 | between the battery and the tray (battery_tray, 1) |
| battery strap, 20 mm hook-and-loop, 250-300 mm, with rubberised face | 2 | battery to tray (battery_tray, 2) |
| double-sided foam tape (VHB), 50 mm disc | 1 | GPS puck on the mast seat (gps_mast, 1) |
| hook-and-loop strap, 20 mm, 300 mm | 1 | HaLowLink 2 round the cradle's walls (halow_cradle, 1) |
| silicone gimbal damper ball, 6.5 mm neck | 4 | camera stage isolation (sensor_nose, 4) |
| zip tie 2.5 mm | 17 | motor leads to the arm cable clip (cable_clip, 8); SMA extension cable to the cradle's arm (halow_cradle, 1); power sense module and 5 V BEC to the four centre slots (hub_bottom_plate, 4); plunger tube to the hanger's saddle (1); servo cables to the keel (3) (launcher_hanger, 4) |
<!-- END GENERATED: fasteners -->

### Assembly notes

- Build order, torques and checks: [parts/aerodrone/README.md](parts/aerodrone/README.md#steps).
- Arm tubes: cut to 180.0 mm, square; they set the 500 mm motor spacing.
- Battery position sets the centre of gravity: about 31 mm behind the hub
  centre on the tray; with the launcher, about 36 mm behind it on the top
  deck (the assembly check computes both); trim by balancing.
- Motor seats must be parallel to the hub before the pinch screws are
  tightened.
- Nothing hangs in the OAK-1's view: the legs splay outward and the battery
  sits above the lens; the assembly check verifies it with a 2 deg margin,
  for the launcher configuration too (the nose carries the camera 25 mm
  further forward for the launcher's barrel).

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
- The printed aircraft: about 2.5 kg, square quad X with 12 in rotors at
  (+-177, +-177) mm, 47 mm above the arm-tube plane, OAK-1 lens about
  142 mm ahead of and 80 mm below the arm-tube plane, rangefinder 177 mm
  ahead, GPS 85 mm behind and about 240 mm above, feet at (+-106, +-106) mm,
  206 mm below. With the foam dart launcher: about 3.2 kg, its muzzle
  307 mm below the arm-tube plane on the centre line, feet at
  (+-230, +-230) mm, 357 mm below.
  Its hover thrust (about 0.31 without the launcher, 0.38 with it) differs
  from the `MOT_THST_HOVER 0.67` in `arducopter.params`, which the Gazebo
  model learned; the hardware overlay `arducopter_hardware.params` starts the
  aircraft at 0.31 and ArduPilot relearns it in the first hovers
  (`MOT_HOVER_LEARN`). Measured camera and
  rangefinder extrinsics belong in `systems/aerodrone/config/sensors.yaml`
  once calibrated.

## Recommended assembly order

1. Print, then assemble the frame, motors, ESCs and power distribution with
   the propellers off ([parts/aerodrone/README.md](parts/aerodrone/README.md)).
2. Mount the Raspberry Pi/Navigator stack and wire the PWM outputs, Power
   Sense Module, GPS/compass and RC receiver; fit the top deck and the
   HaLowLink 2, pair it with the ground station's (hardware/ground_station.md:
   Extender mode, static 192.168.2.252, its 2.4 GHz access point off) and
   check MAVLink at 192.168.2.1:14550.
3. Load the parameters (`arducopter.params`, `arducopter_hardware.params`
   twice with a reboot between, `arducopter_gps_input.params`: "ArduPilot
   parameters" above), check the ESCs' throttle range, then verify
   ArduPilot motor order, RC failsafe, battery monitor, compass, and IMU
   calibration before installing propellers.
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
7. Foam dart launcher (optional): only once the aircraft flies well without
   it. Bring the launcher up on the bench (servo IDs, limits, dry cycle,
   chronograph), fit it with the battery on the top deck and the tall legs,
   balance, run the compass-motor calibration, relearn the hover throttle
   tethered, and fire only with the fire-control software that refuses
   whenever a person is in view (parts/aerodrone/README.md, "Foam dart
   launcher").

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
- Holybro X500 V2 (the frame class; its 2216 KV920 + 10 in propulsion was
  this aircraft's first):
  https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2
- Propulsion: T-Motor MN3510
  (https://store.tmotor.com/product/mn3510-motor-navigator-type.html),
  P12*4 (https://store.tmotor.com/product/polish-carbon-fiber-12x4-prop.html),
  AIR 40A (https://store.tmotor.com/product/air-40a-6s-esc.html); the
  comparison's other sources are in "Propulsion: why 12 in on 4S"
- Hardware parameter overlay: systems/aerodrone/config/arducopter_hardware.params
- Holybro M10/M9N GPS:
  https://docs.holybro.com/gps-and-rtk-system/m8n-m9n-m10-gps
- Morse Micro HaLowLink 2 product brief:
  https://www.morsemicro.com/resources/product_brief/HaLowLink-2_Product-Brief.pdf;
  HaLowLink user guide:
  https://static.gl-inet.com/www/images/products/user-guide/haLowlink2_user-guide.pdf;
  Virtual Wire (APPNOTE-37):
  https://docs.morsemicro.com/application-notes/appnote-37-how-to-configure-virtual-wire
- BlueOS defaults (eth0 192.168.2.2, "GCS Client Link" to 192.168.2.1:14550):
  https://github.com/bluerobotics/BlueOS (`core/services/cable_guy/config.py`,
  `core/services/ardupilot_manager/autopilot_manager.py`)
- Ground station (HaLow access point, laptop, addressing): hardware/ground_station.md
