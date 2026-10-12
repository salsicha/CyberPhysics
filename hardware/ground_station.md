# Ground station bill of materials

Goal: one ground station for the long-range IP link to the vehicles: the
RACECAR Neo and the RACECAR Turret built on it
([hardware/racecarneo.md](racecarneo.md),
[hardware/racecar_turret.md](racecar_turret.md)) and the aerodrone
([hardware/aerodrone.md](aerodrone.md)). The link is Wi-Fi HaLow
(IEEE 802.11ah, sub-GHz) between Morse Micro HaLowLink 2 units: one here as
the access point, one on each vehicle as a client that bridges the
vehicle's Ethernet to it, the user guide's "Virtual Wire". The operator's
laptop, tablet or phone joins the access point by Ethernet or by its
2.4 GHz Wi-Fi and reaches every vehicle on one bridged network.

**HaLow is not a safety link.** Each vehicle keeps its own 2.4 GHz RC
transmitter as the independent stop and failsafe (the car's RC relay in its
e-stop loop, the aircraft's RC failsafe), and is never driven or flown
beyond where that RC link works.

```text
 vehicle                                              ground station
 Jetson (car) or Pi 5 (drone) -- Ethernet --+
                                            |
                      HaLowLink 2, Extender (aqua)  ~~ 902-928 MHz (US) ~~  HaLowLink 2, Access Point (green)
                      (Client, WDS: bridges its                              WAN, LAN, 2.4 GHz Wi-Fi and USB-C
                       Ethernet and USB-C to HaLow)                          all on one bridge
                                                                               |-- Ethernet --- laptop 192.168.2.1
                                                                               '-- 2.4 GHz Wi-Fi -- tablet, phone
 RC receiver  <~~~~~~~~~~~~~~~~ 2.4 GHz, independent of HaLow ~~~~~~~~~~~~~~~~  RC transmitter (one per vehicle)
```

## Purchased parts

### Access point

- 1 x **Morse Micro HaLowLink 2** (sold by GL.iNet, about USD 129,
  https://www.gl-inet.com/en-us/products/halowlink2). From the product
  brief (https://www.morsemicro.com/resources/product_brief/HaLowLink-2_Product-Brief.pdf,
  Rev 9): Morse Micro MM8108 802.11ah radio with 1/2/4/8 MHz channels, TX
  power up to 26 dBm ("limited to 14dbm in EU (inc UK) and 16dBm in Japan"),
  MediaTek MT7621 running OpenWrt 23.05, 2 x Gigabit Ethernet (WAN, LAN), a
  USB-C port for "power and Ethernet data", 2.4 GHz 802.11n 2x2 Wi-Fi with
  internal antennas, an **SMA connector** for the HaLow antenna, a Mode
  button; **5 V, 2 A** on the USB-C; 0-40 C; 88 x 68 x 24 mm, 102 g (168 g
  with its stand and antenna). In the box: the region-specific antenna, the
  stand, an Ethernet cable, a USB-C to USB-C cable and a mains adapter.
  GL.iNet gives "up to 1km (0.6 miles)" of range.
- Buy the **region version** for where it is used: the HaLowLink 2's
  region is locked at the factory to one of US, EU, AU or JP and cannot be
  changed ("Unfortunately due to regulatory compliance reasons you are
  unable to change the region on Halowlink2s", Morse Micro on its
  community forum, https://community.morsemicro.com/t/new-halowlink2-how-to-configure-country/1779;
  the AU version covers only 915-928 MHz). The vehicles' units must be the
  same region as this one.

### Antenna, mast and cable

The range near the ground depends far more on how high the ground antenna
is than on anything else (see "Expected range"), so it goes up a mast.

- Recommended: **the whole access point at the top of the mast**, with its
  own antenna (a 2.34 dBi dipole: the antenna the user guide's ISED
  statement lists it with), and only Ethernet and power coming down. No
  coax loss, and no antenna question with the regulator.
  - 1 x light stand, camera tripod or telescopic mast, **2.5-3 m**, stable
    in wind (sandbag or guy lines); the HaLowLink 2 strapped to its top on
    its stand (hook-and-loop straps or a phone/tablet clamp), the antenna
    vertical.
  - 1 x outdoor Cat6 patch cable, 5-10 m, from the mast to the laptop (or to
    a small switch).
  - Power at the top of the mast (below), or a USB-C cable down the mast
    no longer than about 2 m (5 V over a long thin cable sags).
- Alternative: the access point on the table and **an antenna on the mast**
  through a low-loss coax. Then:
  - 1 x 902-928 MHz (US) or 863-870 MHz (EU) **dipole of at most 2.34 dBi**
    with a mast clamp. In the US an antenna may replace the one the device
    was certified with only if it is of the same type and of equal or
    lower gain (47 CFR 15.204(c)(4)); in Canada the user guide lists only
    the 2.34 dBi dipole. A higher-gain collinear needs the vendor's
    approval.
  - 1 x coax, SMA plug at the HaLowLink 2's end, as short as the mast
    allows: at 900 MHz LMR-240 loses about 0.25 dB/m, LMR-200 about
    0.35 dB/m, RG58 about 0.5 dB/m and RG316 about 1 dB/m. 5 m of LMR-240
    costs about 1.3 dB, which raising the antenna by 2 m more than repays.

### Power

- 1 x **USB-C power bank** with a USB-C output that supplies 5 V at 2 A or
  more (USB-C current advertisement or USB PD), and a USB-C to USB-C cable
  (the unit's own). The user guide asks for a supply that "can provide
  sufficient power" and not to use a USB-A to USB-C adapter. A
  20,000 mAh (74 Wh) bank runs the unit for many hours at its few watts.
- Or the unit's own mains adapter where there is mains.

### Operator devices

- A **laptop** with a browser for each vehicle's web page (the RACECAR's
  page on port 8080 has the map, the cameras, the joysticks and the
  turret's Fire), and, for the aerodrone, **QGroundControl** or Blue
  Robotics' **Cockpit** (MAVLink from BlueOS) and the BlueOS web interface.
  Wired to the access point's LAN or WAN port (it is the most reliable
  path), with the static address below.
- Optionally a **tablet or phone** with a browser, on the access point's
  2.4 GHz Wi-Fi (it gets an address from the access point's DHCP server).
- The vehicles' pages have no login: nothing else joins this network.

### RC transmitters

- Each vehicle's own **2.4 GHz RC transmitter** (the car's, bound to the
  receiver that drives the RC relay in its e-stop loop; the aircraft's,
  bound to its SBUS receiver with its failsafe set). They stay the stop and
  the failsafe whatever HaLow does; their range, not HaLow's, limits how
  far a vehicle may go.

## Network

### Addressing plan

One bridged IPv4 network, **192.168.2.0/24**, static addresses for
everything that serves or receives something. The aerodrone keeps BlueOS's
defaults (BlueOS, https://github.com/bluerobotics/BlueOS: the Pi's `eth0`
is 192.168.2.2 in `core/services/cable_guy/config.py`, and its default GCS
link sends MAVLink to 192.168.2.1:14550 in
`core/services/ardupilot_manager/autopilot_manager.py`), so the ground
laptop is 192.168.2.1. The car takes a free address, **192.168.2.10**.

| Device | Address | Notes |
|---|---|---|
| Ground laptop | 192.168.2.1/24, no gateway | BlueOS's MAVLink target (UDP 14550); the browser for every page |
| Aerodrone, Raspberry Pi 5 `eth0` (BlueOS) | 192.168.2.2 | BlueOS default: `http://192.168.2.2` |
| RACECAR Jetson `eth0` | 192.168.2.10/24, no gateway | the page at `http://192.168.2.10:8080` (`hardware/racecarneo.md`) |
| Tablets and phones | DHCP, 192.168.2.100-149 | from the access point, no gateway or DNS |
| Ground HaLowLink 2 (access point) | 192.168.2.250 | its web interface; runs the DHCP server above |
| Car HaLowLink 2 (Extender) | 192.168.2.251 | its web interface |
| Aerodrone HaLowLink 2 (Extender) | 192.168.2.252 | its web interface |

- No device on this network has a default gateway, so the laptop and the
  phones keep their internet (and the Jetson its own Wi-Fi) on their other
  interfaces. The Jetson's own Wi-Fi network must not also be
  192.168.2.0/24.
- Give each vehicle its own `ROS_DOMAIN_ID` (the compositions read it from
  the shell). The bridge carries DDS discovery (multicast) between the
  vehicles; on different domains their ROS graphs never meet. Do not run
  ROS 2 tools on the ground laptop on a vehicle's domain over HaLow: a
  subscriber to a camera topic would pull raw frames across the link.

### Access point setup (on the bench, before the field)

User guide: HaLowLink user guide 2.11
(https://static.gl-inet.com/www/images/products/user-guide/haLowlink2_user-guide.pdf).
Morse Micro's APPNOTE-37 "How to Configure Virtual Wire"
(https://docs.morsemicro.com/application-notes/appnote-37-how-to-configure-virtual-wire)
describes the same setup for the HaLowLink 1.

1. Fit the antenna first, power the unit, connect the laptop to its LAN
   port, open `https://192.168.12.1` (accept the self-signed certificate)
   and log in with the label's credentials. Update the software (Software
   Updates) and change the admin password.
2. Check the region on the status page (it is locked; see above).
3. Status LED **green**: Access Point mode (a factory reset or a long press
   of Mode returns to it). With the WAN port on any router with DHCP, run
   the Wizard with **"Wi-Fi HaLow devices will get an IP on your existing
   router's network"**: the unit becomes a bridged HaLow access point (the
   guide's "Wi-Fi HaLow Access Point (AP)").
4. Quick Config, Network Interfaces: give the bridged (`wlan`) interface the
   static address **192.168.2.250/24** with a DHCP server for
   192.168.2.100-149 (in Advanced Config, DHCP options: no gateway, option 3
   empty, the way the guide's section 6.7 note describes), then move the
   LAN port, the 2.4 GHz access point and USB-C onto it as well: the
   guide's FAQ "fully bridged" access point. From then on its web interface
   is at `https://192.168.2.250`, and the laptop on any port or on its
   2.4 GHz Wi-Fi is on the vehicles' network. Unplug the router.
5. Security: the HaLow network uses **WPA3 (SAE)**; set your own HaLow SSID
   and passphrase and your own 2.4 GHz passphrase (Quick Config, Wireless)
   before pairing, keep SSH off unless needed, and never connect this
   network to the internet or to a shared LAN: the vehicles' pages have no
   login.
6. Channel: Status, Channel Analysis shows the other HaLow networks; pick
   a quiet channel in Quick Config, Wireless. A narrower channel (1-2 MHz)
   reaches farther at a lower rate, a wider one (4-8 MHz) is faster at short
   range. Mesh modes are not for this setup (and the guide says they are
   not supported in the EU and UK).

### Pairing a vehicle's unit

1. On the vehicle's HaLowLink 2: wait for a solid status LED, hold Mode
   until it flashes quickly aqua, release: **Extender** mode (solid aqua).
2. Press and release Mode on the access point (its HaLow LED flashes
   slowly), then on the Extender. The Extender's HaLow LED turns **solid
   purple** when it has joined; the access point has pushed the network's
   credentials and mode to it. If pairing fails the LEDs flash quickly for
   120 s before another try (user guide, section 3.1).
3. The Extender is now a HaLow client (WDS) that bridges its LAN, WAN and
   USB-C ports to the access point's network: the guide's "Virtual Wire".
   From the laptop, find it in the access point's DHCP leases and set it to
   a static **192.168.2.251** (car) or **.252** (aerodrone) in its own
   Quick Config, so it stays reachable without the DHCP server, and turn
   its 2.4 GHz access point **off** (Quick Config, Wireless): on a vehicle it
   would share 2.4 GHz with the RC link.
4. Cable the Extender's LAN port to the vehicle computer's Ethernet and set
   that computer's static address (table above; the car: `hardware/racecarneo.md`,
   "Long-range link").

## Expected range

- Morse Micro and GL.iNet give **up to about 1 km** line of sight. That is
  for antennas well above the ground.
- Near the ground the reflected ray cancels the direct one: beyond a few
  tens of metres the path loss grows by about 12 dB per doubling of
  distance instead of 6, and the range goes as the square root of the
  product of the two antenna heights. The car's antenna is only about
  0.2 m up (its tip under the RACECAR Turret's launcher), so the car's
  range is set by the ground antenna's height: 3 m instead of 1 m is about
  9.5 dB, or about 1.7 times the range.
- In the EU the transmit power is 12 dB lower (14 dBm), which near the
  ground halves the range.
- Trees, buildings, people and the operator's own body between the
  antennas cost more. Measure it (below); do not plan on 1 km.
- Throughput falls with distance as the link drops to lower rates
  (802.11ah's lowest are a few hundred kbit/s). The RACECAR Turret's page at
  the hardware compositions' HaLow settings needs about 0.5-0.9 Mbit/s per
  open browser (`systems/racecarneo/README.md`, "Long-range link"); keep
  twice that.

## Range test

1. **Bench.** Both units a few metres apart, the vehicle's paired. On the
   access point, Advanced Config, Services, **Range Test**: remote device =
   the vehicle's unit, both directions, TCP and UDP (it runs `iperf3` for
   60 s; Quick Tests for 10 s). Note the throughput: this is the short-range
   ceiling. A result near 900 Mbit/s TCP means the test ran over Ethernet,
   not HaLow (the guide's warning).
2. **Field, the mast up.** Ground antenna at 2.5-3 m, the vehicle on the
   ground (the aircraft too, for the first test), its stack running.
   Walk the vehicle out along the path it will use: 25, 50, 100, 200,
   400 m and on. At each point run the Range Test (enter both positions
   for the distance) and note RSSI, MCS and throughput; Realtime Graphs
   show the link quality live.
3. **With the operator's page.** At each point, on the car: open
   `http://192.168.2.10:8080`, check that the turret and map images keep
   updating, that the joystick still moves the car promptly, and that Stop
   all works. On the aircraft: MAVLink heartbeats in QGroundControl or
   Cockpit without drop-outs.
4. **The RC stop at every point.** Car: wheels up or slow, switch the
   transmitter off: the RC relay must open the e-stop loop. Aircraft: the
   RC failsafe, on the ground. The usable range is the shortest of HaLow's
   (throughput at least twice the page's need), the RC link's, and the
   distance at which you still see the vehicle.
5. Write the result down with the antenna heights, channel and weather; the
   access point's Range Test results are lost at power-off (download the
   CSV).

## Safety and rules

- Keep **20 cm** between any HaLow antenna and people while it transmits
  (the user guide's RF exposure statement), on the mast and on the
  vehicles.
- Use the region version for the country (US: 902-928 MHz, an FCC Part 15
  device; EU: 863-868 MHz) and the antenna rules above.
- HaLow is not the stop: the RC links are (above). Losing HaLow does not
  stop a vehicle: on the car the page's joystick deadman ends manual
  driving within about 0.5 s, but a Nav2 goal runs on and the turret keeps
  tracking (it fires only on a tap); on the aircraft it trips the GCS
  failsafe where that is enabled.
