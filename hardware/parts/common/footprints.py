"""Datasheet footprints of purchased parts that printed parts mate with.

Pure Python. Every class gives its source document and lists in
``UNVERIFIED`` the attributes that the cited document does not state (they
are assumptions or reseller figures); a printed part using any of them must
set ``fit_test`` and is printed first to check the fit.

Conventions: millimetres; hole coordinates are measured from the corner or
edge named in each class.
"""


class RaspberryPi5:
    """Raspberry Pi 5 Model B."""
    SOURCE = ("Raspberry Pi Ltd, 'Raspberry Pi 5' mechanical drawing RP-008347-DS-1 "
              "(https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf)")
    BOARD = (85.0, 56.0)
    # Mounting holes from the board corner at the edge opposite the
    # USB/Ethernet end, on the side opposite the 40-pin header: 3.5 mm in
    # from both edges, 58 x 49 mm apart.
    HOLE_INSET = 3.5
    HOLE_SPACING = (58.0, 49.0)
    HOLE_D = 2.7
    HOLES = ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5))
    # USB and Ethernet jacks stand 3 mm proud of the 85 mm board edge.
    CONNECTOR_OVERHANG = 3.0
    # Not on the drawing: PCB thickness (typical 1.6 mm) and the height of
    # the parts under the board (microSD socket), budgeted at 3 mm.
    PCB_THICKNESS = 1.6
    UNDERSIDE_CLEARANCE = 3.0
    MASS_G = 46.0
    UNVERIFIED = ("PCB_THICKNESS", "UNDERSIDE_CLEARANCE", "MASS_G")


class RaspberryPi4:
    """Raspberry Pi 4 Model B: the same 58 x 49 mm hole pattern as the Pi 5."""
    SOURCE = ("Raspberry Pi Ltd, Raspberry Pi 4 Model B mechanical drawing "
              "(https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-mechanical-drawing.pdf)")
    BOARD = (85.0, 56.0)
    HOLE_SPACING = (58.0, 49.0)
    HOLE_D = 2.7
    UNVERIFIED = ("BOARD", "HOLE_SPACING", "HOLE_D")  # not re-read here; same HAT pattern as the Pi 5


class BlueRoboticsNavigator:
    """Blue Robotics Navigator flight controller (Raspberry Pi HAT)."""
    SOURCE = ("Blue Robotics, Navigator Flight Controller technical details "
              "(https://bluerobotics.com/store/comm-control-power/control/navigator/) and "
              "Navigator Hardware Setup (https://bluerobotics.com/learn/navigator-hardware-setup/)")
    BOARD = (65.0, 56.0)
    HOLE_SPACING = (58.0, 49.0)
    HOLE_D = 2.75
    MASS_G = 26.0
    # Aluminium heatsink between the Pi and the Navigator, M2.5 x 0.45 holes.
    HEATSINK = (65.0, 56.0, 14.0)
    HEATSINK_MASS_G = 30.0
    SUPPLY_V = (5.0, 5.35)
    SUPPLY_MIN_A = 4.5               # "a power supply capable of delivering at least 4.5 amps"
    PI_AND_PERIPHERALS_A = (4.0, 4.5)  # max power to Pi & peripherals: continuous, peak
    # Connector heights above the Navigator PCB are not in the documents;
    # budgeted at 15 mm for plugged servo leads on the PWM rail.
    CONNECTOR_HEIGHT = 15.0
    UNVERIFIED = ("CONNECTOR_HEIGHT",)


class LuxonisOAK1:
    """Luxonis OAK-1 (USB, IMX378). The enclosure is 36 wide, 54.5 high
    (USB-C end to top) and 27.8 deep (lens face to the back fins)."""
    SOURCE = ("Luxonis OAK-1 datasheet "
              "(docs.luxonis.com/projects/hardware/en/latest/pages/BW1093.html, PDF copy "
              "https://media.distrelec.com/Web/Downloads/_t/ds/OAK-1_eng_tds.pdf), dimension drawing "
              "and 'Dimensions and Weight'; FOV of the fixed-focus IMX378 variant from "
              "https://docs.luxonis.com/hardware/products/OAK-1 (auto-focus: 78/66/54 deg)")
    WIDTH = 36.0
    HEIGHT = 54.5
    DEPTH = 27.8
    MASS_G = 53.1
    # Lens centre 32.4 mm from the USB-C end; 1/4"-20 tripod thread, 10 mm
    # deep, in the back 14.5 mm from the USB-C end.
    LENS_FROM_USB_END = 32.4
    TRIPOD_FROM_USB_END = 14.5
    TRIPOD_DEPTH = 10.0
    # The drawing shows the lens and the tripod hole on the centre line
    # across the width but does not dimension it.
    LENS_CENTERED = True
    TRIPOD_CENTERED = True
    HFOV_DEG = 69.0
    VFOV_DEG = 55.0
    DFOV_DEG = 82.0
    POWER_W = (2.5, 3.0)   # base + camera streaming; up to +1 W for AI
    UNVERIFIED = ("LENS_CENTERED", "TRIPOD_CENTERED")


class BenewakeTFminiS:
    """Benewake TFmini-S single-point LiDAR (same housing as the TFmini)."""
    SOURCE = ("Benewake TFmini-S product page (https://en.benewake.com/TFminiS/index.html): "
              "42 x 15 x 16 mm, 5 g, 5 V, <= 140 mA; hole pattern from the TFmini manual "
              "SJ-PM-TFmini-T-01 A03, Figure 5 "
              "(https://cdn-shop.adafruit.com/product-files/3978/3978_manual_SJ-PM-TFmini-T-01_A03ProductManual_EN.pdf)")
    LENGTH = 42.0
    WIDTH = 15.0
    HEIGHT = 16.0
    LENS_BLOCK_LENGTH = 36.0
    EAR_THICKNESS = 5.0
    HOLE_SPACING = 33.0
    HOLE_D = 2.35          # through hole; Benewake recommends ST2.9 self-tapping screws
    HOLE_FROM_SIDE = 7.5   # holes on the centre line of the 15 mm width
    MASS_G = 5.0
    SUPPLY_V = 5.0
    CURRENT_MA = 140
    # The hole pattern is from the TFmini drawing; the TFmini-S datasheet
    # (SJ-PM-TFmini-S-01) could not be read here.
    UNVERIFIED = ("HOLE_SPACING", "HOLE_FROM_SIDE", "EAR_THICKNESS", "LENS_BLOCK_LENGTH")


class BenewakeTFLuna:
    """Benewake TF-Luna single-point LiDAR."""
    SOURCE = ("Benewake TF-Luna datasheet PD-TF-Luna-Datasheet-A05 "
              "(https://en.benewake.com/uploadfiles/2024/04/20240426135921367.pdf)")
    LENGTH = 35.0
    WIDTH = 21.25
    HEIGHT = 12.5
    HOLE_SPACING = 30.5
    HOLE_D = 2.2
    # The drawing does not dimension the holes across the width; assumed
    # on the centre line.
    HOLE_FROM_SIDE = 21.25 / 2.0
    SUPPLY_V = (3.7, 5.2)
    CURRENT_MA = 70
    PEAK_CURRENT_MA = 150
    MASS_G = 5.0
    UNVERIFIED = ("HOLE_FROM_SIDE",)


class Holybro2216KV920:
    """Holybro 2216 KV920 motor (X500 V2 kit), 1045 propellers."""
    SOURCE = ("Holybro PX4 Development Kit X500 V2 specification "
              "(https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2.md): "
              "'Holybro 2216 KV920 Motor', mounting pattern 16x16mm, 1045 propellers, 500 mm wheelbase, "
              "16 mm carbon fiber tubes")
    MOUNT_PATTERN = (16.0, 16.0)
    MOUNT_SCREW = "M3"
    # "16x16mm" read as four M3 holes on a cross, opposite holes 16 mm apart
    # (the usual 22xx base); the specification does not draw it.
    MOUNT_LAYOUT = "cross"
    # Reseller listings, not a Holybro drawing: 28 mm can, 63 g.
    DIAMETER = 28.0
    HEIGHT = 32.0
    MASS_G = 63.0
    # Accepted by the motor mount's slots as well: 16 x 19 and 19 x 19 mm.
    OTHER_PATTERNS = ((16.0, 19.0), (19.0, 19.0))
    UNVERIFIED = ("MOUNT_LAYOUT", "DIAMETER", "HEIGHT", "MASS_G")


class Propeller1045:
    SOURCE = ("T-Motor T1045 propeller, Air Gear 450 II specification "
              "(https://store.tmotor.com/product/air-gear-450II-combo-set.html): 10\" x 4.5 (254 x 114.3 mm), 13 g")
    DIAMETER = 254.0
    MASS_G = 13.0
    UNVERIFIED = ()


class CarbonTube16:
    """Roll-wrapped carbon fibre arm tube."""
    SOURCE = ("Holybro X500 V2 specification (16 mm carbon fiber tubes), "
              "https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2.md")
    OD = 16.0
    # Wall thickness is a purchase choice (16 x 14 mm is common); check the
    # outer diameter with calipers, the clamps take 15.8-16.2 mm.
    ID = 14.0
    UNVERIFIED = ("ID",)


class HolybroM10GPS:
    """Holybro standard M10 (or M9N) GPS with IST8310 compass."""
    SOURCE = ("Holybro standard M10/M9N GPS overview "
              "(https://docs.holybro.com/gps-and-rtk-system/m8n-m9n-m10-gps/standard-m10-m9n-m8n-gps/overview.md)")
    V1 = (50.0, 14.4)   # diameter, height
    V2 = (51.0, 18.5)
    MASS_G = 36.8       # V2
    SUPPLY_V = (4.7, 5.2)
    CURRENT_MA = 200    # "less than 200 mA @ 5 V"
    CONNECTOR = "JST-GH-10P"
    UNVERIFIED = ()


class LipoEnvelope4S:
    """Design envelope (a requirement, not a datasheet) for the 4S LiPo:
    the tray and straps take any pack up to this size."""
    SOURCE = "Requirement of this design: 4S 4000-5000 mAh pack, at most 160 x 50 x 40 mm, about 550 g"
    MAX_SIZE = (160.0, 50.0, 40.0)
    NOMINAL_SIZE = (155.0, 48.0, 35.0)
    MASS_G = 550.0
    UNVERIFIED = ()


class SiliconeDamperBall:
    """Generic silicone gimbal damping ball (purchased), two snap necks."""
    SOURCE = ("No manufacturer drawing: typical listing for 'gimbal damping ball, 6.5 mm "
              "mounting hole, for 1.5-2 mm plates'")
    HOLE_D = 6.5
    PLATE_THICKNESS = 2.0
    PLATE_SPACING = 11.0
    BALL_D = 12.5
    UNVERIFIED = ("HOLE_D", "PLATE_THICKNESS", "PLATE_SPACING", "BALL_D")


class ArduinoUno:
    """Arduino UNO form factor (UNO R3/R4, UNO Q): 2.7 x 2.1 in board,
    holes from the corner at the power-jack/USB edge, bottom side."""
    SOURCE = ("Arduino UNO Rev3 reference design (https://docs.arduino.cc/hardware/uno-rev3), "
              "hole coordinates in mils: (550, 100), (600, 2000), (2600, 300), (2600, 1400)")
    BOARD = (68.58, 53.34)
    HOLES = ((13.97, 2.54), (15.24, 50.8), (66.04, 7.62), (66.04, 35.56))
    HOLE_D = 3.2
    # Not re-read from the Arduino files in this repository's sessions:
    # check against the UNO DXF before printing a mount that uses it.
    UNVERIFIED = ("HOLES", "HOLE_D")


ALL = (RaspberryPi5, RaspberryPi4, BlueRoboticsNavigator, LuxonisOAK1, BenewakeTFminiS,
       BenewakeTFLuna, Holybro2216KV920, Propeller1045, CarbonTube16, HolybroM10GPS,
       LipoEnvelope4S, SiliconeDamperBall, ArduinoUno)
