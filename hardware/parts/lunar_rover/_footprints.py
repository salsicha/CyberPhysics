"""Footprints of the parts the lunar rover's printed parts mate with
(helper, not a part).

Same conventions as hardware/parts/common/footprints.py: millimetres, every
class cites its source and lists in ``UNVERIFIED`` the attributes the cited
document does not state; a printed part that uses one sets ``fit_test``.
They live here, not in common/footprints.py, because that shared file is
frozen while the platforms are added; ``ALL`` lists them for the footprint
test. The OAK-1 is common.footprints.LuxonisOAK1.
"""

REALSENSE_DATASHEET = ("RealSense D400 Series datasheet 337029-017 (Mar 2026) "
                       "(https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf)")


class CubeMarsAK70_10:
    """CubeMars AK70-10 KV100 robotic actuator (24/48 V, 10:1 planetary,
    integrated driver, CAN). Axis along u; u = 0 at the front face of the
    central boss, positive out of the actuator."""
    SOURCE = ("CubeMars AK70-10 KV100 product page (https://www.cubemars.com/product/ak70-10-kv100-robotic-actuator.html): "
              "24/48 V, rated 8.3 N m, peak 24.8 N m, 148/310 rpm rated and 240/480 rpm no-load at 24/48 V, 621 g, "
              "cross-roller bearing 2000 N dynamic / 2520 N static, CAN, 14-bit encoder, NTC, -20 to 50 C; "
              "2D drawing (https://www.cubemars.com/data/cms/202602/ak70-10-robotic-actuator-2d-drawing.pdf): "
              "89 x 50.25 mm, front boss 71 mm x 11.2 mm, rear step 77 mm x 6.5 mm, 8 x M3 x 10 deep on 83 mm and "
              "6 x M3 x 7 deep on 25 mm with a 20 H7 x 2 mm recess on the front, 8 x M3 x 3.5 deep on 83 mm at the rear")
    DIAMETER = 89.0
    LENGTH = 50.25
    FRONT_BOSS = (71.0, 11.2)            # diameter, length (front of the body face)
    REAR_STEP = (77.0, 6.5)
    MOUNT_PCD = 83.0                     # 8 x M3, 10 mm deep, on the body's front face (fixed)
    MOUNT_HOLES = 8
    MOUNT_THREAD_DEPTH = 10.0
    OUTPUT_PCD = 25.0                    # 6 x M3, 7 mm deep, on the central output flange
    OUTPUT_HOLES = 6
    OUTPUT_THREAD_DEPTH = 7.0
    OUTPUT_SPIGOT = (20.0, 2.0)          # 20 H7 recess, 2 mm deep
    RATED_TORQUE_NM = 8.3
    PEAK_TORQUE_NM = 24.8
    RATED_RPM_24V = 148.0
    NO_LOAD_RPM_24V = 240.0
    BEARING_DYNAMIC_N = 2000.0
    MASS_G = 621.0
    TEMP_RANGE_C = (-20.0, 50.0)
    # The drawing does not label which part turns: the central 6 x M3 flange
    # is taken as the output (planet carrier) and the 8 x M3 ring on the
    # body's front face as the fixed mount; the angular offset of the two
    # hole circles is not dimensioned either.
    OUTPUT_IS_CENTRAL_FLANGE = True
    HOLE_PHASE_DEG = 0.0
    UNVERIFIED = ("OUTPUT_IS_CENTRAL_FLANGE", "HOLE_PHASE_DEG")


class Extrusion2040:
    """20 x 40 mm aluminium extrusion, slot 6 (Misumi HFS5-2040 or any
    20-series B-type profile)."""
    SOURCE = ("Misumi HFS5 series 20 mm, 2040 profile (https://us.misumi-ec.com/pdf/fa/2012/p2_0511.pdf): "
              "20 x 40 mm, slot width 6 mm, two slots on each 40 mm face, one on each 20 mm face")
    SIZE = (20.0, 40.0)
    SLOT_W = 6.0
    UNVERIFIED = ()


class Extrusion2020:
    """20 x 20 mm aluminium extrusion, slot 6 (Misumi HFS5-2020)."""
    SOURCE = ("Misumi HFS5 series, 20 mm square (https://us.misumi-ec.com/pdf/fa/2012/p2_0511.pdf): "
              "20 x 20 mm, slot width 6 mm, centre hole 4.2 mm")
    SIZE = 20.0
    SLOT_W = 6.0
    UNVERIFIED = ()


class RealSenseD455:
    """Intel RealSense D455 (front hazard camera)."""
    SOURCE = (f"{REALSENSE_DATASHEET}: Table 3-53 (124 x 29 x 26 mm, 116 g), Figure 10-12 (two M4 on the back "
              "95 mm apart, max insertion 4 mm, 0.4 N m; 1/4\"-20 underneath, 9 mm), Table 3-49 (depth 87 x 58 deg, "
              "colour 90 x 65 deg), Table 4-12 (min-Z 350 mm at 848 x 480)")
    WIDTH = 124.0
    HEIGHT = 29.0
    DEPTH = 26.0
    MASS_G = 116.0
    BACK_M4_SPACING = 95.0
    BACK_M4_MAX_INSERTION = 4.0
    DEPTH_FOV_DEG = (87.0, 58.0)
    MIN_Z_MM = 350.0
    # Not dimensioned on Figure 10-12: the M4 pair drawn on the case's
    # centre lines.
    M4_CENTRED = True
    UNVERIFIED = ("M4_CENTRED",)


class LivoxMid360:
    """Livox Mid-360 3D lidar."""
    SOURCE = ("Livox Mid-360 User Manual v1.0 (2023.01), 'Dimensions', 'Mounting Notice' and 'Specifications' "
              "(https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Mid-360_User_Manual_EN.pdf): "
              "65 x 65 x 60 mm, 265 g, 4 x M3 x 5 deep on the bottom 48 x 36 mm apart, two 3.0 mm locating holes, "
              "FOV 360 x -7..52 deg, 9-27 V, 6.5 W (14 W at start-up in the cold), IP67; mount on a metal plate at "
              "least 3 mm thick and 10,000 mm2, 10 mm free around it; origin 47 mm above the bottom")
    SIZE = (65.0, 65.0, 60.0)
    CONNECTOR_OVERHANG = 8.0            # 73 mm across the M12 connector
    HOLES = (48.0, 36.0)                # 4 x M3, 5 mm deep
    HOLE_DEPTH = 5.0
    ORIGIN_HEIGHT = 47.0
    VFOV_DEG = (-7.0, 52.0)
    SUPPLY_V = (9.0, 27.0)
    POWER_W = 6.5
    MASS_G = 265.0
    HEATSINK_MIN = (3.0, 10000.0)        # plate thickness mm, area mm2
    # The 48 x 36 mm pattern is drawn on the body's centre lines but its
    # offset from the centre is not dimensioned.
    HOLES_CENTRED = True
    UNVERIFIED = ("HOLES_CENTRED",)


class SchneiderXB5EStop:
    """Schneider Harmony XB5AS8442 emergency stop, 40 mm red mushroom,
    turn to release, 1 NC, 22 mm fixing."""
    SOURCE = ("Schneider Electric XB5AS8442 product page "
              "(https://www.se.com/us/en/product/XB5AS8442/): 22 mm cut-out, 40 mm head, IP66/67/69K")
    HOLE_D = 22.5
    HEAD_D = 40.0
    # Panel thickness range and depth behind the panel from typical 22 mm
    # operators, not read from the XB5 drawing.
    PANEL_T = (1.0, 6.0)
    DEPTH_BEHIND = 45.0
    UNVERIFIED = ("PANEL_T", "DEPTH_BEHIND")


class AdafruitVEML7700:
    """Adafruit VEML7700 lux sensor breakout (product 4162), 0-120 klux."""
    SOURCE = ("Adafruit VEML7700 Lux Sensor, product 4162 (https://www.adafruit.com/product/4162): "
              "0 to about 120,000 lux, I2C, STEMMA QT")
    MAX_LUX = 120000.0
    # Board outline and holes: listings give different sizes (Adafruit's
    # page and distributors); the pocket is sized for up to 26 x 18 mm.
    BOARD = (25.4, 17.8, 4.0)
    UNVERIFIED = ("BOARD",)


class VesaMisD:
    """VESA flat display mounting interface (MIS-D), used by mini PCs."""
    SOURCE = ("VESA Flat Display Mounting Interface Standard (FDMI), MIS-D: four M4 screws on a 75 x 75 or "
              "100 x 100 mm square (https://vesa.org/vesa-standards/)")
    PATTERNS = (75.0, 100.0)
    SCREW = "M4"
    UNVERIFIED = ()


class DinRailTS35:
    """Top-hat DIN rail, 35 x 7.5 mm (EN 60715), fixed with M4 screws
    through its slots."""
    SOURCE = "EN 60715 / IEC 60715 top-hat rail TS35, 35 x 7.5 mm (https://webstore.iec.ch/publication/3628)"
    WIDTH = 35.0
    HEIGHT = 7.5
    SCREW = "M4"
    UNVERIFIED = ()


class LiFePO4Pack24V:
    """Design envelope (a requirement, not a datasheet): the traction battery."""
    SOURCE = ("Requirement of this design: 8S LiFePO4, 25.6 V nominal, 25 Ah (640 Wh, the simulator's 650 Wh), "
              "with BMS, at most 260 x 180 x 140 mm and 7 kg")
    MAX_SIZE = (260.0, 180.0, 140.0)
    MASS_G = 7000.0
    NOMINAL_V = 25.6
    FULL_V = 29.2
    CAPACITY_WH = 640.0
    UNVERIFIED = ()


class SolarPanel50W:
    """Design envelope (a requirement): removable 50 W panel on the body."""
    SOURCE = ("Requirement of this design: 40-50 W high-efficiency monocrystalline panel (the simulator charges at "
              "38 W in full sun), at most 520 x 400 mm, aluminium frame 25-35 mm deep with mounting holes in its "
              "bottom flange, about 3 kg")
    MAX_SIZE = (520.0, 400.0)
    FRAME_DEPTH = (25.0, 35.0)
    FRAME_LIP = 2.0                       # the frame's flange thickness under the clamp
    MASS_G = 3000.0
    UNVERIFIED = ("FRAME_LIP",)


class GnssGroundPlate:
    """Design envelope (a requirement): steel disc under a magnetic-base
    GNSS antenna (u-blox ANN-MB class), also its ground plane."""
    SOURCE = "Requirement of this design: 100 mm diameter, 1.5 mm steel disc (magnet holds the antenna)"
    DISC = (100.0, 1.5)
    UNVERIFIED = ()


ALL = (CubeMarsAK70_10, Extrusion2040, Extrusion2020, RealSenseD455, LivoxMid360, SchneiderXB5EStop,
       AdafruitVEML7700, VesaMisD, DinRailTS35, LiFePO4Pack24V, SolarPanel50W, GnssGroundPlate)
