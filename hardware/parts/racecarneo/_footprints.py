"""Datasheet footprints of the RACECAR Neo's purchased parts (helper, not a
part).

Same conventions as hardware/parts/common/footprints.py: millimetres, every
class cites its source and lists in ``UNVERIFIED`` the attributes the cited
document does not state; a printed part that uses one sets ``fit_test``.
They live here, not in common/footprints.py, only because that shared file
was frozen while these platforms were added; ``ALL`` lists them so the
footprint test can cover them when they move.
"""


class RplidarC1:
    """Slamtec RPLIDAR C1: a 55.6 mm square base with the round optical
    enclosure on top."""
    SOURCE = ("Slamtec RPLIDAR C1 datasheet v1.1, Figure 4-1 'RPLIDAR C1 Mechanical Dimensions' and "
              "section 'Coordinate System Definition' "
              "(https://cdn.shopify.com/s/files/1/0058/3145/8904/files/SLAMTEC_rplidar_datasheet_C1_v1.1_en.pdf); "
              "specification page https://www.slamtec.com/en/c1/spec")
    BASE = 55.6                     # square base, side
    BASE_HEIGHT = 23.1
    HEIGHT = 41.3
    SCAN_HEIGHT = 29.8              # laser receiving and emitting height above the bottom face
    HOLE_SQUARE = 43.0              # 4 x M2.5 on a 43 x 43 mm square, centred
    HOLE_THREAD = "M2.5"
    HOLE_DEPTH = 4.0                # "should be no longer than 4 mm" into the base
    MIN_RANGE = 50.0                # "blind range of only 0.05 meters", range 0.05-12 m
    MAX_RANGE = 12000.0
    SAMPLE_RATE_HZ = 5000
    SCAN_RATE_HZ = 10
    ANGULAR_RESOLUTION_DEG = 0.72
    MASS_G = 110.0
    SUPPLY_V = (4.8, 5.2)
    CURRENT_MA = (230, 260)         # typical, max at 10 Hz; 800 mA at start
    # Drawn but not dimensioned: the optical enclosure diameter (about 49 mm
    # scaled from the drawing) and the base corner radius.
    ENCLOSURE_D = 49.0
    CORNER_R = 3.0
    UNVERIFIED = ("ENCLOSURE_D", "CORNER_R")


class RealSenseD435i:
    """Intel RealSense D435/D435i peripheral case. Frames as in the
    realsense-ros description: the camera_link (depth origin, left imager)
    relative to the centre of the bottom 1/4"-20 thread."""
    SOURCE = ("RealSense D400 Series datasheet 337029-017 (Mar 2026), Table 3-52, Figure 10-9, Table 4-20, "
              "Table 3-49 "
              "(https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf); "
              "realsense-ros _d435.urdf.xacro "
              "(https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_description/urdf/_d435.urdf.xacro)")
    WIDTH = 90.0
    HEIGHT = 25.0
    DEPTH = 25.05
    MASS_G = 75.0
    # 1/4"-20 thread in the bottom, centred across the width; two M3 on the
    # back, 45 mm apart, max insertion 3 mm, 0.4 N m.
    BACK_M3_SPACING = 45.0
    BACK_M3_DEPTH = 3.0
    LEFT_IMAGER_FROM_SCREW = 17.5   # camera_link is this far to the left (+y) of the 1/4"-20 centre line
    # From realsense-ros, not the datasheet: the 1/4"-20 centre is 14.9 mm
    # behind the front face, and camera_link 4.3 mm behind the front face
    # (zero depth 4.2 mm behind the glass, glass 0.1 mm behind the face).
    SCREW_FROM_FRONT = 14.9
    LINK_FROM_FRONT = 4.3
    LINK_HEIGHT = HEIGHT / 2.0
    SCREW_DEPTH = 6.0               # tripod thread depth: not stated for the D43x
    COLOR_FOV_DEG = (69.0, 42.0)
    DEPTH_FOV_DEG = (87.0, 58.0)
    USB = "USB-C on the left end of the case"
    UNVERIFIED = ("SCREW_FROM_FRONT", "LINK_FROM_FRONT", "SCREW_DEPTH")


class RealSenseD455:
    """Intel RealSense D455 (D455f, D456 the same case)."""
    SOURCE = ("RealSense D400 Series datasheet 337029-017 (Mar 2026), Table 3-53, Figure 10-12, Table 4-21, "
              "Table 3-49 "
              "(https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf); "
              "realsense-ros _d455.urdf.xacro")
    WIDTH = 124.0
    HEIGHT = 29.0
    DEPTH = 26.0
    MASS_G = 116.0
    BACK_M4_SPACING = 95.0          # two M4 on the back, max insertion 4 mm
    SCREW_FROM_FRONT = 15.8         # 1/4"-20 centre from the front face (reference dimension)
    SCREW_DEPTH = 9.0               # 1/4"-20 max insertion depth
    LEFT_IMAGER_FROM_SCREW = 47.5   # +-0.1
    # USB-C on the BOTTOM face, 37.01 mm to the right (-y) of the 1/4"-20
    # centre line, about 4 mm from the back face, with 2 x M2 cable-lock holes.
    USB_FROM_SCREW = 37.01
    USB_FROM_BACK = 4.06
    LINK_FROM_FRONT = 4.65          # realsense-ros: zero depth 4.55 mm behind the glass, glass 0.1 mm in
    LINK_HEIGHT = 14.5              # realsense-ros: half the case height
    COLOR_FOV_DEG = (90.0, 65.0)
    DEPTH_FOV_DEG = (87.0, 58.0)
    UNVERIFIED = ("LINK_FROM_FRONT", "LINK_HEIGHT")


class JetsonOrinNanoDevKit:
    """NVIDIA Jetson Orin Nano (Super) Developer Kit, carrier P3768, used
    without its plastic stand."""
    SOURCE = ("NVIDIA Jetson Orin Nano Developer Kit Carrier Board Specification SP-11324-001 v1.3, "
              "Figures 4-1 and 4-2, sections 2-3 "
              "(https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/"
              "jetson_orin_nano_devkit_carrier_board_specification_sp.pdf)")
    CARRIER = (100.0, 79.0)          # +-0.13
    PCB_T = 1.57
    TOP_PARTS_H = 16.7               # max
    BOTTOM_PARTS_H = 4.3             # max
    DEVKIT = (103.0, 90.5, 34.77)    # with the stand, module and fan; +-0.2, +-1.09
    MASS_G = 175.0
    DC_INPUT_V = (9.0, 20.0)         # 5.5/2.5 mm jack, centre positive, 3.5 A max; 19 V supply included
    UART_PINS = "40-pin header pin 8 UART1_TXD, pin 10 UART1_RXD, 3.3 V"
    # The specification draws the four carrier holes but does not dimension
    # them; scaled from Figure 4-1 they are the Jetson Nano / Xavier NX
    # developer kit pattern: 86 x 58 mm, 4.0 and 90.0 mm from the DC-jack
    # end, 17.0 mm from the I/O edge and 3.9 mm from the opposite edge,
    # about 2.7 mm diameter (M2.5). The fan top is about 28.3 mm above the
    # carrier's top face.
    HOLES_FROM_JACK_END = (4.0, 90.0)
    HOLES_FROM_IO_EDGE = (17.0, 75.1)
    HOLE_D = 2.7
    FAN_TOP_ABOVE_PCB = 28.3
    UNVERIFIED = ("HOLES_FROM_JACK_END", "HOLES_FROM_IO_EDGE", "HOLE_D", "FAN_TOP_ABOVE_PCB")


class AdafruitBNO085:
    """Adafruit 9-DOF orientation IMU fusion breakout, BNO085 (product 4754)."""
    SOURCE = ("Adafruit product 4754 (https://www.adafruit.com/product/4754) and its fab print in the learn guide "
              "downloads (https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/downloads)")
    BOARD = (25.4, 22.86)            # 1.00 x 0.90 in
    HOLE_SPACING = (20.32, 17.78)    # 0.80 x 0.70 in, centred
    THICKNESS = 4.6
    MASS_G = 2.5
    # Not dimensioned on the fab print: Adafruit's usual 2.5 mm holes for M2.5.
    HOLE_D = 2.5
    UNVERIFIED = ("HOLE_D",)


class Vesc6MkVI:
    """Trampa VESC 6 MkVI in its CNC aluminium heatsink housing."""
    SOURCE = "Trampa VESC 6 MkVI product page (https://trampa.co.uk/product/vesc6-mk6/)"
    SIZE = (75.0, 70.0, 18.0)
    CURRENT_A = (80, 120)            # continuous, burst
    SUPPLY_V = (9.0, 48.0)           # 3S-12S on trampa.co.uk (trampaboards.com says 11.1-60 V)
    BEC = "5 V 1 A, 3.3 V 0.5 A"
    MASS_G = 232.0                   # trampaboards.com listing
    # No hole dimensions are published: the tray holds it with two straps.
    UNVERIFIED = ("MASS_G",)


class SchneiderXB5AS8442:
    """Schneider Electric Harmony XB5 emergency stop, 40 mm mushroom, turn to
    release, 1 NC, 22 mm."""
    SOURCE = "Schneider Electric XB5AS8442 datasheet (https://www.se.com/us/en/product/download-pdf/XB5AS8442)"
    CUTOUT_D = 22.4                  # 22.3 +0.4/0, 22.5 recommended
    PANEL_T = (1.0, 6.0)
    MUSHROOM_D = 40.0
    DEPTH = 82.0                     # overall depth
    PITCH = (40.0, 30.0)
    CONTACT = "1 NC, positive opening (IEC 60947-5-1 Annex K); Ith 10 A, DC-13 0.55 A at 125 V"
    # How the 82 mm splits between the head above the panel and the body
    # under it is not stated; budgeted 35 mm above, 50 mm below.
    HEAD_H = 35.0
    BODY_H = 50.0
    BODY_D = 30.0
    UNVERIFIED = ("HEAD_H", "BODY_H", "BODY_D")


ALL = (RplidarC1, RealSenseD435i, RealSenseD455, JetsonOrinNanoDevKit, AdafruitBNO085, Vesc6MkVI,
       SchneiderXB5AS8442)
