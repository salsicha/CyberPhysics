"""Datasheet footprints of the turret's purchased parts (helper, not a
part).

Same conventions as hardware/parts/common/footprints.py: millimetres, every
class cites its source and lists in ``UNVERIFIED`` the attributes the cited
document does not state; a printed part that uses one sets ``fit_test``.
They live here, not in common/footprints.py, only because that shared file
was frozen while this platform was added; ``ALL`` lists them.
"""


class ST3215:
    """Waveshare ST3215 (and Feetech STS3215) serial bus servo. Servo
    coordinates: X along the case from the output end (0), the shaft at X =
    SHAFT_X on the centre line across the width; the case thickness runs
    from the idler face to the output face."""
    SOURCE = ("Waveshare ST3215 dimension drawing "
              "(https://www.waveshare.com/img/devkit/accessories/ST3215-Servo/ST3215-Servo-details-size.jpg, "
              "on https://www.waveshare.com/st3215-servo.htm) and Feetech STS3215 outline and horn drawings "
              "(https://www.feetechrc.com/525603.html); ratings from https://www.waveshare.com/wiki/ST3215_Servo")
    LENGTH = 45.22
    WIDTH = 24.72
    THICKNESS = 32.0                # main body; 29 at the output end, 35 over the raised covers
    THICKNESS_OUTPUT_END = 29.0
    THICKNESS_MAX = 35.0
    SHAFT_X = 10.11
    HORN_FACES = 37.25              # outer face to outer face, both horns fitted
    HORN_D = 19.95                  # Feetech aluminium horn (Waveshare drawing: 19.2)
    HORN_HOLE_CIRCLE = 14.0         # 4 holes
    HORN_THREAD = "M3"              # Feetech horn: 4 x M3 tapped; flange 2.5, 4.5 overall
    HORN_FLANGE_T = 2.5
    SPLINE = "25T"
    # Body mounting holes, PA2.0 self-tapping (M2 x 6 in practice), at
    # +-10.25 mm across the width: output face at X = 18.41 and 39.11, idler
    # face at X = 18.41 and 42.86.
    FACE_HOLE_Y = 10.25
    OUTPUT_FACE_HOLES_X = (18.41, 39.11)
    IDLER_FACE_HOLES_X = (18.41, 42.86)
    IDLER_HUB_D = 6.0               # protrudes 4.1 mm, takes a PA3.0 x 5 screw
    MASS_G = 55.0
    STALL_TORQUE_NM = 2.94          # 30 kg.cm at 12 V
    STALL_A = 2.7
    SUPPLY_V = (6.0, 12.6)
    SPEED_S_PER_60 = 0.222          # at 12 V
    # Scaled from the drawings, not dimensioned: the raised label cover on
    # the output face (X 24-42, +-7.3 across, up to the horn plane), the two
    # 5264 connectors on the idler face (X 20.9-26.8, nearly the full width,
    # plugs inserted perpendicular to the face, about 11 mm with the plug),
    # the 20.5 mm hole spacing on the output face, the PA2.0 hole depth, and
    # whether the Waveshare horn's holes are M3 tapped.
    # The 29 mm thick output end is about 15.6 mm long (scaled); the horn's
    # 2.5 mm flange then clears the 32 mm body by only about 0.1 mm, so no
    # horn screw may reach past the flange.
    LABEL_X = (24.0, 42.0)
    LABEL_HALF_W = 7.3
    CONNECTOR_X = (20.9, 26.8)
    PLUG_LENGTH = 11.0
    OUTPUT_END_LENGTH = 15.6
    HORN_HUB_D = 9.0
    IDLER_HORN_FLANGE_T = 2.1
    UNVERIFIED = ("LABEL_X", "LABEL_HALF_W", "CONNECTOR_X", "PLUG_LENGTH", "HORN_THREAD", "OUTPUT_END_LENGTH",
                  "IDLER_HORN_FLANGE_T")


class WaveshareBusServoAdapterA:
    """Waveshare Bus Servo Adapter (A): half-duplex adapter for serial bus
    servos, UART header or USB-C, DC jack and screw terminal."""
    SOURCE = ("Waveshare Bus Servo Adapter (A) product page (https://www.waveshare.com/bus-servo-adapter-a.htm), "
              "wiki (https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)) and schematic")
    BOARD = (42.0, 33.0)
    HOLE_SPACING = (37.0, 28.0)
    HOLE_D = 2.5
    SUPPLY_V = (9.0, 12.6)
    SERVO_PORT_MAX_A = 6.0
    # Component height above the board (DC jack, terminal, connectors) is not
    # given; budgeted 12 mm.
    TOP_PARTS_H = 12.0
    UNVERIFIED = ("TOP_PARTS_H",)


class ArduinoUnoR4:
    """Arduino UNO R4 WiFi (ABX00087) and UNO Q (ABX00162): the UNO outline
    and holes, from the lower-left corner with the USB/power edge on the
    left."""
    SOURCE = ("Arduino UNO R4 WiFi datasheet ABX00087, section 13 "
              "(https://docs.arduino.cc/resources/datasheets/ABX00087-datasheet.pdf) and UNO Q datasheet ABX00162, "
              "section 12 (https://docs.arduino.cc/resources/datasheets/ABX00162-datasheet.pdf)")
    BOARD = (68.58, 53.34)
    HOLES = ((13.97, 2.54), (15.24, 50.8), (66.04, 7.62), (66.04, 35.56))
    HOLE_D = 3.2
    TOP_PARTS_H = 8.5
    # UNO Q: the drawing shows bottom features about 5 mm deep although the
    # text says under 2 mm: standoffs of at least 6 mm.
    BOTTOM_CLEARANCE = 6.0
    UNVERIFIED = ()


class ArducamB0385:
    """Arducam B0385, 1 MP OV9782 colour global-shutter USB camera board
    with an M12 lens."""
    SOURCE = ("Arducam B0385 datasheet V1.0 "
              "(https://www.uctronics.com/download/Amazon/B0385_OV9782_Global_Shutter_UVC_Camera_Datasheet.pdf), "
              "quick-start guide, and Arducam's STEP model (https://download.arducam.com/3D-Drawing/B0385.STEP)")
    BOARD = 38.0                    # square
    PCB_T = 1.74                    # from the STEP model
    HOLES_34 = 34.0                 # 4 x dia. 2.6 on a 34 mm square
    HOLE_34_D = 2.6
    HOLES_28 = 28.0                 # 4 x dia. 2.8 on a 28 mm square
    HOLE_28_D = 2.8
    LENS_FRONT = 16.5               # above the PCB front face (STEP)
    LENS_D = 14.0
    HFOV_DEG = 70.0
    # Side-entry 4-pin JST ZR on the back near the bottom edge (STEP): from
    # the board centre -10.8..-3.3 across, -15.4..-11.9 toward the bottom
    # edge, 4.5 mm off the back face; the plug enters from the bottom edge.
    CONNECTOR = ((-10.8, -3.3), (-15.4, -11.9), 4.5)
    VFOV_DEG = 46.0                 # computed from the sensor size and focal length, not stated
    UNVERIFIED = ("VFOV_DEG",)


class TripodNut:
    """1/4"-20 UNC hex nut for the tripod thread."""
    SOURCE = ("ASME B18.2.2 hex nuts: 1/4\"-20 width across flats 7/16 in, thickness 7/32 in; ISO 1222 tripod "
              "thread 1/4\"-20 UNC")
    ACROSS_FLATS = 11.11
    THICKNESS = 5.56
    UNVERIFIED = ()


ALL = (ST3215, WaveshareBusServoAdapterA, ArduinoUnoR4, ArducamB0385, TripodNut)
