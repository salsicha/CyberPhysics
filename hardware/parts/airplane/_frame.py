"""Airplane layout: the numbers the airplane's printed parts share.

Pure Python. Body frame FLU in millimetres: x forward, y left, z up. The
origin is on the centre line, on the top face of the payload-bay floor, at
the Skywalker X8's centre of gravity mark (430-440 mm behind the nose).
Modules whose name starts with "_" are helpers, not printed parts.

The printed parts are bonded into a purchased EPO foam airframe (reference:
Skywalker X8, see hardware/airplane.md): a battery tray ahead of the CG, the
nadir OAK-1 frame over a cut-out in the belly at the CG, and the avionics
deck with the Pi 5 + Navigator stack behind it. GPS, antennas and the
pitot are outside, on the skin. The X8's bay is not dimensioned by its
maker: hold every part in the real bay before gluing (fit test).
"""
from hardware.parts.airplane._footprints import Lipo6SEnvelope, SkywalkerX8
from hardware.parts.common.footprints import LuxonisOAK1, RaspberryPi5

NOSE_X = sum(SkywalkerX8.CG_FROM_NOSE) / 2          # 435 mm ahead of the origin
MOTOR_X = NOSE_X - SkywalkerX8.FUSELAGE_LENGTH       # -355: pusher motor at the tail

# --------------------------------------------------------------------- deck
DECK_X = (-226.0, -46.0)            # behind the camera frame (which ends at x = -44)
DECK_HALF_W = 52.0
DECK_T = 3.0
BOSS_D = 9.0
BOSS_TOP = 8.0                       # insert bosses for the stack grommets
# Pi 5 + Navigator stack on four TPU grommet pairs; Pi centre, USB/Ethernet
# end forward (toward the camera, whose USB 3 cable plugs in there).
PI_CENTER = (-108.0, -18.0)
STACK_GROMMETS = tuple((PI_CENTER[0] + dx, PI_CENTER[1] + dy) for dx in (-48.5, 48.5) for dy in (20.0, -20.0))
GROMMET_HOLE = 6.2
GROMMET_STACK = (2.0, 3.0, 2.0)      # lower flange, tray thickness (groove), upper flange
TRAY_T = 3.0
TRAY_Z0 = BOSS_TOP + GROMMET_STACK[0]                # 10
PI_BOSS_H = 4.0
PI_Z0 = TRAY_Z0 + TRAY_T + PI_BOSS_H                 # 17: underside of the Pi PCB
# Module bays: name -> ((centre x, y), (size x, y), tie direction). Each is
# held by one zip tie through two printed tie bridges, the tie running over
# the module: power modules and the receiver in a row along the deck's left
# strip, the telemetry radio across its rear. Receiver, radio and BEC sizes
# are envelopes (they vary by model), not datasheets.
MODULES = {
    "rc_receiver": ((-199.0, 32.0), (40.0, 24.0), "y"),
    "power_sense": ((-163.0, 32.0), (24.0, 19.0), "y"),
    "supply_5v": ((-128.5, 32.0), (37.0, 18.0), "y"),
    "servo_bec": ((-88.0, 32.0), (36.0, 22.0), "y"),
    "airspeed": ((-57.5, 32.0), (17.0, 23.0), "y"),
    "telemetry_radio": ((-198.0, -20.0), (30.0, 56.0), "x"),
}


def pi_holes_body():
    """Pi 5 mounting holes in the body frame (USB end forward)."""
    x0 = PI_CENTER[0] - RaspberryPi5.BOARD[0] / 2
    y0 = PI_CENTER[1] - RaspberryPi5.BOARD[1] / 2
    return tuple((x0 + hx, y0 + hy) for hx, hy in RaspberryPi5.HOLES)


# --------------------------------------------------------------------- camera
# OAK-1 lens down, USB-C end toward the tail (image top toward the nose:
# CAMERA_MOUNT_YAW_DEG = 0.0 in systems/airplane/config/blueos.env), on four
# silicone damper balls hanging from a frame bonded around the belly
# cut-out. The lens sits LENS_Z above the floor's top face, so it is
# recessed (belly thickness + LENS_Z) above the belly skin for belly landings.
CAMERA_X = 0.0                       # centre of the OAK-1 body along x
FLANGE_T = 3.0                       # the frame's flange, bonded to the floor
LENS_Z = FLANGE_T + 2.0              # 5: the camera body clears the flange
CAMERA_BACK_Z = LENS_Z + LuxonisOAK1.DEPTH            # 32.8
CRADLE_T = 4.0
DAMPER_TAB_T = 2.0
DAMPER_SPACING = 11.0
CRADLE_Z = (CAMERA_BACK_Z, CAMERA_BACK_Z + CRADLE_T)   # (32.8, 36.8)
FRAME_DECK_T = 4.0
FRAME_DECK_Z = (CRADLE_Z[1] + DAMPER_SPACING, CRADLE_Z[1] + DAMPER_SPACING + FRAME_DECK_T)  # (47.8, 51.8)
DAMPERS = tuple((CAMERA_X + dx, dy) for dx in (-25.0, 25.0) for dy in (32.0, -32.0))
FRAME_HALF = (44.0, 52.0)            # frame and flange outline, x and y half sizes
PILLAR = 8.0                         # square pillars at the frame corners


def pillar_xy():
    """Centres of the four pillars (and of the deck screws)."""
    return tuple((CAMERA_X + sx * (FRAME_HALF[0] - PILLAR / 2), sy * (FRAME_HALF[1] - PILLAR / 2))
                 for sx in (-1, 1) for sy in (-1, 1))


FOV_MARGIN_DEG = 2.0
LENS_APERTURE_R = 7.0                # budget for the lens window (not on the OAK-1 drawing)


def lens_point():
    """Lens centre: 32.4 mm from the USB-C end, which faces the tail."""
    return (CAMERA_X - LuxonisOAK1.HEIGHT / 2 + LuxonisOAK1.LENS_FROM_USB_END, 0.0, LENS_Z)


def cutout_half():
    """Half sizes (x, y) of the belly cut-out around the lens: the camera
    view (+ margin) at the outer skin of the thickest belly, plus 2 mm."""
    import math
    depth = SkywalkerX8.BELLY_T_MAX + LENS_Z
    hx = LENS_APERTURE_R + depth * math.tan(math.radians(LuxonisOAK1.VFOV_DEG / 2 + FOV_MARGIN_DEG)) + 2.0
    hy = LENS_APERTURE_R + depth * math.tan(math.radians(LuxonisOAK1.HFOV_DEG / 2 + FOV_MARGIN_DEG)) + 2.0
    return round(hx, 1), round(hy, 1)


def tripod_x():
    return CAMERA_X - LuxonisOAK1.HEIGHT / 2 + LuxonisOAK1.TRIPOD_FROM_USB_END


# --------------------------------------------------------------------- battery
BATTERY_TRAY_X = (52.0, 277.0)       # 225 mm: the pack slides for the CG
BATTERY_TRAY_HALF_W = 33.0
BATTERY_TRAY_T = 3.0
BATTERY_RAIL_H = 12.0
BATTERY_RAIL_T = 4.0
BATTERY_FACE_Z = BATTERY_TRAY_T      # the pack (on its anti-slip pad) sits on this
BATTERY_HALF_W = Lipo6SEnvelope.MAX_SIZE[1] / 2 + 3.0   # pad and strap room between the rails

# --------------------------------------------------------------------- outside
# Skywalker does not dimension the skin: these are where the assembly check
# puts the outside parts for the CG estimate. The GPS goes on the left wing,
# 250 mm out, at least 150 mm from the OAK-1's USB 3 cable, the battery
# leads and the ESC and motor at the tail; the antennas on the fuselage top
# behind the hatch.
TOP_SKIN_Z = 70.0                    # fuselage top over the deck (estimate)
WING_SKIN_Z = 30.0                   # wing top 250 mm out (estimate)
GPS_XY = (-100.0, 250.0)
ANTENNA_XY = (-200.0, 0.0)
PITOT_X = NOSE_X - 5.0
PITOT_Z = 20.0
