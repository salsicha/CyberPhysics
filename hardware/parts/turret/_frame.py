"""Turret layout: the numbers every turret part shares (helper, not a part).

Pure Python. Turret frame, millimetres: origin on the pan axis at the
surface the pan base stands on (the base plate's top face, or the RACECAR
Turret's plate), x forward (the camera's view at pan 0), y left, z up.

Kinematics as systems/racecar_turret/urdf/racecar_turret.urdf.xacro: pan
first, then tilt. The pan servo stands upright on the pan axis, output up;
its horn turns the tilt bracket, which holds the tilt servo lying on its
side with its shaft across (y) on the tilt axis, TILT_Z above the mounting
surface (the URDF's 0.045 + 0.055 m above turret/base_link, which is the
underside of a 5 mm plate). The camera yoke hangs on both tilt horns and
carries the camera 35 mm forward of the tilt axis.
"""
import math

from hardware.parts.turret._footprints import ST3215 as S

PLATE_T_URDF = 5.0                 # turret/base_link is the 5 mm plate's underside
TILT_Z = 100.0 - PLATE_T_URDF       # 95 above the mounting surface
CAMERA_X = 35.0
PAN_LIMIT_DEG = 90.0
TILT_RANGE_DEG = (math.degrees(-0.65), math.degrees(0.75))

# ST3215 envelope about its case centre: thickness +-16 (body), +-14.5 at
# the output end; horn outer faces at +-HORN_FACES / 2.
HALF_T = S.THICKNESS / 2
HALF_T_END = S.THICKNESS_OUTPUT_END / 2
HORN_FACE = S.HORN_FACES / 2        # 18.625
LABEL_TOP = 18.4                    # raised label cover, scaled from the Waveshare drawing

# --------------------------------------------------------------------- pan servo
# Upright, output up, the case extending backward (-x) from the pan axis.
PAN_CASE_Z = 36.0                   # case centre height
PAN_BOTTOM_Z = PAN_CASE_Z - HALF_T  # 20: room for the plugs under it
PAN_HORN_Z = PAN_CASE_Z + HORN_FACE  # 54.6


def pan_case_x(servo_x):
    """Turret x of a point at servo X (from the output end) on the pan servo."""
    return S.SHAFT_X - servo_x


BASE_BOLT = 25.0                    # pan base to its mount: M3 on a 50 x 50 mm square
BASE_FLANGE_T = 4.0

# --------------------------------------------------------------------- tilt bracket
# The pan hub (on the pan horn) and the tilt bracket (the cradle for the
# tilt servo) are two parts so that both print without supports; four M3
# screws join them.
HUB_T = 3.0                         # on the pan horn: M3 x 5 enter the 2.5 mm horn flange 2 mm, tips inside it
HUB_D = 21.0
PAN_HORN_HOLES_DEG = 0.0            # hole pattern on the axes, clear of the joint inserts
BRACKET_FLOOR_T = 6.0
TILT_FLOOR_TOP = TILT_Z - S.WIDTH / 2 - 0.2      # 82.44
COLUMN = (-16.0, 4.25, 17.5)         # x0, x1, half width: clear of the camera when it tilts down
FLOOR_FRONT_HALF_W = 17.5           # 1.1 mm inside the camera yoke's arms
COLUMN_TOP = TILT_FLOOR_TOP - BRACKET_FLOOR_T    # 76.44
JOINT_INSERTS = ((-12.0, 13.2), (-12.0, -13.2), (1.0, 13.2), (1.0, -13.2))
FLOOR_FRONT_X = 6.0                 # the tilt bracket's floor ends here
KEY_SHAFT_D = 5.9                   # over each pan horn screw, for the hex key (head 5.5)
WALL_T = 4.0
WALL_T_OUTPUT = 6.0                 # the +y wall is thicker: its screw counterbores clear the label relief
WALL_X = (-36.0, -17.0)             # side walls cover the far end of the tilt servo


def tilt_case_x(servo_x):
    """Turret x of a point at servo X on the tilt servo (case toward -x)."""
    return S.SHAFT_X - servo_x


# --------------------------------------------------------------------- camera yoke
# Yoke frame: origin on the tilt axis, x along the camera's view at tilt 0.
YOKE_T = 3.0                        # hub thickness: M3 x 5 end inside the horn flanges
YOKE_ARM_T = 5.0                    # the arms are thicker beyond the hub screws
YOKE_ARM_THICK_X = 10.5
YOKE_HUB_D = 20.5
YOKE_HORN_HOLES_DEG = 45.0
ARM_HALF_H = 7.0
CROSSBAR_X = (17.5, 22.5)
CROSSBAR_Z = (-10.0, 20.0)          # short below the axis: it swings over the pan hub when tilted down
CROSSBAR_INSERTS = ((8.0, 14.0), (-8.0, 14.0), (8.0, -4.0), (-8.0, -4.0))   # (y, z) M3 x 3, camera plates
IDLER_HUB_REAR_R = 10.0            # the idler hub's radius behind the axis: clear of the servo's plugs
CAMERA_PLATE_T = 3.0

# --------------------------------------------------------------------- cable path
# The tilt servo's bus cable and the camera cable cross the pan joint as a
# loose loop from the pan base's rear hook to the bracket's rear hook.
BASE_HOOK = (-40.0, 0.0, 40.0)
BRACKET_HOOK_LOCAL = (-38.0, 0.0, 70.0)


def horn_holes(angle0=45.0):
    """The four horn holes on the 14 mm circle (local 2D)."""
    r = S.HORN_HOLE_CIRCLE / 2
    return [(r * math.cos(math.radians(angle0 + 90 * k)), r * math.sin(math.radians(angle0 + 90 * k)))
            for k in range(4)]


# --------------------------------------------------------------------- foam dart launcher
# The launcher core (hardware/parts/turret/_launcher.py) is built in its
# own frame: origin on the bore at the chamber's rear face, x along the
# bore, z toward the plunger tube's side. On the turret it sits on the
# camera yoke, above the camera, upright and parallel to the camera's
# axis: its origin at LAUNCHER_ORIGIN in the yoke frame (origin on the tilt
# axis), no rotation. The bore is then BORE_Z above the camera's optical
# axis, on its centre line.
BORE_Z = 48.0
LAUNCHER_ORIGIN = (12.0, 0.0, BORE_Z)

# Yoke pad: the breech's pedestal stands on it, two M3 from below into
# inserts in the pedestal.
YOKE_PAD = (10.0, CROSSBAR_X[1], 18.0, 16.5, 20.5)   # x0, x1, half width, z0, z1
PAD_SCREWS = ((14.5, 10.0), (14.5, -10.0))           # (x, y), yoke frame


def to_yoke(p):
    """Launcher-frame point -> yoke frame."""
    return (p[0] + LAUNCHER_ORIGIN[0], p[1] + LAUNCHER_ORIGIN[1], p[2] + LAUNCHER_ORIGIN[2])


def from_yoke(p):
    return (p[0] - LAUNCHER_ORIGIN[0], p[1] - LAUNCHER_ORIGIN[1], p[2] - LAUNCHER_ORIGIN[2])

# The breech's pedestal on the yoke pad (launcher frame): x0, x1, y0, y1,
# z0 (the pad's top), the two pad screws (from below, into inserts).
PEDESTAL = (YOKE_PAD[0] - LAUNCHER_ORIGIN[0], 72.0, -13.6, 13.6, YOKE_PAD[4] - LAUNCHER_ORIGIN[2],
            tuple((x - LAUNCHER_ORIGIN[0], y) for x, y in PAD_SCREWS))


def launcher_to_turret(shape):
    """Launcher-frame shape -> the turret frame (tilt 0, pan 0)."""
    from hardware.parts.common import cad
    return cad.translate(shape, (LAUNCHER_ORIGIN[0], LAUNCHER_ORIGIN[1], LAUNCHER_ORIGIN[2] + TILT_Z))
