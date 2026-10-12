"""Aerodrone frame layout: the numbers every aerodrone part shares.

Pure Python. Body frame FLU in millimetres: x forward, y left, z up; the
origin is the hub centre on the plane of the arm-tube axes. Modules whose
name starts with "_" are helpers, not printed parts.

Frame: 500 mm motor-to-motor (X500 class), quad X, four 16 mm carbon arm
tubes clamped between two printed hub plates; 2216-size ~920 KV motors,
10 x 4.5 in propellers, 4S LiPo, about 2.1 kg take-off mass.

Optional payload: the foam dart launcher of hardware/parts/turret/_launcher.py
hangs under the hub pointing straight down, its bore on the vertical axis
through the hub centre (and the centre of mass). In that configuration the
battery rides on a deck above the Pi stack and taller legs carry the
aircraft; see "Foam dart launcher (optional payload)" below.
"""
import math

from hardware.parts.common.footprints import (
    BenewakeTFminiS,
    CarbonTube16,
    Holybro2216KV920,
    LuxonisOAK1,
    RaspberryPi5,
)

WHEELBASE = 500.0
MOTOR_R = WHEELBASE / 2.0
# ArduPilot quad X (FRAME_CLASS 1, FRAME_TYPE 1, systems/aerodrone/config/
# arducopter.params): output -> (arm angle, degrees CCW from the nose;
# propeller spin seen from above).
MOTORS = {
    1: (-45.0, "CCW"),   # front right
    2: (135.0, "CCW"),   # rear left
    3: (45.0, "CW"),     # front left
    4: (-135.0, "CW"),   # rear right
}
ARM_ANGLES = (45.0, 135.0, -135.0, -45.0)

# Arm tubes
TUBE_OD = CarbonTube16.OD
TUBE_BORE = TUBE_OD + 0.2           # clamp bore; the clamp closes 0.5 mm onto the tube
TUBE_R0 = 54.0                      # inner end, against the hub stop
TUBE_R1 = 234.0                     # outer end, against the motor-mount stop
TUBE_LENGTH = TUBE_R1 - TUBE_R0     # 180 mm, cut square

# Hub plates: 4 mm plates with 30 mm between them; each carries half of
# every arm clamp ("block"), 1 mm apart when tightened.
PLATE_T = 4.0
HUB_GAP = 30.0
BOT_Z = (-HUB_GAP / 2 - PLATE_T, -HUB_GAP / 2)   # (-19, -15)
TOP_Z = (HUB_GAP / 2, HUB_GAP / 2 + PLATE_T)     # (15, 19)
CLAMP_GAP = 1.0
PLATE_HALF = 75.0                   # central square 150 x 150
EAR_LENGTH = 110.0                  # plate ears along the arms, from the centre
EAR_WIDTH = 36.0
BLOCK_R = (50.0, 106.0)             # clamp block along the arm (tube stop 50-54)
BLOCK_W = 32.0
CLAMP_BOLT_R = (64.0, 94.0)         # two pairs of M3 x 25 per arm
CLAMP_BOLT_LAT = 12.0
LEG_BOLT_R = (68.0, 80.0)           # landing-leg M3 inserts, on the arm axis

# Inserts in the underside of the bottom plate (body x, y)
BATTERY_TRAY_INSERTS = ((-45.0, 25.0), (-45.0, -25.0), (35.0, 25.0), (35.0, -25.0))
NOSE_INSERTS = ((56.0, 30.0), (56.0, -30.0), (70.0, 30.0), (70.0, -30.0))
POST_BOSS_D = 9.0
POST_BOSS_H = 5.0

# Pi 5 + Navigator stack on the top plate: Pi centre, USB/Ethernet end forward.
PI_CENTER = (20.0, 0.0)
STACK_GROMMETS = tuple((PI_CENTER[0] + dx, dy) for dx in (-48.5, 48.5) for dy in (20.0, -20.0))
GROMMET_HOLE = 6.2
GROMMET_STACK = (2.0, 3.0, 2.0)     # lower flange, tray thickness (groove), upper flange
TRAY_T = 3.0
TRAY_Z0 = TOP_Z[1] + GROMMET_STACK[0]            # 21
PI_BOSS_H = 4.0
PI_Z0 = TRAY_Z0 + TRAY_T + PI_BOSS_H             # 28: underside of the Pi PCB


def pi_holes_body():
    """Pi 5 mounting holes in the body frame (USB end forward)."""
    x0 = PI_CENTER[0] - RaspberryPi5.BOARD[0] / 2
    y0 = PI_CENTER[1] - RaspberryPi5.BOARD[1] / 2
    return tuple((x0 + hx, y0 + hy) for hx, hy in RaspberryPi5.HOLES)


# GPS mast at the rear of the top plate
MAST_XY = (-57.0, 0.0)              # the flange's bolt circle (three inserts in the top plate)
MAST_FLANGE_D = 36.0
# The column stands behind the bolt circle, over the plate's rear edge: a
# battery on the deck (launcher configuration) then reaches its balance
# point in front of it.
MAST_COLUMN_XY = (-85.0, 0.0)
MAST_BOLT_R = 13.5
MAST_BOLT_ANGLES = (0.0, 120.0, 240.0)
MAST_HEIGHT = 210.0                 # flange underside to the GPS seat: 150 mm from a battery on the deck


def mast_bolts_body():
    return tuple((MAST_XY[0] + MAST_BOLT_R * math.cos(math.radians(a)),
                  MAST_XY[1] + MAST_BOLT_R * math.sin(math.radians(a))) for a in MAST_BOLT_ANGLES)


# Sensor nose under the front of the bottom plate. NOSE_SHIFT moves the
# camera and the rangefinder forward so that the OAK-1's view clears the
# launcher's barrel (assembly check); the nose bolts to the same inserts.
NOSE_SHIFT = 25.0
NOSE_X = (50.0, 160.0 + NOSE_SHIFT)
NOSE_HALF_W = 40.0
NOSE_Z = (BOT_Z[0] - 5.0, BOT_Z[0])             # (-24, -19)
# Camera stage on four silicone damper balls, OAK-1 USB-C end toward the tail
# (image top toward the nose: CAMERA_MOUNT_YAW_DEG = 0).
CAMERA_X = 112.0 + NOSE_SHIFT
DAMPERS = tuple((CAMERA_X + dx, dy) for dx in (-25.0, 25.0) for dy in (32.0, -32.0))
DAMPER_TAB_T = 2.0
DAMPER_SPACING = 11.0
STAGE_Z = (NOSE_Z[0] - DAMPER_SPACING - 4.0, NOSE_Z[0] - DAMPER_SPACING)  # (-39, -35)
CRADLE_GAP = 9.0                    # stage underside to cradle top: room to pitch +-10 deg
CRADLE_T = 4.0
CAMERA_BACK_Z = STAGE_Z[0] - CRADLE_GAP - CRADLE_T                          # -52
CAMERA_LENS_Z = CAMERA_BACK_Z - LuxonisOAK1.DEPTH                          # -79.8
PIVOT_Z = CAMERA_BACK_Z - LuxonisOAK1.DEPTH / 2
PITCH_RANGE_DEG = 10.0
LOCK_R = 12.0                       # lock-screw arc radius about the pivot
# Rangefinder at the front of the nose, long axis across the nose.
RANGEFINDER_X = 152.0 + NOSE_SHIFT
RANGEFINDER_HOLES_Y = (BenewakeTFminiS.HOLE_SPACING / 2,)   # 16.5; slots reach TF-Luna 15.25

# Battery tray under the bottom plate
TRAY_POST_H = 10.0
BATTERY_TRAY_X = (-100.0, 48.0)
BATTERY_TRAY_HALF_W = 32.0
BATTERY_TRAY_T = 3.0
BATTERY_TOP_Z = BOT_Z[0] - TRAY_POST_H - BATTERY_TRAY_T                    # -32

# Landing legs (one per arm, under the plate ear) and TPU feet
LEG_THICKNESS = 12.0
GROUND_Z = -206.0

# Motor mount (in its own frame: u out along the arm, origin on the motor axis)
MOTOR_PLATE_Z = (10.0, 14.0)
MOTOR_SEAT_Z = MOTOR_PLATE_Z[1]
PROP_Z = MOTOR_SEAT_Z + Holybro2216KV920.HEIGHT + 4.0


# --------------------------------------------------------------------- foam dart launcher (optional payload)
# TOY POWER, FOAM DARTS ONLY (hardware/aerodrone.md, "Foam dart launcher").
# The launcher core is built in its own frame (hardware/parts/turret/_launcher.py:
# origin on the bore at the chamber's rear face, x_L along the bore toward the
# muzzle, z_L toward the plunger tube and the clip). On the aerodrone it points
# straight down: x_L -> -z (down), z_L -> -x (the clip toward the tail, away
# from the camera), y_L -> -y; the bore is the vertical line through the hub
# centre. LAUNCHER_ROT and LAUNCHER_ORIGIN place a launcher-frame shape in the
# body frame (_launcher.place()).
LAUNCHER_ROT = (("Y", 90.0), ("Z", 180.0))
LAUNCHER_REAR_X = -159.5            # x_L of the launcher's top end (trigger cradle; rod collar screw primed)
HANGER_FLANGE_T = 4.5               # the hanger's flange under the bottom plate
LAUNCHER_TOP_GAP = 1.0
LAUNCHER_ORIGIN = (0.0, 0.0, BOT_Z[0] - HANGER_FLANGE_T - LAUNCHER_TOP_GAP + LAUNCHER_REAR_X)   # z -184


def launcher_to_body(p):
    """Launcher-frame point (x_L, y_L, z_L) -> body frame."""
    x, y, z = p
    return (LAUNCHER_ORIGIN[0] - z, LAUNCHER_ORIGIN[1] - y, LAUNCHER_ORIGIN[2] - x)


# Hanger: a flange under the bottom plate on the four battery-tray inserts
# (the battery tray is not fitted with the launcher), a keel down the front
# of the launcher (body x KEEL_X, toward the camera), tapered rib in front.
HANGER_FLANGE_X = (-52.0, 46.0)     # front edge 4 mm short of the sensor nose
HANGER_FLANGE_HALF_W = 32.0
HANGER_CABLE_HOLE = (36.0, 3.0, 10.0)      # x, y, dia.: servo bus down from the hub (bottom plate's hole at (30, 0))
KEEL_X = (20.0, 25.0)               # web: 6 mm in front of the launcher (cap, rack hook, tube at x <= 14)
KEEL_Y = (-33.0, 15.0)              # clear of the actuator servo mount (y >= 17.5)
KEEL_BOTTOM_Z = -232.0
RIB_Y = (-12.0, -6.0)
RIB_FRONT = ((70.0, -47.5), (KEEL_X[1], -215.0))   # (x, z) top and bottom of the rib's front edge
# Saddle on the plunger tube (no clamping: the plunger runs inside), a zip tie
# around the tube through it; between the actuator mount and the breech.
SADDLE_XL = (-36.0, -16.0)

# Cam guide: a pad on the breech's underside (four M3 into its UNDER_SCREWS
# inserts), the shelf the turret's pedestal gives the cam plate, and a lip
# over the plate's outer edge; the keel bolts to its front face.
CAM_GUIDE_XL = (-12.0, 88.0)        # along the bore (the breech's length, and room for the counterbores)
CAM_GUIDE_Y = (-44.0, 15.0)         # body y of the pad on the breech
# The breech's underside inserts for it (launcher frame (x_L, y_L)): in the
# rear plate and the front wall, clear of the barrel clamp's inserts.
BREECH_UNDER_SCREWS = ((-4.0, 9.0), (-4.0, -9.0), (80.0, 22.0), (80.0, 38.0))
CAM_GUIDE_WING_Y = (21.6, 50.0)     # body y of the shelf, wall and lip (the cam plate is y 21.3..45.7)
CAM_LIP_XL = (20.0, 60.0)
KEEL_PAD_SCREWS_XL = (10.0, 36.0)   # keel to cam guide, M3 into inserts in the pad's front
KEEL_PAD_SCREWS_Y = (-20.0, 4.0)   # body y, clear of the rib and of the breech screws

# Dart detent: a TPU ring in the breech's barrel seat, in front of the
# chamber; the barrel stops on it, DETENT_T further forward than on the turret.
DETENT_T = 1.5

# Servo drive in the hub, between the plates: the Waveshare Bus Servo
# Adapter (A) on the bottom plate's top face ahead of the power module, the
# 12 V regulator on the right (body x0, x1, y0, y1).
SERVO_ADAPTER_XY = (14.0, 56.0, -16.5, 16.5)
REGULATOR_XY = (-12.7, 12.7, -70.0, -44.6)

# Top deck above the Pi stack, on four posts on inserts in the top plate:
# it carries the HaLowLink 2 (both configurations) and, with the launcher,
# the battery, across the aircraft (155 mm along y), clear of the GPS mast
# column.
DECK_POSTS = ((-40.0, 42.0), (-40.0, -42.0), (12.0, 42.0), (12.0, -42.0))
DECK_Z = (65.0, 69.0)               # 4.8 mm over the Navigator's connector envelope (60.2)
DECK_X = (-92.0, 88.0)               # round the mast column at the rear; the antenna's stay at the front
# The deck's outline: full width behind DECK_MID_X (the battery and its
# straps), DECK_MID_HALF_W to DECK_TONGUE_X (the cradle), then a tongue under
# the cradle's arm to the antenna's stay.
DECK_MID_X = 0.0
DECK_MID_HALF_W = 50.0
DECK_TONGUE_X = 32.0
DECK_TONGUE_Y = (8.0, 34.0)

# HaLowLink 2 (Wi-Fi HaLow link to the ground station) on the deck's front,
# standing on its side: 24 mm along x, 88 along y, 68 tall, the connector
# side (SMA, Ethernet, USB-C) up. Its centre stays within about 35 mm of the
# hub centre, the room the battery's trim range leaves (tray and deck). An
# SMA extension takes the antenna forward to a bulkhead on the cradle's stay
# at HALOW_ANTENNA_XY, where it stands up vertically, 130 mm from the GPS.
HALOW_X = (4.0, 28.0)
HALOW_ANTENNA_XY = (80.0, 21.0)
HALOW_BASE_T = 5.0                  # the cradle screws' heads recessed under the unit
HALOW_SMA_Y = 21.0                  # the SMA end of the unit toward +y
HALOW_CRADLE_SCREWS = ((8.0, 30.0), (8.0, -30.0), (24.0, 30.0), (24.0, -30.0), (62.0, 21.0))   # M3, short inserts in the deck
HALOW_REGULATOR_XY = (-8.9, 8.9, 47.0, 67.3)   # its own 5 V regulator, in the hub on the left (x0, x1, y0, y1)
DECK_HALF_W = 74.0                  # the straps pass under it outside the posts
DECK_POST_D = 12.0

# Tall legs (launcher configuration): a printed socket under each plate ear
# (the landing-leg inserts) holds a 12 mm carbon tube splayed outward along
# the arm; TPU feet. The feet are LAUNCHER_GROUND_CLEARANCE below the
# muzzle and outside the OAK-1's view.
LAUNCHER_GROUND_CLEARANCE = 50.0
TALL_LEG_TUBE = (12.0, 10.0)        # OD, ID
TALL_LEG_TOP = (96.0, -33.0)        # (r, z) where the tube axis leaves the socket's flange
TALL_LEG_FOOT_R = 325.0             # r of the foot centre


def launcher_muzzle_z():
    """Body z of the muzzle (the barrel's end), with the detent ring behind the barrel."""
    from hardware.parts.turret import _launcher as L
    return launcher_to_body((L.MUZZLE_X + DETENT_T, 0.0, 0.0))[2]


def launcher_ground_z():
    """Ground plane of the launcher configuration (the tall legs' soles)."""
    return launcher_muzzle_z() - LAUNCHER_GROUND_CLEARANCE


def tall_leg_axis():
    """(top (r, z), unit direction (dr, dz), length to the sole) of the tall
    legs' tube axis, in the leg frame (r along the arm, z up)."""
    import math
    r0, z0 = TALL_LEG_TOP
    dr, dz = TALL_LEG_FOOT_R - r0, launcher_ground_z() - z0
    n = math.hypot(dr, dz)
    return (r0, z0), (dr / n, dz / n), n
