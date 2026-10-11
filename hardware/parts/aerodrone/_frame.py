"""Aerodrone frame layout: the numbers every aerodrone part shares.

Pure Python. Body frame FLU in millimetres: x forward, y left, z up; the
origin is the hub centre on the plane of the arm-tube axes. Modules whose
name starts with "_" are helpers, not printed parts.

Frame: 500 mm motor-to-motor (X500 class), quad X, four 16 mm carbon arm
tubes clamped between two printed hub plates; 2216-size ~920 KV motors,
10 x 4.5 in propellers, 4S LiPo, about 2.0 kg take-off mass.
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
MAST_XY = (-57.0, 0.0)
MAST_FLANGE_D = 36.0
MAST_BOLT_R = 13.5
MAST_BOLT_ANGLES = (0.0, 120.0, 240.0)
MAST_HEIGHT = 129.0                 # flange underside to the GPS seat


def mast_bolts_body():
    return tuple((MAST_XY[0] + MAST_BOLT_R * math.cos(math.radians(a)),
                  MAST_XY[1] + MAST_BOLT_R * math.sin(math.radians(a))) for a in MAST_BOLT_ANGLES)


# Sensor nose under the front of the bottom plate
NOSE_X = (50.0, 160.0)
NOSE_HALF_W = 40.0
NOSE_Z = (BOT_Z[0] - 5.0, BOT_Z[0])             # (-24, -19)
# Camera stage on four silicone damper balls, OAK-1 USB-C end toward the tail
# (image top toward the nose: CAMERA_MOUNT_YAW_DEG = 0).
CAMERA_X = 112.0
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
RANGEFINDER_X = 152.0
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
