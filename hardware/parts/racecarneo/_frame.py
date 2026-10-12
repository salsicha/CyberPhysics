"""RACECAR Neo layout: the numbers every racecarneo part shares (helper,
not a part).

Pure Python. Vehicle frame = the URDF's base_link (FLU, millimetres): x
forward, y left, z up, origin on the ground under the chassis centre
(systems/racecarneo/urdf/racecarneo.urdf.xacro). The sensor positions are
the URDF's: lidar_link (80, 0, 180), camera_link (180, 0, 220), imu_link
(0, 0, 80); the assembly check reports how close the printed mounts put
them.

Chassis: 1/10 Traxxas Slash 4x4 class (324 mm wheelbase, 296 mm wide, 72 mm
ground clearance; Traxxas specification page). A printed deck on four
TPU-isolated post adapters carries every electronic part above the
chassis; the C1 lidar stands on a short mast, and the RealSense hangs from
an arm that crosses the lidar's scan plane only behind the lidar, inside
its 50 mm blind range, so the scan sees no printed part in front of or
beside the car.
"""
import math

from hardware.parts.racecarneo._footprints import RealSenseD435i, RplidarC1

# --------------------------------------------------------------------- URDF
LIDAR_XY = (80.0, 0.0)
SCAN_Z = 180.0
CAMERA_LINK = (180.0, 0.0, 220.0)
IMU_LINK = (0.0, 0.0, 80.0)
NAV2_ROBOT_RADIUS = 220.0           # systems/racecarneo/config/nav2.yaml

# Nothing printed may cross the scan plane in this band unless it is inside
# the lidar's blind range (the camera arm's fin).
SCAN_BAND = 3.0
SCAN_KEEP_OUT_Z = SCAN_Z - SCAN_BAND        # tops of deck items stay below
BLIND_MARGIN = 3.0                           # inside RplidarC1.MIN_RANGE by this much

# --------------------------------------------------------------------- deck
# Two printed halves joined by a lap (front half on top) that the lidar
# mast flange and two rim screws clamp. Each half fits the 250 mm bed.
DECK_Z = (127.0, 132.0)             # underside, top face
DECK_T = DECK_Z[1] - DECK_Z[0]
DECK_X = (-185.0, 196.0)
DECK_HALF_W = 72.0
DECK_CORNER_R = 12.0
JOINT_X = (36.0, 50.0)              # lap: rear half below, front half on top
LAP_Z = DECK_Z[0] + DECK_T / 2.0    # 129.5
LAP_SCREWS = ((43.0, 62.0), (43.0, -62.0))      # plus the mast flange's screws at x = 43
RIM_W = 4.0
RIM_H = 6.0                          # stiffening rim on the long edges, on top

# Deck mounting points: TPU top-hat grommets in 6.2 mm holes, an M3 screw
# through each into the post adapter's insert.
MOUNTS = {"front": ((150.0, 60.0), (150.0, -60.0)),
          "rear": ((-158.0, 60.0), (-158.0, -60.0))}
GROMMET_HOLE = 6.2
GROMMET_FLANGE_T = 2.0
SEAT_Z = DECK_Z[0] - GROMMET_FLANGE_T       # adapter seat top, 125

# Chassis body posts the adapters clamp. Traxxas does not publish their
# positions or diameter (and the 2020+ "clipless" Slash 4x4 kits have
# latching body mounts instead of posts): measure yours and edit these;
# the adapters reach from the posts to the deck mounting points.
POSTS = {"front": ((150.0, 30.0), (150.0, -30.0)),
         "rear": ((-158.0, 42.0), (-158.0, -42.0))}
POST_D = 6.0
SLEEVE_LEN = 24.0

# --------------------------------------------------------------------- lidar mast
LIDAR_BOTTOM_Z = SCAN_Z - RplidarC1.SCAN_HEIGHT           # 150.2
MAST_FLANGE_X = (22.0, 115.0)
MAST_FLANGE_HALF_W = 32.0
MAST_FLANGE_T = 5.0
MAST_SCREWS = ((28.0, 24.0), (28.0, -24.0), (43.0, 24.0), (43.0, -24.0), (111.5, 24.0), (111.5, -24.0))
SEAT_T = 4.5                         # M2.5 x 8 then enters the C1 3.5 mm (max 4)

# --------------------------------------------------------------------- camera arm
# Profile in x-z, extruded across y; its fin crosses the scan plane behind
# the lidar between FIN_X, within the C1's 50 mm blind range.
ARM_HALF_W = 13.0
FIN_X = (36.0, 49.0)                # 31-44 mm behind the lidar axis
ARM_FOOT_X = (22.0, 49.0)
ARM_FOOT_T = 8.0
ARM_FOOT_SCREWS = ((30.0, 7.0), (30.0, -7.0))
BOOM_Z = (196.0, 238.0)             # over the C1 (top at 191.5)
ARM_FRONT_Z = (188.0, 199.0)        # tongue in front of the lidar, above the scan plane
BOOM_FRONT_X = 148.0                 # the deep boom ends 10 mm behind the camera's back face
TONGUE_X = (BOOM_FRONT_X, 186.0)
TONGUE_Y = (1.0, ARM_HALF_W)         # 12 mm, flush with the arm's +y face

# Camera cradle: pivots on the tongue about a y axis under the camera.
PIVOT = (170.0, 194.0)              # (x, z)
LOCK_R = 11.0                        # lock screw arc radius, behind the pivot
PITCH_RANGE_DEG = 10.0
CHEEK_T = 4.0
CHEEK_GAP = 0.3
CRADLE_SEAT_Z = CAMERA_LINK[2] - RealSenseD435i.LINK_HEIGHT     # 207.5, camera bottom face
CRADLE_T = 3.5
CRADLE_X = (157.0, 184.0)
CRADLE_Y = (-56.0, TONGUE_Y[1] + CHEEK_GAP + CHEEK_T + 0.2)
# 1/4"-20 screw positions that put camera_link on y = 0 (the URDF):
# 17.5 mm (D435i) or 47.5 mm (D455) right of the centre line.
SCREW_X = 169.1
SCREW_Y = (-47.5, -17.5)

# --------------------------------------------------------------------- deck items
# Items bolt to M3 x 3 heat-set inserts pressed into the deck's top face.
# NVIDIA Jetson Orin Nano (Super) developer kit, without its plastic stand:
# carrier 79 mm along x and 100 mm along y, I/O edge facing forward (the
# 40-pin header on the left edge), below the scan plane in front of the C1.
JETSON_X = (116.0, 195.0)
JETSON_Y = (-50.0, 50.0)
JETSON_BOSS_H = 6.0
JETSON_TRAY_T = 3.0
JETSON_TRAY_INSERTS = ((128.0, 28.0), (128.0, -28.0), (186.0, 28.0), (186.0, -28.0))
IMU_XY = (0.0, 0.0)
IMU_INSERTS = ((0.0, 17.0), (0.0, -17.0))
VESC_CENTER = (-97.0, 0.0)
VESC_INSERTS = ((-139.0, 12.0), (-139.0, -12.0), (-55.0, 12.0), (-55.0, -12.0))
# Two power trays, left and right of the IMU (the same part turned 180 deg):
# the 19 V converter for the Jetson on one, the 6 V BEC and the e-stop
# contactor on the other.
POWER_CENTERS = ((-9.5, 46.0), (-9.5, -46.0))
POWER_SIZE = (61.0, 40.0)
POWER_INSERTS = ((-29.5, 46.0), (10.5, 46.0), (-29.5, -46.0), (10.5, -46.0))
ESTOP_XY = (-166.0, 0.0)            # e-stop axis (vertical), at the rear
ESTOP_TOWER = 38.0                   # square tower, outer
ESTOP_TOWER_H = 60.0                 # deck top to the panel top
ESTOP_INSERTS = ((-174.0, 25.0), (-174.0, -25.0), (-158.0, 25.0), (-158.0, -25.0))
WIRE_WINDOW = ((-48.0, -20.0), (-38.0, 20.0))   # motor and battery leads down to the chassis
RIM_CLIP_X = (-120.0, -60.0, 30.0, 110.0, 175.0)  # where the rim cable clips go (both sides)

# --------------------------------------------------------------------- RACECAR Turret
# The RACECAR Turret's plate posts stand on the rear deck
# (systems/racecar_turret: plate at (-100, 0, 240)), in the camera arm
# fin's shadow; each post flange takes two M3 into deck inserts.
TURRET_POSTS = ((-57.0, 48.0), (-57.0, -48.0), (-143.0, 48.0), (-143.0, -48.0))
TURRET_POST_INSERT_DX = 10.0


# --------------------------------------------------------------------- HaLow link
# The car's Morse Micro HaLowLink 2 (hardware/racecarneo.md, "Long-range
# link") stands upright behind the camera arm's fin, 24 mm thick across y,
# its connector side (SMA, WAN, LAN, USB-C) facing the rear: inside the
# fin's shadow, so the lidar does not see it, behind the RealSense, and
# below the RACECAR Turret's sweep (racecar_turret assembly check). Its
# cradle's floor bridges the IMU on four printed spacers; the antenna stands
# at the rear-right corner, its top under the turret's launcher at full
# nose-up tilt.
HALOW_X = (-40.0, 28.0)              # connector (rear) face, front face
HALOW_HALF_T = 12.0
HALOW_FLOOR_Z = (150.0, 154.0)       # the cradle's floor: over the IMU and the arm foot's screw heads
HALOW_Z0 = HALOW_FLOOR_Z[1]          # the unit's bottom
HALOW_SPACERS = ((28.0, 39.0), (28.0, -39.0), (-31.0, 19.0), (-31.0, -19.0))
ANTENNA_XY = (-175.0, -60.0)         # the antenna's vertical axis
ANTENNA_AXIS_Z = 141.0               # the bulkhead's horizontal axis (along y)
ANTENNA_PANEL_Y = (-46.0, -42.5)     # the bracket's wall: the bulkhead's panel, outer face first
ANTENNA_INSERTS = ((-181.5, -35.0), (-168.5, -35.0))


def turret_post_inserts():
    return tuple((x + s * TURRET_POST_INSERT_DX, y) for x, y in TURRET_POSTS for s in (1, -1))


def lidar_holes():
    """C1 M2.5 holes in the vehicle frame (43 mm square about the lidar axis)."""
    h = RplidarC1.HOLE_SQUARE / 2.0
    x, y = LIDAR_XY
    return tuple((x + sx * h, y + sy * h) for sx in (1, -1) for sy in (1, -1))


def fin_shadow_deg():
    """Half-angle of the rear sector the camera arm fin hides from the lidar."""
    return math.degrees(math.atan2(ARM_HALF_W, LIDAR_XY[0] - FIN_X[1]))


def jetson_holes():
    """Carrier mounting holes (inferred 86 x 58 mm pattern), vehicle frame:
    I/O edge at JETSON_X[1], DC-jack end at JETSON_Y[0]."""
    from hardware.parts.racecarneo._footprints import JetsonOrinNanoDevKit as J
    return tuple((JETSON_X[1] - ex, JETSON_Y[0] + jy) for ex in J.HOLES_FROM_IO_EDGE for jy in J.HOLES_FROM_JACK_END)


def jetson_pcb_z():
    """Underside of the carrier PCB."""
    return DECK_Z[1] + JETSON_TRAY_T + JETSON_BOSS_H
