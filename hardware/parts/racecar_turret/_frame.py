"""RACECAR Turret layout (helper, not a part).

Pure Python. Vehicle frame = the RACECAR's base_link (millimetres, x
forward, y left, z up from the ground under the chassis centre), as in
hardware/parts/racecarneo/_frame.py. The turret plate is the URDF's
turret/base_link: its underside at (-100, 0, 240) mm
(systems/racecar_turret/urdf/racecar_turret.urdf.xacro), 5 mm thick, so the
turret frame of hardware/parts/turret/_frame.py (origin on the pan axis on
the surface the pan base stands on) sits at (-100, 0, 245).

The plate stands on four posts from the RACECAR deck at the deck's
TURRET_POSTS; they cross the lidar's scan plane inside the rear sector the
camera arm's fin already hides (racecarneo assembly check).
"""
from hardware.parts.racecarneo import _frame as R
from hardware.parts.turret import _frame as T

PLATE_CENTER = (-100.0, 0.0)
PLATE_Z = (240.0, 240.0 + T.PLATE_T_URDF)
PLATE_HALF = (55.0, 58.0)
PLATE_CORNER_R = 8.0
TURRET_ORIGIN = (PLATE_CENTER[0], PLATE_CENTER[1], PLATE_Z[1])

POSTS = R.TURRET_POSTS
POST_D = 12.0
POST_WAIST_D = 8.0
POST_WAIST_Z = (R.SCAN_Z - 9.0, R.SCAN_Z + 9.0)   # thin where it crosses the scan plane
POST_FLANGE = (30.0, 12.0, 4.0)
POST_INSERT_DX = R.TURRET_POST_INSERT_DX

# Bus Servo Adapter (A) under the plate, components facing down.
ADAPTER_CENTER = (-100.0, 0.0)
ADAPTER_STANDOFF = (6.0, 4.0)       # dia., length below the plate

# Cable pass-throughs in the plate (x0, x1, y0, y1), vehicle frame.
BUS_SLOT = (-108.0, -92.0, -46.0, -36.0)   # servo bus: pan base side window -> adapter below
USB_SLOT = (-152.0, -144.0, -10.0, 10.0)   # camera cable: turret cable loop -> down a rear post


def turret_xyz(p):
    """Turret-frame point -> vehicle frame (pan 0)."""
    return (p[0] + TURRET_ORIGIN[0], p[1] + TURRET_ORIGIN[1], p[2] + TURRET_ORIGIN[2])
