"""Terrain camera cradle: holds the Luxonis OAK-1 under the front of the
body, lens straight down, for slip and visual-odometry measurements over
the ground just ahead of the front wheels' tracks."""
from hardware.parts.common import fasteners, printing
from hardware.parts.common.footprints import LuxonisOAK1
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover._footprints import Extrusion2040

PLATE_T = 5.0
WALL = 3.0
WALL_H = 12.0
FIT = 0.3
FLANGE = (60.0, 20.0)       # along y, along x: on the rail's 20 mm bottom face

PART = {
    "name": "Terrain camera cradle (OAK-1, downward)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "outdoors under the body; ASA for the heat of a sunny test pit",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upside down: the flange on the plate, cradle walls up", wall_loops=4,
                               infill_pct=40, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "Luxonis OAK-1", "dimensions": f"{LuxonisOAK1.WIDTH:g} mm wide between the walls; 1/4\"-20 thread "
                                                f"{LuxonisOAK1.TRIPOD_FROM_USB_END:g} mm from the USB-C end, on the centre line",
         "source": LuxonisOAK1.SOURCE, "verified": False},
        {"part": "2040 front bottom cross rail", "dimensions": "two M5 into drop-in T-nuts in the 20 mm bottom face's slot",
         "source": Extrusion2040.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "1/4\"-20 UNC x 3/8\" button head screw", "qty": 1, "use": "OAK-1 to the cradle (engages about 4.5 mm of the 10 mm thread)"},
        {"item": "M5 x 10 button head screw + M5 drop-in T-nut (slot 6)", "qty": 2, "use": "cradle to the front bottom cross rail"},
    ],
    "fit_test": True,
    "notes": ["USB-C end toward the rear: the cable runs up into the body behind the rail."],
}


def build():
    """In the rover frame: the cradle's flange against the bottom face of
    the front bottom cross rail (z = 260), the camera hanging below it."""
    from hardware.parts.common import cad
    x_c = F.TERRAIN_CAM_X
    z_top = F.BODY_Z[0]
    w = LuxonisOAK1.WIDTH / 2 + FIT
    length = LuxonisOAK1.HEIGHT + 2 * FIT            # the OAK-1's 54.5 mm along x
    # the camera's back (tripod side) against the plate's underside
    x0 = F.BODY_X[1] + 2.0                           # camera ahead of the rail; USB end toward -x
    plate = cad.box(x_c - FLANGE[1] / 2, x0 + length + WALL, -w - WALL, w + WALL, z_top - PLATE_T, z_top)
    flange = cad.box(x_c - FLANGE[1] / 2, x_c + FLANGE[1] / 2, -FLANGE[0] / 2, FLANGE[0] / 2, z_top - PLATE_T, z_top)
    walls = [cad.box(x0 - WALL, x0 + length + WALL, y0, y0 + WALL, z_top - PLATE_T - WALL_H, z_top - PLATE_T + 0.5)
             for y0 in (-w - WALL, w)]
    body = cad.fuse_all([plate, flange] + walls)
    tripod_x = x0 + LuxonisOAK1.TRIPOD_FROM_USB_END
    cutters = [cad.cyl(fasteners.CLEARANCE_1_4_20, PLATE_T + 2, (tripod_x, 0.0, z_top - PLATE_T - 1))]
    for y in (-20.0, 20.0):
        cutters.append(cad.clearance_hole("M5", x_c, y, z_top - PLATE_T - 1, z_top + 1))
    return cad.cut_all(body, cutters)
