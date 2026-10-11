"""Lidar mast: stands the RPLIDAR C1 on the front deck so its scan plane is
the URDF's 180 mm, and splices the two deck halves (its flange spans the
lap joint). The camera arm bolts onto the rear of the flange."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F
from hardware.parts.racecarneo._footprints import RplidarC1

SEAT_HALF = 28.0            # seat plate 56 x 56 under the C1's 55.6 mm base
WALL = 3.0
WINDOW_W = 26.0             # side openings to reach the C1 screws and route its cable

PART = {
    "name": "Lidar mast (RPLIDAR C1)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on top of the car in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flange down on the plate, seat up",
                               wall_loops=4, infill_pct=40),
    "mates_with": [
        {"part": "Slamtec RPLIDAR C1",
         "dimensions": f"4 x {RplidarC1.HOLE_THREAD} on a {RplidarC1.HOLE_SQUARE:g} mm square, at most "
                       f"{RplidarC1.HOLE_DEPTH:g} mm deep; scan plane {RplidarC1.SCAN_HEIGHT} mm above the base",
         "source": RplidarC1.SOURCE, "verified": True},
        {"part": "deck_front, deck_rear (printed)",
         "dimensions": "six M3 screws through the flange and the deck (two through the lap joint)",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M2.5 x 8 socket head cap screw", "qty": 4,
         "use": "RPLIDAR C1 to the mast seat, from below (enters the lidar 3.5 mm; 4 mm is the limit)"},
        {"item": "M3 x 16 socket head cap screw", "qty": 6, "use": "lidar mast flange through the deck (and the lap joint)"},
        {"item": "M3 nyloc nut", "qty": 6, "use": "under the deck, lidar mast flange screws"},
        {"item": "M3 washer", "qty": 6, "use": "under the nyloc nuts of the lidar mast flange"},
    ],
    "fit_test": False,
    "notes": [
        "Screw the C1 on before bolting the mast to the deck: the M2.5 screws go in from below through the side windows.",
        "Turn the C1 so its 0 deg mark faces the front of the car, then confirm with /scan (an object ahead must appear at angle 0).",
    ],
}


def build():
    from hardware.parts.common import cad
    x0, x1 = F.MAST_FLANGE_X
    z0 = F.DECK_Z[1]
    z1 = z0 + F.MAST_FLANGE_T
    seat_top = F.LIDAR_BOTTOM_Z
    seat_bot = seat_top - F.SEAT_T
    lx, ly = F.LIDAR_XY
    flange = cad.rounded_plate(x1 - x0, 2 * F.MAST_FLANGE_HALF_W, F.MAST_FLANGE_T, 6.0,
                               center=((x0 + x1) / 2, 0.0, z0))
    walls = cad.box(lx - SEAT_HALF, lx + SEAT_HALF, -SEAT_HALF, SEAT_HALF, z1 - 0.01, seat_bot + 0.01).cut(
        cad.box(lx - SEAT_HALF + WALL, lx + SEAT_HALF - WALL, -SEAT_HALF + WALL, SEAT_HALF - WALL, z1 - 1, seat_bot + 1))
    seat = cad.box(lx - SEAT_HALF, lx + SEAT_HALF, -SEAT_HALF, SEAT_HALF, seat_bot, seat_top)
    body = cad.fuse_all([flange, walls, seat])

    cutters = []
    for x, y in F.lidar_holes():
        cutters.append(cad.clearance_hole(RplidarC1.HOLE_THREAD, x, y, seat_bot - 1, seat_top + 1))
        cutters.append(cad.cyl(7.0, F.MAST_FLANGE_T + 2, (x, y, z0 - 1)))   # hex key access from below
    # Lightening / access windows: through the seat centre and all four walls.
    cutters.append(cad.cbox(24.0, 24.0, 20.0, center=(lx, ly, seat_top)))
    for ang in (0.0, 90.0):
        w = cad.cbox(2 * SEAT_HALF + 2, WINDOW_W, seat_bot - z1 - 2.0, center=(lx, ly, (z1 + seat_bot) / 2))
        cutters.append(cad.rotate(w, "Z", ang, about=(lx, ly, 0)))
    for x, y in F.MAST_SCREWS:
        cutters.append(cad.clearance_hole("M3", x, y, z0 - 1, z1 + 1))
    for x, y in F.ARM_FOOT_SCREWS:
        cutters.append(cad.clearance_hole("M3", x, y, z0 - 1, z1 + 1))
    return cad.cut_all(body, cutters)
