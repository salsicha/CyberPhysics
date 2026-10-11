"""Camera post foot: a socket for the bottom of the 2020 camera post,
screwed to the station board with four wood screws."""
from hardware.parts.common import fasteners, printing
from hardware.parts.so101 import _station as S
from hardware.parts.so101._footprints import Extrusion2020, StationBoard

HALF = 55.0               # plate half-size
SOCKET_WALL = 6.0
GUSSET_T = 4.0
GUSSET_OFFSET = 8.5       # gusset centre lines from the face centre (the screw head is 9.5 mm)
SCREW_XY = 42.0           # wood screws on the corners of a 84 mm square
SCREW_HOLE = 4.5
WELL = (9.0, 5.0)         # head well diameter, floor under the head
TNUT_SCREW_H = 22.0       # above the socket floor

PART = {
    "name": "Camera post foot",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "indoors; stiff socket for the 500 mm post",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "upright: the base plate on the plate, socket up", wall_loops=4,
                               infill_pct=40),
    "mates_with": [
        {"part": "2020 aluminium extrusion (camera post)",
         "dimensions": f"{S.POST_FIT:g} mm square socket, {S.FOOT_SOCKET_DEPTH:g} mm deep, two M5 into drop-in T-nuts",
         "source": Extrusion2020.SOURCE, "verified": True},
        {"part": "station board", "dimensions": "four 4 mm wood screws on an 84 mm square",
         "source": StationBoard.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 10 button head screw + M5 drop-in T-nut (slot 6)", "qty": 2, "use": "post to the foot"},
        {"item": "4 x 16 mm pan-head wood screw", "qty": 4, "use": "foot to the station board"},
    ],
    "fit_test": False,
    "notes": [
        f"The post stands {S.FOOT_FLOOR - S.TABLE_Z:.1f} mm above the board on the socket floor; with a "
        f"{S.POST_LENGTH:g} mm post that puts the camera head where the scene's camera is.",
    ],
}


def build():
    from hardware.parts.common import cad
    px, py = S.POST_XY
    z0 = S.TABLE_Z
    floor = S.FOOT_FLOOR
    top = floor + S.FOOT_SOCKET_DEPTH
    so = S.POST_FIT / 2 + SOCKET_WALL
    plate = cad.rounded_plate(2 * HALF, 2 * HALF, S.FOOT_PLATE_T, 8.0, center=(px, py, z0))
    socket = cad.box(px - so, px + so, py - so, py + so, z0, top)
    rib = cad.profile_xz([(so - 1, z0 + S.FOOT_PLATE_T - 0.5), (HALF - 14, z0 + S.FOOT_PLATE_T - 0.5),
                          (so + 3, top - 6), (so - 1, top - 6)], -GUSSET_T / 2, GUSSET_T / 2)
    # two gussets per face, either side of the T-nut screw on the face's centre line
    gussets = [cad.translate(cad.rotate_z(cad.translate(rib, (0, off, 0)), a), (px, py, 0))
               for a in (0, 90, 180, 270) for off in (-GUSSET_OFFSET, GUSSET_OFFSET)]
    body = cad.fuse_all([plate, socket] + gussets)
    cutters = [cad.box(px - S.POST_FIT / 2, px + S.POST_FIT / 2, py - S.POST_FIT / 2, py + S.POST_FIT / 2,
                       floor, top + 1)]
    for sx in (1, -1):
        for sy in (1, -1):
            x, y = px + sx * SCREW_XY, py + sy * SCREW_XY
            cutters.append(cad.cyl(SCREW_HOLE, S.FOOT_PLATE_T + 2, (x, y, z0 - 1)))
            cutters.append(cad.cyl(WELL[0], S.FOOT_PLATE_T, (x, y, z0 + WELL[1])))
    z = floor + TNUT_SCREW_H
    for dx, dy in ((1, 0), (0, 1)):
        start = (px + dx * (so + 1), py + dy * (so + 1), z)
        cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], SOCKET_WALL + 2, start, (-dx, -dy, 0)))
    return cad.cut_all(body, cutters)
