"""Lidar mount: on the top of the 400 mm front mast, carries the Livox
Mid-360 on a purchased 100 x 100 x 3 mm aluminium plate, which Livox asks
for as the lidar's heat sink (a printed plate would insulate it)."""
from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover._footprints import Extrusion2020, LivoxMid360

PLATE = (110.0, 6.0)          # square side, thickness
SINK = (100.0, 3.0)           # purchased aluminium plate
FIT = 0.4
WALL = 6.0
SOCKET_H = 30.0
SINK_BOLTS = 40.0             # 4 x M4 on a square of 2 x this
WINDOW = 30.0                 # air windows under the aluminium plate

PART = {
    "name": "Lidar mount (Livox Mid-360 on the mast)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "outdoors at the top of the mast in the sun, under a 6.5 W lidar",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upside down: the plate's top face on the plate, socket up", wall_loops=5,
                               infill_pct=40, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "Livox Mid-360", "dimensions": "4 x M3 on 48 x 36 mm on its bottom, through the aluminium plate",
         "source": LivoxMid360.SOURCE, "verified": True},
        {"part": "Livox Mid-360 hole pattern position", "dimensions": "assumed centred on the body",
         "source": LivoxMid360.SOURCE, "verified": False},
        {"part": "2020 mast", "dimensions": f"{Extrusion2020.SIZE + FIT:g} mm socket, two M5 into drop-in T-nuts",
         "source": Extrusion2020.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 10 socket head cap screw", "qty": 4,
         "use": "lidar through the printed and aluminium plates, heads recessed (4.5 mm into its 5 mm threads)"},
        {"item": "M4 x 12 socket head cap screw + M4 nyloc nut", "qty": 4, "use": "aluminium plate to the mount"},
        {"item": "M5 x 10 button head screw + M5 drop-in T-nut (slot 6)", "qty": 2, "use": "mount to the mast"},
    ],
    "fit_test": True,
    "notes": ["Drill the aluminium plate from this part (use it as the template): 4 x 3.4 mm for the lidar, 4 x 4.5 mm for the M4."],
}


def build():
    """Local frame: the mast's top end at z = 0, its axis on z; the M12
    connector toward -x (the rear)."""
    from hardware.parts.common import cad
    side, t = PLATE
    s = Extrusion2020.SIZE + FIT
    so = s / 2 + WALL
    socket = cad.box(-so, so, -so, so, -SOCKET_H, 2.0)
    plate = cad.rounded_plate(side, side, t, 6.0, center=(0, 0, 2.0))
    body = socket.fuse(plate).clean()
    cutters = [cad.box(-s / 2, s / 2, -s / 2, s / 2, -SOCKET_H - 1, 0.0)]
    hx, hy = LivoxMid360.HOLES[0] / 2, LivoxMid360.HOLES[1] / 2
    for x in (-hx, hx):
        for y in (-hy, hy):
            cutters.append(cad.clearance_hole("M3", x, y, -1.0, 2.0 + t + 1))
            cutters.append(cad.cyl(fasteners.SHCS_HEAD["M3"][0] + 1.0, 3.5, (x, y, 2.0 - 0.01)))   # heads under the plate
    for x in (-SINK_BOLTS, SINK_BOLTS):
        for y in (-SINK_BOLTS, SINK_BOLTS):
            cutters.append(cad.clearance_hole("M4", x, y, 1.0, 2.0 + t + 1))
    for x, y in ((0.0, 32.0), (0.0, -32.0), (32.0, 0.0), (-32.0, 0.0)):
        cutters.append(cad.rounded_plate(WINDOW if x == 0 else 12.0, 12.0 if x == 0 else WINDOW - 6.0, t + 2, 3.0,
                                         center=(x, y, 1.0)))
    z = -SOCKET_H / 2
    cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], WALL + 2, (so + 1, 0.0, z), (-1, 0, 0)))
    cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], WALL + 2, (0.0, so + 1, z), (0, -1, 0)))
    return cad.cut_all(body, cutters)
