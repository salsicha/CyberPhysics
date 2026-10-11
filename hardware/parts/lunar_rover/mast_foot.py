"""Mast foot: stands a 2020 mast on a 2040 top side rail of the body (the
400 mm front mast for the lidar at the front left, the 200 mm rear mast
for the GNSS antenna at the rear right). One apron down the rail's outer
face; the lid stays clear inside."""
from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover._footprints import Extrusion2020, Extrusion2040

FIT = 0.4
WALL = 6.0
SOCKET_H = 40.0
FLANGE = (60.0, 20.0, 6.0)      # along y, along x (the rail's 20 mm top face), thickness
APRON = 28.0                    # apron down the rail's outer 40 mm face

PART = {
    "name": "Mast foot (2020 on a 2040 side rail)",
    "quantity": 2,
    "material": "ASA",
    "material_reason": "outdoors on the top of the rover, in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "on its back: the apron's outer face on the plate", wall_loops=5,
                               infill_pct=40, rotate=[("Y", 90)]),
    "mates_with": [
        {"part": "2020 mast", "dimensions": f"{Extrusion2020.SIZE + FIT:g} mm socket, two M5 into drop-in T-nuts",
         "source": Extrusion2020.SOURCE, "verified": True},
        {"part": "2040 top side rail", "dimensions": "two M5 into the top slot, two through the apron into the outer face",
         "source": Extrusion2040.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 10 button head screw + M5 drop-in T-nut (slot 6)", "qty": 6, "use": "foot to the rail (4) and mast to the foot (2)"},
    ],
    "fit_test": False,
    "notes": ["The same part at both ends: the front one carries the lidar mast, the rear one the GNSS mast."],
}


def build():
    """Local frame: the rail's top face at z = 0, rail along y centred on
    x = 0 (its outer face toward +x), the mast's axis on z."""
    from hardware.parts.common import cad
    s = Extrusion2020.SIZE + FIT
    so = s / 2 + WALL
    rail_w = Extrusion2040.SIZE[0]
    flange = cad.box(-rail_w / 2, so, -FLANGE[0] / 2, FLANGE[0] / 2, 0.0, FLANGE[2])
    aprons = [cad.box(rail_w / 2, so, -FLANGE[0] / 2, FLANGE[0] / 2, -APRON, FLANGE[2])]   # flush with the socket
    socket = cad.box(-so, so, -so, so, 0.0, F.MAST_FOOT_FLOOR + SOCKET_H)
    body = cad.fuse_all([flange, socket] + aprons)
    cutters = [cad.box(-s / 2, s / 2, -s / 2, s / 2, F.MAST_FOOT_FLOOR, F.MAST_FOOT_FLOOR + SOCKET_H + 1)]
    for y in (-22.0, 22.0):
        cutters.append(cad.clearance_hole("M5", 0.0, y, -1.0, FLANGE[2] + 1))
        cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], so - rail_w / 2 + 2, (so + 1, y, -10.0), (-1, 0, 0)))
    z = F.MAST_FOOT_FLOOR + SOCKET_H / 2
    cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], WALL + 2, (so + 1, 0.0, z), (-1, 0, 0)))
    cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], WALL + 2, (0.0, so + 1, z), (0, -1, 0)))
    return cad.cut_all(body, cutters)
