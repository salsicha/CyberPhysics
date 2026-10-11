"""Solar panel support: four posts on the top side rails carry the panel
80 mm above the body (clear of the differential bar and its rod ends). The panel's frame
bolts to each post through its own bottom-flange holes; the post's top is
cross-slotted because flange holes differ between panels."""
from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover._footprints import Extrusion2040, SolarPanel50W

BASE = (40.0, 20.0, 6.0)          # along x (the rail), across (the rail's 20 mm top face), thickness
COLUMN = (24.0, 40.0)             # along x, along y: deep under the top plate's reach
TOP = (40.0, 75.0, 8.0)           # top plate: along x, along y (outward), thickness
SLOT = (30.0, 40.0)               # slot lengths along x and along y, M5

PART = {
    "name": "Solar panel post",
    "quantity": 4,
    "material": "ASA",
    "material_reason": "outdoors, in the sun under the panel's edge",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "on its side: the post's profile flat on the plate", wall_loops=5, infill_pct=40,
                               rotate=[("X", 90)]),
    "mates_with": [
        {"part": "2040 top side rail", "dimensions": "two M5 into drop-in T-nuts in the 20 mm top face's slot",
         "source": Extrusion2040.SOURCE, "verified": True},
        {"part": "solar panel frame bottom flange", "dimensions": "one M5 through a flange hole, anywhere in the cross slot",
         "source": SolarPanel50W.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 10 button head screw + M5 drop-in T-nut (slot 6)", "qty": 2, "use": "post to the top side rail"},
        {"item": "M5 x 16 socket head cap screw + 2 M5 washers + M5 nyloc nut", "qty": 1, "use": "panel flange to the post"},
    ],
    "fit_test": True,
    "notes": ["Check the panel's flange holes fall in the four cross slots before printing all four."],
}


def build():
    """Local frame: the rail's top face at z = 0, rail along x, the rail's
    centre line at y = 0, the panel's edge toward +y."""
    from hardware.parts.common import cad
    h = F.PANEL_Z - F.BODY_Z[1]
    y0 = -BASE[1] / 2
    base = cad.box(-BASE[0] / 2, BASE[0] / 2, y0, BASE[1] / 2, 0.0, BASE[2])
    col = cad.box(-COLUMN[0] / 2, COLUMN[0] / 2, y0, y0 + COLUMN[1], 0.0, h)
    top = cad.box(-TOP[0] / 2, TOP[0] / 2, y0, y0 + TOP[1], h - TOP[2], h)
    body = cad.fuse_all([base, col, top])
    cutters = [cad.clearance_hole("M5", x, 0.0, -1.0, BASE[2] + 1) for x in (-14.0, 14.0)]
    cy = y0 + TOP[1] - 20.0
    cutters.append(cad.slot(-SLOT[0] / 2, cy, SLOT[0] / 2, cy, fasteners.CLEARANCE["M5"], h - TOP[2] - 1, h + 1))
    cutters.append(cad.slot(0.0, y0 + COLUMN[1] + 6.0, 0.0, y0 + TOP[1] - 8.0, fasteners.CLEARANCE["M5"],
                            h - TOP[2] - 1, h + 1))
    return cad.cut_all(body, cutters)
