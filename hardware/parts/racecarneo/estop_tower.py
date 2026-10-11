"""E-stop tower: a hollow column at the rear of the deck that carries the
22 mm emergency-stop mushroom on top, where it can be hit from behind or
above; the switch body and its leads are inside, the leads leave through
a slot at the bottom."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F
from hardware.parts.racecarneo._footprints import SchneiderXB5AS8442 as E

WALL = 3.0
PANEL_T = 4.0
EAR = (12.0, 12.2, 5.0)          # ear (x, y, t) each side, with two screws
LEAD_SLOT = (14.0, 10.0)         # width, height, at the bottom of the front wall

PART = {
    "name": "E-stop tower (22 mm mushroom)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "outdoors, and it is hit by hand: ASA keeps its stiffness in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: ears on the plate, panel on top (bridged)",
                               wall_loops=4, infill_pct=40),
    "mates_with": [
        {"part": "Schneider XB5AS8442 emergency stop",
         "dimensions": f"{E.CUTOUT_D} mm cut-out in a {PANEL_T:g} mm panel (1-6 mm allowed), {E.MUSHROOM_D:g} mm mushroom",
         "source": E.SOURCE, "verified": True},
        {"part": "XB5AS8442 body under the panel",
         "dimensions": f"budgeted {E.BODY_H:g} mm long, dia. {E.BODY_D:g} mm (the 82 mm overall depth is not split in the datasheet)",
         "source": E.SOURCE, "verified": False},
        {"part": "deck_rear (printed)", "dimensions": "four M3 x 3 inserts", "source": "hardware/parts/racecarneo/_frame.py",
         "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "M3 x 8 socket head cap screw", "qty": 4, "use": "e-stop tower ears to the rear deck inserts"}],
    "fit_test": True,
    "notes": [
        "The e-stop's NC contact opens the contactor coil (see the BOM): a 22 mm switch must never carry motor current.",
        "Its top is in the lidar's rear blind sector (behind the camera arm's fin), so it may stand above the scan plane.",
    ],
}


def build():
    from hardware.parts.common import cad
    cx, cy = F.ESTOP_XY
    z0 = F.DECK_Z[1]
    h = F.ESTOP_TOWER_H
    a = F.ESTOP_TOWER / 2
    shell = cad.box(cx - a, cx + a, cy - a, cy + a, z0, z0 + h).cut(
        cad.box(cx - a + WALL, cx + a - WALL, cy - a + WALL, cy + a - WALL, z0 - 1, z0 + h - PANEL_T))
    ears = [cad.box(ix - EAR[0] / 2, ix + EAR[0] / 2, *sorted((iy - (EAR[1] / 2) * (1 if iy > 0 else -1),
                                                               iy + (EAR[1] / 2) * (1 if iy > 0 else -1))), z0, z0 + EAR[2])
            for ix, iy in F.ESTOP_INSERTS]
    body = cad.fuse_all([shell] + ears)
    cutters = [cad.cyl(E.CUTOUT_D + 0.1, PANEL_T + 2, (cx, cy, z0 + h - PANEL_T - 1)),
               cad.box(cx + a - WALL - 1, cx + a + 1, cy - LEAD_SLOT[0] / 2, cy + LEAD_SLOT[0] / 2, z0 - 1, z0 + LEAD_SLOT[1])]
    for ix, iy in F.ESTOP_INSERTS:
        cutters.append(cad.clearance_hole("M3", ix, iy, z0 - 1, z0 + EAR[2] + 1))
    return cad.cut_all(body, cutters)
