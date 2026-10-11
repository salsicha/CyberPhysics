"""Tether spool crank handle: a sleeve that turns on an M5 screw in one
flange's crank boss."""
from hardware.parts.common import printing
from hardware.parts.submarine import _frame as F

SIZE = (24.0, 55.0)             # outside diameter, length
BORE = 5.8                      # turns freely on the M5 screw
SCREW_LENGTH = 70.0             # washer + handle + washer + boss + flange: the nyloc sits flush in the flange

PART = {
    "name": "Tether spool crank handle",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on deck in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright on one end", wall_loops=4, infill_pct=30),
    "mates_with": [
        {"part": "spool_flange (printed)", "dimensions": "M5 screw through the crank boss",
         "source": "hardware/parts/submarine/spool_flange.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 70 socket head cap screw, A4 (316) stainless", "qty": 1, "use": "crank handle axle, through the flange boss"},
        {"item": "M5 washer, A4 (316) stainless", "qty": 2, "use": "crank handle: under the head, and between handle and boss"},
        {"item": "M5 nyloc nut, A4 (316) stainless", "qty": 1, "use": "crank handle, in the hex pocket on the flange's inner face"},
    ],
    "fit_test": False,
    "notes": ["Tighten only until the handle still spins freely on the screw; the nut sits in a pocket flush with the flange's inner face so the tether cannot catch on it."],
}


def build():
    """In the spool frame: on the crank axis, from the flange boss outward (-z)."""
    from hardware.parts.common import cad
    from hardware.parts.submarine import spool_flange
    cx, cy = cad.polar(F.CRANK_R, F.CRANK_ANGLE)
    z0 = -F.SPOOL_FLANGE[1] - spool_flange.CRANK_BOSS[1] - 1.0 - SIZE[1]
    sleeve = cad.cyl(SIZE[0], SIZE[1], (cx, cy, z0))
    return sleeve.cut(cad.cyl(BORE, SIZE[1] + 2, (cx, cy, z0 - 1))).clean()
