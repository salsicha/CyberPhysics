"""HaLowLink cradle spacer: a tube that stands the cradle's floor 18 mm
above the deck, over the IMU; an M3 screw passes through it into a deck
insert. Four per car, the same part."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F

OD = 8.0

PART = {
    "name": "HaLowLink cradle spacer",
    "quantity": 4,
    "material": "ASA",
    "material_reason": "on the deck in the sun, under the cradle's screws",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: an end on the plate", wall_loops=4, infill_pct=100),
    "mates_with": [
        {"part": "halow_cradle (printed) and deck_rear",
         "dimensions": f"{F.HALOW_FLOOR_Z[0] - F.DECK_Z[1]:g} mm from the deck's top to the cradle's floor, an M3 "
                       "through it",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": ["The cradle's M3 x 25 screws pass through the spacers (they are listed with the cradle)."],
}


def build():
    """One spacer at x, y = 0 between the deck's top and the cradle's floor."""
    from hardware.parts.common import cad
    z0, z1 = F.DECK_Z[1], F.HALOW_FLOOR_Z[0]
    return cad.cyl(OD, z1 - z0, (0, 0, z0)).cut(cad.clearance_hole("M3", 0, 0, z0 - 1, z1 + 1))
