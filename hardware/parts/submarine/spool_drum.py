"""Tether spool drum: the core of a topside hand spool for the Fathom
tether (about 60 m of the 7.6 mm standard tether or 200 m of the 4 mm slim
one); two spool flanges screw onto its ends."""
from hardware.parts.common import printing
from hardware.parts.submarine import _frame as F
from hardware.parts.submarine._footprints import FathomTether

TETHER_SLOT = (12.0, 30.0)       # width (around), length (along the axle) from the flange end

PART = {
    "name": "Tether spool drum",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on deck in the sun, wet from the tether",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: one end on the plate", wall_loops=4, infill_pct=25),
    "mates_with": [
        {"part": "spool_flange (printed, x2)", "dimensions": f"six M4 inserts in each end on a {F.SPOOL_PCD:g} mm circle",
         "source": "hardware/parts/submarine/_frame.py", "verified": True},
        {"part": "Fathom tether", "dimensions": f"{TETHER_SLOT[0]:g} mm slot for the tether end "
                                                f"(standard {FathomTether.STANDARD_D} mm, slim {FathomTether.SLIM_D} mm)",
         "source": FathomTether.SOURCE, "verified": True},
    ],
    "inserts": [{"size": "M4", "qty": 12, "where": "both end faces, six each"}],
    "fasteners": [],
    "fit_test": False,
    "notes": [
        "The tether's topside end goes in through the slot and out through the flange's tether hole, so it can be plugged into the topside interface while the spool sits still; do not run the ROV with the tether wound tight on the drum.",
        "Brass inserts on the topside spool: rinse the spool in fresh water after salt-water days.",
    ],
}


def build():
    """In the spool frame: axle on z, the drum from z = 0 to its length."""
    from hardware.parts.common import cad
    od, idia, length = F.SPOOL_DRUM
    drum = cad.cyl(od, length, (0, 0, 0)).cut(cad.cyl(idia, length + 2, (0, 0, -1)))
    cutters = []
    for a in F.SPOOL_SCREW_ANGLES:
        x, y = cad.polar(F.SPOOL_PCD / 2, a)
        cutters.append(cad.insert_hole("M4", x, y, length))
        cutters.append(cad.insert_hole("M4", x, y, 0.0, direction=1))
    slot = cad.box(idia / 2 - 1, od / 2 + 1, -TETHER_SLOT[0] / 2, TETHER_SLOT[0] / 2, -1, TETHER_SLOT[1])
    cutters.append(cad.rotate_z(slot, F.TETHER_ANGLE))
    return cad.cut_all(drum, cutters)
