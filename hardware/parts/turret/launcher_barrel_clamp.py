"""Launcher barrel clamp: a split collar around the brass barrel in front
of the breech, with an M3 pinch screw; two M3 into the breech."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher barrel clamp",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "outdoors on the RACECAR Turret",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "the rear face on the plate", wall_loops=4, infill_pct=60, rotate=[("Y", -90)]),
    "mates_with": [{"part": "K&S Precision Metals 8140 brass tube (barrel)",
                    "dimensions": f"17/32 in OD ({L.BARREL['od']} mm) in a {L.BARREL['od'] + 0.15:.2f} mm split collar",
                    "source": "ksmetals.com, round brass tube 17/32 in x 0.014 in wall, SKU 8140", "verified": True}],
    "inserts": [],
    "fasteners": [{"item": "M3 x 14 socket head cap screw", "qty": 2, "use": "barrel clamp to the breech's front wall"},
                  {"item": "M3 x 25 socket head cap screw", "qty": 1, "use": "barrel clamp pinch screw"},
                  {"item": "M3 nyloc nut", "qty": 1, "use": "barrel clamp pinch screw"}],
    "fit_test": False,
    "notes": [],
}


def build():
    return F.launcher_to_turret(L.barrel_clamp())
