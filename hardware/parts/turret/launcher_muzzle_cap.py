"""Launcher muzzle cap: a soft cup over the barrel's end for transport and
maintenance, with a tether eye."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher muzzle cap",
    "quantity": 1,
    "material": "TPU95A",
    "material_reason": "grips the barrel and does not scratch it",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("TPU95A", "closed end on the plate", wall_loops=3, infill_pct=40, rotate=[("Y", 90)]),
    "mates_with": [{"part": "K&S Precision Metals 8140 brass tube (barrel)",
                    "dimensions": f"{L.BARREL['od'] + 0.25:.2f} mm bore, {L.CAP_DEPTH:g} mm deep",
                    "source": "ksmetals.com, round brass tube 17/32 in x 0.014 in wall, SKU 8140", "verified": True}],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": ["Fit it, and the safety, whenever the launcher is carried or worked on."],
}


def build():
    return F.launcher_to_turret(L.muzzle_cap())
