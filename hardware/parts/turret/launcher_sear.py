"""Launcher sear: a plate sliding up and down in the housing; its notch
drops into the rod's groove and holds the drawn plunger. The trigger's pin
lifts it by the lug; the safety's tongue blocks it."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher sear",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "tough, low wear on the rod",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat: the plate on the build plate", wall_loops=4, infill_pct=100,
                               rotate=[("Y", 90)]),
    "mates_with": [{"part": "launcher_plunger, launcher_cap (printed)",
                    "dimensions": f"notch {L.SEAR_SLOT_W} mm over the {L.GROOVE_CORE_D:g} mm groove core; "
                                  f"{L.sear_release_lift():.2f} mm lift releases it",
                    "source": "hardware/parts/turret/_launcher.py", "verified": True}],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": [],
}


def build():
    return F.launcher_to_turret(L.sear())
