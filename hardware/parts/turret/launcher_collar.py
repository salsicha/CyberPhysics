"""Launcher rod collar: screwed onto the plunger rod's tail after the rod
is through the cap; the priming rack's hook pushes it back."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher rod collar",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "the rack's hook pushes it: tough",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat", wall_loops=4, infill_pct=100, rotate=[("Y", 90)]),
    "mates_with": [{"part": "launcher_plunger, launcher_rack (printed)",
                    "dimensions": f"{L.COLLAR_D:g} mm on the {L.ROD_D:g} mm rod's end (M3 insert); the hook's slot is "
                                  f"{L.ROD_D + 0.8:g} mm",
                    "source": "hardware/parts/turret/_launcher.py", "verified": True}],
    "inserts": [],
    "fasteners": [{"item": "M3 x 8 socket head cap screw", "qty": 1, "use": "rod collar to the plunger rod's tail"}],
    "fit_test": False,
    "notes": [],
}


def build():
    return F.launcher_to_turret(L.collar())
