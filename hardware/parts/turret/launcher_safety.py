"""Launcher safety catch: a slider in the sear housing. Pushed in (SAFE)
its tongue sits 0.8 mm over the caught sear, which needs 4.45 mm to
release; pulled back 5 mm (FIRE) it is clear."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher safety catch",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "a sliding part with a detent",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat, grip up", wall_loops=4, infill_pct=100),
    "mates_with": [{"part": "launcher_cap, launcher_sear (printed)", "dimensions": "slot 8.4 x 3.4 mm in the housing",
                    "source": "hardware/parts/turret/_launcher.py", "verified": True}],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": ["SAFE engages only with the plunger caught (the sear down); uncocked, nothing can fire."],
}


def build():
    return F.launcher_to_turret(L.safety())
