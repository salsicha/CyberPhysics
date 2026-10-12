"""Launcher cam plate: under the rack's front; its slot moves the
shuttle's pin 17 mm sideways during the first 30 mm of the rack's stroke."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher cam plate",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "the pin slides in its slot",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat", wall_loops=4, infill_pct=60),
    "mates_with": [{"part": "3 x 10 mm dowel pin (in the shuttle)", "dimensions": f"slot {L.SLOT_W} mm",
                    "source": "hardware/parts/turret/_launcher.py", "verified": True}],
    "inserts": [{"size": "M3-short", "qty": 2, "where": "rack screws, from the top"}],
    "fasteners": [],
    "fit_test": False,
    "notes": [],
}


def build():
    return F.launcher_to_turret(L.cam())
