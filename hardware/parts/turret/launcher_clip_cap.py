"""Launcher clip cap: the clip spring's top seat, under the two pins."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher clip cap",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "takes the clip spring",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat, spring boss up", wall_loops=3, infill_pct=40, rotate=[("X", 180)]),
    "mates_with": [{"part": "launcher_clip (printed)", "dimensions": "under the two M3 pins",
                    "source": "hardware/parts/turret/_launcher.py", "verified": True}],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": [],
}


def build():
    return F.launcher_to_turret(L.clip_cap())
