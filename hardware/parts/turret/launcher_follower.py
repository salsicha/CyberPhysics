"""Launcher clip follower: pushed down by the clip spring onto the darts;
wider than the outlet, so it stops on the cover when the clip is empty."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher clip follower",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "slides in the clip",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat, spring boss up", wall_loops=3, infill_pct=40),
    "mates_with": [{"part": "launcher_clip (printed)", "dimensions": "14.8 mm in the 15 mm clip; the outlet is 13.4 mm",
                    "source": "hardware/parts/turret/_launcher.py", "verified": True}],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": [],
}


def build():
    return F.launcher_to_turret(L.follower())
