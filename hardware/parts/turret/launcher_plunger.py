"""Launcher plunger: the head (O-ring) and its rod, the spring's guide,
with the sear's groove; the rack's hook pushes the collar screwed on its
tail."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher plunger (head and rod)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "tough: it takes the shot's impact and the sear's load",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "head down, rod vertical", wall_loops=4, infill_pct=100, rotate=[("Y", 90)]),
    "mates_with": [
        {"part": "NBR 70 O-ring 27 x 2.5 mm (plunger head)", "dimensions": f"groove root {L.HEAD_GROOVE[0]} mm, "
                                                                             f"{L.HEAD_GROOVE[1]} mm wide, in the {L.PT_BORE:g} mm bore",
         "source": "standard metric O-ring size; groove for a light dynamic squeeze", "verified": False},
        {"part": "launcher_cap, launcher_sear, launcher_rack (printed)",
         "dimensions": f"rod {L.ROD_D:g} mm; groove core {L.GROOVE_CORE_D:g} mm; collar {L.COLLAR_D:g} mm",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
    ],
    "inserts": [{"size": "M3", "qty": 1, "where": "rod's tail end, along the rod (the collar's screw)"}],
    "fasteners": [{"item": "NBR 70 O-ring 27 x 2.5 mm", "qty": 1, "use": "plunger head seal"}],
    "fit_test": True,
    "notes": ["Silicone grease on the O-ring. The collar goes on after the rod is through the cap."],
}


def build():
    return F.launcher_to_turret(L.plunger())
