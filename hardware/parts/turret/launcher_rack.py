"""Launcher priming rack: module 1.5 teeth down over the actuator servo's
pinion; at its rear the hook that pushes the plunger rod's collar back, at
its front a step onto the cam plate."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher priming rack",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "tough at the teeth and the hook; slides on the ASA mount",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "on its side: the tooth profile on the plate", wall_loops=4, infill_pct=60,
                               rotate=[("X", 90)]),
    "mates_with": [{"part": "launcher_pinion, launcher_servo_mount, launcher_cam, launcher_plunger (printed)",
                    "dimensions": "module 1.5, 20 deg stub teeth; hook on the 14 mm collar; two M3 into the cam plate",
                    "source": "hardware/parts/turret/_gears.py", "verified": True}],
    "inserts": [],
    "fasteners": [{"item": "M3 x 6 socket head cap screw", "qty": 2, "use": "rack to the cam plate"}],
    "fit_test": False,
    "notes": ["PTFE dry lube on the teeth and where it slides."],
}


def build():
    return F.launcher_to_turret(L.rack())
