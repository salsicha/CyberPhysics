"""Launcher actuator servo mount: hangs from the plunger tube's two lugs;
a cradle for the actuator ST3215 (ID 14) under the rack, a shoe and a lip
that keep the rack on the pinion."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher actuator servo mount",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "holds a servo that runs warm; outdoors on the RACECAR Turret",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "on its side: the outer wall on the plate", wall_loops=4, infill_pct=25,
                               rotate=[("X", 90)]),
    "mates_with": [
        {"part": "Waveshare ST3215 servo (actuator, ID 14)",
         "dimensions": f"case {S.LENGTH} x {S.WIDTH} x {S.THICKNESS:g} mm; M2 into the idler-face hole pair at X = "
                       f"{S.IDLER_FACE_HOLES_X[1]}, +-{S.FACE_HOLE_Y} mm",
         "source": S.SOURCE, "verified": True},
        {"part": "ST3215 bus plugs", "dimensions": "window in the wall at X 20.9-26.8 mm (scaled)",
         "source": S.SOURCE, "verified": False},
        {"part": "launcher_tube (printed)", "dimensions": "two M3 x 10 into the lugs' inserts",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "M3 x 10 socket head cap screw", "qty": 2, "use": "actuator servo mount to the tube's lugs"},
                  {"item": "M2 x 6 socket head cap screw (or PA2.0 x 6 self-tapping)", "qty": 2,
                   "use": "actuator servo into the mount's wall (heads in counterbores)"},
                  {"item": "zip tie 2.5 mm", "qty": 1, "use": "actuator servo cable at the mount"}],
    "fit_test": True,
    "notes": [],
}


def build():
    return F.launcher_to_turret(L.servo_mount())
