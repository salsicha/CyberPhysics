"""Launcher rear cap: the spring's rear seat and the rod's bushing; behind
it the sear housing (sear slot and spring, the safety's slot), and at +y
the trigger servo's cradle."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher rear cap, sear housing and trigger servo cradle",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "holds a servo that runs warm; outdoors on the RACECAR Turret",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "the cap's front face on the plate, housing and cradle up", wall_loops=4,
                               infill_pct=25, rotate=[("Y", 90)]),
    "mates_with": [
        {"part": "Waveshare ST3215 servo (trigger, ID 15)",
         "dimensions": f"case {S.LENGTH} x {S.WIDTH} x {S.THICKNESS:g} mm; M2 into the idler-face hole pair at X = "
                       f"{S.IDLER_FACE_HOLES_X[1]}, +-{S.FACE_HOLE_Y} mm",
         "source": S.SOURCE, "verified": True},
        {"part": "ST3215 bus plugs", "dimensions": "on the idler face at X 20.9-26.8 mm, clear of the cradle wall (scaled)",
         "source": S.SOURCE, "verified": False},
        {"part": "launcher_tube, launcher_sear, launcher_safety (printed)",
         "dimensions": "three M3 x 12 into the tube's rear flange; sear slot; safety slot",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "M3 x 12 socket head cap screw", "qty": 3, "use": "rear cap to the plunger tube's rear flange"},
                  {"item": "M2 x 6 socket head cap screw (or PA2.0 x 6 self-tapping)", "qty": 2,
                   "use": "trigger servo into the cap's cradle wall (heads in counterbores)"},
                  {"item": "zip tie 2.5 mm", "qty": 1, "use": "trigger servo cable at the cap's cradle"}],
    "fit_test": True,
    "notes": ["Fit the cap screws before the trigger servo: its horn covers them."],
}


def build():
    return F.launcher_to_turret(L.cap())
