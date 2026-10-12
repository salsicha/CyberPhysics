"""Launcher pinion: 18 teeth, module 1.5 (13.5 mm pitch radius), on the
actuator servo's horn; it drives the priming rack: 38 mm of stroke is 161
deg of the servo."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher pinion (on the actuator servo's horn)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on a servo that runs warm; runs against the PETG rack",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: the horn face on the plate, counterbores up", wall_loops=4,
                               infill_pct=100, rotate=[("X", 90)]),
    "mates_with": [
        {"part": "Waveshare ST3215 output horn (actuator servo)",
         "dimensions": f"4 x {S.HORN_THREAD} on a {S.HORN_HOLE_CIRCLE:g} mm circle; M3 x 5 through 3 mm of gear",
         "source": S.SOURCE, "verified": True},
        {"part": "Waveshare ST3215 horn thread", "dimensions": "M3 tapped (Feetech's horn drawing; Waveshare's does not state it)",
         "source": S.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [{"item": "M3 x 5 socket head cap screw", "qty": 4,
                   "use": "pinion to the actuator servo's horn (no longer: the tip must stay inside the horn flange)"}],
    "fit_test": True,
    "notes": [],
}


def build():
    return F.launcher_to_turret(L.pinion())
