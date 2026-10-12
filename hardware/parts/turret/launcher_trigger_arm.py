"""Launcher trigger arm: on the trigger servo's horn; its pin lifts the
sear's lug when the servo turns to the fire angle."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher trigger arm (on the trigger servo's horn)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on a servo that runs warm",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: the hub on the plate, pin up", wall_loops=4, infill_pct=100,
                               rotate=[("X", -90)]),
    "mates_with": [
        {"part": "Waveshare ST3215 output horn (trigger servo)",
         "dimensions": f"4 x {S.HORN_THREAD} on a {S.HORN_HOLE_CIRCLE:g} mm circle; M3 x 5 through the 3 mm hub",
         "source": S.SOURCE, "verified": True},
        {"part": "Waveshare ST3215 horn thread", "dimensions": "M3 tapped (Feetech's horn drawing; Waveshare's does not state it)",
         "source": S.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [{"item": "M3 x 5 socket head cap screw", "qty": 4,
                   "use": "trigger arm to the trigger servo's horn (no longer: the tip must stay inside the horn flange)"}],
    "fit_test": True,
    "notes": [],
}


def build():
    return F.launcher_to_turret(L.trigger_arm())
