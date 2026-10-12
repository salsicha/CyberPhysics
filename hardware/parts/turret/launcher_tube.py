"""Launcher plunger tube: a 32 mm bore with its front wall, the outlet
spigot (off-centre, on the bore) into the breech, a square front flange,
the rear flange for the cap and two lugs for the actuator servo mount."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher plunger tube",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "a pressure vessel on the tilt stage outdoors",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "standing on the rear flange, bore vertical (round); tree supports under the "
                               "front flange", wall_loops=6, infill_pct=40, supports="tree(auto), on build plate only",
                               rotate=[("Y", -90)]),
    "mates_with": [
        {"part": "NBR 70 O-ring 10 x 2 mm (outlet spigot)", "dimensions": f"groove root {L.SPIGOT_GROOVE[0]} mm, "
                                                                           f"{L.SPIGOT_GROOVE[1]} mm wide, in a {L.SPIGOT_D + 0.3:.1f} mm socket",
         "source": "standard metric O-ring size; groove for about 15 % squeeze", "verified": False},
        {"part": "launcher_plunger (printed)", "dimensions": f"bore {L.PT_BORE:g} mm, head {L.HEAD_D} mm",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
        {"part": "launcher_breech, launcher_cap, launcher_servo_mount (printed)",
         "dimensions": "four M3 through the front flange; three M3 inserts in the rear flange, two in the lugs",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
    ],
    "inserts": [{"size": "M3", "qty": 3, "where": "rear flange (cap screws), along the tube"},
                {"size": "M3", "qty": 2, "where": "lugs at -y (actuator servo mount), from below"}],
    "fasteners": [{"item": "M3 x 10 socket head cap screw", "qty": 2, "use": "tube flange to the breech, lower pair"},
                  {"item": "M3 x 8 socket head cap screw", "qty": 2, "use": "tube flange to the breech, upper pair"},
                  {"item": "NBR 70 O-ring 10 x 2 mm", "qty": 1, "use": "tube outlet spigot in the breech"}],
    "fit_test": True,
    "notes": ["Six walls make the 2.4 mm wall solid (airtight). Smooth the bore with fine wet sandpaper and silicone "
              "grease; the plunger's O-ring must slide without catching."],
}


def build():
    return F.launcher_to_turret(L.tube())
