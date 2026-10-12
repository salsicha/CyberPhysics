"""Launcher clip: the breech's cover (the chamber's roof, with the outlet
over the shuttle's load position) and the straight clip of six darts on it,
one part. A follower and a light spring push the darts down; two M3 pins
hold the clip cap."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher clip and breech cover",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on the tilt stage outdoors; the roof must stay flat",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "roof down: the cover's underside on the plate, clip up", wall_loops=3,
                               infill_pct=20),
    "mates_with": [
        {"part": "Elite-size foam dart", "dimensions": f"outlet {L.OUTLET[3] - L.OUTLET[2]:.1f} x {L.OUTLET[1] - L.OUTLET[0]:.1f} mm, "
                                                         f"clip {L.CLIP_IN_Y[1] - L.CLIP_IN_Y[0]:g} mm wide, six darts",
         "source": "community measurements (blasterhub.com darts comparison, blastercentral.com); no Hasbro drawing",
         "verified": False},
        {"part": "launcher_breech (printed)", "dimensions": "four M3 x 10 into the wall bosses",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "M3 x 10 socket head cap screw", "qty": 4, "use": "launcher clip (cover) to the breech"},
                  {"item": "M3 x 25 socket head cap screw", "qty": 2, "use": "clip cap pins, across the clip"},
                  {"item": "M3 nyloc nut", "qty": 2, "use": "clip cap pins"}],
    "fit_test": True,
    "notes": ["Reload: take out the two pins, lift the cap, spring and follower out, drop up to six darts in, refit."],
}


def build():
    return F.launcher_to_turret(L.clip())
