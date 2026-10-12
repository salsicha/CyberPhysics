"""Launcher shuttle: the sliding breech block. Its trough is the chamber;
the rack's cam moves it 17 mm sideways under the clip and back onto the
bore through the pin in its tab."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher shuttle (sliding breech block)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "slides in the ASA breech: PETG on ASA wears less than ASA on ASA",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "upright: the underside on the plate, trough up", wall_loops=4, infill_pct=40),
    "mates_with": [
        {"part": "Elite-size foam dart", "dimensions": f"{L.DART_D} mm x up to {L.DART_LEN} mm in a {2 * L.TROUGH_R:g} mm "
                                                         f"trough {L.CHAMBER[1] - L.CHAMBER[0]:g} mm long",
         "source": "community measurements (blasterhub.com darts comparison, blastercentral.com); no Hasbro drawing",
         "verified": False},
        {"part": "launcher_breech (printed)", "dimensions": f"{L.SHUTTLE_CLEAR} mm clearance in the channel",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
        {"part": "3 x 10 mm steel dowel pin", "dimensions": f"pressed into a {L.PIN_HOLE_D} mm hole in the tab",
         "source": "ISO 2338 / ISO 8734 dowel pins, 3 mm nominal", "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "3 x 10 mm steel dowel pin (ISO 8734)", "qty": 1, "use": "cam pin in the shuttle's tab, pressed in"}],
    "fit_test": True,
    "notes": ["Grease its top and end faces lightly (silicone grease): they seal the chamber."],
}


def build():
    return F.launcher_to_turret(L.shuttle())
