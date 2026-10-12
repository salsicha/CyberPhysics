"""Dart detent: a soft TPU ring in the breech's barrel seat, between the
chamber and the barrel. Pointing down, a chambered dart would slide out of
the shuttle's trough into the 12.78 mm barrel (and jam the shuttle at the
next priming); the ring's lip grips its foam in the chamber, and the shot's
air pushes it through. It sits between the barrel's rear face and the
shuttle's front face."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.turret import _launcher as L

OD = L.BARREL["od"] + 0.35          # light press fit in the 13.69 mm seat
LIP_ID = 12.0                       # 0.35 mm of squeeze on the 12.7 mm dart's foam

PART = {
    "name": "Launcher dart detent (TPU ring, aerodrone)",
    "quantity": 1,
    "material": "TPU95A",
    "material_reason": "a soft lip the dart's foam pushes through",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("TPU95A", "flat", wall_loops=2, infill_pct=100),
    "min_wall_mm": 0.8,
    "mates_with": [
        {"part": "launcher_breech_down (printed): barrel seat",
         "dimensions": f"OD {OD:.2f} mm in the {L.BARREL['od'] + 0.2:.2f} mm seat, {F.DETENT_T:g} mm thick, against the "
                       "chamber's front face",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
        {"part": "Elite-size foam dart",
         "dimensions": f"lip {LIP_ID:g} mm for a {L.DART_D:g} mm dart",
         "source": "community measurements (see _launcher.py); no Hasbro drawing", "verified": False},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": True,
    "notes": ["Push it into the barrel seat from the front, before the barrel, until it is flush with the chamber's "
              "front face (a straight edge across the chamber); the barrel then stops on it.",
              "Check by hand (safety on, no spring force: before the tube is fitted): a dart dropped into the chamber "
              "muzzle down must stay when the launcher is shaken; pushed with a finger it must pass with a light push. "
              "Darts vary: too tight, set LIP_ID to 12.3 mm; too loose, 11.7 mm; and repeat the chronograph check."],
}


def build_local():
    """The ring in the launcher frame (x_L along the bore)."""
    from hardware.parts.common import cad
    x0 = L.CHAMBER[1] + 0.01
    return cad.cyl(OD, F.DETENT_T, (x0, 0, 0), (1, 0, 0)).cut(
        cad.cyl(LIP_ID, F.DETENT_T + 2, (x0 - 1, 0, 0), (1, 0, 0)))


def build():
    return L.place(build_local(), F.LAUNCHER_ORIGIN, F.LAUNCHER_ROT)
