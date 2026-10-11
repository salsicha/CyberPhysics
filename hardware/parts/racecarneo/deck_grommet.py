"""TPU top-hat grommet: two per deck mounting point (one from above, one
from below) isolate the deck, and every sensor on it, from the chassis."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F

FLANGE_D = 10.5
SLEEVE_D = 6.0
SLEEVE_L = (F.DECK_T - 0.2) / 2      # the two sleeves meet in the deck hole
BORE_D = 3.3

PART = {
    "name": "Deck grommet (TPU top hat)",
    "quantity": 8,
    "material": "TPU95A",
    "material_reason": "damper: no hard contact between the deck and the chassis",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("TPU95A", "flange down, sleeve up", wall_loops=3, infill_pct=100,
                               top_shell_layers=4, bottom_shell_layers=4),
    "mates_with": [
        {"part": "deck_front, deck_rear (printed)", "dimensions": f"{F.GROMMET_HOLE} mm hole in a {F.DECK_T:g} mm deck",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": [
        "Tighten the deck screws only until the flanges start to bulge: crushed TPU passes vibration straight through.",
    ],
}


def build():
    from hardware.parts.common import cad
    hat = cad.fuse_all([
        cad.cyl(FLANGE_D, F.GROMMET_FLANGE_T, (0, 0, 0)),
        cad.cyl(SLEEVE_D, SLEEVE_L, (0, 0, F.GROMMET_FLANGE_T)),
    ])
    return hat.cut(cad.cyl(BORE_D, F.GROMMET_FLANGE_T + SLEEVE_L + 2, (0, 0, -1))).clean()
