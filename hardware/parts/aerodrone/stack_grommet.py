"""TPU top-hat grommet: two per corner (one from above, one from below)
decouple the stack tray from the top plate."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing

FLANGE_D = 10.5
FLANGE_T = 2.0
SLEEVE_D = 6.0
SLEEVE_L = (F.TRAY_T - 0.2) / 2      # the two sleeves meet in the tray hole
BORE_D = 3.3

PART = {
    "name": "Stack grommet (TPU top hat)",
    "quantity": 8,
    "material": "TPU95A",
    "material_reason": "damper: no hard contact between the stack tray and the airframe",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("TPU95A", "flange down, sleeve up", wall_loops=3, infill_pct=100,
                               top_shell_layers=4, bottom_shell_layers=4),
    "mates_with": [
        {"part": "stack_tray (printed)", "dimensions": f"{F.GROMMET_HOLE} mm hole in a {F.TRAY_T:g} mm tray",
         "source": "hardware/parts/aerodrone/stack_tray.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": [
        "Tighten the bolts only until the flanges start to bulge: crushed TPU passes vibration straight through.",
        ("TPU 95A this size is stiff: it removes high-frequency buzz and hard contact, not the 80-170 Hz rotor band. "
         "Balance the props and use ArduPilot's harmonic notch; if VIBE stays above 30 m/s/s, fit softer silicone "
         "grommets in the same holes."),
    ],
}


def build():
    from hardware.parts.common import cad
    hat = cad.fuse_all([
        cad.cyl(FLANGE_D, FLANGE_T, (0, 0, 0)),
        cad.cyl(SLEEVE_D, SLEEVE_L, (0, 0, FLANGE_T)),
    ])
    return hat.cut(cad.cyl(BORE_D, FLANGE_T + SLEEVE_L + 2, (0, 0, -1))).clean()
