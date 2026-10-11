"""TPU foot pressed onto the end of each landing leg."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.aerodrone.landing_leg import TENON
from hardware.parts.common import printing

SIZE = (20.0, 22.0)    # along the arm, across it
PRESS = printing.PRESS_FIT_MM

PART = {
    "name": "Landing foot",
    "quantity": 4,
    "material": "TPU95A",
    "material_reason": "soft pad that takes the landing shock and grips the ground",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings(
        "TPU95A", "socket opening up, flat sole on the plate",
        wall_loops=3, infill_pct=30, rotate=[]),
    "mates_with": [
        {"part": "landing_leg (printed)",
         "dimensions": f"tenon {TENON[0][1] - TENON[0][0]:g} x {F.LEG_THICKNESS:g} mm, socket {-2 * PRESS:g} mm smaller (press fit)",
         "source": "hardware/parts/aerodrone/landing_leg.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": ["Push onto the leg tenon; a drop of CA glue keeps it on if it works loose."],
}


def build():
    from hardware.parts.common import cad
    (r0, r1), (z_t0, z_t1) = TENON
    rc = (r0 + r1) / 2
    sole = F.GROUND_Z
    foot = cad.rounded_plate(SIZE[0], SIZE[1], z_t1 - sole, 4.0, center=(rc, 0.0, sole))
    socket = cad.cbox(r1 - r0 + 2 * PRESS, F.LEG_THICKNESS + 2 * PRESS, z_t1 - z_t0 + 1,
                      center=(rc, 0.0, (z_t0 + z_t1) / 2 + 0.5))
    return foot.cut(socket).clean()
