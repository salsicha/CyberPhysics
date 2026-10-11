"""Landing leg: one per arm, bolted under the hub plate ear, splayed
outward so it stays out of the nadir camera's view."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing

# Leg frame: x along the arm (radius from the hub centre), y across it,
# z up (body z). The leg is this profile extruded LEG_THICKNESS in y.
PROFILE = [
    (62.0, F.BOT_Z[0]), (106.0, F.BOT_Z[0]), (106.0, -29.0), (104.0, -29.0),
    (156.0, -188.0), (155.0, -188.0), (155.0, -200.0), (145.0, -200.0), (145.0, -188.0),
    (144.0, -188.0), (97.2, -55.0), (86.0, -29.0), (62.0, -29.0),
]
TENON = ((145.0, 155.0), (-200.0, -188.0))   # r range, z range: the TPU foot's peg

PART = {
    "name": "Landing leg",
    "quantity": 4,
    "material": "ASA",
    "material_reason": "outdoor, in the sun; takes landing loads",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings(
        "ASA", "lying flat on its side (the 12 mm thickness vertical)",
        wall_loops=5, infill_pct=40, top_shell_layers=5, bottom_shell_layers=5,
        rotate=[("X", 90)]),
    "mates_with": [
        {"part": "hub_bottom_plate (printed)", "dimensions": "two M3 inserts on the arm axis at r = 68 and 80 mm",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
        {"part": "leg_foot (printed TPU)", "dimensions": "10 x 12 mm tenon, 12 mm long",
         "source": "hardware/parts/aerodrone/leg_foot.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "M3 x 16 socket head cap screw", "qty": 2, "use": "landing leg to bottom plate"}],
    "fit_test": False,
    "notes": [
        "The legs splay 20 deg outward: the feet sit 150 mm from the centre, outside the OAK-1's 69 x 55 deg view.",
        "Ground clearance: 126 mm under the OAK-1 lens, about 130 mm under the battery.",
    ],
}


def build():
    from hardware.parts.common import cad
    t = F.LEG_THICKNESS / 2
    leg = cad.profile_xz(PROFILE, -t, t)
    holes = [cad.clearance_hole("M3", r, 0.0, -30.0, F.BOT_Z[0] + 1) for r in F.LEG_BOLT_R]
    return cad.cut_all(leg, holes)
