"""Tall landing leg socket (launcher configuration): replaces a landing leg
under each plate ear (same two inserts) and holds a 12 mm carbon tube
splayed outward along the arm, in a split clamp. The foam dart launcher
hangs about 290 mm under the hub; these legs keep its muzzle
LAUNCHER_GROUND_CLEARANCE above the ground and their feet outside the
OAK-1's view."""
import math

from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing

SLAB_HALF_T = 8.0                   # across the arm (y): clear of the sensor nose
BLOCK_HALF_W = 14.0                 # round the tube axis
BLOCK_LEN = 40.0                    # along the tube from where it leaves the flange
BORE = F.TALL_LEG_TUBE[0] + 0.2
STOP_S = 1.5                        # the tube's top end stops here
SLOT = (14.0, 2.0)                  # pinch slot: from this far down the axis, width (in y)
PINCH_S = 27.0
PINCH_N = 10.5
FLANGE = (62.0, 106.0, F.BOT_Z[0] - 10.0, F.BOT_Z[0])     # r0, r1, z0, z1 (as the landing leg's)
CBORE = (6.5, 12.0)

PART = {
    "name": "Tall leg socket (launcher configuration)",
    "quantity": 4,
    "material": "ASA",
    "material_reason": "outdoor, in the sun; takes the landing loads of the long legs",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upside down: the flange's top face on the plate (the block leans out at "
                               "36 deg, no supports)", wall_loops=5, infill_pct=40, top_shell_layers=6,
                               bottom_shell_layers=6, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "hub_bottom_plate (printed)", "dimensions": "two M3 inserts on the arm axis at r = 68 and 80 mm "
                                                             "(the landing leg's)", "source": "hardware/parts/aerodrone/_frame.py",
         "verified": True},
        {"part": f"carbon fibre tube {F.TALL_LEG_TUBE[0]:g} x {F.TALL_LEG_TUBE[1]:g} mm",
         "dimensions": f"{BORE:.1f} mm split bore, two M3 pinch screws", "source": "design requirement (purchased tube)",
         "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 16 socket head cap screw", "qty": 2, "use": "tall leg socket to the bottom plate (heads in counterbores)"},
        {"item": "M3 x 20 socket head cap screw", "qty": 2, "use": "tall leg socket pinch clamp on the carbon tube"},
        {"item": "M3 nyloc nut", "qty": 2, "use": "tall leg socket pinch clamp"},
    ],
    "fit_test": False,
    "notes": ["The tube goes in from below up to the stop; tighten the pinch screws until the tube cannot turn by hand.",
              "Only with the launcher: without it, fit the standard landing legs (the aircraft stands lower)."],
}


def axis():
    """(top point (r, z), unit direction (dr, dz), outward normal (nr, nz), length to the ground)."""
    (r0, z0), (dr, dz), length = F.tall_leg_axis()
    return (r0, z0), (dr, dz), (-dz, dr), length


def point(s, n=0.0):
    (r0, z0), (dr, dz), (nr, nz), _ = axis()
    return (r0 + s * dr + n * nr, z0 + s * dz + n * nz)


def along(shape):
    """A shape built round the local +z axis (local x = outward normal) -> the leg frame on the tube axis."""
    from hardware.parts.common import cad
    (r0, z0), (dr, dz), _, _ = axis()
    # local z -> (dr, 0, dz): about y by atan2(dr, dz); local x -> (dz, 0, -dr) = -normal, so turn 180 about z first
    beta = math.degrees(math.atan2(dr, dz))
    return cad.translate(cad.rotate(cad.rotate(shape, "Z", 180.0), "Y", beta), (r0, 0.0, z0))


def build():
    from hardware.parts.common import cad
    r0, r1, fz0, fz1 = FLANGE
    pts = [(r0, fz1), (r1, fz1), point(-2.0, BLOCK_HALF_W), point(BLOCK_LEN, BLOCK_HALF_W),
           point(BLOCK_LEN, -BLOCK_HALF_W), point(-2.0, -BLOCK_HALF_W), (r0, fz0)]
    slab = cad.profile_xz(pts, -SLAB_HALF_T, SLAB_HALF_T)
    cutters = [along(cad.cyl(BORE, BLOCK_LEN + 10.0, (0, 0, STOP_S))),
               along(cad.box(-BLOCK_HALF_W - 1, BLOCK_HALF_W + 1, -SLOT[1] / 2, SLOT[1] / 2, SLOT[0], BLOCK_LEN + 5))]
    for n in (PINCH_N, -PINCH_N):
        cutters.append(along(cad.cyl(3.4, 2 * SLAB_HALF_T + 2, (n, -SLAB_HALF_T - 1, PINCH_S), (0, 1, 0))))
    for r in F.LEG_BOLT_R:
        cutters.append(cad.clearance_hole("M3", r, 0.0, fz0 - 30, fz1 + 1))
        cutters.append(cad.cyl(CBORE[0], 30.0, (r, 0.0, fz0 - 30.0)))
    return cad.cut_all(slab, cutters)


def tube_length():
    """Cut length of the carbon tube: from the stop to the foot's socket bottom."""
    from hardware.parts.aerodrone import tall_leg_foot as FT
    return FT.tube_end_s() - STOP_S
