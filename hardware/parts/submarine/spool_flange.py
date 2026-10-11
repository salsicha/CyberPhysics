"""Tether spool flange: one each end of the drum; a hub for the hand-held
20 mm axle, lightening windows, the tether exit hole and a boss for the
crank handle."""
from hardware.parts.common import fasteners, printing
from hardware.parts.submarine import _frame as F

WINDOWS = (6, 34.0, 82.0)       # count, diameter, pitch-circle radius
CRANK_BOSS = (22.0, 8.0)        # diameter, height (outward)
TETHER_HOLE_D = 14.0
NUT_POCKET = (8.0 + 0.3, 5.2)   # M5 nyloc (DIN 985: 8 mm AF, 5 mm high), flush in the inner face
AXLE_FIT = 1.0                  # diametral clearance on the axle: it turns freely

PART = {
    "name": "Tether spool flange",
    "quantity": 2,
    "material": "ASA",
    "material_reason": "on deck in the sun, wet from the tether",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "inner face (against the drum) on the plate, hub and crank boss up",
                               wall_loops=4, infill_pct=25, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "spool_drum (printed)", "dimensions": f"six M4 screws on a {F.SPOOL_PCD:g} mm circle into its inserts",
         "source": "hardware/parts/submarine/_frame.py", "verified": True},
        {"part": "20 mm axle (wooden dowel or aluminium tube)", "dimensions": f"{F.SPOOL_AXLE + AXLE_FIT:g} mm hub bore",
         "source": "requirement of this design", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M4 x 10 socket head cap screw, A4 (316) stainless", "qty": 6, "use": "spool flange to the drum inserts"},
    ],
    "fit_test": False,
    "notes": [
        "Both flanges are the same; the crank handle goes on one of them, the tether leaves through the other's tether hole.",
    ],
}


def build():
    """In the spool frame: the flange's inner face (against the drum) at z = 0,
    its outer face and hub toward -z."""
    import math

    from hardware.parts.common import cad
    d, t = F.SPOOL_FLANGE
    hub_d, hub_l = F.SPOOL_HUB
    cx, cy = cad.polar(F.CRANK_R, F.CRANK_ANGLE)
    body = cad.fuse_all([
        cad.cyl(d, t, (0, 0, -t)),
        cad.cyl(hub_d, hub_l + 0.01, (0, 0, -t - hub_l)),
        cad.cyl(CRANK_BOSS[0], CRANK_BOSS[1] + 0.01, (cx, cy, -t - CRANK_BOSS[1])),
    ])
    head_d = fasteners.SHCS_HEAD["M4"][0]
    cutters = [cad.cyl(F.SPOOL_AXLE + AXLE_FIT, t + hub_l + 2, (0, 0, -t - hub_l - 1)),
               cad.cyl(5.5, t + CRANK_BOSS[1] + 2, (cx, cy, -t - CRANK_BOSS[1] - 1))]
    af = NUT_POCKET[0]
    r = af / math.sqrt(3)
    hexagon = [(cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a))) for a in range(0, 360, 60)]
    cutters.append(cad.polygon_prism(hexagon, -NUT_POCKET[1], 0.01))
    tx, ty = cad.polar(F.TETHER_HOLE_R, F.TETHER_ANGLE)
    cutters.append(cad.cyl(TETHER_HOLE_D, t + 2, (tx, ty, -t - 1)))
    n, wd, wr = WINDOWS
    for k in range(n):
        wx, wy = cad.polar(wr, k * 360.0 / n)
        cutters.append(cad.cyl(wd, t + 2, (wx, wy, -t - 1)))
    for a in F.SPOOL_SCREW_ANGLES:
        x, y = cad.polar(F.SPOOL_PCD / 2, a)
        cutters.append(cad.clearance_hole("M4", x, y, -t - 1, 1))
    assert head_d + 1 < 2 * (F.SPOOL_PCD / 2 - hub_d / 2)
    return cad.cut_all(body, cutters)
