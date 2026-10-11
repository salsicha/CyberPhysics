"""Wheel segment: one eighth of a 360 mm wheel (0.18 m rolling radius at
the grouser tips, the simulator's): a 120 mm wide rim with a web bolted to
the hub, an outboard stiffening lip and end walls bolted to the
neighbouring segments, so the eight form a closed ring."""
import math

from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover import _wheel as W

PART = {
    "name": "Wheel segment (1/8 of a wheel)",
    "quantity": 6 * F.SEGMENTS,
    "material": "ASA",
    "material_reason": "outdoors in the sun and abraded by regolith simulant; ASA keeps its strength in the heat",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("ASA", "on one end wall: the rim arcs up to 45 deg, no supports", wall_loops=4,
                               infill_pct=25, rotate=[("Y", -F.SEGMENT_DEG / 2)]),
    "mates_with": [
        {"part": "wheel_hub (printed)", "dimensions": f"two M4 at {F.HUB_BOLT_R:g} mm radius, +-{F.SEGMENT_DEG / 4:g} deg",
         "source": "hardware/parts/lunar_rover/wheel_hub.py", "verified": True},
        {"part": "grouser (printed)", "dimensions": "two M3 per grouser through the rim",
         "source": "hardware/parts/lunar_rover/grouser.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M4 x 14 socket head cap screw + M4 washer", "qty": 2, "use": "segment web to the hub inserts (7 mm into the 8.1 mm insert)"},
        {"item": "M4 x 16 socket head cap screw + M4 nyloc nut", "qty": 2,
         "use": "end wall to the next segment's end wall (one segment of each pair carries the screws)"},
    ],
    "fit_test": False,
    "notes": [
        "Layers lie across the rim's width: the rim's bending along the axle runs in the layers, not across them.",
        "Wear part: carry two spare segments per wheel for regolith runs.",
    ],
}


def build():
    from hardware.parts.common import cad
    half = F.SEGMENT_DEG / 2
    r_in = F.RIM_OUTER_R - F.RIM_T
    rim = W.sector(r_in, F.RIM_OUTER_R, F.RIM_U[0], F.RIM_U[1], -half, half)
    web = cad.fuse_all([W.sector(F.WEB_R[0], F.WEB_INNER_R, F.WEB_U[0], F.WEB_U[1], -half, half),
                        W.sector(F.WEB_OUTER_R, F.WEB_R[1], F.WEB_U[0], F.WEB_U[1], -half, half)] +
                       [_spoke(da) for da in (-F.SEGMENT_DEG / 4, F.SEGMENT_DEG / 4)])
    lip = W.sector(F.LIP_R[0], F.LIP_R[1], F.LIP_U[0], F.LIP_U[1], -half, half)
    t = F.END_WALL_T
    arc_t = math.degrees(t / F.END_WALL_R[0])
    walls = [W.sector(F.END_WALL_R[0], F.END_WALL_R[1], F.RIM_U[0], F.RIM_U[1], a0, a1)
             for a0, a1 in ((-half, -half + arc_t), (half - arc_t, half))]
    seg = cad.fuse_all([rim, web, lip] + walls)
    cutters = []
    for da in (-F.SEGMENT_DEG / 4, F.SEGMENT_DEG / 4):
        x, _, z = W.polar_xz(F.HUB_BOLT_R, da, 0.0)
        cutters.append(cad.cyl(fasteners.CLEARANCE["M4"], F.WEB_U[1] - F.WEB_U[0] + 2, (x, F.WEB_U[0] - 1, z), (0, 1, 0)))
    for a in W.grouser_angles():
        for y in W.GROUSER_BOLT_Y:
            p0 = W.polar_xz(r_in - 1, a, y)
            n = (math.cos(math.radians(a)), 0.0, math.sin(math.radians(a)))
            cutters.append(cad.cyl(fasteners.CLEARANCE["M3"], F.RIM_T + 2, p0, n))
    for sign in (-1, 1):
        a = sign * half
        tangent = (-math.sin(math.radians(a)) * sign, 0.0, math.cos(math.radians(a)) * sign)
        for r, y in W.END_BOLTS:
            x, _, z = W.polar_xz(r, a, y)
            start = (x + 3 * tangent[0], y, z + 3 * tangent[2])
            cutters.append(cad.cyl(fasteners.CLEARANCE["M4"], 3 * t, start, (-tangent[0], 0, -tangent[2])))
    return cad.cut_all(seg, cutters)


def _spoke(angle_deg):
    """Radial spoke of the web from the bolt ring to the outer band."""
    from hardware.parts.common import cad
    s = cad.box(F.WEB_INNER_R - 2.0, F.WEB_OUTER_R + 2.0, F.WEB_U[0], F.WEB_U[1], -F.SPOKE_W / 2, F.SPOKE_W / 2)
    return cad.rotate(s, "Y", -angle_deg)
