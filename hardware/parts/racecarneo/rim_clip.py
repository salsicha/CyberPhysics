"""Rim cable clip: snaps over the deck's edge rim and holds a cable bundle
(USB, power leads) up to 10 mm across along the deck edge."""
import math

from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F

WIDTH = 10.0                  # along the rim
GRIP = F.RIM_W + 0.15         # slight interference after printing shrink
WALL = 2.0
RING_ID = 10.0
RING_WALL = 2.0
OPENING_DEG = 70.0

PART = {
    "name": "Rim cable clip",
    "quantity": 10,
    "material": "PETG",
    "material_reason": "flexes to snap on and to take the cable without cracking",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "on its side: the clip profile flat on the plate",
                               wall_loops=4, infill_pct=100, rotate=[("Y", 90)]),
    "mates_with": [
        {"part": "deck_front, deck_rear rims (printed)", "dimensions": f"{F.RIM_W:g} x {F.RIM_H:g} mm rim",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": [
        "Push down over the rim, ring inboard; open the ring with a thumb to lay the cable in.",
        f"Positions: x = {', '.join(f'{x:g}' for x in F.RIM_CLIP_X)} mm on both sides (the assembly figure shows them).",
    ],
}


def build():
    """Clip on the left (+y) rim at x = 0, ring inboard (toward -y)."""
    import cadquery as cq

    from hardware.parts.common import cad
    w = F.DECK_HALF_W
    z_top = F.DECK_Z[1] + F.RIM_H
    y_in, y_out = w - F.RIM_W, w
    cap = cad.box(-WIDTH / 2, WIDTH / 2, y_in - WALL, y_out + WALL, z_top, z_top + WALL)
    legs = [cad.box(-WIDTH / 2, WIDTH / 2, y_in - WALL, y_in - (GRIP - F.RIM_W) / 2, z_top - F.RIM_H + 1.0, z_top + 0.01),
            cad.box(-WIDTH / 2, WIDTH / 2, y_out + (GRIP - F.RIM_W) / 2, y_out + WALL, z_top - F.RIM_H + 1.0, z_top + 0.01)]
    # Cable ring beside the cap, inboard, opening up.
    ro, ri = RING_ID / 2 + RING_WALL, RING_ID / 2
    rc_y, rc_z = y_in - WALL - ri, z_top + WALL - ro
    half = math.radians(OPENING_DEG / 2)
    a0, a1 = math.pi / 2 + half, math.pi / 2 - half + 2 * math.pi
    am = (a0 + a1) / 2

    def p(r, a):
        return (rc_y + r * math.cos(a), rc_z + r * math.sin(a))

    ring = (cq.Workplane("YZ", origin=(-WIDTH / 2, 0, 0))
            .moveTo(*p(ro, a0)).threePointArc(p(ro, am), p(ro, a1))
            .lineTo(*p(ri, a1)).threePointArc(p(ri, am), p(ri, a0))
            .close().extrude(WIDTH)).val()
    return cad.fuse_all([cap, ring] + legs)
