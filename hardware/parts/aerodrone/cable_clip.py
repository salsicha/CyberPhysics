"""Snap-on clip for the 16 mm arm tube with a cable-tie tab underneath:
holds the motor leads and ESC signal wire along each arm."""
import math

from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.common.footprints import CarbonTube16

WIDTH = 10.0               # along the tube
RING_ID = F.TUBE_OD - 0.2  # slight grip
RING_WALL = 2.4
OPENING_DEG = 100.0        # gap at the top, centred on +z
TAB = (12.0, 5.0)          # width across the tube, thickness
TIE_SLOT = (3.0, 1.8)      # along the tube, across

PART = {
    "name": "Arm cable clip",
    "quantity": 8,
    "material": "PETG",
    "material_reason": "flexes to snap on without cracking",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "on its side: the C profile flat on the plate",
                               wall_loops=4, infill_pct=100, rotate=[("Y", 90)]),
    "mates_with": [
        {"part": "16 mm carbon arm tube", "dimensions": f"{RING_ID:.1f} mm ring snaps over the {CarbonTube16.OD:g} mm tube",
         "source": CarbonTube16.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "zip tie 2.5 mm", "qty": 1, "use": "motor leads to the arm cable clip"}],
    "fit_test": False,
    "notes": ["Two per arm: snap on from above, the tab under the tube; tie the motor and ESC leads to the tab."],
}


def build():
    import cadquery as cq

    from hardware.parts.common import cad
    ro, ri = RING_ID / 2 + RING_WALL, RING_ID / 2
    half = math.radians(OPENING_DEG / 2)
    # C profile in the y-z plane, extruded along x.
    a0 = math.pi / 2 + half
    a1 = math.pi / 2 - half + 2 * math.pi
    am = (a0 + a1) / 2

    def p(r, a):
        return (r * math.cos(a), r * math.sin(a))

    ring = (cq.Workplane("YZ", origin=(-WIDTH / 2, 0, 0))
            .moveTo(*p(ro, a0)).threePointArc(p(ro, am), p(ro, a1))
            .lineTo(*p(ri, a1))
            .threePointArc(p(ri, am), p(ri, a0))
            .close().extrude(WIDTH)).val()
    tab = cad.box(-WIDTH / 2, WIDTH / 2, -TAB[0] / 2, TAB[0] / 2, -ro - TAB[1], -ri - 0.5)
    clip = ring.fuse(tab).clean()
    slots = [cad.cbox(TIE_SLOT[0], TIE_SLOT[1], 20, center=(0, y, -ro - TAB[1] / 2)) for y in (-3.5, 3.5)]
    return cad.cut_all(clip, slots)
