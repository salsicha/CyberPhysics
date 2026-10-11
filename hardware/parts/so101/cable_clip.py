"""Cable clip for the 2020 camera post: an M5 screw into a drop-in T-nut
holds it in the post's slot; the RealSense USB cable snaps into the hook."""
import math

from hardware.parts.common import fasteners, printing
from hardware.parts.so101._footprints import Extrusion2020

LENGTH = 12.0             # along the post
BASE = (24.0, 4.0)        # across the slot, thickness
SCREW_X = -6.0
HOOK_X = 5.5
HOOK_ID = 6.5             # USB 3 Type-C cables are about 4.5-5.5 mm
HOOK_WALL = 2.0
OPENING = 4.2             # snap-in gap, under the cable diameter
KEY = (5.6, 1.5, 2.5)     # two keys into the 6 mm slot opening, either side of the screw: width, height, length

PART = {
    "name": "Post cable clip",
    "quantity": 3,
    "material": "PETG",
    "material_reason": "flexes so the cable snaps in without cracking",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "on its side: the clip profile flat on the plate", wall_loops=4,
                               infill_pct=100, rotate=[("X", 90)]),
    "mates_with": [
        {"part": "2020 aluminium extrusion (camera post)",
         "dimensions": f"{KEY[0]} mm key in the {Extrusion2020.SLOT_W:g} mm slot, M5 into a drop-in T-nut",
         "source": Extrusion2020.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 10 button head screw + M5 drop-in T-nut (slot 6)", "qty": 1, "use": "cable clip to the post"},
    ],
    "fit_test": False,
    "notes": ["Three on the post: the camera's USB cable runs down the post's rear slot to the board."],
}


def build():
    import cadquery as cq

    from hardware.parts.common import cad
    base = cad.box(-BASE[0] / 2, BASE[0] / 2, -LENGTH / 2, LENGTH / 2, 0.0, BASE[1])
    keys = [cad.box(SCREW_X - KEY[0] / 2, SCREW_X + KEY[0] / 2, y0, y0 + KEY[2], -KEY[1], 0.01)
            for y0 in (-LENGTH / 2, LENGTH / 2 - KEY[2])]
    ro, ri = HOOK_ID / 2 + HOOK_WALL, HOOK_ID / 2
    cz = BASE[1] + ri - 0.5
    half = math.asin(OPENING / 2 / ri)
    # C ring in the x-z plane, opening at the top, extruded along y
    a0, a1 = math.pi / 2 + half, math.pi / 2 - half + 2 * math.pi
    am = (a0 + a1) / 2

    def p(r, a):
        return (HOOK_X + r * math.cos(a), cz + r * math.sin(a))

    ring = (cq.Workplane("XZ", origin=(0, LENGTH / 2, 0))
            .moveTo(*p(ro, a0)).threePointArc(p(ro, am), p(ro, a1))
            .lineTo(*p(ri, a1)).threePointArc(p(ri, am), p(ri, a0)).close()
            .extrude(LENGTH)).val()
    clip = cad.fuse_all([base, ring] + keys)
    hole = cad.cyl(fasteners.CLEARANCE["M5"], BASE[1] + KEY[1] + 2, (SCREW_X, 0, -KEY[1] - 1))
    cable = cad.cyl(HOOK_ID, LENGTH + 2, (HOOK_X, -LENGTH / 2 - 1, cz), (0, 1, 0))
    return cad.cut_all(clip, [hole, cable])
