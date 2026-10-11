"""Cable clip for the 20-series extrusions (rocker and bogie beams, legs,
masts): an M5 screw into a drop-in T-nut holds it in the slot; a cable
up to 6.5 mm snaps into the hook (actuator power and CAN, sensor cables)."""
import math

from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover._footprints import Extrusion2040

LENGTH = 12.0
BASE = (24.0, 4.0)
SCREW_X = -6.0
HOOK_X = 5.5
HOOK_ID = 7.0
HOOK_WALL = 2.0
OPENING = 4.6
KEY = (5.6, 1.5, 2.5)

PART = {
    "name": "Extrusion cable clip",
    "quantity": 24,
    "material": "ASA",
    "material_reason": "outdoors in the sun; flexes enough for the cable to snap in",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "on its side: the clip profile flat on the plate", wall_loops=4,
                               infill_pct=100, rotate=[("X", 90)]),
    "mates_with": [
        {"part": "20-series extrusion slot", "dimensions": f"{KEY[0]} mm keys in the {Extrusion2040.SLOT_W:g} mm slot, M5 into a drop-in T-nut",
         "source": Extrusion2040.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "M5 x 10 button head screw + M5 drop-in T-nut (slot 6)", "qty": 1, "use": "cable clip to an extrusion"}],
    "fit_test": False,
    "notes": ["Four per side along the rocker and bogie (actuator cables), the rest on the masts."],
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
