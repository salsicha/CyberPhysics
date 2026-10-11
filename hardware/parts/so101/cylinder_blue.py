"""Blue cylinder: the scene's round distractor (54 mm diameter, 60 mm
tall, 35 g). The infill is chosen so the estimated mass matches."""
from hardware.parts.common import printing
from hardware.parts.so101 import _objects as O
from hardware.parts.so101 import _station as S

NAME = "blue_cylinder_distractor"

PART = {
    "name": "Blue cylinder (distractor, 54 x 60 mm)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "blue PETG, matte if available",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "standing on an end face", wall_loops=2, infill_pct=12),
    "mates_with": [
        {"part": "scene object blue_cylinder_distractor",
         "dimensions": "{:g} mm diameter, {:g} mm tall, {:g} g".format(
             S.scene_box(NAME)[1][0], S.scene_box(NAME)[1][2], O.scene_mass_g(NAME)),
         "source": "systems/so101/scenarios/picking_table.json", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": ["Weigh it: 26-47 g is inside the scene's randomised mass range (0.75-1.35x)."],
}


def build():
    import cadquery as cq
    d, _, h = S.scene_box(NAME)[1]
    return (cq.Workplane("XY").circle(d / 2).extrude(h).edges().chamfer(O.CHAMFER)).val()
