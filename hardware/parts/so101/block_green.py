"""Green block: the scene's distractor and the second task's target
(40 x 55 x 40 mm, 40 g). The infill is chosen so the estimated mass
matches the scene's."""
from hardware.parts.common import printing
from hardware.parts.so101 import _objects as O
from hardware.parts.so101 import _station as S

NAME = "green_block_distractor"

PART = {
    "name": "Green block (distractor, 40 x 55 x 40 mm)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "green PETG, matte if available",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat on a 40 x 55 mm face", wall_loops=3, infill_pct=25),
    "mates_with": [
        {"part": "scene object green_block_distractor",
         "dimensions": "{:g} x {:g} x {:g} mm, {:g} g".format(*S.scene_box(NAME)[1], O.scene_mass_g(NAME)),
         "source": "systems/so101/scenarios/picking_table.json", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": ["Weigh it: 30-54 g is inside the scene's randomised mass range (0.75-1.35x)."],
}


def build():
    return O.box_object(NAME)
