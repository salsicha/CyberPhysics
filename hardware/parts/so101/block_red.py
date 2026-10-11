"""Red block: the scene's target object (45 mm cube, 45 g). The infill is
chosen so the estimated mass matches the scene's."""
from hardware.parts.common import printing
from hardware.parts.so101 import _objects as O
from hardware.parts.so101 import _station as S

NAME = "red_block_target"

PART = {
    "name": "Red block (task target, 45 mm cube)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "red PETG, matte if available (glossy prints reflect the RealSense projector)",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat on one face", wall_loops=3, infill_pct=30),
    "mates_with": [
        {"part": "scene object red_block_target",
         "dimensions": "{:g} x {:g} x {:g} mm, {:g} g".format(*S.scene_box(NAME)[1], O.scene_mass_g(NAME)),
         "source": "systems/so101/scenarios/picking_table.json", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": ["Weigh it: the scene randomises object mass by 0.75-1.35x, so 34-61 g is inside the simulated range."],
}


def build():
    return O.box_object(NAME)
