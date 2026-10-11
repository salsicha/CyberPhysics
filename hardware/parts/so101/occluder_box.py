"""Occluder: the scene's matte grey box (50 x 160 x 70 mm) that hides
part of the work area from the camera."""
from hardware.parts.common import printing
from hardware.parts.so101 import _objects as O
from hardware.parts.so101 import _station as S

NAME = "occluder_box"

PART = {
    "name": "Occluder box (50 x 160 x 70 mm)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "matte grey PETG (the scene's matte grey cardboard; a cardboard box of the same size also works)",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat on its 50 x 160 mm face", wall_loops=2, infill_pct=8),
    "mates_with": [
        {"part": "scene asset occluder_box",
         "dimensions": "{:g} x {:g} x {:g} mm".format(*S.scene_box(NAME)[1]),
         "source": "systems/so101/scenarios/picking_table.json", "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "double-sided tape pad, 20 mm", "qty": 2, "use": "occluder to the board at its scene pose"}],
    "fit_test": False,
    "notes": ["Scene pose: centre at station (460, -20) mm, turned 0.12 rad (6.9 deg) about z."],
}


def build():
    return O.box_object(NAME)
