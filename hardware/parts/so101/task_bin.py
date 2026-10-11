"""Task bin: the scene's left (blue) and right (red) target bins,
180 x 120 x 35 mm outside, as an open tray standing on the table."""
from hardware.parts.common import printing
from hardware.parts.so101 import _station as S

WALL = 2.0
FLOOR = 2.0
CORNER_R = 8.0

PART = {
    "name": "Task bin (left blue, right red)",
    "quantity": 2,
    "material": "PETG",
    "material_reason": "indoors; print one in blue and one in red PETG to match the scene's materials",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "upright: the floor on the plate", wall_loops=2, infill_pct=15),
    "mates_with": [
        {"part": "scene bins left_bin and right_bin",
         "dimensions": "{:g} x {:g} x {:g} mm outside".format(*S.scene_box("left_bin")[1]),
         "source": "systems/so101/scenarios/picking_table.json", "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "double-sided tape pad, 20 mm", "qty": 2, "use": "bin to the board at its scene position"}],
    "fit_test": False,
    "notes": [
        "The scene models each bin as a solid box; the physical bins are trays of the same outside size.",
        "Left bin centre at station (300, 250) mm, right bin at (300, -250) mm.",
    ],
}


def build():
    from hardware.parts.common import cad
    sx, sy, sz = S.scene_box("left_bin")[1]
    outer = cad.rounded_plate(sx, sy, sz, CORNER_R)
    inner = cad.rounded_plate(sx - 2 * WALL, sy - 2 * WALL, sz, CORNER_R - WALL, center=(0, 0, FLOOR))
    return outer.cut(inner)
