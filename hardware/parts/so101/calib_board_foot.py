"""Calibration board foot: two hold the 280 x 200 mm AprilGrid board
upright on the table for camera and hand-eye calibration."""
from hardware.parts.common import printing
from hardware.parts.so101 import _station as S

BASE = (70.0, 50.0, 4.0)      # along the board, across, thickness
BLOCK = (70.0, 20.0, 28.0)    # upright block with the slot
SLOT_W = 6.4                  # boards up to 6 mm (3 mm aluminium composite, 5 mm foam board)
SLOT_DEPTH = 22.0

PART = {
    "name": "Calibration board foot",
    "quantity": 2,
    "material": "PETG",
    "material_reason": "indoors; any filament works",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "upright: the base on the plate", wall_loops=3, infill_pct=20),
    "mates_with": [
        {"part": "calibration board (AprilGrid on a rigid panel)",
         "dimensions": "{:g} x {:g} mm, up to 6 mm thick".format(*[1000 * v for v in S.SCENE["calibration_board"]["size"][:2]]),
         "source": "systems/so101/scenarios/picking_table.json (calibration_board)", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": ["Push the board's 280 mm edge into both slots, about 200 mm apart; add a strip of tape if the board is thinner than 6 mm."],
}


def build():
    from hardware.parts.common import cad
    base = cad.rounded_plate(BASE[0], BASE[1], BASE[2], 6.0)
    block = cad.box(-BLOCK[0] / 2, BLOCK[0] / 2, -BLOCK[1] / 2, BLOCK[1] / 2, 0.0, BLOCK[2])
    body = base.fuse(block).clean()
    slot = cad.box(-BLOCK[0] / 2 - 1, BLOCK[0] / 2 + 1, -SLOT_W / 2, SLOT_W / 2, BLOCK[2] - SLOT_DEPTH, BLOCK[2] + 1)
    return body.cut(slot).clean()
