"""Battery end block: two hold the 24 V LiFePO4 pack at its ends on the
body floor; each has two slots for the two straps that run over the top of
the pack from one block to the other."""
from hardware.parts.common import printing
from hardware.parts.lunar_rover._footprints import LiFePO4Pack24V

LIP = 12.0                 # how far the block reaches over the pack's end
H = 70.0
WALL = 6.0
CLEAR = 1.0                # around the pack envelope
STRAP = (22.0, 4.0)        # strap slot: width, thickness
STRAP_Y = (-50.0, 50.0)    # two straps over the pack

PART = {
    "name": "Battery end block",
    "quantity": 2,
    "material": "PETG",
    "material_reason": "inside the body; tough enough to take the pack's inertia in a bump",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("PETG", "base on the plate", wall_loops=4, infill_pct=40),
    "mates_with": [
        {"part": "24 V LiFePO4 pack (envelope)",
         "dimensions": "{:g} x {:g} x {:g} mm at most".format(*LiFePO4Pack24V.MAX_SIZE),
         "source": LiFePO4Pack24V.SOURCE, "verified": True},
        {"part": "body floor panel", "dimensions": "three M5 through the floor, nyloc nuts below",
         "source": "hardware/parts/lunar_rover/_frame.py (body layout)", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 20 socket head cap screw + M5 nyloc nut + M5 washer", "qty": 3, "use": "end block to the floor panel"},
        {"item": "battery strap, 20 mm, 700 mm, cam buckle", "qty": 1,
         "use": "over the top of the pack, through a slot in each end block (two straps in all)"},
        {"item": "closed-cell foam pad, 3 mm", "qty": 1, "use": "between the pack and the end block"},
        {"item": "closed-cell foam pad, 6 mm, 130 x 180 mm", "qty": 1, "use": "under the pack (two pads side by side)"},
    ],
    "fit_test": False,
    "notes": ["Pad a pack smaller than the envelope with foam so it cannot slide."],
}


def build():
    """One end block in its local frame: the pack's end face at x = 0, the
    pack toward -x, floor at z = 0."""
    from hardware.parts.common import cad
    lx, ly, _ = LiFePO4Pack24V.MAX_SIZE
    half = ly / 2 + CLEAR
    base = cad.box(-LIP, WALL + 20.0, -half - WALL, half + WALL, 0.0, WALL)
    back = cad.box(CLEAR, CLEAR + WALL, -half - WALL, half + WALL, 0.0, H)
    sides = [cad.box(-LIP, CLEAR + WALL, y0, y0 + WALL, 0.0, H * 0.6) for y0 in (-half - WALL, half)]
    body = cad.fuse_all([base, back] + sides)
    cutters = [cad.box(CLEAR - 0.5, CLEAR + WALL + 0.5, y - STRAP[0] / 2, y + STRAP[0] / 2, H - 14.0 - STRAP[1], H - 14.0)
               for y in STRAP_Y]
    for y in (-half + 25.0, 0.0, half - 25.0):
        cutters.append(cad.clearance_hole("M5", CLEAR + WALL + 10.0, y, -1.0, WALL + 1.0))
    return cad.cut_all(body, cutters)
