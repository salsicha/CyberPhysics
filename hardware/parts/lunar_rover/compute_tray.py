"""Compute tray: the onboard x86-64 mini PC on its VESA 75 or 100 mounting
holes, standing on feet on the body floor (front bay)."""
from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover._footprints import VesaMisD

T = 5.0
FOOT = 14.0                    # feet: square, at the corners
FLOOR_BOLTS = 30.0             # floor bolts this far in from the corners

PART = {
    "name": "Compute tray (VESA 75/100)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "inside the closed body, out of the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "upside down: the top face on the plate, feet up", wall_loops=3,
                               infill_pct=25, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "mini PC VESA mount", "dimensions": "4 x M4 on 75 x 75 and on 100 x 100 mm",
         "source": VesaMisD.SOURCE, "verified": True},
        {"part": "body floor panel", "dimensions": "four M4 through the floor, nyloc nuts below",
         "source": "hardware/parts/lunar_rover/_frame.py (body layout)", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M4 x 10 socket head cap screw", "qty": 4, "use": "mini PC to the tray (VESA holes, from below)"},
        {"item": "M4 x 25 socket head cap screw + M4 nyloc nut + M4 washer", "qty": 4, "use": "tray to the floor panel"},
        {"item": "hook-and-loop strap, 20 mm, 400 mm", "qty": 1, "use": "mini PC over the tray, through the strap slots"},
    ],
    "fit_test": False,
    "notes": ["Ventilation: the windows in the tray let air under the PC; keep its fan intakes clear."],
}


def build():
    from hardware.parts.common import cad
    x0, x1 = F.COMPUTE_TRAY_X
    w = F.TRAY_HALF_W
    z0 = F.FLOOR_Z + F.FLOOR_T
    z1 = z0 + F.TRAY_FEET
    plate = cad.box(x0, x1, -w, w, z1, z1 + T)
    feet = [cad.box(x - FOOT / 2, x + FOOT / 2, y - FOOT / 2, y + FOOT / 2, z0, z1 + 0.5)
            for x in (x0 + FOOT / 2, x1 - FOOT / 2) for y in (-w + FOOT / 2, w - FOOT / 2)]
    ribs = [cad.box(x0, x1, y - 2.0, y + 2.0, z0 + 3.0, z1 + 0.5) for y in (-w + 2, w - 2)]
    body = cad.fuse_all([plate] + feet + ribs)
    cx = (x0 + x1) / 2
    cutters = []
    for p in VesaMisD.PATTERNS:
        for sx in (-1, 1):
            for sy in (-1, 1):
                cutters.append(cad.clearance_hole("M4", cx + sx * p / 2, sy * p / 2, z1 - 1, z1 + T + 1))
    for x in (x0 + FOOT / 2, x1 - FOOT / 2):
        for y in (-w + FOOT / 2, w - FOOT / 2):
            cutters.append(cad.clearance_hole("M4", x, y, z0 - 1, z1 + T + 1))
            cutters.append(cad.cyl(fasteners.SHCS_HEAD["M4"][0] + 1.5, T + 1, (x, y, z1 + 1.5)))
    for y in (-w + 30.0, w - 30.0):
        cutters.append(cad.slot(cx - 25, y, cx + 25, y, 4.0, z1 - 1, z1 + T + 1))
    for y in (-45.0, 45.0):
        cutters.append(cad.rounded_plate(40.0, 30.0, T + 2, 5.0, center=(cx, y, z1 - 1)))
    return cad.cut_all(body, cutters)
