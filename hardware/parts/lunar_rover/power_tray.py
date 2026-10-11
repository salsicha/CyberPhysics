"""Power tray: two TS35 DIN rails for the power and safety electronics
(main fuse and breakers, the e-stop contactor, DC-DC converters, the
safety controller, the Ethernet switch, terminal blocks), standing on
feet on the body floor (rear bay)."""
from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover._footprints import DinRailTS35

T = 5.0
FOOT = 14.0
RAIL_X = (-235.0, -175.0)          # rail centre lines (rails along y)
RAIL_INSERT_Y = (-100.0, -50.0, 0.0, 50.0, 100.0)

PART = {
    "name": "Power tray (two DIN rails)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "inside the closed body, out of the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "upside down: the top face on the plate, feet up", wall_loops=3,
                               infill_pct=25, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "TS35 DIN rail, 240 mm", "dimensions": f"{DinRailTS35.WIDTH:g} mm rails, M4 through the rail slots into inserts every 50 mm",
         "source": DinRailTS35.SOURCE, "verified": True},
        {"part": "body floor panel", "dimensions": "four M4 through the floor, nyloc nuts below",
         "source": "hardware/parts/lunar_rover/_frame.py (body layout)", "verified": True},
    ],
    "inserts": [{"size": "M4", "qty": 2 * len(RAIL_INSERT_Y), "where": "top face, DIN rail screws"}],
    "fasteners": [
        {"item": "M4 x 8 pan head screw + M4 washer", "qty": 2 * len(RAIL_INSERT_Y), "use": "DIN rails to the tray"},
        {"item": "M4 x 25 socket head cap screw + M4 nyloc nut + M4 washer", "qty": 4, "use": "tray to the floor panel"},
    ],
    "fit_test": False,
    "notes": ["Cut the two rails to 240 mm. The 24 V main fuse and the contactor go on the rail nearest the battery."],
}


def build():
    from hardware.parts.common import cad
    x0, x1 = F.POWER_TRAY_X
    w = F.TRAY_HALF_W
    z0 = F.FLOOR_Z + F.FLOOR_T
    z1 = z0 + F.TRAY_FEET
    plate = cad.box(x0, x1, -w, w, z1, z1 + T)
    feet = [cad.box(x - FOOT / 2, x + FOOT / 2, y - FOOT / 2, y + FOOT / 2, z0, z1 + 0.5)
            for x in (x0 + FOOT / 2, x1 - FOOT / 2) for y in (-w + FOOT / 2, w - FOOT / 2)]
    under = [cad.box(x - 6.0, x + 6.0, -w + FOOT, w - FOOT, z0 + 4.0, z1 + 0.5) for x in RAIL_X]   # insert bosses
    body = cad.fuse_all([plate] + feet + under)
    cutters = []
    for x in RAIL_X:
        for y in RAIL_INSERT_Y:
            cutters.append(cad.insert_hole("M4", x, y, z1 + T))
    for x in (x0 + FOOT / 2, x1 - FOOT / 2):
        for y in (-w + FOOT / 2, w - FOOT / 2):
            cutters.append(cad.clearance_hole("M4", x, y, z0 - 1, z1 + T + 1))
            cutters.append(cad.cyl(fasteners.SHCS_HEAD["M4"][0] + 1.5, T + 1, (x, y, z1 + 1.5)))
    for y in (-75.0, 75.0):          # cable pass-throughs between the rails
        cutters.append(cad.slot((x0 + x1) / 2, y - 20, (x0 + x1) / 2, y + 20, 10.0, z1 - 1, z1 + T + 1))
    return cad.cut_all(body, cutters)
