"""VESC tray: corner guides and two strap slots hold a VESC 6 MkVI-size
controller (75 x 70 x 18 mm) on the rear deck, between the turret post
positions; open sides for the battery and motor leads."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F
from hardware.parts.racecarneo._footprints import Vesc6MkVI as V

CLEAR = 0.6
GUIDE_H = 8.0
GUIDE_LEN = 14.0
WALL = 3.0
PLATE_T = 3.0
STRAP_SLOT = (3.5, 22.0)            # 20 mm hook-and-loop straps, across y
X = (-143.0, -51.0)
HALF_W = 39.0                       # clear of the RACECAR Turret's post flanges
STRAP_X = (-12.0, 12.0)             # relative to the VESC centre
TAB_HALF_W = 46.5

PART = {
    "name": "VESC tray",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "carries the motor controller, whose heatsink runs hot",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: plate on the build plate, guides up", wall_loops=3, infill_pct=30),
    "mates_with": [
        {"part": "Trampa VESC 6 MkVI", "dimensions": f"{V.SIZE[0]:g} x {V.SIZE[1]:g} x {V.SIZE[2]:g} mm housing "
                                                      f"between corner guides ({CLEAR:g} mm clearance)",
         "source": V.SOURCE, "verified": True},
        {"part": "deck_rear (printed)", "dimensions": "four M3 x 3 inserts", "source": "hardware/parts/racecarneo/_frame.py",
         "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 6 socket head cap screw", "qty": 4, "use": "VESC tray to the rear deck inserts"},
        {"item": "hook-and-loop strap, 20 mm, 200 mm", "qty": 2, "use": "VESC to its tray"},
        {"item": "silicone thermal pad or rubber pad, 70 x 60 x 1 mm", "qty": 1, "use": "between the VESC and its tray"},
    ],
    "fit_test": False,
    "notes": [
        "A smaller VESC (e.g. Flipsky Mini FSESC 6.7, 67 x 39 mm) fits between the guides with a foam spacer, but "
        "check its input range: the Mini FSESC 6.7 needs 4S or more and does not run on this car's 3S pack.",
        "Battery leads to the e-stop contactor, motor leads down through the deck's wire window ahead of the tray.",
    ],
}


def build():
    from hardware.parts.common import cad
    cx, cy = F.VESC_CENTER
    z0 = F.DECK_Z[1]
    z1 = z0 + PLATE_T
    hx, hy = V.SIZE[0] / 2 + CLEAR, V.SIZE[1] / 2 + CLEAR
    plate = cad.fuse_all([cad.rounded_plate(X[1] - X[0], 2 * HALF_W, PLATE_T, 5.0, center=((X[0] + X[1]) / 2, cy, z0))]
                         + [cad.rounded_plate(STRAP_SLOT[1] + 4, 2 * TAB_HALF_W, PLATE_T, 3.0, center=(cx + dx, cy, z0))
                            for dx in STRAP_X])
    guides = []
    for sx in (1, -1):
        for sy in (1, -1):
            ox, oy = cx + sx * hx, cy + sy * hy
            guides.append(cad.box(*sorted((ox, ox + sx * WALL)), *sorted((oy - sy * GUIDE_LEN, oy + sy * WALL)),
                                  z1 - 0.01, z1 + GUIDE_H))
            guides.append(cad.box(*sorted((ox - sx * GUIDE_LEN, ox + sx * WALL)), *sorted((oy, oy + sy * WALL)),
                                  z1 - 0.01, z1 + GUIDE_H))
    body = cad.fuse_all([plate] + guides)
    cutters = []
    for x, y in F.VESC_INSERTS:
        cutters.append(cad.clearance_hole("M3", x, y, z0 - 1, z1 + 1))
    for dx in STRAP_X:
        for sy in (1, -1):
            cutters.append(cad.cbox(STRAP_SLOT[1], STRAP_SLOT[0], 10, center=(cx + dx, cy + sy * (TAB_HALF_W - 3.5), z0)))
    cutters.append(cad.rounded_plate(2 * hx - 24, 2 * hy - 24, 10, 8.0, center=(cx, cy, z0 - 1)))
    return cad.cut_all(body, cutters)
