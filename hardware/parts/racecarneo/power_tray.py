"""Power tray: a slotted plate for the DC-DC converters, the fuse holder
and the e-stop contactor, beside the IMU on the rear deck (two per car,
the same part turned 180 deg); everything is held with zip ties or
double-sided foam tape, since the modules' sizes vary."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F

T = 3.0
LIP_H = 4.0
LIP_W = 2.0
SLOT = (2.6, 5.0)
SLOT_X = (-26.0, -8.0, 8.0, 26.0)
SLOT_Y = (-13.0, 0.0, 13.0)

PART = {
    "name": "Power tray (converters, fuse, contactor)",
    "quantity": 2,
    "material": "ASA",
    "material_reason": "converters and the contactor coil run warm; on the deck in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: plate on the build plate, lip up", wall_loops=3, infill_pct=25),
    "mates_with": [
        {"part": "deck_rear (printed)", "dimensions": "two M3 x 3 inserts", "source": "hardware/parts/racecarneo/_frame.py",
         "verified": True},
        {"part": "DC-DC converters, fuse holder, contactor",
         "dimensions": f"any module up to about {F.POWER_SIZE[0] - 8:g} x {F.POWER_SIZE[1] - 6:g} mm, held by zip ties "
                       "through the slots or foam tape",
         "source": "Requirement of this design: module sizes vary by vendor", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 6 socket head cap screw", "qty": 2, "use": "power tray to the rear deck inserts"},
        {"item": "zip tie 2.5 mm", "qty": 4, "use": "modules and leads on the power trays"},
    ],
    "fit_test": False,
    "notes": [
        "Left tray (+y): the 19 V converter for the Jetson. Right tray (-y): the 6 V servo BEC, the main fuse and "
        "the e-stop contactor.",
    ],
}


def build():
    from hardware.parts.common import cad
    cx, cy = F.POWER_CENTERS[0]
    sx, sy = F.POWER_SIZE
    z0 = F.DECK_Z[1]
    plate = cad.rounded_plate(sx, sy, T, 4.0, center=(cx, cy, z0))
    lip = cad.rounded_plate(sx, sy, LIP_H, 4.0, center=(cx, cy, z0 + T - 0.01)).cut(
        cad.rounded_plate(sx - 2 * LIP_W, sy - 2 * LIP_W, LIP_H + 2, 2.5, center=(cx, cy, z0 + T - 1)))
    body = plate.fuse(lip).clean()
    cutters = []
    for x, y in F.POWER_INSERTS[:2]:
        cutters.append(cad.clearance_hole("M3", x, y, z0 - 1, z0 + T + LIP_H + 1))
    for dx in SLOT_X:
        for dy in SLOT_Y:
            cutters.append(cad.cbox(SLOT[0], SLOT[1], 10, center=(cx + dx, cy + dy, z0)))
    # Lip gaps for the leads, at both ends.
    for sxn in (1, -1):
        cutters.append(cad.cbox(6.0, 16.0, LIP_H + 1, center=(cx + sxn * sx / 2, cy, z0 + T + LIP_H / 2 + 0.5)))
    return cad.cut_all(body, cutters)
