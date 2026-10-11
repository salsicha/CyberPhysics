"""IMU mount: holds the BNO085 breakout at the car's centre (the URDF's
imu_link x, y), its x axis forward, on the rear deck."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F
from hardware.parts.racecarneo._footprints import AdafruitBNO085 as B

PLATE = (30.0, 44.0, 3.0)
BOSS_H = 5.0
BOSS_D = 7.5
ARROW = 6.0

PART = {
    "name": "IMU mount (BNO085 breakout)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on the deck in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: plate on the build plate, bosses up", wall_loops=3, infill_pct=40),
    "mates_with": [
        {"part": "Adafruit BNO085 breakout (4754)",
         "dimensions": f"{B.BOARD[0]:g} x {B.BOARD[1]:g} mm board, holes {B.HOLE_SPACING[0]:g} x {B.HOLE_SPACING[1]:g} mm",
         "source": B.SOURCE, "verified": True},
        {"part": "Adafruit BNO085 hole diameter", "dimensions": f"{B.HOLE_D} mm for M2.5 (not dimensioned)",
         "source": B.SOURCE, "verified": False},
        {"part": "deck_rear (printed)", "dimensions": "two M3 x 3 inserts", "source": "hardware/parts/racecarneo/_frame.py",
         "verified": True},
    ],
    "inserts": [{"size": "M2.5", "qty": 4, "where": "IMU bosses, from the top"}],
    "fasteners": [
        {"item": "M2.5 x 6 socket head cap screw", "qty": 4, "use": "BNO085 breakout to the IMU mount"},
        {"item": "M3 x 6 socket head cap screw", "qty": 2, "use": "IMU mount to the rear deck inserts"},
    ],
    "fit_test": True,
    "notes": [
        "The arrow points forward: fit the breakout with its silkscreen x axis along the arrow (imu_link rpy 0).",
        "The IMU sits 62 mm above the URDF's nominal imu_link z (0.08 m), on the isolated deck with the lidar and "
        "camera; set imu_joint z to 0.142 m or leave it (an IMU's height does not affect its readings).",
    ],
}


def build():
    from hardware.parts.common import cad
    x, y = F.IMU_XY
    z0 = F.DECK_Z[1]
    z1 = z0 + PLATE[2]
    plate = cad.rounded_plate(PLATE[0], PLATE[1], PLATE[2], 4.0, center=(x, y, z0))
    sx, sy = B.HOLE_SPACING[0] / 2, B.HOLE_SPACING[1] / 2
    bosses = [cad.cyl(BOSS_D, BOSS_H + 0.01, (x + dx, y + dy, z1 - 0.01)) for dx in (sx, -sx) for dy in (sy, -sy)]
    arrow = cad.polygon_prism([(x + PLATE[0] / 2 - 1, y), (x + PLATE[0] / 2 - 1 - ARROW, y + ARROW / 2),
                               (x + PLATE[0] / 2 - 1 - ARROW, y - ARROW / 2)], z1 - 0.01, z1 + 1.0)
    body = cad.fuse_all([plate, arrow] + bosses)
    cutters = [cad.insert_hole("M2.5", x + dx, y + dy, z1 + BOSS_H) for dx in (sx, -sx) for dy in (sy, -sy)]
    for ix, iy in F.IMU_INSERTS:
        cutters.append(cad.clearance_hole("M3", ix, iy, z0 - 1, z1 + 1))
    return cad.cut_all(body, cutters)
