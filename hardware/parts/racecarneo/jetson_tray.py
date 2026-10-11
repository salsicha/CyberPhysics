"""Jetson tray: carries the Jetson Orin Nano (Super) developer kit's carrier
board (without its plastic stand) on four bosses, in front of the lidar and
below its scan plane, I/O edge forward."""
from hardware.parts.common import fasteners, printing
from hardware.parts.racecarneo import _frame as F
from hardware.parts.racecarneo._footprints import JetsonOrinNanoDevKit as J

X = (F.JETSON_X[0], F.DECK_X[1] - 1.0)
HALF_W = 54.0
BOSS_D = 7.5
MOUNT_RELIEF_D = 13.0           # clears the deck grommet washers
WINDOW = (134.0, 176.0, -30.0, 30.0)

PART = {
    "name": "Jetson tray (Orin Nano developer kit carrier)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "under the Jetson's carrier board, which runs warm, and in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: plate on the build plate, bosses up", wall_loops=3, infill_pct=30),
    "mates_with": [
        {"part": "Jetson Orin Nano developer kit carrier (P3768)",
         "dimensions": f"{J.CARRIER[0]:g} x {J.CARRIER[1]:g} mm board; bottom parts at most {J.BOTTOM_PARTS_H:g} mm "
                       f"(bosses {F.JETSON_BOSS_H:g} mm)",
         "source": J.SOURCE, "verified": True},
        {"part": "Jetson carrier mounting holes",
         "dimensions": "4 x M2.5, 86 x 58 mm, 17.0 mm from the I/O edge and 4.0 mm from the DC-jack end "
                       "(scaled from the drawing; the specification does not dimension them)",
         "source": J.SOURCE, "verified": False},
        {"part": "deck_front (printed)", "dimensions": "four M3 x 3 inserts",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
    ],
    "inserts": [{"size": "M2.5", "qty": 4, "where": "carrier bosses, from the top"}],
    "fasteners": [
        {"item": "M2.5 x 6 socket head cap screw", "qty": 4, "use": "Jetson carrier to the tray bosses"},
        {"item": "M3 x 6 socket head cap screw", "qty": 4, "use": "Jetson tray to the front deck inserts"},
        {"item": "zip tie 2.5 mm", "qty": 4, "use": "USB cables along the Jetson tray's sides"},
    ],
    "fit_test": True,
    "notes": [
        "Take the carrier off its plastic stand (four screws). Keep the stand: the tray replaces it on the car only.",
        "Screw the tray to the deck before fitting the Jetson: the tray screws sit under the carrier.",
        "The fan's top ends about 9 mm under the lidar's scan plane; nothing may be stacked on the Jetson.",
    ],
}


def build():
    from hardware.parts.common import cad
    z0 = F.DECK_Z[1]
    z1 = z0 + F.JETSON_TRAY_T
    plate = cad.rounded_plate(X[1] - X[0], 2 * HALF_W, F.JETSON_TRAY_T, 6.0,
                              center=((X[0] + X[1]) / 2, 0.0, z0))
    holes = F.jetson_holes()
    body = cad.fuse_all([plate] + [cad.cyl(BOSS_D, F.JETSON_BOSS_H + 0.01, (x, y, z1 - 0.01)) for x, y in holes])
    cutters = [cad.insert_hole("M2.5", x, y, z1 + F.JETSON_BOSS_H) for x, y in holes]
    head_d, head_h = fasteners.SHCS_HEAD["M3"]
    for x, y in F.JETSON_TRAY_INSERTS:
        cutters.append(cad.clearance_hole("M3", x, y, z0 - 1, z1 + 1))
    for x, y in F.MOUNTS["front"]:
        cutters.append(cad.cyl(MOUNT_RELIEF_D, 10, (x, y, z0 - 1)))
    wx0, wx1, wy0, wy1 = WINDOW
    cutters.append(cad.rounded_plate(wx1 - wx0, wy1 - wy0, 10, 8.0, center=((wx0 + wx1) / 2, (wy0 + wy1) / 2, z0 - 1)))
    for x in (128.0, 186.0):
        for y in (41.5, 50.5, -41.5, -50.5):
            cutters.append(cad.cbox(2.6, 4.0, 10, center=(x, y, z0)))
    return cad.cut_all(body, cutters)
