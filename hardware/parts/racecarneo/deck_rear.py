"""Deck, rear half: carries the IMU at the car's centre, the power
converters, the VESC, the e-stop at the rear, the HaLowLink 2's cradle and
antenna, and (RACECAR Turret) the turret plate's posts. Its front end is the
lower layer of the lap joint."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _deck as D
from hardware.parts.racecarneo import _frame as F

X = (F.DECK_X[0], F.JOINT_X[1])

PART = {
    "name": "Deck, rear half",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "the top of the car, in the sun; the VESC heatsink sits on it",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: underside on the plate, rims up",
                               wall_loops=3, infill_pct=25, brim="outer brim only, 5 mm"),
    "mates_with": [
        {"part": "deck_front (printed)", "dimensions": f"lap joint over x = {F.JOINT_X[0]:g}-{F.JOINT_X[1]:g} mm",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
        {"part": "post_adapter_rear (printed) through deck_grommet (printed TPU)",
         "dimensions": f"{F.GROMMET_HOLE} mm grommet holes at {F.MOUNTS['rear'][0]}",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
        {"part": "imu_mount, power_tray, vesc_tray, estop_tower, halow_spacer, antenna_mount (printed)",
         "dimensions": "M3 x 3 inserts in the top face", "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
    ],
    "inserts": [{"size": D.INSERT, "qty": sum(1 for x, _ in D.insert_holes() if X[0] <= x <= X[1]),
                 "where": "top face, through holes, pressed flush: IMU, power tray, VESC tray, e-stop tower, "
                          "HaLowLink cradle and antenna mount, and 8 for the RACECAR Turret's plate posts"}],
    "fasteners": [
        {"item": "M3 x 16 socket head cap screw", "qty": 2, "use": "deck to the rear post adapters, through a pair of deck grommets"},
        {"item": "M3 washer", "qty": 2, "use": "on the upper deck grommets, rear"},
        {"item": "M3 x 12 socket head cap screw", "qty": 2, "use": "lap joint rim screws at x = 43 mm"},
        {"item": "M3 nyloc nut", "qty": 2, "use": "lap joint rim screws"},
    ],
    "fit_test": False,
    "notes": [
        "The eight inserts near the four turret post positions are for the RACECAR Turret; press them anyway.",
    ],
}


def build():
    return D.half(X[0], X[1], "bottom")
