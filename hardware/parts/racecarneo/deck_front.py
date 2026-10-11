"""Deck, front half: carries the lidar mast and camera arm and, in front of
the lidar and below its scan plane, the Jetson tray. Its rear end is the
upper layer of the lap joint with the rear half."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _deck as D
from hardware.parts.racecarneo import _frame as F

X = (F.JOINT_X[0], F.DECK_X[1])

PART = {
    "name": "Deck, front half",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "the top of the car, in the sun; the Jetson warms it from above",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: underside on the plate, rims up",
                               wall_loops=3, infill_pct=25, brim="outer brim only, 5 mm"),
    "mates_with": [
        {"part": "deck_rear (printed)", "dimensions": f"lap joint over x = {F.JOINT_X[0]:g}-{F.JOINT_X[1]:g} mm, "
                                                       f"{F.DECK_T / 2:g} mm each",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
        {"part": "post_adapter_front (printed) through deck_grommet (printed TPU)",
         "dimensions": f"{F.GROMMET_HOLE} mm grommet holes at {F.MOUNTS['front'][0]}",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
        {"part": "lidar_mast, camera_arm, jetson_tray (printed)", "dimensions": "M3 through holes and M3 x 3 inserts",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
    ],
    "inserts": [{"size": D.INSERT, "qty": sum(1 for x, _ in D.insert_holes() if X[0] <= x <= X[1]),
                 "where": "top face, through holes, pressed flush: Jetson tray"}],
    "fasteners": [
        {"item": "M3 x 16 socket head cap screw", "qty": 2, "use": "deck to the front post adapters, through a pair of deck grommets"},
        {"item": "M3 washer", "qty": 2, "use": "on the upper deck grommets, front"},
    ],
    "fit_test": False,
    "notes": [
        "Lay the front half over the rear half's lap; the lidar mast flange screws and the two rim screws at x = 43 clamp the joint.",
    ],
}


def build():
    return D.half(X[0], X[1], "top")
