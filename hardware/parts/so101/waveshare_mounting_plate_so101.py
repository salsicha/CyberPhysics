"""SO-101 waveshare board mounting plate: carries the Waveshare bus servo adapter on the back of the base."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Waveshare board mounting plate (SO-101, upstream)",
    "quantity": 2,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("as published: flat on the plate"),
    "vendored": U.vendored("WaveShare_Mounting_Plate_SO101.stl", "STL/SO101/Individual/WaveShare_Mounting_Plate_SO101.stl",
                           "6275f05ef28cd407be13ea882735282c4a8b2ef64930c77a9e7b5a3811740be1"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Two: one for the follower and one for the leader arm.",
        "Fix the board with the spare screws of the servo accessory bags.",
    ],
}
