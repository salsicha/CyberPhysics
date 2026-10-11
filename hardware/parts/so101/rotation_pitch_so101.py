"""SO-101 shoulder: the rotating shoulder on the shoulder-pan horns."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Shoulder (rotation-pitch) (SO-101, upstream)",
    "quantity": 2,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("on its side: the large flat face on the plate", rotate=[("Y", -90)]),
    "vendored": U.vendored("Rotation_Pitch_SO101.stl", "STL/SO101/Individual/Rotation_Pitch_SO101.stl",
                           "5ef34a5420071627d01323be13e80b7ac8af0760c72d8382c0664c914f281800"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M3_SERVO, "qty": 8, "use": "shoulder to the shoulder-pan horns, four on top and four underneath"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Two: one for the follower and one for the leader arm.",
    ],
}
