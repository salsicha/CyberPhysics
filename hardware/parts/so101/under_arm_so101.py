"""SO-101 forearm: between the elbow and wrist-flex servos."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Forearm (under arm) (SO-101, upstream)",
    "quantity": 2,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("lying flat: the large flat face on the plate", rotate=[("X", -90)]),
    "vendored": U.vendored("Under_arm_SO101.stl", "STL/SO101/Individual/Under_arm_SO101.stl",
                           "537b03e68ff7c061da4cc70bf0b5a61018a76c00bccc72d61f022d1644230db5"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M3_SERVO, "qty": 8, "use": "forearm to the elbow horns, four each side"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Two: one for the follower and one for the leader arm.",
    ],
}
