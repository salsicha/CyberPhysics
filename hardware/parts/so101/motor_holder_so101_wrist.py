"""SO-101 wrist motor holder: holds the wrist-flex servo (joint 4) in the forearm."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Wrist motor holder (SO-101, upstream)",
    "quantity": 2,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("on its side: the flat end face on the plate", rotate=[("Y", 90)]),
    "vendored": U.vendored("Motor_holder_SO101_Wrist.stl", "STL/SO101/Individual/Motor_holder_SO101_Wrist.stl",
                           "936c84ed2c918b3ec14e2c3ec3f742f47bed10a4e3d0a2c9bf7869d0a2a55149"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M2_SERVO, "qty": 4, "use": "wrist-flex servo"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Two: one for the follower and one for the leader arm.",
    ],
}
