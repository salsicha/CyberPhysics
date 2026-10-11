"""SO-101 upper arm: between the shoulder-lift and elbow servos; holds the elbow servo (joint 3)."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Upper arm (SO-101, upstream)",
    "quantity": 2,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("lying flat: the large flat face on the plate", rotate=[("X", 90)]),
    "vendored": U.vendored("Upper_arm_SO101.stl", "STL/SO101/Individual/Upper_arm_SO101.stl",
                           "96333913c296b13ecb1c81e63e57e4352fb6b5dea7722c84ec1fbd5073e23865"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M3_SERVO, "qty": 8, "use": "upper arm to the shoulder-lift horns, four each side"},
        {"item": U.M2_SERVO, "qty": 4, "use": "elbow servo"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Two: one for the follower and one for the leader arm.",
    ],
}
