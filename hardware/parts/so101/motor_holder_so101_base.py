"""SO-101 shoulder motor holder: holds the shoulder-lift servo (joint 2) on the rotating shoulder."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Shoulder motor holder (SO-101, upstream)",
    "quantity": 2,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("on its side: the flat end face on the plate", rotate=[("Y", 90)]),
    "vendored": U.vendored("Motor_holder_SO101_Base.stl", "STL/SO101/Individual/Motor_holder_SO101_Base.stl",
                           "f9971aa805e2d6fb61726d2787e752a6fe0ff21d3c66023296ee4c57a2b100a4"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M2_SERVO, "qty": 4, "use": "shoulder-lift servo"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Two: one for the follower and one for the leader arm.",
    ],
}
