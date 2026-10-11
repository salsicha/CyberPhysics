"""SO-101 base motor holder: slides over the shoulder-pan servo in the base."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Base motor holder (SO-101, upstream)",
    "quantity": 2,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("on its back: the large flat face on the plate", rotate=[("X", 90)]),
    "min_wall_mm": 1.0,
    "vendored": U.vendored("Base_motor_holder_SO101.stl", "STL/SO101/Individual/Base_motor_holder_SO101.stl",
                           "e10eb1296560bc00dca91af5f35d9be637be026a5b544c0ebfcac51501aa3c70"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M2_SERVO, "qty": 2, "use": "base motor holder, one each side"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Two: one for the follower and one for the leader arm.",
        "The published design has 1.0 mm walls (two 0.42 mm lines and gap fill), so its minimum wall is 1.0 mm, not the default 1.2 mm.",
    ],
}
