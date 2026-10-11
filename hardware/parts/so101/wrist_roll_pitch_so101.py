"""SO-101 wrist: on the wrist-flex horns; holds the wrist-roll servo (joint 5)."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Wrist (roll-pitch) (SO-101, upstream)",
    "quantity": 2,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("on its side: the flat face on the plate", rotate=[("Y", -90)]),
    "vendored": U.vendored("Wrist_Roll_Pitch_SO101.stl", "STL/SO101/Individual/Wrist_Roll_Pitch_SO101.stl",
                           "14eb2f45ab1b279025ed2aea358f0091aad6aff967faef58527fd216f28fb6e5"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M3_SERVO, "qty": 8, "use": "wrist to the wrist-flex horns, four each side"},
        {"item": U.M2_SERVO, "qty": 2, "use": "wrist-roll servo, front screws"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Two: one for the follower and one for the leader arm.",
    ],
}
