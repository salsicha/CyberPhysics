"""SO-101 leader holder: the leader's wrist piece that carries the handle and the trigger servo."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Leader holder (wrist roll) (SO-101 leader, upstream)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("on its side: the flat face on the plate", rotate=[("Y", -90)]),
    "vendored": U.vendored("Wrist_Roll_SO101.stl", "STL/SO101/Individual/Wrist_Roll_SO101.stl",
                           "de3a65044dd4ae8bcb9659d8ca2b49598e3f5571edf89f45ad975e9776a7ffee"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M3_SERVO, "qty": 4, "use": "leader holder to the wrist-roll horn"},
        {"item": U.M2_SERVO, "qty": 4, "use": "trigger servo, two each side"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Leader arm only.",
    ],
}
