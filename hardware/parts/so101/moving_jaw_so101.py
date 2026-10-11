"""SO-101 moving jaw: the follower gripper's moving jaw on the gripper-servo horns."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Moving jaw (SO-101 follower, upstream)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("upside down: the published top face on the plate (supports under the curved finger)", rotate=[("X", 180)]),
    "vendored": U.vendored("Moving_Jaw_SO101.stl", "STL/SO101/Individual/Moving_Jaw_SO101.stl",
                           "403763bbab8656cc7ce5ea6374741d0860bd19426b5b4b0041de3a2c1275f90b"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M3_SERVO, "qty": 8, "use": "moving jaw to the gripper horns, four each side"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Follower arm only.",
        "The mesh has one small enclosed void (76 mm3), part of the published design.",
    ],
}
