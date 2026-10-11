"""SO-101 gripper body / fixed jaw: the follower gripper: fixed jaw, gripper servo (joint 6) on the wrist-roll horn."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Gripper body / fixed jaw (SO-101 follower, upstream)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("on its side: the flat face on the plate", rotate=[("Y", -90)]),
    "vendored": U.vendored("Wrist_Roll_Follower_SO101.stl", "STL/SO101/Individual/Wrist_Roll_Follower_SO101.stl",
                           "3f47a4b889b95a4c71046bda9ddfc9399d96d80a57ad7fc079700e240323a302"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M3_SERVO, "qty": 4, "use": "gripper to the wrist-roll horn"},
        {"item": U.M2_SERVO, "qty": 4, "use": "gripper servo, two each side"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Follower arm only.",
    ],
}
