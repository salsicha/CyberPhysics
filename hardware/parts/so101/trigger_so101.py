"""SO-101 leader trigger: the leader's trigger on the gripper-servo horn: it commands the follower's gripper."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Leader trigger (SO-101 leader, upstream)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("upside down: the published top face on the plate", rotate=[("X", 180)]),
    "vendored": U.vendored("Trigger_SO101.stl", "STL/SO101/Individual/Trigger_SO101.stl",
                           "48ecec3a3710cffdc0ae96d28547e49ddf4cbc93ccd915be7549f78e00ad2850"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M3_SERVO, "qty": 4, "use": "trigger to the gripper-servo horn"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Leader arm only.",
    ],
}
