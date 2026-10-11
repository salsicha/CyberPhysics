"""SO-101 base: holds the shoulder-pan servo (joint 1); the follower's base bolts to station_base_plate, the leader's is clamped to the table."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Base (SO-101, upstream)",
    "quantity": 2,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("as published: standing on its rear face, the two lattice clamp lobes upright (their teardrop screw holes need no support)"),
    "vendored": U.vendored("Base_SO101.stl", "STL/SO101/Individual/Base_SO101.stl",
                           "19f3c6baafc23a3bc92805762e906df9514d1008be84d490a90a9096ffdcf3c8"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M2_SERVO, "qty": 4, "use": "shoulder-pan servo into the base, two from the top and two from the bottom"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Two: one for the follower and one for the leader arm.",
        "Four 5 mm through holes in the clamp lobes take the M4 bolts of station_base_plate (follower only).",
    ],
}
