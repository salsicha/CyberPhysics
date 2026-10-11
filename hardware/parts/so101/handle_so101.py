"""SO-101 leader handle: the leader arm's handle."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "Leader handle (SO-101 leader, upstream)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": U.MATERIAL_REASON,
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("lying flat: the flat side on the plate", rotate=[("X", -90)]),
    "vendored": U.vendored("Handle_SO101.stl", "STL/SO101/Individual/Handle_SO101.stl",
                           "fb8757bdff009c04c207481dd664813ccdac2ad989acea6057df780b52327281"),
    "mates_with": [U.STS3215],
    "inserts": [],
    "fasteners": [
        {"item": U.M2_SERVO, "qty": 1, "use": "handle to the leader holder"},
    ],
    "fit_test": False,
    "notes": [
        U.UPSTREAM_NOTE,
        "Leader arm only.",
    ],
}
