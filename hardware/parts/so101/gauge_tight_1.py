"""STS3215 fit gauge, tight (upstream): printed first to check the printer's fit on a real STS3215 servo (upstream README, step 3)."""
from hardware.parts.so101 import _upstream as U

PART = {
    "name": "STS3215 fit gauge, tight (upstream)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "the same filament and settings as the arm parts it checks",
    "material_fallback": None,
    "flight_critical": False,
    "print": U.settings("as published: flat on the plate", supports="none"),
    "vendored": U.vendored("Gauge_tight_1.stl", "STL/Gauges/Gauge_tight_1.STL",
                           "8cc075ed3ead3a7de08fbfd15e44e9be0fae61798b8672ebff1df2005ff918dc"),
    "mates_with": [{"part": "Feetech STS3215 servo body", "dimensions": "the servo slides into the gauge as into the arm's pockets", "source": f"upstream README, Printing the Parts, step 3: {U.README}", "verified": True}],
    "inserts": [],
    "fasteners": [],
    "fit_test": True,
    "notes": [
        U.UPSTREAM_NOTE,
        "Fit test, with gauge_0 and gauge_tight_1 printed together: an STS3215 should slide into gauge_0 with light friction and be clearly tighter in gauge_tight_1 (upstream README, step 3). If it is loose in gauge_0 or will not enter it, tune the filament flow or XY compensation before printing the arm.",
    ],
}
