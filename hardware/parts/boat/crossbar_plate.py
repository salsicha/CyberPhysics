"""Crossbar plate: an accessory deck bolted on top of a BlueBoat crossbar
through four of its 8 mm holes, with a 50 mm grid of M4 inserts for the
camera mast and other top-side payloads."""
from hardware.parts.boat import _frame as F
from hardware.parts.boat._footprints import BlueBoat, Iso4762M6, Iso7089M6Washer
from hardware.parts.common import printing

COUNTERBORE = (Iso7089M6Washer.D2 + 1.0, 4.0)   # diameter, depth: washer and screw head
CORNER_R = 8.0

PART = {
    "name": "Crossbar plate (accessory deck)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on deck in sun and salt spray; ASA is UV-stable",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat underside on the plate", wall_loops=4, infill_pct=25,
                               brim="outer brim only, 5 mm"),
    "mates_with": [
        {"part": "BlueBoat crossbar",
         "dimensions": f"four M6 screws through {BlueBoat.CROSSBAR_HOLE_D:g} mm holes {2 * F.PITCH:g} mm apart "
                       f"(every second {F.PITCH:g} mm hole), in Blue Robotics' frame bushings",
         "source": BlueBoat.SOURCE, "verified": True},
        {"part": "BlueBoat crossbar height (screw length)",
         "dimensions": f"M6 x 60 for a {F.BAR_H:g} mm crossbar: plate, washers, bar and nyloc nut",
         "source": "not in the BlueBoat documents: measure the bar", "verified": False},
        {"part": "camera_mast (printed) and other accessories", "dimensions": "M4 inserts on a 50 mm grid",
         "source": "hardware/parts/boat/_frame.py", "verified": True},
    ],
    "inserts": [{"size": "M4", "qty": len(F.grid()), "where": "top face, 50 mm accessory grid"}],
    "fasteners": [
        {"item": "M6 x 60 socket head cap screw, A4 (316) stainless", "qty": 4, "use": "crossbar plate through the crossbar"},
        {"item": "M6 washer, A4 (316) stainless", "qty": 8, "use": "under the crossbar plate screw heads and nuts"},
        {"item": "M6 nyloc nut, A4 (316) stainless", "qty": 4, "use": "under the crossbar"},
    ],
    "fit_test": True,
    "notes": [
        "Use the plastic frame bushings that come with the BlueBoat in the crossbar holes (they keep stainless off the aluminium and take M6).",
        "Measure the crossbar height first: screw length = bar height + 20 mm (rounded up to a stock length).",
        "Rinse with fresh water after salt-water runs; brass inserts tarnish but hold for years above the waterline.",
    ],
}


def build():
    from hardware.parts.common import cad
    sx, sy, t = F.PLATE
    plate = cad.rounded_plate(sx, sy, t, CORNER_R, center=(0.0, 0.0, 0.0))
    cutters = []
    for y in F.PLATE_BOLTS_Y:
        cutters.append(cad.cyl(Iso7089M6Washer.CLEARANCE, t + 2, (0.0, y, -1)))
        cutters.append(cad.cyl(COUNTERBORE[0], COUNTERBORE[1] + 1, (0.0, y, t - COUNTERBORE[1])))
    cutters += [cad.insert_hole("M4", x, y, t) for x, y in F.grid()]
    assert Iso4762M6.HEAD[0] < COUNTERBORE[0]
    return cad.cut_all(plate, cutters)
