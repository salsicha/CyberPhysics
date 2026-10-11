"""Differential clevis: holds an M6 rod end in double shear on a 2020 or
2040 face. Two hang under the rocker beams 100 mm ahead of the rocker
pivots, two sit on the ends of the differential bar under the body;
vertical M6 links join them, so the body's pitch is the average of the two
rockers'."""
from hardware.parts.common import printing
from hardware.parts.lunar_rover._footprints import Extrusion2040

BASE = (50.0, 20.0, 8.0)          # along the beam, across, thick
GAP = 9.5                         # between the cheeks: M6 rod-end ball width up to 9 mm
CHEEK_T = 6.0
PIN_Z = 22.0                      # rod-end bolt axis above the face

PART = {
    "name": "Differential clevis (M6 rod end)",
    "quantity": 4,
    "material": "ASA",
    "material_reason": "outdoors on the top of the rover, in the sun",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("ASA", "base on the plate, cheeks up", wall_loops=5, infill_pct=50),
    "mates_with": [
        {"part": "2040 rocker beam top face / 2020 differential bar", "dimensions": "two M5 into drop-in T-nuts in a 6 mm slot",
         "source": Extrusion2040.SOURCE, "verified": True},
        {"part": "M6 rod end (DIN ISO 12240-4 type E, female thread)", "dimensions": f"ball width up to {GAP - 0.5:g} mm, M6 bolt",
         "source": "ISO 12240-4 dimension series E: d = 6 mm, ball width B = 6.75 mm (typical rod-end listings 6.75-9 mm)",
         "verified": False},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 12 socket head cap screw + M5 drop-in T-nut (slot 6)", "qty": 2, "use": "clevis to the beam"},
        {"item": "M6 x 30 socket head cap screw + M6 nyloc nut + 2 M6 washers", "qty": 1, "use": "rod end in the clevis"},
    ],
    "fit_test": True,
    "notes": ["Use a rod end with a PTFE liner; shim with M6 washers if the ball is narrower than the gap."],
}


def build():
    from hardware.parts.common import cad
    lx, ly, t = BASE
    base = cad.box(-lx / 2, lx / 2, -ly / 2, ly / 2, 0.0, t)
    half = GAP / 2 + CHEEK_T
    cheeks = []
    for y0 in (-half, GAP / 2):
        cheek = cad.box(-12.0, 12.0, y0, y0 + CHEEK_T, 0.0, PIN_Z)
        cheek = cheek.fuse(cad.cyl(24.0, CHEEK_T, (0.0, y0, PIN_Z), (0, 1, 0)))
        cheeks.append(cheek)
    body = cad.fuse_all([base] + cheeks)
    cutters = [cad.cyl(6.4, 2 * half + 2, (0.0, -half - 1, PIN_Z), (0, 1, 0))]
    for x in (-18.0, 18.0):
        cutters.append(cad.clearance_hole("M5", x, 0.0, -1.0, t + 1.0))
    return cad.cut_all(body, cutters)
