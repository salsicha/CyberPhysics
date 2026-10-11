"""Fastener and heat-set insert dimensions, with their sources.

Pure Python. Values are the nominal standard or vendor numbers; the
clearance holes already include the slack a printed hole needs.
"""
from dataclasses import dataclass

CNC_KITCHEN_INSERTS = (
    "CNC Kitchen, 'Dimensions & Design Guidelines, metric size inserts' table "
    "(https://cnckitchen.store/products/gewindeeinsatz-threaded-insert-set-all-sizes, "
    "table image table_metric.png): L length, D1 insert diameter, D2 hole diameter, "
    "W min. wall thickness; blind holes at least L + 1 mm deep"
)
ISO_273 = "ISO 273 clearance holes for bolts and screws, medium series"
ISO_4762 = "ISO 4762 socket head cap screws (head diameter dk max, head height k = d)"
ISO_10511 = "ISO 10511 prevailing-torque (nylon insert) thin hex nuts, width across flats s, height h max"
ASME_B18_3 = "ASME B18.3 socket head cap screws, inch series"
UNC_1_4_20 = "ASME B1.1 1/4\"-20 UNC thread, basic major diameter 0.250 in (6.35 mm)"


@dataclass(frozen=True)
class Insert:
    """A CNC Kitchen-style brass heat-set insert."""
    thread: str
    length: float      # L
    diameter: float    # D1, knurl outer diameter
    hole: float        # D2, the hole to print
    min_wall: float    # W, wall around the hole

    @property
    def hole_depth(self):
        """Minimum blind-hole depth (L + 1 mm)."""
        return self.length + 1.0

    @property
    def boss_diameter(self):
        """Smallest boss that keeps the minimum wall around the hole."""
        return self.hole + 2.0 * self.min_wall


INSERTS = {
    "M2": Insert("M2", 3.0, 3.6, 3.2, 1.3),
    "M2.5": Insert("M2.5", 4.0, 4.6, 4.0, 1.6),
    "M3": Insert("M3", 5.7, 4.6, 4.0, 1.6),
    "M3-short": Insert("M3", 3.0, 4.6, 4.0, 1.6),
    "M4": Insert("M4", 8.1, 6.3, 5.7, 2.1),
}

# ISO 273 medium clearance holes (mm).
CLEARANCE = {"M2": 2.4, "M2.5": 2.9, "M3": 3.4, "M4": 4.5, "M5": 5.5}
# 1/4"-20 camera screw: ASME B18.2.8 normal fit for 1/4" (0.281 in = 7.14 mm),
# rounded down to 7.0 mm because a printed hole comes out slightly small.
CLEARANCE_1_4_20 = 7.0
THREAD_1_4_20_MAJOR = 6.35

# Nominal (major) thread diameters, used for screw bodies in the assembly check.
MAJOR = {"M2": 2.0, "M2.5": 2.5, "M3": 3.0, "M4": 4.0, "M5": 5.0}

# ISO 4762 socket head cap screws: head diameter dk (max) and height k.
SHCS_HEAD = {"M2": (3.8, 2.0), "M2.5": (4.5, 2.5), "M3": (5.5, 3.0), "M4": (7.0, 4.0)}
# ASME B18.3 1/4"-20 socket head: 0.375 in head, 0.250 in high.
SHCS_HEAD_1_4_20 = (9.53, 6.35)

# ISO 10511 nyloc nuts: width across flats s, height h.
NYLOC_NUT = {"M3": (5.5, 4.0), "M4": (7.0, 5.0)}

# Counterbore for a socket head: head diameter + 1 mm.
def counterbore(size):
    return SHCS_HEAD[size][0] + 1.0
