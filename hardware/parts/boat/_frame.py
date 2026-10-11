"""Boat layout: the numbers the boat's printed parts share.

Pure Python. Boat frame FLU in millimetres: x forward, y left, z up; the
origin is on the boat's centre line on the top face of the crossbar the
accessory plate is bolted to. The BlueBoat itself is purchased (see
hardware/boat.md); the printed parts are top-side accessories on its
crossbars and hatch lids.
"""
from hardware.parts.boat._footprints import BlueBoat

PITCH = BlueBoat.CROSSBAR_PITCH                 # 25 mm hole pitch along the crossbar
BAR_W, BAR_H = BlueBoat.CROSSBAR_SECTION

# Crossbar plate: centred on the crossbar, its long side along it (y).
PLATE = (100.0, 225.0, 12.0)                    # x, y, thickness
PLATE_BOLTS_Y = (-3 * PITCH, -PITCH, PITCH, 3 * PITCH)   # 50 mm apart: every second crossbar hole
GRID_X = (-25.0, 25.0)
GRID_Y = (-100.0, -50.0, 0.0, 50.0, 100.0)      # M4 inserts for accessories, 50 mm grid


def grid():
    return tuple((x, y) for x in GRID_X for y in GRID_Y)


# Camera mast: on the plate, its flange screws into four grid inserts.
MAST_XY = (0.0, 25.0)
MAST_SCREWS = tuple((MAST_XY[0] + dx, MAST_XY[1] + dy) for dx in (-25.0, 25.0) for dy in (-25.0, 25.0))
MAST_FLANGE = (70.0, 70.0, 6.0)
MAST_HEIGHT = 200.0                             # plate top to the stud
