"""Submarine layout: the numbers the submarine's printed parts share.

Pure Python. ROV frame FLU in millimetres: x forward, y left, z up; the
origin is on the underside of the BlueROV2's bottom panel, at its centre.
The BlueROV2 (enclosures, end caps, penetrators, thrusters, frame) is
purchased; the printed parts bolt under the bottom panel through holes
drilled with the parts as templates (Blue Robotics' own instruction for
accessories), plus a topside tether spool. None is part of a pressure
boundary or a seal.
"""
from hardware.parts.submarine._footprints import (
    BlueROV2,
    Iso7089M16Washer,
    WaterLinkedDVLA50,
)

PANEL_T = BlueROV2.BOTTOM_PANEL_T
M5_CLEARANCE = 5.5                          # ISO 273 medium

# DVL A50 under the panel centre (on the CG fore and aft, so it adds no
# pitch moment), back plate against the mount's flange, beams down.
DVL_XY = (0.0, 0.0)
DVL_FLANGE = (100.0, 100.0, 6.0)            # x, y, thickness
DVL_BOLTS = tuple((DVL_XY[0] + sx * 40.0, DVL_XY[1] + sy * 40.0) for sx in (-1, 1) for sy in (-1, 1))
DVL_BACK_Z = -DVL_FLANGE[2]                 # -6: the back plate
DVL_FACE_Z = DVL_BACK_Z - WaterLinkedDVLA50.HEIGHT   # -31: the transducer face
DVL_RING_H = 16.0
DVL_FIT = 0.3                               # radial clearance round the PEEK housing

# Skids: two rails along x under the panel, outside the DVL's beams.
SKID_Y = (100.0, -100.0)
SKID = (240.0, 16.0, 40.0)                  # length, width, height
SKID_BOLTS_X = (-85.0, 0.0, 85.0)          # clear of the end ramps
SKID_BOTTOM_Z = -SKID[2]

# Trim weight holders: M16 stainless washers on a spigot, one near each
# corner of the clear area, inside the skids.
TRIM_XY = tuple((sx * 130.0, sy * 60.0) for sx in (1, -1) for sy in (1, -1))
TRIM_FLANGE = (36.0, 4.0)                   # diameter, thickness
TRIM_SPIGOT = (Iso7089M16Washer.D1 - 0.6, 27.0)   # diameter, length: the screw head stays above the skid line
TRIM_WASHERS_MAX = int(TRIM_SPIGOT[1] // Iso7089M16Washer.H)

# Topside tether spool (its own frame: axle along z, drum from z = 0).
SPOOL_DRUM = (100.0, 80.0, 110.0)           # OD, ID, length
SPOOL_FLANGE = (240.0, 5.0)                 # diameter, thickness
SPOOL_PCD = 90.0
SPOOL_SCREW_ANGLES = (0.0, 60.0, 120.0, 180.0, 240.0, 300.0)
SPOOL_AXLE = 20.0                           # purchased dowel or tube
SPOOL_HUB = (44.0, 12.0)                    # boss diameter, length (outward)
TETHER_HOLE_R = 30.0                        # tether end leaves through the flange here (inside the drum)
TETHER_ANGLE = 180.0
CRANK_R = 100.0
CRANK_ANGLE = 30.0
