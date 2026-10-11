"""Antenna mount: bonded to the top skin behind the GPS; holds the
telemetry radio's antenna upright on an RP-SMA bulkhead extension and the
RC receiver's two antenna tubes at 90 deg to each other (polarisation
diversity), all outside the carbon-free foam but clear of the GPS."""
import math

from hardware.parts.airplane import _frame as F
from hardware.parts.airplane._footprints import (
    ReceiverAntennaTube,
    RpSmaBulkhead,
    SkywalkerX8,
)
from hardware.parts.common import printing

PAD = (56.0, 50.0, 3.0)
BRIDGE_W = 26.0               # inside, across y
BRIDGE_WALL = 4.0
BRIDGE_H = 16.0               # pad top to the underside of the top plate
TOP_T = 4.0                   # within the bulkhead's panel range
BRIDGE_LEN = 22.0             # along x
SLEEVE_D = 9.0
SLEEVE_L = 18.0
SLEEVE_BORE = 4.2             # takes 2.5-4 mm tubes; heat-shrink a thin one to fit
SLEEVE_X = 22.0               # ahead of the bridge, clear of its walls
SLEEVE_Y = 16.0

PART = {
    "name": "Antenna mount (telemetry RP-SMA and RC receiver antennas)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on the top skin in full sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: pad on the plate; the sleeves lean at 45 deg and need no supports",
                               wall_loops=4, infill_pct=30),
    "mates_with": [
        {"part": "RP-SMA bulkhead extension (telemetry antenna)",
         "dimensions": f"dia. {RpSmaBulkhead.HOLE_D} mm hole through a {TOP_T:g} mm plate for the 1/4\"-36 thread",
         "source": RpSmaBulkhead.SOURCE, "verified": True},
        {"part": "RP-SMA bulkhead panel range",
         "dimensions": f"{TOP_T:g} mm plate (bulkheads typically take up to {RpSmaBulkhead.MAX_PANEL:g} mm)",
         "source": RpSmaBulkhead.SOURCE, "verified": False},
        {"part": "RC receiver antenna tubes",
         "dimensions": f"{SLEEVE_BORE} mm sleeves at +-45 deg from vertical",
         "source": ReceiverAntennaTube.SOURCE, "verified": False},
        {"part": "Skywalker X8 top skin",
         "dimensions": f"{PAD[0]:g} x {PAD[1]:g} mm pad bonded to the foam",
         "source": SkywalkerX8.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "RP-SMA bulkhead extension cable, 150-200 mm (to suit the telemetry radio)", "qty": 1,
         "use": "telemetry antenna through the antenna mount"},
    ],
    "fit_test": True,
    "notes": [
        "Drill matching holes through the skin under the bridge (RP-SMA cable) and under each sleeve (receiver antennas) before bonding.",
        "Antenna tips at least 150 mm from the GPS puck; never coil spare coax next to the GPS.",
    ],
}


def sleeve_axes():
    """(base point, unit direction) of the two receiver-antenna sleeves."""
    x, y = F.ANTENNA_XY
    z = F.TOP_SKIN_Z + PAD[2]
    s = math.sqrt(0.5)
    return [((x + SLEEVE_X, y + sy * SLEEVE_Y, z - 1.0), (0.0, sy * s, s)) for sy in (1, -1)]


def build():
    import cadquery as cq

    from hardware.parts.common import cad
    x, y = F.ANTENNA_XY
    z0 = F.TOP_SKIN_Z
    zp = z0 + PAD[2]
    pad = cad.rounded_plate(PAD[0], PAD[1], PAD[2], 6.0, center=(x + 6.0, y, z0))
    half = BRIDGE_W / 2
    walls = [cad.box(x - BRIDGE_LEN / 2, x + BRIDGE_LEN / 2, ya, yb, zp - 0.01, zp + BRIDGE_H)
             for ya, yb in ((y + half, y + half + BRIDGE_WALL), (y - half - BRIDGE_WALL, y - half))]
    top = cad.box(x - BRIDGE_LEN / 2, x + BRIDGE_LEN / 2, y - half - BRIDGE_WALL, y + half + BRIDGE_WALL,
                  zp + BRIDGE_H - 0.01, zp + BRIDGE_H + TOP_T)
    sleeves = [cad.cyl(SLEEVE_D, SLEEVE_L, base, d) for base, d in sleeve_axes()]
    body = cad.fuse_all([pad, top] + walls + sleeves)
    cutters = [cad.cyl(RpSmaBulkhead.HOLE_D, TOP_T + 2, (x, y, zp + BRIDGE_H - 1)),
               cad.box(x - BRIDGE_LEN / 2 + 3, x + BRIDGE_LEN / 2 - 3, y - half + 2, y + half - 2, z0 - 1, zp + 0.01)]
    for (bx, by, bz), d in sleeve_axes():
        # 45 deg bore up the sleeve, joined by a vertical hole through the
        # pad (a slanted exit would leave a knife edge on the bonding face)
        r = (SLEEVE_BORE + 1.0) / 2
        cutters.append(cad.cyl(SLEEVE_BORE, SLEEVE_L + 2, (bx, by, bz), d))
        cutters.append(cad.cyl(2 * r, bz - z0 + 1.0, (bx, by, z0 - 1)))
        cutters.append(cq.Solid.makeSphere(r, cad.V(bx, by, bz), angleDegrees1=-90, angleDegrees2=90))
    # the sleeves start inside the pad; trim anything below the bonding face
    cutters.append(cad.box(x - 60, x + 60, y - 60, y + 60, z0 - 30, z0))
    return cad.cut_all(body, cutters)
