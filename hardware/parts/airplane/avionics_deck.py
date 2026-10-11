"""Avionics deck: bonded to the payload-bay floor behind the camera; carries
the Pi 5 + Navigator stack tray on four TPU grommet pairs and holds the
power modules and radios with zip ties through printed tie bridges."""
from hardware.parts.airplane import _frame as F
from hardware.parts.airplane._footprints import (
    BlueRobotics5V6A,
    BlueRoboticsPowerSense,
    HolybroAirspeedMS4525DO,
    SkywalkerX8,
)
from hardware.parts.common import fasteners, printing

CORNER_R = 8.0
BRIDGE = (6.0, 7.0)          # along the tie, across it
TUNNEL = (3.4, 1.8)          # width (across the tie direction), height
ROOF = 1.6
NOTCH = 6.0                  # V-notch in the front edge: points to the nose

PART = {
    "name": "Avionics deck",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "inside a closed foam fuselage with the Pi 5 running; ASA keeps its shape where PETG could creep in a hot bay",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("ASA", "flat underside (the bonding face) on the plate, bosses up",
                               wall_loops=3, infill_pct=20, brim="outer brim only, 5 mm"),
    "mates_with": [
        {"part": "Skywalker X8 payload-bay floor",
         "dimensions": f"{F.DECK_X[1] - F.DECK_X[0]:g} x {2 * F.DECK_HALF_W:g} mm flat face bonded to the foam; "
                       f"needs a floor about {SkywalkerX8.BAY_FLOOR_WIDTH:g} mm wide",
         "source": SkywalkerX8.SOURCE, "verified": False},
        {"part": "stack_grommet (printed TPU) under the stack tray",
         "dimensions": "four M3 inserts in 11 mm bosses at the Pi 5 + Navigator stack corners",
         "source": "hardware/parts/airplane/_frame.py", "verified": True},
        {"part": "Blue Robotics Power Sense Module and 5 V 6 A supply",
         "dimensions": f"{BlueRoboticsPowerSense.SIZE[0]:g} x {BlueRoboticsPowerSense.SIZE[1]:g} mm and "
                       f"{BlueRobotics5V6A.SIZE[0]:g} x {BlueRobotics5V6A.SIZE[1]:g} mm bays between tie bridges",
         "source": BlueRoboticsPowerSense.SOURCE + "; " + BlueRobotics5V6A.SOURCE, "verified": True},
        {"part": "Holybro MS4525DO airspeed module",
         "dimensions": f"{HolybroAirspeedMS4525DO.MODULE[0]:g} x {HolybroAirspeedMS4525DO.MODULE[1]:g} mm bay",
         "source": HolybroAirspeedMS4525DO.SOURCE, "verified": True},
        {"part": "servo BEC, RC receiver, telemetry radio",
         "dimensions": "36 x 22, 40 x 24 and 30 x 56 mm bays (envelopes: these vary by model)",
         "source": "envelopes of this design, not datasheets", "verified": False},
    ],
    "inserts": [{"size": "M3", "qty": 4, "where": "stack grommet bosses, from the top"}],
    "fasteners": [
        {"item": "zip tie 2.5 mm, 150 mm", "qty": 6, "use": "power modules and radios to the deck's tie bridges"},
    ],
    "fit_test": True,
    "notes": [
        "Hold the deck in the bay before bonding: it must lie flat on the floor, the V-notch toward the nose, with the stack clear of the hatch when closed.",
        "Bond with a foam-safe adhesive over the whole face (5-minute epoxy or UHU Por); scuff the foam's skin first. Never cyanoacrylate without foam-safe primer: it dissolves EPO.",
    ],
}


def tie_bridges():
    """(x, y, axis) of every tie bridge: one each side of every module, the
    tie running along ``axis`` over the module."""
    out = []
    for (cx, cy), (sx, sy), axis in F.MODULES.values():
        for s in (-1, 1):
            if axis == "x":
                out.append((cx + s * (sx / 2 + BRIDGE[0] / 2 + 1.0), cy, "x"))
            else:
                out.append((cx, cy + s * (sy / 2 + BRIDGE[0] / 2 + 1.0), "y"))
    return out


def build():
    from hardware.parts.common import cad
    x0, x1 = F.DECK_X
    w = F.DECK_HALF_W
    t = F.DECK_T
    plate = cad.rounded_plate(x1 - x0, 2 * w, t, CORNER_R, center=((x0 + x1) / 2, 0.0, 0.0))
    ins = fasteners.INSERTS["M3"]
    bosses = [cad.cyl(11.0, F.BOSS_TOP - t + 0.01, (x, y, t - 0.01)) for x, y in F.STACK_GROMMETS]
    h = TUNNEL[1] + ROOF + 0.01
    bridges = [cad.cbox(*((BRIDGE[0], BRIDGE[1]) if a == "x" else (BRIDGE[1], BRIDGE[0])), h,
                        center=(x, y, t + h / 2 - 0.01)) for x, y, a in tie_bridges()]
    body = cad.fuse_all([plate] + bosses + bridges)
    cutters = [cad.insert_hole("M3", x, y, F.BOSS_TOP) for x, y in F.STACK_GROMMETS]
    # Tunnels through each bridge along the tie, on the deck surface.
    for x, y, a in tie_bridges():
        size = (BRIDGE[0] + 2, TUNNEL[0]) if a == "x" else (TUNNEL[0], BRIDGE[0] + 2)
        cutters.append(cad.cbox(*size, TUNNEL[1] + 0.01, center=(x, y, t + (TUNNEL[1] + 0.01) / 2)))
    cutters.append(cad.polygon_prism([(x1 + 0.01, -NOTCH / 2), (x1 - NOTCH, 0.0), (x1 + 0.01, NOTCH / 2)], -1, t + 1))
    assert ins.boss_diameter <= 11.0
    return cad.cut_all(body, cutters)
