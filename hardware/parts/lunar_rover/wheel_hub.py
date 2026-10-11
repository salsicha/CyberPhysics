"""Wheel hub: a 226 mm disc bolted to the AK70-10's output flange (six M3
and the 20 mm spigot), carrying the eight rim segments of one wheel."""
import math

from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover._footprints import CubeMarsAK70_10 as AK

SPIGOT_CLEARANCE = 0.05
CBORE = (6.5, 3.5)            # M3 head recess on the outboard face: diameter, depth

PART = {
    "name": "Wheel hub",
    "quantity": 6,
    "material": "ASA",
    "material_reason": "outdoors in the sun and on a warm actuator; ASA keeps its strength to about 90 C",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("ASA", "outboard face on the plate, spigot up", wall_loops=5, top_shell_layers=5,
                               bottom_shell_layers=5, infill_pct=25, brim="outer brim only, 5 mm",
                               rotate=[("X", -90)]),
    "mates_with": [
        {"part": "CubeMars AK70-10 output flange",
         "dimensions": f"{AK.OUTPUT_HOLES} x M3 ({AK.OUTPUT_THREAD_DEPTH:g} mm deep) on {AK.OUTPUT_PCD:g} mm, "
                       f"{AK.OUTPUT_SPIGOT[0]:g} H7 x {AK.OUTPUT_SPIGOT[1]:g} mm recess",
         "source": AK.SOURCE, "verified": True},
        {"part": "CubeMars AK70-10: which flange turns",
         "dimensions": "the central 6 x M3 flange taken as the output; the 2D drawing does not label it",
         "source": AK.SOURCE, "verified": False},
        {"part": "wheel_segment (printed)",
         "dimensions": f"{F.SEGMENTS * 2} M4 inserts on {2 * F.HUB_BOLT_R:g} mm",
         "source": "hardware/parts/lunar_rover/wheel_segment.py", "verified": True},
    ],
    "inserts": [{"size": "M4", "qty": F.SEGMENTS * 2, "where": "outboard face, segment web bolts"}],
    "fasteners": [
        {"item": "M3 x 16 socket head cap screw, class 12.9", "qty": AK.OUTPUT_HOLES,
         "use": "hub to the actuator's output flange (5.5 mm into the 7 mm threads), medium thread locker"},
    ],
    "fit_test": True,
    "notes": [
        "Before printing the hub, check on the actuator which flange turns (power it and command a slow speed): the hub bolts to the central one.",
        "Re-torque the six M3 after the first hour of driving, then check them every outing; printed plastic under a screw head settles.",
    ],
}


def build():
    from hardware.parts.common import cad
    disc = cad.cyl(2 * F.HUB_R, F.HUB_T, (0, 0, 0), (0, 1, 0))
    d_sp = AK.OUTPUT_SPIGOT[0] - 2 * SPIGOT_CLEARANCE
    spigot = cad.cyl(d_sp, AK.OUTPUT_SPIGOT[1] - 0.2, (0, -(AK.OUTPUT_SPIGOT[1] - 0.2), 0), (0, 1, 0))
    hub = disc.fuse(spigot).clean()
    cutters = []
    for k in range(AK.OUTPUT_HOLES):
        x, _, z = (AK.OUTPUT_PCD / 2 * math.cos(math.radians(60 * k)), 0, AK.OUTPUT_PCD / 2 * math.sin(math.radians(60 * k)))
        cutters.append(cad.cyl(fasteners.CLEARANCE["M3"], F.HUB_T + 6, (x, -4, z), (0, 1, 0)))
        cutters.append(cad.cyl(CBORE[0], CBORE[1] + 0.01, (x, F.HUB_T - CBORE[1], z), (0, 1, 0)))
    for s in range(F.SEGMENTS):
        for da in (-F.SEGMENT_DEG / 4, F.SEGMENT_DEG / 4):
            a = math.radians(s * F.SEGMENT_DEG + da)
            x, z = F.HUB_BOLT_R * math.cos(a), F.HUB_BOLT_R * math.sin(a)
            cutters.append(cad.insert_hole_axis("M4", (x, F.HUB_T, z), (0, -1, 0)))
    # lightening windows between the bolt ring and the centre
    for s in range(F.SEGMENTS):
        a = s * F.SEGMENT_DEG + F.SEGMENT_DEG / 2
        x, z = 64.0 * math.cos(math.radians(a)), 64.0 * math.sin(math.radians(a))
        cutters.append(cad.cyl(38.0, F.HUB_T + 2, (x, -1, z), (0, 1, 0)))
    return cad.cut_all(hub, cutters)
