"""Turret plate: the URDF's turret/base_link. Carries the turret's pan base
on four inserts, stands on the four posts, holds the Waveshare Bus Servo
Adapter (A) underneath (components down, jumper and DC jack reachable from
the side) and has pass-throughs for the servo bus and the camera cable."""
from hardware.parts.common import printing
from hardware.parts.racecar_turret import _frame as F
from hardware.parts.turret import _frame as T
from hardware.parts.turret._footprints import WaveshareBusServoAdapterA as B

PART = {
    "name": "Turret plate (with the servo adapter mount)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "the top of the car, in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upside down: the top face on the plate, adapter standoffs up",
                               wall_loops=4, infill_pct=40, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "turret pan_base (printed)", "dimensions": f"four M3 x 3 inserts on a {2 * T.BASE_BOLT:g} mm square",
         "source": "hardware/parts/turret/_frame.py", "verified": True},
        {"part": "plate_post (printed)", "dimensions": "four M3 screws into the posts' top inserts",
         "source": "hardware/parts/racecar_turret/_frame.py", "verified": True},
        {"part": "Waveshare Bus Servo Adapter (A)",
         "dimensions": f"{B.BOARD[0]:g} x {B.BOARD[1]:g} mm, dia. {B.HOLE_D} holes on {B.HOLE_SPACING[0]:g} x "
                       f"{B.HOLE_SPACING[1]:g} mm",
         "source": B.SOURCE, "verified": True},
    ],
    "inserts": [
        {"size": "M3-short", "qty": 4, "where": "pan base screws, from the top, through holes pressed flush"},
        {"size": "M2", "qty": 4, "where": "adapter standoffs under the plate"},
    ],
    "fasteners": [
        {"item": "M3 x 10 socket head cap screw", "qty": 4, "use": "turret plate onto the posts' top inserts"},
        {"item": "M3 washer", "qty": 4, "use": "turret plate screws"},
        {"item": "M2 x 5 socket head cap screw", "qty": 4, "use": "Bus Servo Adapter (A) to the plate's standoffs"},
        {"item": "zip tie 2.5 mm", "qty": 2, "use": "servo bus and camera cable at the plate's pass-throughs"},
    ],
    "fit_test": False,
    "notes": [
        "The turret's pan base bolts on with the same four M3 x 10 as on its own base plate.",
        "The adapter hangs under the plate with its components down: set its jumper (A, UART) before fitting it.",
    ],
}


def build():
    from hardware.parts.common import cad
    cx, cy = F.PLATE_CENTER
    z0, z1 = F.PLATE_Z
    hx, hy = F.PLATE_HALF
    plate = cad.rounded_plate(2 * hx, 2 * hy, z1 - z0, F.PLATE_CORNER_R, center=(cx, cy, z0))
    ax, ay = F.ADAPTER_CENTER
    sx, sy = B.HOLE_SPACING[0] / 2, B.HOLE_SPACING[1] / 2
    sd, sl = F.ADAPTER_STANDOFF
    standoffs = [cad.cyl(sd, sl + 0.01, (ax + dx, ay + dy, z0 - sl)) for dx in (sx, -sx) for dy in (sy, -sy)]
    body = cad.fuse_all([plate] + standoffs)
    cutters = []
    for bx in (T.BASE_BOLT, -T.BASE_BOLT):
        for by in (T.BASE_BOLT, -T.BASE_BOLT):
            cutters.append(cad.cyl(4.0, z1 - z0 + 2, (cx + bx, cy + by, z0 - 1)))
    for px, py in F.POSTS:
        cutters.append(cad.clearance_hole("M3", px, py, z0 - 1, z1 + 1))
    for dx in (sx, -sx):
        for dy in (sy, -sy):
            cutters.append(cad.insert_hole("M2", ax + dx, ay + dy, z0 - sl, direction=1))
    for x0, x1, y0, y1 in (F.BUS_SLOT, F.USB_SLOT):
        cutters.append(cad.rounded_plate(x1 - x0, y1 - y0, z1 - z0 + 2, min(x1 - x0, y1 - y0) / 2 - 0.01,
                                         center=((x0 + x1) / 2, (y0 + y1) / 2, z0 - 1)))
    return cad.cut_all(body, cutters)
