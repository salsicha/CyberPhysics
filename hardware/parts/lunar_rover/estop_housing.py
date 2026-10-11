"""E-stop housing: a box on the rear face of the body, behind the solar
panel, with the 40 mm emergency stop on top where it can be hit from
behind or above. Its contact block hangs inside; the cable leaves through
an M16 cable gland in the side; the bottom is open for the screws."""
from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover._footprints import Extrusion2040, SchneiderXB5EStop

BOX = (56.0, 70.0, 60.0)        # x (out from the body), y, z
WALL = 3.0
TOP_T = 4.0                     # panel thickness for the 22 mm operator
GLAND_HOLE = 16.5               # M16 x 1.5 cable gland

PART = {
    "name": "E-stop housing",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "outdoors, in the sun",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("ASA", "upside down: the top (button) face on the plate", wall_loops=4,
                               infill_pct=30, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "Schneider Harmony XB5AS8442 emergency stop",
         "dimensions": f"{SchneiderXB5EStop.HOLE_D:g} mm hole in a {TOP_T:g} mm top",
         "source": SchneiderXB5EStop.SOURCE, "verified": True},
        {"part": "22 mm operator panel thickness and depth", "dimensions": f"panel 1-6 mm, {SchneiderXB5EStop.DEPTH_BEHIND:g} mm behind it",
         "source": SchneiderXB5EStop.SOURCE, "verified": False},
        {"part": "2040 rear top cross rail", "dimensions": "two M5 into drop-in T-nuts in the rear face's upper slot",
         "source": Extrusion2040.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 12 button head screw + M5 drop-in T-nut (slot 6)", "qty": 2, "use": "housing to the rear top cross rail"},
        {"item": "M16 x 1.5 cable gland (4-8 mm cable)", "qty": 1, "use": "e-stop cable, right side"},
    ],
    "fit_test": True,
    "notes": ["The e-stop opens the contactor coil circuit: wire it as described in the README, never through software."],
}


def build():
    """In the rover frame: against the rear face of the rear top cross rail
    (x = -300), top face level with the body's top."""
    from hardware.parts.common import cad
    bx, by, bz = BOX
    x1 = F.BODY_X[0]
    x0 = x1 - bx
    z1 = F.BODY_Z[1]
    z0 = z1 - bz
    outer = cad.box(x0, x1, -by / 2, by / 2, z0, z1)
    inner = cad.box(x0 + WALL, x1 - WALL, -by / 2 + WALL, by / 2 - WALL, z0 - 1, z1 - TOP_T)
    body = outer.cut(inner)
    cx = (x0 + x1 - WALL) / 2
    cutters = [cad.cyl(SchneiderXB5EStop.HOLE_D, TOP_T + 2, (cx, 0.0, z1 - TOP_T - 1)),
               cad.cyl(GLAND_HOLE, WALL + 2, (cx, -by / 2 - 1, z0 + 22.0), (0, 1, 0)),
               cad.box(x1 - WALL - 1, x1 + 1, -12.0, 12.0, z0 + 6.0, z0 + 22.0)]   # wiring into the body
    for y in (-22.0, 22.0):
        cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], WALL + 2, (x1 - WALL - 1, y, z1 - 10.0), (1, 0, 0)))
    return cad.cut_all(body, cutters)
