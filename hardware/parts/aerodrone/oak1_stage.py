"""OAK-1 camera stage: hangs from the sensor nose on four silicone damper
balls; its two cheeks carry the camera cradle on a pitch pivot, locked
anywhere within +-10 deg."""
import math

from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.common.footprints import LuxonisOAK1, SiliconeDamperBall

HALF_LEN = 32.0
HALF_W = 27.5
CHEEK_Y = (23.5, 27.5)             # inner and outer face, both sides
CHEEK_HALF_LEN = 14.0
CHEEK_BOTTOM = F.PIVOT_Z - 7.0
POCKET_D = 13.0
LOBE_D = 17.0
SLOT_W = 3.4


def tripod_x():
    return F.CAMERA_X - LuxonisOAK1.HEIGHT / 2 + LuxonisOAK1.TRIPOD_FROM_USB_END


PART = {
    "name": "OAK-1 camera stage (isolated, pitch pivot)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "shaded under the nose; general structure",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings(
        "PETG", "upside down: flat top on the plate, cheeks up",
        wall_loops=4, infill_pct=40, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "silicone gimbal damper balls (x4)",
         "dimensions": f"dia. {SiliconeDamperBall.HOLE_D} mm holes in {SiliconeDamperBall.PLATE_THICKNESS:g} mm tabs",
         "source": SiliconeDamperBall.SOURCE, "verified": False},
        {"part": "oak1_cradle (printed)", "dimensions": "M3 pivot and M3 lock screw on each side",
         "source": "hardware/parts/aerodrone/oak1_cradle.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 8 socket head cap screw", "qty": 4, "use": "pitch pivot (2) and pitch lock (2), into the cradle inserts"},
        {"item": "M3 washer", "qty": 2, "use": "under the pitch-lock screws"},
    ],
    "fit_test": True,
    "notes": [
        "Set the pitch so the lens looks straight down with the aircraft level (or at the cruise attitude you care about), then tighten the lock screws.",
    ],
}


def _arc_slot(cx, cz, radius, a0, a1, width, y0, y1):
    """Arc slot about (cx, cz) in the x-z plane, from angle a0 to a1 (deg,
    from +x toward +z), through y0..y1."""
    import cadquery as cq
    ro, ri, h = radius + width / 2, radius - width / 2, width / 2

    def p(r, a, tangent=0.0):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        return (cx + r * c - tangent * s, cz + r * s + tangent * c)

    am = (a0 + a1) / 2
    wp = (cq.Workplane("XZ", origin=(0, y1, 0))
          .moveTo(*p(ro, a0))
          .threePointArc(p(ro, am), p(ro, a1))
          .threePointArc(p(radius, a1, h), p(ri, a1))
          .threePointArc(p(ri, am), p(ri, a0))
          .threePointArc(p(radius, a0, -h), p(ro, a0))
          .close().extrude(y1 - y0))
    return wp.val()


def build():
    from hardware.parts.common import cad
    z0, z1 = F.STAGE_Z
    cx = F.CAMERA_X
    plate = cad.fuse_all(
        [cad.rounded_plate(2 * HALF_LEN, 2 * HALF_W, z1 - z0, 5.0, center=(cx, 0.0, z0))]
        + [cad.cyl(LOBE_D, z1 - z0, (x, y, z0)) for x, y in F.DAMPERS]
        + [cad.box(x - LOBE_D / 2, x + LOBE_D / 2, min(y, math.copysign(HALF_W - 3, y)),
                   max(y, math.copysign(HALF_W - 3, y)), z0, z1) for x, y in F.DAMPERS])
    cheeks = [cad.box(cx - CHEEK_HALF_LEN, cx + CHEEK_HALF_LEN, ya, yb, CHEEK_BOTTOM, z0 + 0.01)
              for ya, yb in ((CHEEK_Y[0], CHEEK_Y[1]), (-CHEEK_Y[1], -CHEEK_Y[0]))]
    body = cad.fuse_all([plate] + cheeks)

    tab_bottom = z1 - F.DAMPER_TAB_T
    cutters = []
    for x, y in F.DAMPERS:
        cutters.append(cad.cyl(SiliconeDamperBall.HOLE_D, z1 - z0 + 2, (x, y, z0 - 1)))
        cutters.append(cad.cyl(POCKET_D, tab_bottom - z0 + 1, (x, y, z0 - 1)))
    cutters.append(cad.cyl(SLOT_W, 2 * CHEEK_Y[1] + 2, (cx, -CHEEK_Y[1] - 1, F.PIVOT_Z), (0, 1, 0)))
    r = F.PITCH_RANGE_DEG
    for y0, y1 in ((CHEEK_Y[0] - 0.5, CHEEK_Y[1] + 0.5), (-CHEEK_Y[1] - 0.5, -CHEEK_Y[0] + 0.5)):
        cutters.append(_arc_slot(cx, F.PIVOT_Z, F.LOCK_R, 90 - r, 90 + r, SLOT_W, y0, y1))
    cutters.append(cad.cyl(9.0, z1 - z0 + 2, (tripod_x(), 0.0, z0 - 1)))
    return cad.cut_all(body, cutters)
