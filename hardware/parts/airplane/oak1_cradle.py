"""OAK-1 cradle: holds the camera lens-down by its 1/4"-20 tripod thread
between two side walls, and hangs from the camera deck on four silicone
damper balls."""
import math

from hardware.parts.airplane import _frame as F
from hardware.parts.common import fasteners, printing
from hardware.parts.common.footprints import LuxonisOAK1, SiliconeDamperBall

FIT = printing.CLOSE_FIT_MM
HALF_LEN = 32.0
HALF_W = 27.5
WALL = 3.0
INNER_HALF_W = LuxonisOAK1.WIDTH / 2 + FIT + 0.1        # 18.2
OUTER_HALF_W = INNER_HALF_W + WALL                       # 21.2
WALL_BOTTOM = 14.0                                       # leaves the lens end of the camera free
LOBE_D = 17.0
POCKET_D = 13.0
TRIPOD_SLOT = 2.0                                        # +-1 mm along the camera for tolerance

PART = {
    "name": "OAK-1 cradle",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "touches the OAK-1 housing, which runs warm (up to ~4 W); ASA keeps its shape",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upside down: flat top on the plate, walls up",
                               wall_loops=4, infill_pct=40, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "Luxonis OAK-1",
         "dimensions": f"{LuxonisOAK1.WIDTH:g} mm wide body between walls {2 * INNER_HALF_W:.1f} mm apart; 1/4\"-20 thread "
                       f"{LuxonisOAK1.TRIPOD_FROM_USB_END:g} mm from the USB-C end, {LuxonisOAK1.TRIPOD_DEPTH:g} mm deep",
         "source": LuxonisOAK1.SOURCE, "verified": True},
        {"part": "Luxonis OAK-1 tripod thread across the width",
         "dimensions": "on the centre line (drawn, not dimensioned)", "source": LuxonisOAK1.SOURCE, "verified": False},
        {"part": "silicone gimbal damper balls (x4)",
         "dimensions": f"dia. {SiliconeDamperBall.HOLE_D} mm holes in {SiliconeDamperBall.PLATE_THICKNESS:g} mm tabs",
         "source": SiliconeDamperBall.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "1/4\"-20 UNC x 1/2\" button or socket head screw", "qty": 1,
         "use": "OAK-1 to cradle (engages about 8 mm of the 10 mm thread)"},
    ],
    "fit_test": True,
    "notes": [
        ("USB-C end of the OAK-1 toward the tail: with the lens down the image top then points to the nose, "
         "matching CAMERA_MOUNT_YAW_DEG = 0.0 in systems/airplane/config/blueos.env (check with a still image)."),
        "Screw the camera on before the cradle goes onto the balls; the screw is reached through the deck window later.",
    ],
}


def build():
    from hardware.parts.common import cad
    cx = F.CAMERA_X
    z0, z1 = F.CRADLE_Z
    plate = cad.fuse_all(
        [cad.rounded_plate(2 * HALF_LEN, 2 * HALF_W, z1 - z0, 5.0, center=(cx, 0.0, z0))]
        + [cad.cyl(LOBE_D, z1 - z0, (x, y, z0)) for x, y in F.DAMPERS]
        + [cad.box(x - LOBE_D / 2, x + LOBE_D / 2, min(y, math.copysign(HALF_W - 3, y)),
                   max(y, math.copysign(HALF_W - 3, y)), z0, z1) for x, y in F.DAMPERS])
    half = LuxonisOAK1.HEIGHT / 2
    walls = [cad.box(cx - half, cx + half, ya, yb, WALL_BOTTOM, z0 + 0.01)
             for ya, yb in ((INNER_HALF_W, OUTER_HALF_W), (-OUTER_HALF_W, -INNER_HALF_W))]
    body = cad.fuse_all([plate] + walls)
    tab_bottom = z1 - F.DAMPER_TAB_T
    tx = F.tripod_x()
    cutters = [cad.slot(tx - TRIPOD_SLOT / 2, 0, tx + TRIPOD_SLOT / 2, 0, fasteners.CLEARANCE_1_4_20, z0 - 1, z1 + 1)]
    for x, y in F.DAMPERS:
        cutters.append(cad.cyl(SiliconeDamperBall.HOLE_D, z1 - z0 + 2, (x, y, z0 - 1)))
        cutters.append(cad.cyl(POCKET_D, tab_bottom - z0 + 1, (x, y, z0 - 1)))
    return cad.cut_all(body, cutters)
