"""OAK-1 cradle: holds the camera lens-down by its 1/4"-20 tripod thread
between two side walls, and pivots in the camera stage's cheeks."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import fasteners, printing
from hardware.parts.common.footprints import LuxonisOAK1

FIT = printing.CLOSE_FIT_MM
WALL = 3.0
INNER_HALF_W = LuxonisOAK1.WIDTH / 2 + FIT + 0.1        # 18.2
OUTER_HALF_W = INNER_HALF_W + WALL                       # 21.2
BOSS_T = 2.0
PIVOT_BOSS_D = 10.0
LOCK_BOSS_D = 8.0
WALL_BOTTOM = F.PIVOT_Z - 6.0                            # leaves the lens end of the camera free
TRIPOD_SLOT = 2.0                                        # +-1 mm along the camera for tolerance

PART = {
    "name": "OAK-1 cradle",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "touches the OAK-1 housing, which runs warm (up to ~4 W); ASA keeps its shape",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings(
        "ASA", "upside down: camera seat on the plate, walls up",
        wall_loops=4, infill_pct=40, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "Luxonis OAK-1",
         "dimensions": f"{LuxonisOAK1.WIDTH:g} mm wide body between walls {2 * INNER_HALF_W:.1f} mm apart; 1/4\"-20 thread "
                       f"{LuxonisOAK1.TRIPOD_FROM_USB_END:g} mm from the USB-C end, {LuxonisOAK1.TRIPOD_DEPTH:g} mm deep",
         "source": LuxonisOAK1.SOURCE, "verified": True},
        {"part": "Luxonis OAK-1 tripod thread across the width",
         "dimensions": "on the centre line (drawn, not dimensioned)", "source": LuxonisOAK1.SOURCE, "verified": False},
        {"part": "oak1_stage (printed)", "dimensions": "pivot and lock bosses on the stage cheek axis",
         "source": "hardware/parts/aerodrone/oak1_stage.py", "verified": True},
    ],
    "inserts": [
        {"size": "M3-short", "qty": 4, "where": "side bosses: pitch pivot (2) and pitch lock (2), pressed flush from outside (through holes)"},
    ],
    "fasteners": [
        {"item": "1/4\"-20 UNC x 1/2\" button or socket head screw", "qty": 1,
         "use": "OAK-1 to cradle (engages about 8 mm of the 10 mm thread)"},
    ],
    "fit_test": True,
    "notes": [
        ("USB-C end of the OAK-1 toward the tail: with the lens down the image top then points to the nose, "
         "matching CAMERA_MOUNT_YAW_DEG = 0 in systems/aerodrone/config/topics.env (check with a still image)."),
        "Fit the camera before the cradle goes into the stage: the tripod screw is reached from above.",
    ],
}


def build():
    from hardware.parts.common import cad
    cx = F.CAMERA_X
    half_len = LuxonisOAK1.HEIGHT / 2
    top = F.CAMERA_BACK_Z + F.CRADLE_T
    seat = cad.box(cx - half_len, cx + half_len, -OUTER_HALF_W, OUTER_HALF_W, F.CAMERA_BACK_Z, top)
    walls = [cad.box(cx - half_len, cx + half_len, ya, yb, WALL_BOTTOM, F.CAMERA_BACK_Z + 0.01)
             for ya, yb in ((INNER_HALF_W, OUTER_HALF_W), (-OUTER_HALF_W, -INNER_HALF_W))]
    bosses = []
    for sign in (1, -1):
        y0 = sign * OUTER_HALF_W - (0.01 if sign > 0 else -0.01)
        for d, z in ((PIVOT_BOSS_D, F.PIVOT_Z), (LOCK_BOSS_D, F.PIVOT_Z + F.LOCK_R)):
            bosses.append(cad.cyl(d, BOSS_T + 0.01, (cx, y0, z), (0, sign, 0)))
    body = cad.fuse_all([seat] + walls + bosses)

    tripod_x = cx - half_len + LuxonisOAK1.TRIPOD_FROM_USB_END
    cutters = [cad.slot(tripod_x - TRIPOD_SLOT / 2, 0, tripod_x + TRIPOD_SLOT / 2, 0,
                        fasteners.CLEARANCE_1_4_20, F.CAMERA_BACK_Z - 1, top + 1)]
    insert = fasteners.INSERTS["M3-short"]
    for sign in (1, -1):
        # Through the wall: a blind hole would leave a 1 mm skin. The screw
        # (M3 x 8 from the stage cheek) stops 1.3 mm short of the camera.
        face = sign * (OUTER_HALF_W + BOSS_T + 1)
        for z in (F.PIVOT_Z, F.PIVOT_Z + F.LOCK_R):
            cutters.append(cad.cyl(insert.hole, WALL + BOSS_T + 2, (cx, face, z), (0, -sign, 0)))
    return cad.cut_all(body, cutters)
