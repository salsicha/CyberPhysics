"""Camera plate with a 1/4"-20 shelf: bolts to the camera yoke's crossbar
and carries any camera with a tripod thread in its base (RealSense D435i,
OAK-1, cased UVC cameras), its back against the plate, its base on the
shelf so that a 25 mm tall camera is centred on the tilt axis."""
from hardware.parts.common import fasteners, printing
from hardware.parts.racecarneo._footprints import RealSenseD435i
from hardware.parts.turret import _frame as F

X0 = F.CROSSBAR_X[1]
BACK_T = 6.0                        # counterbored M3 heads stay behind the camera's back face
CB = (6.5, 3.2)
HALF_W = 22.0
Z_TOP = 20.0
SHELF_TOP = -RealSenseD435i.HEIGHT / 2      # camera base 12.5 mm below the tilt axis
SHELF_T = 4.0
SHELF_LEN = 26.0
SLOT_X = (4.5, 15.5)                # slot centre range from the back face

PART = {
    "name": "Camera plate, 1/4\"-20 shelf",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "carries a warm camera; stays stiff in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: the shelf's underside on the plate", wall_loops=4, infill_pct=40),
    "mates_with": [
        {"part": "camera with a 1/4\"-20 tripod thread (e.g. RealSense D435i)",
         "dimensions": f"slot {fasteners.CLEARANCE_1_4_20} mm wide, {SLOT_X[0]:g}-{SLOT_X[1]:g} mm from the back face "
                       f"(D435i: {RealSenseD435i.DEPTH - RealSenseD435i.SCREW_FROM_FRONT:.1f} mm)",
         "source": RealSenseD435i.SOURCE, "verified": True},
        {"part": "camera_yoke (printed)", "dimensions": "four M3 x 3 inserts in the crossbar",
         "source": "hardware/parts/turret/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "1/4\"-20 UNC x 3/8\" button head screw", "qty": 1, "use": "camera to the tripod plate shelf"},
        {"item": "M3 x 6 socket head cap screw", "qty": 4, "use": "tripod plate to the camera yoke's crossbar inserts"},
    ],
    "fit_test": False,
    "notes": [
        "For the standalone turret with a RealSense or another cased camera; the RACECAR Turret uses the B0385 plate.",
        "A heavier camera than the D435i (75 g) slows the tilt and loads the servo: keep it under about 150 g.",
    ],
}


def build():
    from hardware.parts.common import cad
    tz = F.TILT_Z
    x1 = X0 + BACK_T
    back = cad.box(X0, x1, -HALF_W, HALF_W, tz + SHELF_TOP - SHELF_T, tz + Z_TOP)
    shelf = cad.box(x1 - 0.01, x1 + SHELF_LEN, -HALF_W, HALF_W, tz + SHELF_TOP - SHELF_T, tz + SHELF_TOP)
    body = back.fuse(shelf).clean()
    cutters = []
    for y, z in F.CROSSBAR_INSERTS:
        cutters.append(cad.cyl(3.4, BACK_T + 2, (X0 - 1, y, tz + z), (1, 0, 0)))
        cutters.append(cad.cyl(CB[0], CB[1] + 1, (x1 - CB[1], y, tz + z), (1, 0, 0)))
    cutters.append(cad.slot(x1 + SLOT_X[0], 0, x1 + SLOT_X[1], 0, fasteners.CLEARANCE_1_4_20,
                            tz + SHELF_TOP - SHELF_T - 1, tz + SHELF_TOP + 1))
    return cad.cut_all(body, cutters)
