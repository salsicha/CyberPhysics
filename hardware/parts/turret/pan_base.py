"""Pan base: holds the pan ST3215 upright with its output shaft on the pan
axis, its case in a close pocket, two M2 screws into its idler face, and
room under it for the servo's two bus plugs. Bolts to the turret's base
plate or to the RACECAR Turret's plate with four M3 on a 50 mm square."""
from hardware.parts.common import fasteners, printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret._footprints import ST3215 as S

CLEAR = 0.25
WALL = 3.0
WALL_TOP = F.PAN_CASE_Z + 4.0
FLANGE = (-41.0, 31.0, 31.0)        # x0, x1, half width
NEAR_LEDGE_X = 4.5                  # the ledge under the output end starts here (clear of the idler hub)
FAR_BLOCK_X = -20.0
SCREW_POCKET = (5.2, 14.0)          # from below: M2 head and driver
CABLE_WINDOW = (-19.0, 3.5, 13.0)   # x0, x1, height above the flange
HOOK = (4.0, 12.0)                  # rear hook post (x depth, y width)

PART = {
    "name": "Pan base (pan servo)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "holds a servo that runs warm under load; the RACECAR Turret's is in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flange down on the plate, pocket up", wall_loops=4, infill_pct=40),
    "mates_with": [
        {"part": "Waveshare ST3215 servo (pan)",
         "dimensions": f"{S.LENGTH} x {S.WIDTH} mm case in a pocket with {CLEAR} mm clearance; shaft {S.SHAFT_X} mm from "
                       f"the output end; M2 into the idler-face holes at X = {S.IDLER_FACE_HOLES_X[1]}, +-{S.FACE_HOLE_Y}",
         "source": S.SOURCE, "verified": True},
        {"part": "ST3215 bus connectors and PA2.0 hole depth",
         "dimensions": "plugs under the idler face at X 20.9-26.8 mm (scaled); M2 x 10 enter the case about 4 mm",
         "source": S.SOURCE, "verified": False},
        {"part": "base_plate (printed) or racecar_turret turret_plate (printed)",
         "dimensions": f"four M3 inserts on a {2 * F.BASE_BOLT:g} mm square about the pan axis",
         "source": "hardware/parts/turret/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M2 x 10 pan head screw (or PA2.0 x 10 self-tapping)", "qty": 2,
         "use": "pan servo to the pan base, from below into the idler-face holes"},
        {"item": "M3 x 10 socket head cap screw", "qty": 4, "use": "pan base to the base plate or turret plate"},
        {"item": "zip tie 2.5 mm", "qty": 1, "use": "servo cables at the pan base's rear hook"},
    ],
    "fit_test": True,
    "notes": [
        "Fit the servo before bolting the base down: its two M2 screws go in from below.",
        "The servo's output end (the shaft) faces forward; both bus plugs stay in the cavity under it and leave "
        "through the side windows.",
    ],
}


def build():
    from hardware.parts.common import cad
    x0 = F.pan_case_x(S.LENGTH) - CLEAR          # -35.36
    x1 = F.pan_case_x(0.0) + CLEAR               # 10.36
    hw = S.WIDTH / 2 + CLEAR
    ft = F.BASE_FLANGE_T
    fx0, fx1, fhw = FLANGE
    flange = cad.rounded_plate(fx1 - fx0, 2 * fhw, ft, 6.0, center=((fx0 + fx1) / 2, 0.0, 0.0))
    walls = cad.box(x0 - WALL, x1 + WALL, -hw - WALL, hw + WALL, ft - 0.01, WALL_TOP).cut(
        cad.box(x0, x1, -hw, hw, ft, WALL_TOP + 1))
    # The output end of the case is 29 mm thick, 1.5 mm less than the body: the ledge under it is higher.
    ledge = cad.box(NEAR_LEDGE_X, x1 + 0.01, -hw - 0.01, hw + 0.01, ft - 0.01, F.PAN_CASE_Z - F.HALF_T_END)
    block = cad.box(x0 - 0.01, FAR_BLOCK_X, -hw - 0.01, hw + 0.01, ft - 0.01, F.PAN_BOTTOM_Z)
    hook = cad.box(x0 - WALL - HOOK[0], x0 - WALL + 0.01, -HOOK[1] / 2, HOOK[1] / 2, ft - 0.01, WALL_TOP + 4.0)
    body = cad.fuse_all([flange, walls, ledge, block, hook])

    cutters = []
    sx = F.pan_case_x(S.IDLER_FACE_HOLES_X[1])
    for sy in (S.FACE_HOLE_Y, -S.FACE_HOLE_Y):
        cutters.append(cad.clearance_hole("M2", sx, sy, -1, F.PAN_BOTTOM_Z + 1))
        cutters.append(cad.cyl(SCREW_POCKET[0], SCREW_POCKET[1] + 1, (sx, sy, -1)))
    for bx in (F.BASE_BOLT, -F.BASE_BOLT):
        for by in (F.BASE_BOLT, -F.BASE_BOLT):
            cutters.append(cad.clearance_hole("M3", bx, by, -1, ft + 1))
            head = fasteners.SHCS_HEAD["M3"][0] + 1.0
            cutters.append(cad.cyl(head, WALL_TOP, (bx, by, ft)))       # head clearance if a wall is near
    wx0, wx1, wh = CABLE_WINDOW
    cutters.append(cad.box(wx0, wx1, -hw - WALL - 1, hw + WALL + 1, ft, ft + wh))
    cutters.append(cad.cbox(HOOK[0] + 2, 5.0, 2.6, center=(x0 - WALL - HOOK[0] / 2, 0.0, WALL_TOP)))
    return cad.cut_all(body, cutters)
