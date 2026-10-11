"""Tilt bracket: bolted on the pan hub, turns with the pan servo and holds
the tilt ST3215 lying on its side, shaft across (y) on the tilt axis, its
case reaching back: a floor under the case, two side walls with M2 screws
into the servo's far face holes, a rear stop, and a hook for the cable
loop."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret._footprints import ST3215 as S

CLEAR = 0.2
REAR_STOP_T = 3.5
WALL_Z = (F.TILT_FLOOR_TOP - F.BRACKET_FLOOR_T, F.TILT_Z + S.WIDTH / 2 + 1.5)
LABEL_RELIEF = 2.6                  # pocket in the +y wall for the raised label cover
JOINT_CB = (6.0, 3.2)               # counterbores for the M3 heads in the floor
WALL_CB = (4.6, 2.0)                # counterbores for the M2 heads in the side walls
HOOK = (6.0, 10.0)                  # rear hook (x depth, y width) under the floor

PART = {
    "name": "Tilt bracket (on the pan horn)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "holds a servo that runs warm under load; the RACECAR Turret's is in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "floor on the plate, walls up", wall_loops=4, infill_pct=40),
    "mates_with": [
        {"part": "pan_hub (printed)", "dimensions": "four M3 x 8 into its top-face inserts; key holes over its shafts",
         "source": "hardware/parts/turret/_frame.py", "verified": True},
        {"part": "Waveshare ST3215 servo (tilt)",
         "dimensions": f"case {S.LENGTH} x {S.WIDTH} x {S.THICKNESS:g} mm; M2 into the output-face hole pair at X = "
                       f"{S.OUTPUT_FACE_HOLES_X[1]} and the idler-face pair at X = {S.IDLER_FACE_HOLES_X[1]}",
         "source": S.SOURCE, "verified": True},
        {"part": "ST3215 raised label and bus connectors",
         "dimensions": "label cover X 24-42 mm, +-7.3 mm (relief in the +y wall); connectors at X 20.9-26.8 mm on the "
                       "idler face (the -y wall stops behind them); scaled from the drawings",
         "source": S.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 8 socket head cap screw", "qty": 4, "use": "tilt bracket to the pan hub (heads recessed in the floor)"},
        {"item": "M2 x 6 socket head cap screw (or PA2.0 x 6 self-tapping)", "qty": 2,
         "use": "tilt servo into the tilt bracket's -y wall (heads in counterbores, 4 mm into the case)"},
        {"item": "M2 x 8 socket head cap screw (or PA2.0 x 8 self-tapping)", "qty": 2,
         "use": "tilt servo into the tilt bracket's +y wall (heads in counterbores, 4 mm into the case)"},
        {"item": "zip tie 2.5 mm", "qty": 1, "use": "cable loop at the tilt bracket's rear hook"},
    ],
    "fit_test": True,
    "notes": [
        "Screw it to the pan hub before the tilt servo goes in: the four screw heads sit under the servo.",
        "Lay the tilt servo in without its horns (they do not pass the walls), screw it in, then fit the horns.",
    ],
}


def build():
    from hardware.parts.common import cad
    fz1 = F.TILT_FLOOR_TOP
    fz0 = fz1 - F.BRACKET_FLOOR_T
    case_x0 = F.tilt_case_x(S.LENGTH) - CLEAR       # -35.31, far end
    hy_in = F.HALF_T + CLEAR                        # walls' inner faces
    hy_out = hy_in + F.WALL_T
    hy_out_o = hy_in + F.WALL_T_OUTPUT
    wx0, wx1 = F.WALL_X

    cx0 = F.COLUMN[0]
    floor = cad.box(case_x0 - REAR_STOP_T, cx0, -hy_out, hy_out_o, fz0, fz1).fuse(
        cad.box(cx0 - 0.01, F.FLOOR_FRONT_X, -F.FLOOR_FRONT_HALF_W, F.FLOOR_FRONT_HALF_W, fz0, fz1)).clean()
    rear = cad.box(case_x0 - REAR_STOP_T, case_x0, -hy_out, hy_out_o, fz1 - 0.01, WALL_Z[1])
    walls = [cad.box(wx0, wx1, hy_in, hy_out_o, fz1 - 0.01, WALL_Z[1]),
             cad.box(wx0, wx1 - 2.0, -hy_out, -hy_in, fz1 - 0.01, WALL_Z[1])]
    hook = cad.box(case_x0 - REAR_STOP_T - HOOK[0], case_x0 - REAR_STOP_T + 0.01, -HOOK[1] / 2, HOOK[1] / 2,
                   fz0, fz1 + 6.0)
    body = cad.fuse_all([floor, rear, hook] + walls)

    cutters = []
    for x, y in F.JOINT_INSERTS:
        cutters.append(cad.clearance_hole("M3", x, y, fz0 - 1, fz1 + 1))
        cutters.append(cad.cyl(JOINT_CB[0], JOINT_CB[1] + 1, (x, y, fz1 - JOINT_CB[1])))
    # M2 screws into the servo's far-face holes (along y through the walls).
    tz = F.TILT_Z
    xo = F.tilt_case_x(S.OUTPUT_FACE_HOLES_X[1])
    xi = F.tilt_case_x(S.IDLER_FACE_HOLES_X[1])
    for dz in (S.FACE_HOLE_Y, -S.FACE_HOLE_Y):
        cutters.append(cad.cyl(2.4, F.WALL_T_OUTPUT + 2, (xo, hy_in - 1, tz + dz), (0, 1, 0)))
        cutters.append(cad.cyl(2.4, F.WALL_T + 2, (xi, -hy_in + 1, tz + dz), (0, -1, 0)))
        # Counterbores: an M2 x 6 then enters the servo case 4 mm.
        cutters.append(cad.cyl(WALL_CB[0], WALL_CB[1] + 1, (xo, hy_out_o + 1, tz + dz), (0, -1, 0)))
        cutters.append(cad.cyl(WALL_CB[0], WALL_CB[1] + 1, (xi, -hy_out - 1, tz + dz), (0, 1, 0)))
    # Relief for the raised label cover on the output face (+y).
    lx0, lx1 = F.tilt_case_x(S.LABEL_X[1]) - 0.5, F.tilt_case_x(S.LABEL_X[0]) + 0.5
    cutters.append(cad.box(lx0, lx1, hy_in - 0.1, hy_in + LABEL_RELIEF, tz - S.LABEL_HALF_W - 0.5,
                           tz + S.LABEL_HALF_W + 0.5))
    cutters.append(cad.cbox(HOOK[0] + 2, 5.0, 2.6, center=(case_x0 - REAR_STOP_T - HOOK[0] / 2, 0.0, fz1 + 3.0)))
    return cad.cut_all(body, cutters)
