"""Camera plate for the Arducam B0385 (38 mm board, M12 lens): bolts to the
camera yoke's crossbar and holds the board on four standoffs, lens on the
tilt axis, about 35 mm in front of it (the URDF's turret/camera_link)."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret._footprints import ArducamB0385 as A

X0 = F.CROSSBAR_X[1]
T = F.CAMERA_PLATE_T
HALF_W = 22.0
Z = (-20.2, 20.0)
BOSS_H = 6.0                        # clears the board's back-side connector (4.5 mm)
BOSS_D = 6.4

PART = {
    "name": "Camera plate, Arducam B0385",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "outdoors on the RACECAR Turret; stays stiff in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: the plate on the build plate, standoffs up",
                               wall_loops=3, infill_pct=40, rotate=[("Y", -90)]),
    "mates_with": [
        {"part": "Arducam B0385 camera board",
         "dimensions": f"{A.BOARD:g} mm square board, 4 x dia. {A.HOLE_34_D} holes on a {A.HOLES_34:g} mm square, lens on "
                       f"the centre; back-side connector {A.CONNECTOR[2]} mm tall",
         "source": A.SOURCE, "verified": True},
        {"part": "camera_yoke (printed)", "dimensions": "four M3 x 3 inserts in the crossbar",
         "source": "hardware/parts/turret/_frame.py", "verified": True},
    ],
    "inserts": [{"size": "M2", "qty": 4, "where": "standoffs, from the top"}],
    "fasteners": [
        {"item": "M2 x 5 socket head cap screw", "qty": 4, "use": "Arducam B0385 board to the camera plate standoffs"},
        {"item": "M3 x 6 socket head cap screw", "qty": 4, "use": "camera plate to the camera yoke's crossbar inserts"},
        {"item": "zip tie 2.5 mm", "qty": 1, "use": "camera USB cable at the camera plate's bottom slot"},
    ],
    "fit_test": False,
    "notes": [
        "Board connector toward the bottom edge: its side-entry plug then points down, and the cable leaves under "
        "the crossbar; tie it through the plate's bottom slot with a loop before it runs back to the pan stage.",
    ],
}


def build():
    from hardware.parts.common import cad
    tz = F.TILT_Z
    x1 = X0 + T
    plate = cad.box(X0, x1, -HALF_W, HALF_W, tz + Z[0], tz + Z[1])
    h = A.HOLES_34 / 2
    bosses = [cad.cyl(BOSS_D, BOSS_H + 0.01, (x1 - 0.01, sy * h, tz + sz * h), (1, 0, 0))
              for sy in (1, -1) for sz in (1, -1)]
    body = cad.fuse_all([plate] + bosses)
    cutters = []
    for sy in (1, -1):
        for sz in (1, -1):
            cutters.append(cad.insert_hole_axis("M2", (x1 + BOSS_H, sy * h, tz + sz * h), (-1, 0, 0)))
    for y, z in F.CROSSBAR_INSERTS:
        cutters.append(cad.cyl(3.4, T + 2, (X0 - 1, y, tz + z), (1, 0, 0)))
    cutters.append(cad.cbox(T + 2, 5.0, 2.6, center=((X0 + x1) / 2, 0.0, tz + Z[0] + 3.5)))
    return cad.cut_all(body, cutters)
