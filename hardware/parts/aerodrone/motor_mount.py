"""Motor mount: clamps the outer end of a 16 mm arm tube and carries a
2216-size motor (16 x 16 mm cross; 16 x 19 and 19 x 19 also fit)."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.common.footprints import CarbonTube16, Holybro2216KV920

# Mount frame: u (x) out along the arm, v (y) to the left of it, z up;
# origin on the motor axis at the arm-tube axis height.
BODY_U = (-52.0, -14.0)
BODY_HALF_W = 13.0
BODY_Z = (-17.0, F.MOTOR_PLATE_Z[0])
PLATE_DISK_D = 34.0
TUBE_END_U = F.TUBE_R1 - F.MOTOR_R        # -16
PINCH_U = (-44.0, -26.0)
PINCH_Z = -12.5
SLIT = 1.5
CENTER_HOLE_D = 8.5

PART = {
    "name": "Motor mount",
    "quantity": 4,
    "material": "ASA",
    "material_reason": "next to the motor and ESC, in the sun; ASA keeps its stiffness when the motor runs hot",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings(
        "ASA", "upside down: motor seat on the plate, tube clamp up (slit on top)",
        wall_loops=5, infill_pct=50, top_shell_layers=6, bottom_shell_layers=6,
        rotate=[("X", 180)]),
    "mates_with": [
        {"part": "Holybro 2216 KV920 motor",
         "dimensions": f"four M3 holes on a {Holybro2216KV920.MOUNT_PATTERN[0]:g} x {Holybro2216KV920.MOUNT_PATTERN[1]:g} mm cross; "
                       "slots reach 16 x 19 and 19 x 19 mm",
         "source": Holybro2216KV920.SOURCE, "verified": True},
        {"part": "16 mm carbon arm tube",
         "dimensions": f"OD {CarbonTube16.OD} mm in a {F.TUBE_BORE} mm split bore, tube end on the stop",
         "source": CarbonTube16.SOURCE, "verified": True},
        {"part": "motor base: hole layout, thread depth, centre",
         "dimensions": "the 16 x 16 mm pattern read as a cross (opposite holes 16 mm apart); M3 x 8 screws engage "
                       "4 mm through the 4 mm plate (check they stop short of the windings); 8.5 mm centre hole "
                       "for the shaft end and circlip",
         "source": "not in the Holybro specification: check on the motor", "verified": False},
    ],
    "inserts": [{"size": "M3", "qty": 2, "where": "far side of the pinch clamp"}],
    "fasteners": [
        {"item": "M3 x 8 socket head cap screw (or the motor's own screws)", "qty": 4, "use": "motor to mount"},
        {"item": "M3 x 25 socket head cap screw", "qty": 2, "use": "pinch clamp on the arm tube"},
    ],
    "fit_test": True,
    "notes": [
        "Before tightening the pinch clamp, set the motor seat parallel to the hub top plate (a straight edge across both); a twisted mount tilts the thrust.",
        "Turn the motor bell by hand after fitting the screws: nothing may rub or touch the windings.",
        "Blue (medium) thread locker on the motor screws, which go into metal; none on screws into inserts.",
    ],
}


def build():
    from hardware.parts.common import cad

    z_p0, z_p1 = F.MOTOR_PLATE_Z
    body = cad.box(BODY_U[0], BODY_U[1], -BODY_HALF_W, BODY_HALF_W, BODY_Z[0], BODY_Z[1])
    plate = cad.fuse_all([
        cad.box(BODY_U[0], 0.0, -BODY_HALF_W, BODY_HALF_W, z_p0, z_p1),
        cad.cyl(PLATE_DISK_D, z_p1 - z_p0, (0, 0, z_p0)),
    ])
    shape = body.fuse(plate).clean()

    cutters = [
        cad.cyl(F.TUBE_BORE, TUBE_END_U - BODY_U[0] + 1, (BODY_U[0] - 1, 0, 0), (1, 0, 0)),
        cad.box(BODY_U[0] - 1, TUBE_END_U, -SLIT / 2, SLIT / 2, BODY_Z[0] - 1, -4.0),
        cad.cyl(CENTER_HOLE_D, z_p1 - z_p0 + 2, (0, 0, z_p0 - 1)),
    ]
    for u in PINCH_U:
        cutters.append(cad.cyl(3.4, 2 * BODY_HALF_W + 2, (u, -BODY_HALF_W - 1, PINCH_Z), (0, 1, 0)))
        cutters.append(cad.insert_hole_axis("M3", (u, BODY_HALF_W, PINCH_Z), (0, -1, 0)))
    r_in = Holybro2216KV920.MOUNT_PATTERN[0] / 2
    r_out = max(Holybro2216KV920.OTHER_PATTERNS[-1]) / 2
    for ang in (0.0, 90.0, 180.0, 270.0):
        x0, y0 = cad.polar(r_in, ang)
        x1, y1 = cad.polar(r_out, ang)
        cutters.append(cad.slot(x0, y0, x1, y1, 3.4, z_p0 - 1, z_p1 + 1))
    return cad.cut_all(shape, cutters)
