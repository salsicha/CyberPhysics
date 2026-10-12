"""Motor mount: clamps the outer end of a 16 mm arm tube and carries a
35-size motor, the T-Motor MN3510 (four M3 on a 25 mm circle; the slots
also take a 19 mm circle)."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.common.footprints import CarbonTube16

# Mount frame: u (x) out along the arm, v (y) to the left of it, z up;
# origin on the motor axis at the arm-tube axis height.
BODY_U = (-52.0, -14.0)
BODY_HALF_W = 13.0
BODY_Z = (-17.0, F.MOTOR_PLATE_Z[0])
PLATE_DISK_D = 42.0                       # the motor's 41.8 mm can sits on the whole seat
TUBE_END_U = F.TUBE_R1 - F.MOTOR_R        # -16
PINCH_U = (-44.0, -26.0)
PINCH_Z = -12.5
SLIT = 1.5
CENTER_HOLE_D = F.MOTOR.BASE_CENTRE_CLEARANCE_D
# Motor screws: four radial slots from a 19 mm to a 25 mm circle, turned
# 45 deg off the arm so that no screw head lands under the clamp body.
MOTOR_SLOT_R = (9.5, F.MOTOR.MOUNT_PCD / 2)
MOTOR_SLOT_ANGLES = (45.0, 135.0, 225.0, 315.0)

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
        {"part": "T-Motor MN3510 KV700 motor",
         "dimensions": f"four M3 holes on a {F.MOTOR.MOUNT_PCD:g} mm circle, 90 deg apart; the slots run from "
                       f"{2 * MOTOR_SLOT_R[0]:g} to {2 * MOTOR_SLOT_R[1]:g} mm; {F.MOTOR.DIAMETER:g} mm can on a "
                       f"{PLATE_DISK_D:g} mm seat",
         "source": F.MOTOR.SOURCE, "verified": True},
        {"part": "16 mm carbon arm tube",
         "dimensions": f"OD {CarbonTube16.OD} mm in a {F.TUBE_BORE} mm split bore, tube end on the stop",
         "source": CarbonTube16.SOURCE, "verified": True},
        {"part": "motor base: thread depth, centre",
         "dimensions": f"M3 x 8 screws engage {F.MOTOR.THREAD_DEPTH:g} mm through the 4 mm seat (check they stop "
                       f"short of the windings); {CENTER_HOLE_D:g} mm centre hole for the shaft end and circlip",
         "source": "not dimensioned on T-Motor's page: check on the motor", "verified": False},
    ],
    "inserts": [{"size": "M3", "qty": 2, "where": "far side of the pinch clamp"}],
    "fasteners": [
        {"item": "M3 x 8 socket head cap screw", "qty": 4, "use": "motor to mount (T-Motor's two M3 x 12 are for the propeller)"},
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
    r_in, r_out = MOTOR_SLOT_R
    for ang in MOTOR_SLOT_ANGLES:
        x0, y0 = cad.polar(r_in, ang)
        x1, y1 = cad.polar(r_out, ang)
        cutters.append(cad.slot(x0, y0, x1, y1, 3.4, z_p0 - 1, z_p1 + 1))
    return cad.cut_all(shape, cutters)
