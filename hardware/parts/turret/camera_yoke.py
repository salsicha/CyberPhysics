"""Camera yoke: hangs on both of the tilt servo's horns and carries a
camera plate on its crossbar, in front of the tilt axis. The idler-side
hub is smaller behind the axis so it clears the servo's bus plugs."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret._footprints import ST3215 as S

PART = {
    "name": "Camera yoke (tilt stage)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on the tilt servo, which runs warm; the RACECAR Turret's is in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "standing on the crossbar's front face, hubs up",
                               wall_loops=4, infill_pct=40, rotate=[("Y", 90)]),
    "mates_with": [
        {"part": "Waveshare ST3215 output and idler horns (tilt servo)",
         "dimensions": f"4 x {S.HORN_THREAD} on a {S.HORN_HOLE_CIRCLE:g} mm circle each, horn faces "
                       f"{S.HORN_FACES} mm apart",
         "source": S.SOURCE, "verified": True},
        {"part": "Waveshare ST3215 horn thread and bus plug position",
         "dimensions": "M3 (Feetech's drawing); behind the axis the idler hub is 10 mm in radius, clear of the "
                       "plugs at X 20.9-26.8 mm (scaled)",
         "source": S.SOURCE, "verified": False},
        {"part": "camera_plate_b0385, camera_plate_tripod (printed)", "dimensions": "four M3 x 3 inserts in the crossbar",
         "source": "hardware/parts/turret/_frame.py", "verified": True},
    ],
    "inserts": [{"size": "M3-short", "qty": 4, "where": "crossbar, from the front, through holes pressed flush"}],
    "fasteners": [{"item": "M3 x 5 socket head cap screw", "qty": 8,
                   "use": "camera yoke to the tilt servo's output and idler horns (no longer: the tip must stay "
                          "inside the horn flange)"}],
    "fit_test": True,
    "notes": [
        "With the tilt servo at its centre (step 2048), press both horns on so a hole pair lies at 45 deg, then fit "
        "the yoke with the crossbar forward; set the camera level with the zero offset afterwards.",
    ],
}


def build_local():
    """The yoke in its own frame: origin on the tilt axis, x forward."""
    from hardware.parts.common import cad
    y_in, y_out = F.HORN_FACE, F.HORN_FACE + F.YOKE_T
    cx0, cx1 = F.CROSSBAR_X
    cz0, cz1 = F.CROSSBAR_Z
    parts = []
    for sign in (1, -1):
        ya, yb = sorted((sign * y_in, sign * y_out))
        hub = cad.cyl(F.YOKE_HUB_D, F.YOKE_T, (0, ya, 0), (0, 1, 0))
        if sign < 0:
            # Behind the axis the idler hub is smaller: whatever the tilt, it
            # stays clear of the bus plugs on that face of the servo.
            hub = hub.cut(cad.box(-F.YOKE_HUB_D, 0.0, ya - 1, yb + 1, -F.YOKE_HUB_D, F.YOKE_HUB_D)).fuse(
                cad.cyl(2 * F.IDLER_HUB_REAR_R, F.YOKE_T, (0, ya, 0), (0, 1, 0)).cut(
                    cad.box(0.0, F.YOKE_HUB_D, ya - 1, yb + 1, -F.YOKE_HUB_D, F.YOKE_HUB_D))).clean()
        parts.append(hub)
        parts.append(cad.box(0.0, cx1, ya, yb, -F.ARM_HALF_H, F.ARM_HALF_H))
        ta, tb = sorted((sign * y_in, sign * (y_in + F.YOKE_ARM_T)))
        parts.append(cad.box(F.YOKE_ARM_THICK_X, cx1, ta, tb, -F.ARM_HALF_H, F.ARM_HALF_H))
    y_arm = y_in + F.YOKE_ARM_T
    parts.append(cad.box(cx0, cx1, -y_arm, y_arm, cz0, cz1))
    yoke = cad.fuse_all(parts)
    cutters = []
    for sign in (1, -1):
        face = sign * y_out
        for x, z in F.horn_holes(F.YOKE_HORN_HOLES_DEG):
            cutters.append(cad.cyl(3.4, F.YOKE_T + 2, (x, face + sign, z), (0, -sign, 0)))
    for y, z in F.CROSSBAR_INSERTS:
        cutters.append(cad.cyl(4.0, cx1 - cx0 + 2, (cx0 - 1, y, z), (1, 0, 0)))
    return cad.cut_all(yoke, cutters)


def build():
    from hardware.parts.common import cad
    return cad.translate(build_local(), (0, 0, F.TILT_Z))
