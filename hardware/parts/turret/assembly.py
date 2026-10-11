"""Turret assembly: the printed parts on the base plate with simple
datasheet-sized models of the two ST3215 servos (case, label cover, horns,
bus plugs), the camera, the controller boards and the screws, and the
checks run by tools/build_printable_parts.py:

  * interference at the home pose (pan 0, tilt 0), screws modelled through
    their holes;
  * tilt sweep over the joint limits (-0.65..0.75 rad) for both camera
    plates (Arducam B0385 board; a RealSense D435i on the 1/4"-20 plate):
    the tilt stage against the pan stage, with the smallest clearance;
  * pan sweep over +-90 deg: the pan stage against the pan base, the pan
    servo and the base plate, with the smallest clearance;
  * camera view: nothing printed in the B0385's 70 x 46 deg view (plus a
    margin) at any tilt;
  * the tilt axis height and the camera position against the RACECAR
    Turret's URDF, the cable loop the pan range needs, and the masses and
    tilt load against the servo's torque.
"""
import math

from hardware.parts.racecarneo import _checks as C
from hardware.parts.racecarneo._footprints import RealSenseD435i
from hardware.parts.turret import _frame as F
from hardware.parts.turret._footprints import (
    ST3215,
    ArducamB0385,
    ArduinoUnoR4,
    WaveshareBusServoAdapterA,
)

FOV_MARGIN_DEG = 2.0
SWEEP_STEP_DEG = 5.0
MIN_CLEARANCE_MM = 0.5
G = 9.81

MASS_G = {"servo": ST3215.MASS_G, "b0385": 15.0, "d435i": RealSenseD435i.MASS_G}   # B0385 board+lens: estimate


# --------------------------------------------------------------------- models

def st3215(rotations, origin, hole_deg=45.0, idler_horn=True):
    """ST3215 model. Local frame: X along the case from the shaft toward the
    far end, W across the width, T toward the output face; ``rotations``
    ([(axis, deg)], applied in order) turn it into the turret frame and
    ``origin`` is where the shaft axis meets the case's mid-plane. Returns
    [(name, shape)]."""
    from hardware.parts.common import cad
    S = ST3215

    def place(shape):
        for axis, deg in rotations:
            shape = cad.rotate(shape, axis, deg)
        return cad.translate(shape, origin)

    s0 = -S.SHAFT_X                                   # case starts at the output end
    hw = S.WIDTH / 2
    step = s0 + S.OUTPUT_END_LENGTH
    body = cad.box(step, s0 + S.LENGTH, -hw, hw, -F.HALF_T, F.HALF_T).fuse(
        cad.box(s0, step + 0.01, -hw, hw, -F.HALF_T_END, F.HALF_T_END))
    lx0, lx1 = s0 + S.LABEL_X[0], s0 + S.LABEL_X[1]
    body = body.fuse(cad.box(lx0, lx1, -S.LABEL_HALF_W, S.LABEL_HALF_W, F.HALF_T - 0.01, F.LABEL_TOP))
    for hx in S.OUTPUT_FACE_HOLES_X:
        for hy in (S.FACE_HOLE_Y, -S.FACE_HOLE_Y):
            body = body.cut(cad.cyl(2.0, 6.0, (s0 + hx, hy, F.HALF_T - 6.0)))
    for hx in S.IDLER_FACE_HOLES_X:
        for hy in (S.FACE_HOLE_Y, -S.FACE_HOLE_Y):
            body = body.cut(cad.cyl(2.0, 6.0, (s0 + hx, hy, -F.HALF_T)))
    out = [("case", place(body))]
    holes = [cad.cyl(3.0, 10.0, (x, y, -5.0)) for x, y in F.horn_holes(hole_deg)]
    flange_out = F.HORN_FACE - S.HORN_FLANGE_T
    horn = cad.cyl(S.HORN_D, S.HORN_FLANGE_T, (0, 0, flange_out)).fuse(
        cad.cyl(S.HORN_HUB_D, flange_out - F.HALF_T_END + 0.01, (0, 0, F.HALF_T_END)))
    out.append(("horn_out", place(cad.cut_all(horn, [cad.translate(h, (0, 0, F.HORN_FACE)) for h in holes]))))
    if idler_horn:
        flange_in = -F.HORN_FACE + S.IDLER_HORN_FLANGE_T
        ih = cad.cyl(S.HORN_D, S.IDLER_HORN_FLANGE_T, (0, 0, -F.HORN_FACE)).fuse(
            cad.cyl(S.HORN_HUB_D, -F.HALF_T_END - flange_in + 0.01, (0, 0, flange_in - 0.01)))
        out.append(("horn_idler", place(cad.cut_all(ih, [cad.translate(h, (0, 0, -F.HORN_FACE)) for h in holes]))))
    cx0, cx1 = s0 + S.CONNECTOR_X[0], s0 + S.CONNECTOR_X[1]
    out.append(("plugs", place(cad.box(cx0, cx1, -11.0, 11.0, -F.HALF_T - S.PLUG_LENGTH, -F.HALF_T - 0.01))))
    return out


def b0385_board(plate_x1):
    """B0385 board (with its lens) on the standoffs, in the yoke frame."""
    from hardware.parts.common import cad
    from hardware.parts.turret import camera_plate_b0385 as P
    A = ArducamB0385
    x0 = plate_x1 + P.BOSS_H
    h = A.BOARD / 2
    board = cad.box(x0, x0 + A.PCB_T, -h, h, -h, h)
    for sy in (1, -1):
        for sz in (1, -1):
            board = board.cut(cad.cyl(A.HOLE_34_D, 4, (x0 - 1, sy * A.HOLES_34 / 2, sz * A.HOLES_34 / 2), (1, 0, 0)))
    lens = cad.cyl(A.LENS_D, A.LENS_FRONT, (x0 + A.PCB_T, 0, 0), (1, 0, 0))
    (c0, c1), (z0, z1), ch = A.CONNECTOR
    conn = cad.box(x0 - ch, x0 + 0.01, c0, c1, z0, z1)
    return board.fuse(lens).fuse(conn).clean(), x0 + A.PCB_T, x0 + A.PCB_T + A.LENS_FRONT


def stages(camera="b0385"):
    """(static, pan_stage, tilt_stage) lists of (name, shape) in the turret
    frame at pan 0, tilt 0. The tilt stage is in its own frame (origin on the
    tilt axis) so it can be rotated before it is placed."""
    from hardware.parts.common import cad
    from hardware.parts.turret import (
        base_plate,
        camera_plate_b0385,
        camera_plate_tripod,
        camera_yoke,
        pan_base,
        pan_hub,
        tilt_bracket,
    )
    static, pan_st, tilt_st = [], [], []
    static.append(("base_plate", base_plate.build()))
    static.append(("pan_base", pan_base.build()))
    for n, s in st3215([("Z", 180)], (0, 0, F.PAN_CASE_Z), F.PAN_HORN_HOLES_DEG, idler_horn=False):
        (pan_st if n == "horn_out" else static).append((f"pan_servo_{n}", s))
    # Controller boards on the base plate
    U, B = ArduinoUnoR4, WaveshareBusServoAdapterA
    ux, uy = base_plate.UNO_ORIGIN
    uz = base_plate.UNO_BOSS[1]
    uno = cad.box(ux, ux + U.BOARD[0], uy, uy + U.BOARD[1], uz, uz + 1.6)
    for x, y in base_plate.uno_holes():
        uno = uno.cut(cad.cyl(U.HOLE_D, 4, (x, y, uz - 1)))
        static.append((f"uno_screw_{x:g}_{y:g}", C.screw((x, y, uz + 1.6), (0, 0, -1), 5.0, "M3")))
    static.append(("arduino_uno", uno))
    bx, by = base_plate.BSA_ORIGIN
    bz = base_plate.BSA_BOSS[1]
    bsa = cad.box(bx, bx + B.BOARD[0], by, by + B.BOARD[1], bz, bz + 1.6)
    for x, y in base_plate.bsa_holes():
        bsa = bsa.cut(cad.cyl(B.HOLE_D, 4, (x, y, bz - 1)))
        static.append((f"bsa_screw_{x:g}_{y:g}", C.screw((x, y, bz + 1.6), (0, 0, -1), 5.0, "M2")))
    static.append(("servo_adapter", bsa))
    for sx in (F.BASE_BOLT, -F.BASE_BOLT):
        for sy in (F.BASE_BOLT, -F.BASE_BOLT):
            static.append((f"base_bolt_{sx:g}_{sy:g}", C.screw((sx, sy, F.BASE_FLANGE_T), (0, 0, -1), 10.0, "M3")))
    sxp = F.pan_case_x(ST3215.IDLER_FACE_HOLES_X[1])
    for sy in (ST3215.FACE_HOLE_Y, -ST3215.FACE_HOLE_Y):
        static.append((f"pan_servo_screw_{sy:g}", C.screw((sxp, sy, F.PAN_BOTTOM_Z - 6.0), (0, 0, 1), 10.0, "M2")))

    # Pan stage: hub, bracket, tilt servo (its horns go with the tilt stage), screws.
    pan_st.append(("pan_hub", pan_hub.build()))
    pan_st.append(("tilt_bracket", tilt_bracket.build()))
    for x, y in F.horn_holes(F.PAN_HORN_HOLES_DEG):
        pan_st.append((f"hub_screw_{x:.0f}_{y:.0f}", C.screw((x, y, F.PAN_HORN_Z + F.HUB_T), (0, 0, -1), 5.0, "M3")))
    for x, y in F.JOINT_INSERTS:
        z = F.TILT_FLOOR_TOP - tilt_bracket.JOINT_CB[1]
        pan_st.append((f"joint_screw_{x:g}_{y:g}", C.screw((x, y, z), (0, 0, -1), 8.0, "M3")))
    for n, s in st3215([("X", 90), ("Z", 180)], (0, 0, F.TILT_Z), F.YOKE_HORN_HOLES_DEG):
        if n.startswith("horn"):
            tilt_st.append((f"tilt_servo_{n}", cad.translate(s, (0, 0, -F.TILT_Z))))
        else:
            pan_st.append((f"tilt_servo_{n}", s))
    hy_in = F.HALF_T + tilt_bracket.CLEAR
    for dz in (ST3215.FACE_HOLE_Y, -ST3215.FACE_HOLE_Y):
        xo = F.tilt_case_x(ST3215.OUTPUT_FACE_HOLES_X[1])
        xi = F.tilt_case_x(ST3215.IDLER_FACE_HOLES_X[1])
        cb = tilt_bracket.WALL_CB[1]
        pan_st.append((f"wall_screw_o_{dz:g}", C.screw((xo, hy_in + F.WALL_T_OUTPUT - cb, F.TILT_Z + dz), (0, -1, 0), 8.0, "M2")))
        pan_st.append((f"wall_screw_i_{dz:g}", C.screw((xi, -hy_in - F.WALL_T + cb, F.TILT_Z + dz), (0, 1, 0), 6.0, "M2")))

    # Tilt stage (yoke frame): yoke, horn screws, camera plate, camera.
    tilt_st.append(("camera_yoke", camera_yoke.build_local()))
    for sign in (1, -1):
        face = sign * (F.HORN_FACE + F.YOKE_T)
        for x, z in F.horn_holes(F.YOKE_HORN_HOLES_DEG):
            tilt_st.append((f"yoke_screw_{sign}_{x:.0f}_{z:.0f}", C.screw((x, face, z), (0, -sign, 0), 5.0, "M3")))
    if camera == "b0385":
        plate = cad.translate(camera_plate_b0385.build(), (0, 0, -F.TILT_Z))
        tilt_st.append(("camera_plate_b0385", plate))
        board, _, _ = b0385_board(camera_plate_b0385.X0 + camera_plate_b0385.T)
        tilt_st.append(("b0385", board))
        px1 = camera_plate_b0385.X0 + camera_plate_b0385.T
        for y, z in F.CROSSBAR_INSERTS:
            tilt_st.append((f"plate_screw_{y:g}_{z:g}", C.screw((px1, y, z), (-1, 0, 0), 6.0, "M3")))
    else:
        plate = cad.translate(camera_plate_tripod.build(), (0, 0, -F.TILT_Z))
        tilt_st.append(("camera_plate_tripod", plate))
        P = camera_plate_tripod
        x0 = P.X0 + P.BACK_T
        R = RealSenseD435i
        tilt_st.append(("d435i", cad.box(x0, x0 + R.DEPTH, -R.WIDTH / 2, R.WIDTH / 2, P.SHELF_TOP, P.SHELF_TOP + R.HEIGHT)))
    return static, pan_st, tilt_st


def place_tilt(shape, tilt_deg):
    """Yoke-frame shape at this tilt (positive nose up), in the turret frame."""
    from hardware.parts.common import cad
    return cad.translate(cad.rotate(shape, "Y", -tilt_deg), (0, 0, F.TILT_Z))


def place_pan(shape, pan_deg):
    """Pan-stage shape at this pan (positive turns right)."""
    from hardware.parts.common import cad
    return cad.rotate(shape, "Z", -pan_deg)


def pose(pan_deg=0.0, tilt_deg=0.0, camera="b0385"):
    static, pan_st, tilt_st = stages(camera)
    bodies = list(static)
    bodies += [(n, place_pan(s, pan_deg)) for n, s in pan_st]
    bodies += [(n, place_pan(place_tilt(s, tilt_deg), pan_deg)) for n, s in tilt_st]
    return bodies


# --------------------------------------------------------------------- checks

def _min_distance(movers, others):
    best = (math.inf, None)
    for n1, s1 in movers:
        b1 = s1.BoundingBox()
        for n2, s2 in others:
            b2 = s2.BoundingBox()
            gap = max(b1.xmin - b2.xmax, b2.xmin - b1.xmax, b1.ymin - b2.ymax, b2.ymin - b1.ymax,
                      b1.zmin - b2.zmax, b2.zmin - b1.zmax)
            if gap > best[0]:
                continue
            d = s1.distance(s2)
            if d < best[0]:
                best = (d, f"{n1} / {n2}")
    return best


def tilt_sweep(camera):
    static, pan_st, tilt_st = stages(camera)
    lo, hi = F.TILT_RANGE_DEG
    step = int(SWEEP_STEP_DEG)
    angles = [lo] + [a for a in range(-90, 91, step) if lo < a < hi] + [hi]
    # Moving bodies: the yoke, plate and camera (not the screws inside the horns).
    movers0 = [(n, s) for n, s in tilt_st if not n.startswith("tilt_servo_horn")]
    others = [(n, s) for n, s in pan_st + static if not n.startswith(("base_plate", "arduino", "uno_", "bsa_",
                                                                       "servo_adapter", "base_bolt"))]
    worst = (math.inf, None, None)
    for a in angles:
        movers = [(n, place_tilt(s, a)) for n, s in movers0]
        d, pair = _min_distance(movers, others)
        if d < worst[0]:
            worst = (d, pair, a)
    return worst


def pan_sweep():
    static, pan_st, tilt_st = stages("b0385")
    movers0 = [(n, s) for n, s in pan_st if not n.startswith("pan_servo_horn")]
    movers0 += [(n, place_tilt(s, 0.0)) for n, s in tilt_st]
    others = [(n, s) for n, s in static if not n.startswith("pan_servo_horn")]
    worst = (math.inf, None, None)
    for a in range(-int(F.PAN_LIMIT_DEG), int(F.PAN_LIMIT_DEG) + 1, 15):
        movers = [(n, place_pan(s, a)) for n, s in movers0]
        d, pair = _min_distance(movers, others)
        if d < worst[0]:
            worst = (d, pair, a)
    return worst


def lens_front():
    from hardware.parts.turret import camera_plate_b0385 as P
    _, sensor_x, front_x = b0385_board(P.X0 + P.T)
    return sensor_x, front_x


def check(entries=None):
    """Run the assembly checks; returns a report dict ('passed' is the
    overall result)."""
    entries = entries or {}
    report = {}
    bodies = pose(0.0, 0.0)
    press = (("pan_servo_horn", "pan_hub", "hub disc on the horn"),
             ("tilt_servo_horn", "camera_yoke", "yoke hubs on the horns"))
    overlaps, allowed = C.interferences(bodies, press)
    report["bodies"] = len(bodies)
    report["interferences"] = overlaps or "none"
    report["intended_press_fits"] = len(allowed)

    tilt_ok = True
    for cam in ("b0385", "d435i"):
        d, pair, a = tilt_sweep(cam)
        report[f"tilt_sweep_{cam}"] = (f"clear from {F.TILT_RANGE_DEG[0]:.1f} to {F.TILT_RANGE_DEG[1]:.1f} deg; "
                                       f"smallest gap {d:.1f} mm ({pair} at {a:.0f} deg)")
        tilt_ok = tilt_ok and d >= MIN_CLEARANCE_MM
    d, pair, a = pan_sweep()
    report["pan_sweep"] = f"clear over +-{F.PAN_LIMIT_DEG:g} deg; smallest gap {d:.1f} mm ({pair} at {a:+.0f} deg)"
    pan_ok = d >= MIN_CLEARANCE_MM

    # Camera view at the tilt limits and level.
    sensor_x, front_x = lens_front()
    hits = []
    for tilt in (F.TILT_RANGE_DEG[0], 0.0, F.TILT_RANGE_DEG[1]):
        p = math.radians(tilt)
        apex = (front_x * math.cos(p), 0.0, F.TILT_Z + front_x * math.sin(p))
        fov = C.frustum(apex, (math.cos(p), 0, math.sin(p)), (-math.sin(p), 0, math.cos(p)),
                        ArducamB0385.HFOV_DEG / 2 + FOV_MARGIN_DEG, ArducamB0385.VFOV_DEG / 2 + FOV_MARGIN_DEG, 1500.0)
        for hit in C.in_view([(n, s) for n, s in pose(0.0, tilt) if n != "b0385"], fov):
            hits.append(f"tilt {tilt:+.0f}: {hit}")
    report["camera_view_clear"] = hits or (f"yes (B0385 {ArducamB0385.HFOV_DEG:g} x {ArducamB0385.VFOV_DEG:g} deg + "
                                           f"{FOV_MARGIN_DEG:g} deg margin, at both tilt limits and level)")
    report["tilt_axis_mm"] = (f"{F.TILT_Z:g} above the mounting surface ({F.TILT_Z + F.PLATE_T_URDF:g} above "
                              "turret/base_link with a 5 mm plate; URDF 0.045 + 0.055 m)")
    report["camera_mm"] = (f"B0385 sensor {sensor_x:.1f}, lens front {front_x:.1f} ahead of the tilt axis, on its "
                           f"height; URDF turret/camera_link {F.CAMERA_X:g}")

    # Cable loop from the pan base's hook to the bracket's hook.
    bx, by, bz = F.BASE_HOOK
    from hardware.parts.turret import tilt_bracket as TB
    hx = F.tilt_case_x(ST3215.LENGTH) - TB.CLEAR - TB.REAR_STOP_T - TB.HOOK[0] / 2
    hz = F.TILT_FLOOR_TOP + 3.0
    spans = []
    for a in range(-90, 91, 15):
        r = math.radians(-a)
        x, y = hx * math.cos(r), hx * math.sin(r)
        spans.append(math.dist((bx, by, bz), (x, y, hz)))
    report["cable_loop_mm"] = (f"hook to hook {min(spans):.0f}-{max(spans):.0f} mm over the pan range: leave "
                               f"{math.ceil((max(spans) + 40) / 10) * 10:.0f} mm of cable between the two ties")

    m = {stem: e.get("estimate", {}).get("mass_g", 0.0) for stem, e in entries.items()}
    tilt_mass = m.get("camera_yoke", 0) + m.get("camera_plate_b0385", 0) + MASS_G["b0385"]
    report["moving_mass_g"] = (f"tilt stage {tilt_mass:.0f}, pan stage "
                               f"{tilt_mass + MASS_G['servo'] + m.get('pan_hub', 0) + m.get('tilt_bracket', 0):.0f}")
    # Static tilt load: the camera stage's weight at its centre of mass.
    lever = 0.030
    torque = tilt_mass / 1000.0 * G * lever
    report["tilt_load"] = (f"about {torque:.3f} N m with the B0385 ({ST3215.STALL_TORQUE_NM:g} N m stall); "
                           f"{(F.CAMERA_X + 15) / 1000 * RealSenseD435i.MASS_G / 1000 * G:.3f} N m with a D435i")
    report["passed"] = not overlaps and tilt_ok and pan_ok and not hits
    return report


# --------------------------------------------------------------------- figures

def figures(out_dir, entries=None):
    """Render the assembly-instruction figures into ``out_dir``."""
    from pathlib import Path

    from hardware.parts.common import render
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    report = check(entries)
    materials = {stem: e.get("material", "ASA") for stem, e in (entries or {}).items()}
    printed_ids = set(materials) | {"base_plate", "pan_base", "pan_hub", "tilt_bracket", "camera_yoke",
                                    "camera_plate_b0385", "camera_plate_tripod"}

    def items(bodies, offsets=None, only=None):
        out = []
        offsets = offsets or {}
        for n, s in bodies:
            if only and not any(n.startswith(p) for p in only):
                continue
            m = C.mesh(s)
            off = next((v for k, v in offsets.items() if n.startswith(k)), None)
            if off:
                m = m.copy().apply_translation(off)
            col = render.MATERIAL_COLORS.get(materials.get(n, "ASA"), "#e8833a") if n in printed_ids \
                else render.PURCHASED_COLOR
            out.append((m, col))
        return out

    home = pose(0.0, 0.0)
    render.render(items(home), out_dir / "overview.png", elev=22, azim=-55, zoom=1.0,
                  title="Turret: printed parts (colour), purchased parts (grey)")
    exploded = {"pan_servo": (0, 0, 30), "pan_hub": (0, 0, 70), "hub_screw": (0, 0, 80), "tilt_bracket": (0, 0, 100),
                "joint_screw": (0, 0, 115), "tilt_servo": (0, 0, 130), "wall_screw": (0, 0, 130),
                "camera_yoke": (40, 0, 150), "yoke_screw": (40, 0, 150), "camera_plate": (70, 0, 150),
                "plate_screw": (60, 0, 150), "b0385": (95, 0, 150), "pan_base": (0, 0, 15), "base_bolt": (0, 0, 15),
                "pan_servo_screw": (0, 0, -10)}
    render.render(items(home, exploded), out_dir / "exploded.png", elev=15, azim=-60, zoom=0.95,
                  title="Exploded view")
    render.render(items(home, only=("base_plate", "pan_base", "pan_servo", "base_bolt", "arduino", "uno_", "servo_adapter",
                                    "bsa_", "pan_hub", "hub_screw")),
                  out_dir / "step_pan.png", elev=30, azim=-50, zoom=1.0,
                  title="Pan base, pan servo, pan hub; Arduino and servo adapter on the base plate")
    render.render(items(home, only=("pan_hub", "tilt_bracket", "joint_screw", "tilt_servo", "wall_screw", "camera_yoke",
                                    "yoke_screw", "camera_plate", "plate_screw", "b0385")),
                  out_dir / "step_tilt.png", elev=20, azim=-40, zoom=1.0,
                  title="Tilt bracket, tilt servo, camera yoke and the B0385 plate")
    lo, hi = F.TILT_RANGE_DEG
    sweep = []
    for a in (lo, hi):
        sweep += [(f"{n}", s) for n, s in pose(0.0, a) if n.startswith(("camera_yoke", "camera_plate", "b0385"))]
    sweep += [(n, s) for n, s in home if n.startswith(("pan_hub", "tilt_bracket", "tilt_servo_case", "pan_base",
                                                       "pan_servo_case", "tilt_servo_plugs"))]
    render.render(items(sweep), out_dir / "tilt_range.png", elev=0, azim=-90, zoom=1.0,
                  title=f"Tilt limits {lo:.0f} and +{hi:.0f} deg (side view)")
    return report
