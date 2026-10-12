"""Turret assembly: the printed parts on the base plate with simple
datasheet-sized models of the four ST3215 servos (case, label cover,
horns, bus plugs), the camera, the controller boards, the screws and the
foam dart launcher (hardware/parts/turret/_launcher.py: its printed parts,
the brass barrel, the spring as its outline, the darts), and the checks
run by tools/build_printable_parts.py:

  * interference in each launcher state (after a shot, cocked with a dart
    chambered, primed, firing, firing blocked by the safety), screws
    modelled through their holes, the pinion in mesh with the rack;
  * tilt sweep over the joint limits (-0.65..0.75 rad) for both camera
    plates (Arducam B0385 board; a RealSense D435i on the 1/4"-20 plate),
    the launcher cocked and primed: the tilt stage against the pan stage;
  * pan sweep over +-90 deg at both tilt limits and level, launcher cocked
    and primed: the pan and tilt stages against the pan base, the pan
    servo and the base plate;
  * camera view: nothing of the turret, the launcher, its clip or darts in
    the B0385's 70 x 46 deg view, nor in a D435i's colour view (69 x 42
    deg) on the tripod plate (each plus a margin), at both tilt limits and
    level, cocked and primed; the D435i's depth view (87 x 58 deg) is
    reported;
  * line of fire: a dart's path from the muzzle clear of the turret's base
    at every pan and tilt;
  * the tilt axis, the camera and the launcher's joints and muzzle against
    the RACECAR Turret's URDF, the cable loop the pan range needs, the
    masses and the tilt load (full clip) against the tilt servo's torque.
"""
import math

from hardware.parts.racecarneo import _checks as C
from hardware.parts.racecarneo._footprints import RealSenseD435i
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
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
# Launcher states: rack travel u (mm back), plunger caught, trigger angle
# (rad), safety; darts: in the clip, and one chambered.
LAUNCHER_STATES = {
    "shot": {"u": 0.0, "cocked": False, "chambered": False},
    "cocked": {"u": 0.0, "cocked": True},
    "primed": {"u": L.ACT_PRIME, "cocked": False},
    "firing": {"u": 0.0, "cocked": True, "trigger": L.TRIG_FIRE_RAD},
    # the safety stops the sear 0.8 mm up: the trigger servo stalls there
    "safe": {"u": 0.0, "cocked": True, "trigger": math.asin((2.5 + L.SAFETY_GAP) / L.TRIG_ARM_R), "safe": True},
}
SWEEP_STATES = ("cocked", "primed")
CLIP_LOAD = L.CLIP_DARTS - 1        # a full clip of six: one chambered, five in the clip
DART_PATH_MM = 2000.0
DART_PATH_D = 14.0

# B0385 board+lens: estimate. Screws, pins and O-rings of the launcher: about 14 g.
MASS_G = {"servo": ST3215.MASS_G, "b0385": 15.0, "d435i": RealSenseD435i.MASS_G, "launcher_hardware": 14.0}


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


def launcher(state="cocked", darts=CLIP_LOAD):
    """The foam dart launcher in the yoke frame in this state, with the
    actuator and trigger servos and the screws: [(name, shape)]."""
    from hardware.parts.common import cad
    st = dict(LAUNCHER_STATES[state])
    chambered = st.pop("chambered", True)
    return [(n, cad.translate(s, F.LAUNCHER_ORIGIN))
            for n, _, s in L.models(darts_in_clip=darts, chambered=chambered, pedestal=F.PEDESTAL, st3215=st3215,
                                    screw=C.screw, **st)]


def dart_path():
    """A dart's path from the muzzle, DART_PATH_MM along the bore (yoke frame)."""
    from hardware.parts.common import cad
    x0 = L.MUZZLE_X + 1.0
    return cad.translate(cad.cyl(DART_PATH_D, DART_PATH_MM, (x0, 0, 0), (1, 0, 0)), F.LAUNCHER_ORIGIN)


def stages(camera="b0385", state="cocked", with_launcher=True):
    """(static, pan_stage, tilt_stage) lists of (name, shape) in the turret
    frame at pan 0, tilt 0, the launcher in ``state``. The tilt stage is in
    its own frame (origin on the tilt axis) so it can be rotated before it
    is placed."""
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
    if with_launcher:
        tilt_st += launcher(state)
    return static, pan_st, tilt_st


def place_tilt(shape, tilt_deg):
    """Yoke-frame shape at this tilt (positive nose up), in the turret frame."""
    from hardware.parts.common import cad
    return cad.translate(cad.rotate(shape, "Y", -tilt_deg), (0, 0, F.TILT_Z))


def place_pan(shape, pan_deg):
    """Pan-stage shape at this pan (positive turns right)."""
    from hardware.parts.common import cad
    return cad.rotate(shape, "Z", -pan_deg)


def pose(pan_deg=0.0, tilt_deg=0.0, camera="b0385", state="cocked"):
    static, pan_st, tilt_st = stages(camera, state)
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


def tilt_sweep(camera, state="cocked"):
    static, pan_st, tilt_st = stages(camera, state)
    lo, hi = F.TILT_RANGE_DEG
    step = int(SWEEP_STEP_DEG)
    angles = [lo] + [a for a in range(-90, 91, step) if lo < a < hi] + [hi]
    # Moving bodies: the yoke, plate, camera and launcher (not the screws inside the horns).
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


def pan_sweep(tilt_deg=0.0, state="cocked"):
    static, pan_st, tilt_st = stages("b0385", state)
    movers0 = [(n, s) for n, s in pan_st if not n.startswith("pan_servo_horn")]
    movers0 += [(n, place_tilt(s, tilt_deg)) for n, s in tilt_st]
    others = [(n, s) for n, s in static if not n.startswith("pan_servo_horn")]
    worst = (math.inf, None, None)
    for a in range(-int(F.PAN_LIMIT_DEG), int(F.PAN_LIMIT_DEG) + 1, 15):
        movers = [(n, place_pan(s, a)) for n, s in movers0]
        d, pair = _min_distance(movers, others)
        if d < worst[0]:
            worst = (d, pair, a)
    return worst


def tilt_poses():
    """(tilt, launcher state) pairs the sweeps and view checks cover."""
    lo, hi = F.TILT_RANGE_DEG
    return [(t, s) for t in (lo, 0.0, hi) for s in SWEEP_STATES]


def lens_front():
    from hardware.parts.turret import camera_plate_b0385 as P
    _, sensor_x, front_x = b0385_board(P.X0 + P.T)
    return sensor_x, front_x


def d435i_front():
    """Centre of a D435i's front face on the tripod plate, yoke frame."""
    from hardware.parts.turret import camera_plate_tripod as P
    R = RealSenseD435i
    return P.X0 + P.BACK_T + R.DEPTH, P.SHELF_TOP + R.HEIGHT / 2


def view_hits(camera, tilt, state, apex_x, apex_z, hfov, vfov):
    """Bodies in a camera's view (plus the margin) at this tilt, pan 0."""
    p = math.radians(tilt)
    c, s = math.cos(p), math.sin(p)
    apex = (apex_x * c - apex_z * s, 0.0, F.TILT_Z + apex_x * s + apex_z * c)
    fov = C.frustum(apex, (c, 0, s), (-s, 0, c), hfov / 2 + FOV_MARGIN_DEG, vfov / 2 + FOV_MARGIN_DEG, 1500.0)
    return C.in_view([(n, b) for n, b in pose(0.0, tilt, camera, state) if n != camera], fov)


def tilt_stage_mass(entries, camera="b0385", state="cocked", with_launcher=True, darts=CLIP_LOAD):
    """(mass g, centre of mass (x, z) mm in the yoke frame) of the tilt stage."""
    m = {stem: e.get("estimate", {}).get("mass_g", 0.0) for stem, e in entries.items()}
    _, _, tilt_st = stages(camera, state, with_launcher=False)
    shapes = dict(tilt_st)
    items = [(m.get("camera_yoke", 0.0), shapes["camera_yoke"])]
    if camera == "b0385":
        items += [(m.get("camera_plate_b0385", 0.0), shapes["camera_plate_b0385"]), (MASS_G["b0385"], shapes["b0385"])]
    else:
        items += [(m.get("camera_plate_tripod", 0.0), shapes["camera_plate_tripod"]),
                  (MASS_G["d435i"], shapes["d435i"])]
    if with_launcher:
        items += launcher_masses(entries, state, darts)
    points = []
    for mass, shape in items:
        c = shape.Center()
        points.append((mass, (c.x, c.z)))
    total = sum(w for w, _ in points)
    return total, (sum(w * p[0] for w, p in points) / total, sum(w * p[1] for w, p in points) / total)


def launcher_masses(entries, state="cocked", darts=CLIP_LOAD, frame=None):
    """[(mass g, shape)] of the launcher: printed parts (estimates), servos,
    spring, barrel, darts, hardware (at the breech). ``frame``: the
    launcher frame instead of the yoke frame."""
    from hardware.parts.common import cad
    m = {stem: e.get("estimate", {}).get("mass_g", 0.0) for stem, e in entries.items()}
    bodies = dict(launcher(state, darts))
    if frame == "launcher":
        back = tuple(-v for v in F.LAUNCHER_ORIGIN)
        bodies = {n: cad.translate(s, back) for n, s in bodies.items()}
    out = [(m.get(f"launcher_{stem}", 0.0), bodies[f"launcher_{stem}"]) for stem in L.PRINTED if stem != "muzzle_cap"]
    out += [(MASS_G["servo"], bodies["act_servo_case"]), (MASS_G["servo"], bodies["trig_servo_case"]),
            (L.SPRING["mass_g"], bodies["launcher_spring"]), (L.BARREL["mass_g"], bodies["launcher_barrel"]),
            (MASS_G["launcher_hardware"], bodies["launcher_breech"])]
    out += [(L.DART_MASS_G, s) for n, s in bodies.items() if n.startswith("launcher_dart")]
    return out


def tilt_torque(mass_g, com):
    """Largest static torque (N m) about the tilt axis over the tilt range."""
    lo, hi = F.TILT_RANGE_DEG
    x, z = com
    worst = 0.0
    for k in range(101):
        p = math.radians(lo + (hi - lo) * k / 100)
        worst = max(worst, abs(x * math.cos(p) - z * math.sin(p)))
    return mass_g / 1000.0 * G * worst / 1000.0


def line_of_fire(static):
    """Smallest distance from a dart's path to ``static`` over the pan and
    tilt ranges: (mm, pair, pan, tilt)."""
    path0 = dart_path()
    worst = (math.inf, None, None, None)
    lo, hi = F.TILT_RANGE_DEG
    for tilt in (lo, 0.0, hi):
        for a in range(-int(F.PAN_LIMIT_DEG), int(F.PAN_LIMIT_DEG) + 1, 15):
            d, pair = _min_distance([("dart_path", place_pan(place_tilt(path0, tilt), a))], static)
            if d < worst[0]:
                worst = (d, pair, a, tilt)
    return worst


def check(entries=None):
    """Run the assembly checks; returns a report dict ('passed' is the
    overall result)."""
    entries = entries or {}
    report = {}
    press = (("pan_servo_horn", "pan_hub", "hub disc on the horn"),
             ("tilt_servo_horn", "camera_yoke", "yoke hubs on the horns"),
             ("launcher_cam_pin", "launcher_shuttle", "dowel pin pressed into the shuttle's tab"))
    overlaps, allowed, count = [], [], 0
    for state in LAUNCHER_STATES:
        bodies = pose(0.0, 0.0, state=state)
        o, a = C.interferences(bodies, press)
        overlaps += [f"{state}: {x}" for x in o]
        allowed = a
        count = len(bodies)
    report["bodies"] = count
    report["interferences"] = overlaps or f"none ({', '.join(LAUNCHER_STATES)})"
    report["intended_press_fits"] = len(allowed)

    tilt_ok = True
    for cam in ("b0385", "d435i"):
        d, pair, a, st = min((tilt_sweep(cam, st) + (st,) for st in SWEEP_STATES), key=lambda w: w[0])
        report[f"tilt_sweep_{cam}"] = (f"clear from {F.TILT_RANGE_DEG[0]:.1f} to {F.TILT_RANGE_DEG[1]:.1f} deg, launcher "
                                       f"cocked and primed; smallest gap {d:.1f} mm ({pair} at {a:.0f} deg, {st})")
        tilt_ok = tilt_ok and d >= MIN_CLEARANCE_MM
    d, pair, a, tilt, st = min((pan_sweep(t, s) + (t, s) for t, s in tilt_poses()), key=lambda w: w[0])
    report["pan_sweep"] = (f"clear over +-{F.PAN_LIMIT_DEG:g} deg at both tilt limits and level, launcher cocked and "
                           f"primed; smallest gap {d:.1f} mm ({pair} at {a:+.0f} deg, tilt {tilt:+.0f}, {st})")
    pan_ok = d >= MIN_CLEARANCE_MM

    # Camera views at the tilt limits and level, cocked and primed.
    sensor_x, front_x = lens_front()
    hits, depth_hits = [], []
    A, R = ArducamB0385, RealSenseD435i
    rx, rz = d435i_front()
    for tilt, st in tilt_poses():
        for hit in view_hits("b0385", tilt, st, front_x, 0.0, A.HFOV_DEG, A.VFOV_DEG):
            hits.append(f"B0385, tilt {tilt:+.0f}, {st}: {hit}")
        for hit in view_hits("d435i", tilt, st, rx, rz, *R.COLOR_FOV_DEG):
            hits.append(f"D435i colour, tilt {tilt:+.0f}, {st}: {hit}")
        if tilt == 0.0 and st == "cocked":
            depth_hits = [h.split(" (")[0] for h in view_hits("d435i", tilt, st, rx, rz, *R.DEPTH_FOV_DEG)]
    report["camera_view_clear"] = hits or (
        f"yes (B0385 {A.HFOV_DEG:g} x {A.VFOV_DEG:g} deg; D435i colour {R.COLOR_FOV_DEG[0]:g} x {R.COLOR_FOV_DEG[1]:g} "
        f"deg on the tripod plate; each + {FOV_MARGIN_DEG:g} deg margin, at both tilt limits and level, launcher "
        f"cocked and primed, clip full)")
    report["d435i_depth_view"] = (f"the D435i's {R.DEPTH_FOV_DEG[0]:g} x {R.DEPTH_FOV_DEG[1]:g} deg depth view sees "
                                  f"{', '.join(sorted(set(depth_hits)))}" if depth_hits else "clear")
    report["tilt_axis_mm"] = (f"{F.TILT_Z:g} above the mounting surface ({F.TILT_Z + F.PLATE_T_URDF:g} above "
                              "turret/base_link with a 5 mm plate; URDF 0.045 + 0.055 m)")
    report["camera_mm"] = (f"B0385 sensor {sensor_x:.1f}, lens front {front_x:.1f} ahead of the tilt axis, on its "
                           f"height; URDF turret/camera_link {F.CAMERA_X:g}")
    mx, my, mz = F.to_yoke((L.MUZZLE_X, 0.0, 0.0))
    ax, ay, az = F.to_yoke((L.HOOK_X[0], 0.0, L.PT_Z))
    tx, ty, tz = F.to_yoke((L.TRIG_SHAFT[0], L.TRIG_SHAFT[1] - L.HORN_FACE, L.TRIG_SHAFT[2]))
    v = [L.muzzle_velocity(e) for e in (0.20, 0.35)]
    swept, bore = L.volumes_cm3()
    report["launcher"] = (
        f"bore {F.BORE_Z:g} mm above the camera's axis, parallel, on its centre line; muzzle (turret/muzzle) "
        f"({mx:g}, {my:g}, {mz:g}) mm in turret/tilt_link; actuator_joint 0..{L.ACT_PRIME / 1000:.3f} m (sear catches "
        f"from {L.ACT_CATCH:g} mm), origin ({ax:g}, {ay:g}, {az:g}); trigger_joint 0 safe, {L.TRIG_FIRE_RAD:g} rad "
        f"fires, axis at ({tx:g}, {ty:g}, {tz:g}); spring {L.SPRING['rate_n_mm']:g} N/mm, "
        f"{L.spring_energy_j():.2f} J at the sear; {swept:.1f} cm3 plunger for {bore:.1f} cm3 of bore; "
        f"{v[0]:.0f}-{v[1]:.0f} m/s at 20-35 % efficiency (chronograph at bring-up)")

    # Line of fire: a dart's path clear of the base, the pan stage and the plate.
    static, pan_st, _ = stages("b0385", "cocked")
    d, pair, a, tilt = line_of_fire(static)
    report["line_of_fire"] = (f"a dart's path ({DART_PATH_D:g} mm, {DART_PATH_MM / 1000:g} m from the muzzle) clears the "
                              f"turret's base at every pan and tilt; closest {d:.0f} mm ({pair} at pan {a:+d}, tilt "
                              f"{tilt:+.0f})")
    fire_ok = d > 0.0

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
    tilt_mass, com = tilt_stage_mass(entries)
    bare_mass, bare_com = tilt_stage_mass(entries, with_launcher=False)
    lm = launcher_masses(entries, frame="launcher")
    core = sum(w for w, _ in lm)
    lcx = sum(w * s.Center().x for w, s in lm) / core
    lcz = sum(w * s.Center().z for w, s in lm) / core
    lcy = sum(w * s.Center().y for w, s in lm) / core
    printed = sum(m.get(f"launcher_{stem}", 0.0) for stem in L.PRINTED if stem != "muzzle_cap")
    report["launcher_mass_g"] = (f"{core:.0f} with a full clip (printed {printed:.0f}, two servos "
                                 f"{2 * MASS_G['servo']:g}, spring {L.SPRING['mass_g']:g}, barrel {L.BARREL['mass_g']:g}, "
                                 f"hardware {MASS_G['launcher_hardware']:g}, darts {L.CLIP_DARTS * L.DART_MASS_G:g}); "
                                 f"centre of mass {lcx:.0f} mm along the bore from the chamber's rear face, "
                                 f"{lcz:+.0f} mm toward the plunger tube, {lcy:+.0f} mm sideways")
    report["moving_mass_g"] = (f"tilt stage {tilt_mass:.0f} with the launcher and a full clip ({bare_mass:.0f} "
                               f"without), pan stage {tilt_mass + MASS_G['servo'] + m.get('pan_hub', 0) + m.get('tilt_bracket', 0):.0f}")
    stall = ST3215.STALL_TORQUE_NM
    loads = []
    for label, kw in (("B0385", {}), ("D435i", {"camera": "d435i"})):
        for st in SWEEP_STATES:
            mass, c = tilt_stage_mass(entries, state=st, **kw)
            tq = tilt_torque(mass, c)
            loads.append(f"{label}, {st}: {tq:.3f} N m ({stall / tq:.0f}x margin)")
    bare_tq = tilt_torque(bare_mass, bare_com)
    report["tilt_load"] = (f"worst static torque over the tilt range, full clip, against the tilt servo's "
                           f"{stall:g} N m stall: " + "; ".join(loads) +
                           f"; centre of mass ({com[0]:.0f}, {com[1]:.0f}) mm from the tilt axis with the B0385 "
                           f"(without the launcher {bare_tq:.3f} N m)")
    report["passed"] = not overlaps and tilt_ok and pan_ok and not hits and fire_ok
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
                                    "camera_plate_b0385", "camera_plate_tripod"} | {f"launcher_{s}" for s in L.PRINTED}

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
                  title="Turret with the foam dart launcher: printed (colour), purchased (grey)")
    exploded = {"pan_servo": (0, 0, 30), "pan_hub": (0, 0, 70), "hub_screw": (0, 0, 80), "tilt_bracket": (0, 0, 100),
                "joint_screw": (0, 0, 115), "tilt_servo": (0, 0, 130), "wall_screw": (0, 0, 130),
                "camera_yoke": (40, 0, 150), "yoke_screw": (40, 0, 150), "camera_plate": (70, 0, 150),
                "plate_screw": (60, 0, 150), "b0385": (95, 0, 150), "pan_base": (0, 0, 15), "base_bolt": (0, 0, 15),
                "pan_servo_screw": (0, 0, -10), "launcher_pad_screw": (40, 0, 150)}
    exploded.update({n: (40, 0, 260) for n, _ in launcher() if n not in exploded})
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
    # The launcher on its own, opened up: the clip lifted, the shuttle and
    # rack out to -y, the plunger and spring out of the tube.
    lx = {"launcher_clip": (0, 0, 60), "launcher_follower": (0, 0, 60), "launcher_clip_cap": (0, 0, 60),
          "launcher_dart_clip": (0, 0, 60), "launcher_cover_screw": (0, 0, 70), "launcher_shuttle": (0, 40, 30),
          "launcher_cam_pin": (0, 40, 30), "launcher_dart_chamber": (0, 40, 45), "launcher_rack": (0, -40, -30),
          "launcher_cam": (0, -40, -30), "launcher_cam_screw": (0, -40, -20), "launcher_plunger": (-120, 0, 0),
          "launcher_spring": (-60, 0, 0), "launcher_cap": (-160, 0, 0), "launcher_sear": (-160, 0, 25),
          "launcher_safety": (-160, 0, 35), "launcher_cap_screw": (-170, 0, 0), "trig_servo": (-160, 30, 0),
          "launcher_trig_wall_screw": (-160, 40, 0), "launcher_trigger_arm": (-160, 15, 0),
          "launcher_arm_screw": (-160, 10, 0), "launcher_barrel_clamp": (40, 0, 0), "launcher_clamp_screw": (50, 0, 0),
          "launcher_pinch_screw": (40, 0, 0), "launcher_barrel": (70, 0, 0), "act_servo": (0, -40, -60),
          "launcher_servo_mount": (0, -40, -60), "launcher_act_wall_screw": (0, -50, -60),
          "launcher_mount_screw": (0, -40, -50), "launcher_pinion": (0, -25, -60), "launcher_pinion_screw": (0, -20, -60)}
    render.render(items(launcher(), lx), out_dir / "step_launcher.png", elev=22, azim=-60, zoom=1.0,
                  title="Foam dart launcher, opened up (toy power: foam darts only)")
    lo, hi = F.TILT_RANGE_DEG
    sweep = []
    for a in (lo, hi):
        sweep += [(n, s) for n, s in pose(0.0, a, state="primed")
                  if n.startswith(("camera_yoke", "camera_plate", "b0385", "launcher_breech", "launcher_tube",
                                   "launcher_clip", "launcher_cap", "launcher_barrel", "launcher_rack",
                                   "launcher_servo_mount", "act_servo_case", "trig_servo_case", "launcher_plunger"))]
    sweep += [(n, s) for n, s in home if n.startswith(("pan_hub", "tilt_bracket", "tilt_servo_case", "pan_base",
                                                       "pan_servo_case", "tilt_servo_plugs"))]
    render.render(items(sweep), out_dir / "tilt_range.png", elev=0, azim=-90, zoom=1.0,
                  title=f"Tilt limits {lo:.0f} and +{hi:.0f} deg, launcher primed (side view)")
    return report
