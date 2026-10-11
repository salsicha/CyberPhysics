"""Aerodrone assembly: printed parts placed on the vehicle with simple
datasheet-sized models of the purchased parts, and the checks run by
tools/build_printable_parts.py:

  * interference: no two bodies overlap (screws are modelled at their
    nominal diameter through the clearance or insert holes, so a hole in
    the wrong place shows up as an overlap), except declared press fits;
  * camera view: nothing enters the OAK-1's 69 x 55 deg view (plus a
    margin) below the lens;
  * rangefinder beam and propeller discs are clear;
  * centre of gravity: where the battery must sit for the CG to be on the
    hub centre, and the take-off mass.

``figures()`` renders the assembly-instruction images.
"""
import math

from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import fasteners
from hardware.parts.common.footprints import (
    BenewakeTFminiS,
    BlueRoboticsNavigator,
    CarbonTube16,
    Holybro2216KV920,
    HolybroM10GPS,
    LipoEnvelope4S,
    LuxonisOAK1,
    Propeller1045,
    RaspberryPi5,
    SiliconeDamperBall,
)

OVERLAP_TOLERANCE_MM3 = 0.5
FOV_MARGIN_DEG = 2.0
SCREW_SLACK = 0.02          # screw bodies are this much under nominal diameter
CLIP_R = (176.0, 192.0)
ESC_R = (118.0, 168.0)
ESC_SIZE = (50.0, 22.0, 8.0)     # envelope, not a datasheet
CARBON_DENSITY = 1.55            # g/cm3, roll-wrapped carbon tube

# Pairs (name prefixes) that overlap on purpose.
PRESS_FITS = (
    ("landing_leg", "leg_foot", "TPU foot pressed onto the leg tenon"),
    ("cable_clip", "arm_tube", "clip grips the tube"),
)

# Masses of purchased parts (g) and where they come from.
PURCHASED_MASS = {
    "motor": (Holybro2216KV920.MASS_G, "reseller listing (unverified)"),
    "prop": (Propeller1045.MASS_G, "T-Motor T1045 specification"),
    "pi5": (RaspberryPi5.MASS_G, "typical (unverified)"),
    "heatsink": (BlueRoboticsNavigator.HEATSINK_MASS_G, "Blue Robotics"),
    "navigator": (BlueRoboticsNavigator.MASS_G, "Blue Robotics"),
    "oak1": (LuxonisOAK1.MASS_G, "Luxonis datasheet"),
    "gps": (HolybroM10GPS.MASS_G, "Holybro"),
    "tfmini": (BenewakeTFminiS.MASS_G, "Benewake"),
    "battery": (LipoEnvelope4S.MASS_G, "design envelope"),
    "esc": (25.0, "estimate for a 20-30 A ESC with leads"),
    "power_module": (20.0, "estimate"),
    "bec": (25.0, "estimate"),
    "wiring_rc_telemetry": (150.0, "estimate: wiring, XT60, RC receiver, telemetry radio, USB-UART, cables"),
    "screw": (1.0, "estimate per M3 screw"),
}


# --------------------------------------------------------------------- placement

def _arm_place(shape, angle, r=0.0):
    from hardware.parts.common import cad
    return cad.rotate_z(cad.translate(shape, (r, 0, 0)), angle)


def _screw(axis_point, direction, length, size, head=True, slack=SCREW_SLACK):
    """Socket head screw: head on the outside of ``axis_point`` (opposite to
    ``direction``), shank ``length`` along ``direction``."""
    from hardware.parts.common import cad
    d = fasteners.MAJOR[size] - slack
    dx, dy, dz = direction
    x, y, z = axis_point
    body = cad.cyl(d, length, (x, y, z), direction)
    if head:
        hd, hh = fasteners.SHCS_HEAD[size]
        body = body.fuse(cad.cyl(hd, hh, (x - dx * hh, y - dy * hh, z - dz * hh), direction))
    return body


def _purchased():
    """[(name, shape, mass_g)] for every purchased part in the model."""
    import cadquery as cq

    from hardware.parts.common import cad
    out = []

    def add(name, shape, mass=0.0):
        out.append((name, shape, mass))

    tube_len = F.TUBE_LENGTH
    tube_mass = math.pi / 4 * (CarbonTube16.OD ** 2 - CarbonTube16.ID ** 2) * tube_len / 1000 * CARBON_DENSITY
    for i, a in enumerate(F.ARM_ANGLES):
        tube = cad.cyl(CarbonTube16.OD, tube_len, (F.TUBE_R0, 0, 0), (1, 0, 0)).cut(
            cad.cyl(CarbonTube16.ID, tube_len + 2, (F.TUBE_R0 - 1, 0, 0), (1, 0, 0)))
        add(f"arm_tube_{i}", _arm_place(tube, a), tube_mass)
        # motor with four threaded holes on the 16 mm cross
        motor = cad.cyl(Holybro2216KV920.DIAMETER, Holybro2216KV920.HEIGHT, (0, 0, F.MOTOR_SEAT_Z))
        for ang in (0, 90, 180, 270):
            hx, hy = cad.polar(Holybro2216KV920.MOUNT_PATTERN[0] / 2, ang)
            motor = motor.cut(cad.cyl(fasteners.MAJOR["M3"], 6.0, (hx, hy, F.MOTOR_SEAT_Z - 0.01)))
            add(f"motor_screw_{i}_{ang}", _arm_place(
                _screw((hx, hy, F.MOTOR_PLATE_Z[0]), (0, 0, 1), 8.0, "M3"), a, F.MOTOR_R),
                PURCHASED_MASS["screw"][0])
        add(f"motor_{i}", _arm_place(motor, a, F.MOTOR_R), PURCHASED_MASS["motor"][0])
        prop = cad.cyl(Propeller1045.DIAMETER, 3.0, (0, 0, F.PROP_Z))
        add(f"prop_{i}", _arm_place(prop, a, F.MOTOR_R), PURCHASED_MASS["prop"][0])
        esc = cad.box(ESC_R[0], ESC_R[1], -ESC_SIZE[1] / 2, ESC_SIZE[1] / 2,
                      -CarbonTube16.OD / 2 - ESC_SIZE[2] - 0.5, -CarbonTube16.OD / 2 - 0.5)
        add(f"esc_{i}", _arm_place(esc, a), PURCHASED_MASS["esc"][0])
        # pinch screws of the motor mount
        from hardware.parts.aerodrone import motor_mount as MM
        for u in MM.PINCH_U:
            add(f"pinch_screw_{i}_{u:g}", _arm_place(
                _screw((u, -MM.BODY_HALF_W, MM.PINCH_Z), (0, 1, 0), 25.0, "M3"), a, F.MOTOR_R),
                PURCHASED_MASS["screw"][0])

    from hardware.parts.aerodrone import _hub
    for k, (x, y) in enumerate(_hub.clamp_bolt_xy()):
        add(f"clamp_bolt_{k}", _screw((x, y, F.TOP_Z[1]), (0, 0, -1), 25.0, "M3"), PURCHASED_MASS["screw"][0])
    for k, (x, y) in enumerate(_hub.leg_bolt_xy()):
        add(f"leg_bolt_{k}", _screw((x, y, -29.0), (0, 0, 1), 16.0, "M3"), PURCHASED_MASS["screw"][0])

    # Pi 5 + heatsink + Navigator
    px0 = F.PI_CENTER[0] - RaspberryPi5.BOARD[0] / 2
    py0 = F.PI_CENTER[1] - RaspberryPi5.BOARD[1] / 2
    z = F.PI_Z0
    pcb = RaspberryPi5.PCB_THICKNESS
    pi = cad.box(px0, px0 + RaspberryPi5.BOARD[0], py0, py0 + RaspberryPi5.BOARD[1], z, z + pcb)
    holes = F.pi_holes_body()
    for x, y in holes:
        pi = pi.cut(cad.cyl(RaspberryPi5.HOLE_D, pcb + 2, (x, y, z - 1)))
    add("pi5", pi, PURCHASED_MASS["pi5"][0])
    # USB/Ethernet jacks: envelope at the front end, 3 mm proud of the board
    px1 = px0 + RaspberryPi5.BOARD[0]
    add("pi5_jacks", cad.box(px0 + BlueRoboticsNavigator.BOARD[0] + 0.5, px1 + RaspberryPi5.CONNECTOR_OVERHANG,
                             py0 + 2, py0 + RaspberryPi5.BOARD[1] - 2, z + pcb, z + pcb + 16))
    hs_x, hs_y, hs_z = BlueRoboticsNavigator.HEATSINK
    zh = z + pcb
    hs = cad.box(px0, px0 + hs_x, py0, py0 + hs_y, zh, zh + hs_z)
    for x, y in holes:
        hs = hs.cut(cad.cyl(fasteners.MAJOR["M2.5"], hs_z + 2, (x, y, zh - 1)))
    add("navigator_heatsink", hs, PURCHASED_MASS["heatsink"][0])
    zn = zh + hs_z
    nav = cad.box(px0, px0 + BlueRoboticsNavigator.BOARD[0], py0, py0 + BlueRoboticsNavigator.BOARD[1], zn, zn + 1.6)
    add("navigator", nav, PURCHASED_MASS["navigator"][0])
    add("navigator_connectors", cad.box(px0, px0 + BlueRoboticsNavigator.BOARD[0], py0, py0 + BlueRoboticsNavigator.BOARD[1],
                                        zn + 1.6, zn + 1.6 + BlueRoboticsNavigator.CONNECTOR_HEIGHT))
    head_h = fasteners.SHCS_HEAD["M2.5"][1]
    for k, (x, y) in enumerate((holes[0], holes[3])):
        add(f"pi_screw_{k}", _screw((x, y, F.TRAY_Z0 + head_h), (0, 0, 1), 12.0, "M2.5"))
    for k, (x, y) in enumerate(F.STACK_GROMMETS):
        top = F.TRAY_Z0 + F.TRAY_T + F.GROMMET_STACK[2]
        add(f"grommet_bolt_{k}", _screw((x, y, top), (0, 0, -1), 12.0, "M3"), PURCHASED_MASS["screw"][0])

    # GPS on the mast
    seat = F.TOP_Z[1] + F.MAST_HEIGHT
    add("gps", cad.cyl(HolybroM10GPS.V2[0], HolybroM10GPS.V2[1], (F.MAST_XY[0], F.MAST_XY[1], seat)),
        PURCHASED_MASS["gps"][0])
    for k, (x, y) in enumerate(F.mast_bolts_body()):
        add(f"mast_bolt_{k}", _screw((x, y, F.TOP_Z[1] + 5.0), (0, 0, -1), 10.0, "M3"), PURCHASED_MASS["screw"][0])

    # OAK-1, tripod screw, dampers, pivot and lock screws
    cx = F.CAMERA_X
    half = LuxonisOAK1.HEIGHT / 2
    oak = cad.box(cx - half, cx + half, -LuxonisOAK1.WIDTH / 2, LuxonisOAK1.WIDTH / 2, F.CAMERA_LENS_Z, F.CAMERA_BACK_Z)
    tripod_x = cx - half + LuxonisOAK1.TRIPOD_FROM_USB_END
    oak = oak.cut(cad.cyl(fasteners.THREAD_1_4_20_MAJOR, LuxonisOAK1.TRIPOD_DEPTH + 0.01,
                          (tripod_x, 0, F.CAMERA_BACK_Z - LuxonisOAK1.TRIPOD_DEPTH)))
    add("oak1", oak, PURCHASED_MASS["oak1"][0])
    cradle_top = F.CAMERA_BACK_Z + F.CRADLE_T
    tripod = cad.cyl(fasteners.THREAD_1_4_20_MAJOR - SCREW_SLACK, 12.7, (tripod_x, 0, cradle_top), (0, 0, -1)).fuse(
        cad.cyl(11.1, 3.35, (tripod_x, 0, cradle_top)))
    add("tripod_screw", tripod, 5.0)
    zc = (F.NOSE_Z[0] + F.STAGE_Z[1]) / 2
    for k, (x, y) in enumerate(F.DAMPERS):
        ball = cq.Solid.makeSphere(SiliconeDamperBall.PLATE_SPACING / 2, cad.V(x, y, zc), angleDegrees1=-90, angleDegrees2=90)
        neck = cad.cyl(SiliconeDamperBall.HOLE_D - 0.1, SiliconeDamperBall.PLATE_SPACING + 2 * F.DAMPER_TAB_T,
                       (x, y, F.STAGE_Z[1] - F.DAMPER_TAB_T))
        add(f"damper_{k}", ball.fuse(neck), 2.0)
    from hardware.parts.aerodrone import oak1_stage as ST
    for sign in (1, -1):
        for z_s in (F.PIVOT_Z, F.PIVOT_Z + F.LOCK_R):
            add(f"cradle_screw_{sign}_{z_s:.0f}",
                _screw((cx, sign * ST.CHEEK_Y[1], z_s), (0, -sign, 0), 8.0, "M3"), PURCHASED_MASS["screw"][0])

    # TFmini-S, long axis across the nose, ears against the nose underside
    rx = F.RANGEFINDER_X
    ear_z = (F.NOSE_Z[0] - BenewakeTFminiS.EAR_THICKNESS, F.NOSE_Z[0])
    tf = cad.box(rx - BenewakeTFminiS.WIDTH / 2, rx + BenewakeTFminiS.WIDTH / 2,
                 -BenewakeTFminiS.LENGTH / 2, BenewakeTFminiS.LENGTH / 2, *ear_z).fuse(
        cad.box(rx - BenewakeTFminiS.WIDTH / 2, rx + BenewakeTFminiS.WIDTH / 2,
                -BenewakeTFminiS.LENS_BLOCK_LENGTH / 2, BenewakeTFminiS.LENS_BLOCK_LENGTH / 2,
                F.NOSE_Z[0] - BenewakeTFminiS.HEIGHT, ear_z[0] + 0.01))
    for s in (1, -1):
        tf = tf.cut(cad.cyl(BenewakeTFminiS.HOLE_D, 10, (rx, s * BenewakeTFminiS.HOLE_SPACING / 2, ear_z[0] - 1)))
        add(f"tfmini_screw_{s}", cad.cyl(BenewakeTFminiS.HOLE_D - 0.05, 9.5,
                                         (rx, s * BenewakeTFminiS.HOLE_SPACING / 2, F.NOSE_Z[1]), (0, 0, -1)).fuse(
            cad.cyl(5.6, 2.2, (rx, s * BenewakeTFminiS.HOLE_SPACING / 2, F.NOSE_Z[1]))))
    add("tfmini_s", tf, PURCHASED_MASS["tfmini"][0])
    for k, (x, y) in enumerate(F.NOSE_INSERTS):
        add(f"nose_bolt_{k}", _screw((x, y, F.NOSE_Z[0]), (0, 0, 1), 10.0, "M3"), PURCHASED_MASS["screw"][0])

    # power module and BEC inside the hub, battery under the tray (x set later)
    z = F.BOT_Z[1]
    add("power_module", cad.box(-12, 12, -10, 10, z, z + 10), PURCHASED_MASS["power_module"][0])
    add("bec", cad.box(-40, -22, -10, 10, z, z + 8), PURCHASED_MASS["bec"][0])
    for k, (x, y) in enumerate(F.BATTERY_TRAY_INSERTS):
        z_b = F.BOT_Z[0] - F.TRAY_POST_H - F.BATTERY_TRAY_T + 3.5
        add(f"tray_bolt_{k}", _screw((x, y, z_b), (0, 0, 1), 14.0, "M3"), PURCHASED_MASS["screw"][0])
    return out


def battery(x_center):
    from hardware.parts.common import cad
    sx, sy, sz = LipoEnvelope4S.NOMINAL_SIZE
    return cad.box(x_center - sx / 2, x_center + sx / 2, -sy / 2, sy / 2, F.BATTERY_TOP_Z - sz, F.BATTERY_TOP_Z)


def printed():
    """[(name, part id, shape)] for every printed piece in place."""
    from hardware.parts.aerodrone import (
        battery_tray,
        cable_clip,
        gps_mast,
        hub_bottom_plate,
        hub_top_plate,
        landing_leg,
        leg_foot,
        motor_mount,
        oak1_cradle,
        oak1_stage,
        sensor_nose,
        stack_grommet,
        stack_tray,
    )
    from hardware.parts.common import cad
    out = []
    for mod in (hub_bottom_plate, hub_top_plate, stack_tray, gps_mast, sensor_nose, oak1_stage, oak1_cradle, battery_tray):
        stem = mod.__name__.rsplit(".", 1)[1]
        out.append((stem, stem, mod.build()))
    mm, leg, foot, clip = motor_mount.build(), landing_leg.build(), leg_foot.build(), cable_clip.build()
    for i, a in enumerate(F.ARM_ANGLES):
        out.append((f"motor_mount_{i}", "motor_mount", _arm_place(mm, a, F.MOTOR_R)))
        out.append((f"landing_leg_{i}", "landing_leg", _arm_place(leg, a)))
        out.append((f"leg_foot_{i}", "leg_foot", _arm_place(foot, a)))
        for j, r in enumerate(CLIP_R):
            out.append((f"cable_clip_{i}_{j}", "cable_clip", _arm_place(clip, a, r)))
    g = stack_grommet.build()
    for k, (x, y) in enumerate(F.STACK_GROMMETS):
        out.append((f"stack_grommet_lower_{k}", "stack_grommet", cad.translate(g, (x, y, F.TOP_Z[1]))))
        upper = cad.rotate(g, "X", 180)
        top = F.TRAY_Z0 + F.TRAY_T + F.GROMMET_STACK[2]
        out.append((f"stack_grommet_upper_{k}", "stack_grommet", cad.translate(upper, (x, y, top))))
    return out


# --------------------------------------------------------------------- checks

def _bbox_overlap(a, b, tol=0.01):
    return not (a.xmax < b.xmin - tol or b.xmax < a.xmin - tol or a.ymax < b.ymin - tol or
                b.ymax < a.ymin - tol or a.zmax < b.zmin - tol or b.zmax < a.zmin - tol)


def _allowed(n1, n2):
    for a, b, why in PRESS_FITS:
        if (n1.startswith(a) and n2.startswith(b)) or (n1.startswith(b) and n2.startswith(a)):
            return why
    return None


def _frustum(apex, half_x_deg, half_y_deg, depth, start=0.5):
    """Downward pyramid from ``apex`` (lens centre), image x along body x."""
    import cadquery as cq
    x, y, z = apex
    tx, ty = math.tan(math.radians(half_x_deg)), math.tan(math.radians(half_y_deg))
    z0, z1 = z - start, z - depth
    w0 = (2 * tx * start, 2 * ty * start)
    w1 = (2 * tx * depth, 2 * ty * depth)
    return (cq.Workplane("XY", origin=(x, y, z0)).rect(*w0)
            .workplane(offset=z1 - z0).rect(*w1).loft(combine=True).val())


def lens_point():
    return (F.CAMERA_X - LuxonisOAK1.HEIGHT / 2 + LuxonisOAK1.LENS_FROM_USB_END, 0.0, F.CAMERA_LENS_Z)


def battery_position(masses, centroids):
    """Battery centre x that puts the CG on the hub centre, given the other
    masses (g) and centroids (mm)."""
    m = sum(masses)
    mx = sum(mi * c[0] for mi, c in zip(masses, centroids))
    return -mx / LipoEnvelope4S.MASS_G, m + LipoEnvelope4S.MASS_G


def check(entries=None):
    """Run the assembly checks; returns a report dict ('passed' is the
    overall result)."""
    from hardware.parts.common import cad
    entries = entries or {}
    report = {}
    bodies = []
    masses, centroids = [], []
    for name, stem, shape in printed():
        bodies.append((name, shape))
        mass = entries.get(stem, {}).get("estimate", {}).get("mass_g", 0.0)
        masses.append(mass)
        centroids.append(shape.Center().toTuple())
    for name, shape, mass in _purchased():
        bodies.append((name, shape))
        if mass:
            masses.append(mass)
            centroids.append(shape.Center().toTuple())
    misc = PURCHASED_MASS["wiring_rc_telemetry"][0]
    masses.append(misc)
    centroids.append((0.0, 0.0, 0.0))
    x_batt, auw = battery_position(masses, centroids)
    bodies.append(("battery", battery(x_batt)))
    masses.append(LipoEnvelope4S.MASS_G)
    centroids.append(bodies[-1][1].Center().toTuple())
    m = sum(masses)
    cg = [sum(mi * c[k] for mi, c in zip(masses, centroids)) / m for k in range(3)]
    tray_ok = (F.BATTERY_TRAY_X[0] <= x_batt - LipoEnvelope4S.NOMINAL_SIZE[0] / 2 + 15 and
               x_batt + LipoEnvelope4S.NOMINAL_SIZE[0] / 2 - 15 <= F.BATTERY_TRAY_X[1])
    report["battery_centre_x_mm"] = round(x_batt, 1)
    report["battery_on_tray"] = tray_ok
    report["cg_mm"] = [round(v, 1) for v in cg]
    report["takeoff_mass_g"] = round(auw)

    boxes = [(n, s, s.BoundingBox()) for n, s in bodies]
    overlaps, allowed = [], []
    for i in range(len(boxes)):
        n1, s1, b1 = boxes[i]
        for j in range(i + 1, len(boxes)):
            n2, s2, b2 = boxes[j]
            if not _bbox_overlap(b1, b2):
                continue
            v = s1.intersect(s2).Volume()
            if v > OVERLAP_TOLERANCE_MM3:
                why = _allowed(n1, n2)
                if why:
                    allowed.append(f"{n1}/{n2} {v:.0f} mm3 ({why})")
                else:
                    overlaps.append(f"{n1} / {n2}: {v:.1f} mm3")
    report["bodies"] = len(bodies)
    report["interferences"] = overlaps or "none"
    report["intended_press_fits"] = len(allowed)

    lens = lens_point()
    fov = _frustum(lens, LuxonisOAK1.VFOV_DEG / 2 + FOV_MARGIN_DEG, LuxonisOAK1.HFOV_DEG / 2 + FOV_MARGIN_DEG, 1500.0)
    fb = fov.BoundingBox()
    in_view = []
    for n, s, b in boxes:
        if n == "oak1" or not _bbox_overlap(fb, b):
            continue
        v = s.intersect(fov).Volume()
        if v > OVERLAP_TOLERANCE_MM3:
            in_view.append(f"{n} ({v:.0f} mm3)")
    report["camera_view_clear"] = in_view or f"yes (69 x 55 deg + {FOV_MARGIN_DEG:g} deg margin)"

    rx = F.RANGEFINDER_X
    beam = cad.cone(20.0, 20.0 + 2 * 1000 * math.tan(math.radians(2.0)), 1000.0,
                    (rx, 0, F.NOSE_Z[0] - BenewakeTFminiS.HEIGHT - 0.5), (0, 0, -1))
    bb = beam.BoundingBox()
    blocked = [n for n, s, b in boxes if not n.startswith("tfmini") and _bbox_overlap(bb, b)
               and s.intersect(beam).Volume() > OVERLAP_TOLERANCE_MM3]
    report["rangefinder_beam_clear"] = blocked or "yes"

    prop_hits = []
    for i, a in enumerate(F.ARM_ANGLES):
        disc = _arm_place(cad.cyl(Propeller1045.DIAMETER + 10.0, 12.0, (0, 0, F.PROP_Z - 3.0)), a, F.MOTOR_R)
        db = disc.BoundingBox()
        for n, s, b in boxes:
            if n in (f"prop_{i}", f"motor_{i}") or not _bbox_overlap(db, b):
                continue
            if s.intersect(disc).Volume() > OVERLAP_TOLERANCE_MM3:
                prop_hits.append(f"prop {i}: {n}")
    report["propeller_clearance"] = prop_hits or "yes (5 mm radial, 3 mm below and 9 mm above the disc)"
    report["passed"] = not overlaps and not in_view and not blocked and not prop_hits and tray_ok
    return report


# --------------------------------------------------------------------- figures

def _mesh(shape, tol=0.25):
    import numpy as np
    import trimesh
    verts, tris = shape.tessellate(tol, 0.4)
    return trimesh.Trimesh(np.array([v.toTuple() for v in verts]), np.array(tris), process=False)


def figures(out_dir, entries=None):
    """Render the assembly-instruction figures into ``out_dir``."""
    from pathlib import Path

    from hardware.parts.common import render
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    report = check(entries)
    materials = {stem: e.get("material", "PETG") for stem, e in (entries or {}).items()}
    pieces = [(n, stem, s) for n, stem, s in printed()]
    buy = [(n, s) for n, s, _ in _purchased() if not n.startswith(("prop_",))]
    buy.append(("battery", battery(report["battery_centre_x_mm"])))
    meshes = {}

    def mesh(name, shape):
        if name not in meshes:
            meshes[name] = _mesh(shape)
        return meshes[name]

    def color(stem):
        return render.MATERIAL_COLORS.get(materials.get(stem, "PETG"), "#4c9be8")

    def sel(prefixes, purchased_prefixes=(), offsets=None):
        items = []
        offsets = offsets or {}
        for n, stem, s in pieces:
            if any(n.startswith(p) for p in prefixes):
                m = mesh(n, s)
                off = next((v for k, v in offsets.items() if n.startswith(k)), None)
                items.append((m.copy().apply_translation(off) if off else m, color(stem)))
        for n, s in buy:
            if any(n.startswith(p) for p in purchased_prefixes):
                m = mesh(n, s)
                off = next((v for k, v in offsets.items() if n.startswith(k)), None)
                items.append((m.copy().apply_translation(off) if off else m, render.PURCHASED_COLOR))
        return items

    every = ("",)
    render.render(sel(every, every), out_dir / "overview.png", zoom=1.15,
                  title="Aerodrone: printed parts (colour) and purchased parts (grey)")
    exploded = {"hub_top_plate": (0, 0, 60), "stack_grommet_upper": (0, 0, 120), "stack_grommet_lower": (0, 0, 60),
                "stack_tray": (0, 0, 110), "pi5": (0, 0, 140), "navigator": (0, 0, 140), "pi_screw": (0, 0, 110),
                "grommet_bolt": (0, 0, 150), "gps_mast": (0, 0, 180), "gps": (0, 0, 200), "mast_bolt": (0, 0, 190),
                "clamp_bolt": (0, 0, 100), "battery_tray": (0, 0, -60), "battery": (0, 0, -110), "tray_bolt": (0, 0, -100),
                "sensor_nose": (40, 0, -40), "nose_bolt": (40, 0, -60), "oak1_stage": (50, 0, -80),
                "damper": (45, 0, -60), "oak1_cradle": (60, 0, -120), "oak1": (60, 0, -160), "tripod_screw": (60, 0, -100),
                "cradle_screw": (50, 0, -80), "tfmini": (40, 0, -70), "landing_leg": (0, 0, -40), "leg_foot": (0, 0, -60),
                "leg_bolt": (0, 0, -30)}
    render.render(sel(every, every, exploded), out_dir / "exploded.png", elev=18, azim=-60, zoom=0.95,
                  title="Exploded view")
    render.render(sel(("hub_", "motor_mount", "cable_clip"), ("arm_tube", "motor", "clamp_bolt", "pinch_screw", "esc")),
                  out_dir / "step_frame.png", elev=35, azim=-50, zoom=0.9, title="Frame: hub plates, arm tubes, motor mounts")
    render.render(sel(("hub_top_plate", "stack_", "gps_mast"),
                      ("pi5", "navigator", "pi_screw", "grommet_bolt", "gps", "mast_bolt"),
                      {"stack_grommet_upper": (0, 0, 25), "stack_tray": (0, 0, 15), "pi5": (0, 0, 30), "navigator": (0, 0, 30),
                       "pi_screw": (0, 0, 10), "grommet_bolt": (0, 0, 35)}),
                  out_dir / "step_stack.png", elev=25, azim=-40, title="Pi 5 + Navigator stack and GPS mast")
    render.render(sel(("sensor_nose", "oak1_"), ("oak1", "tripod_screw", "damper", "cradle_screw", "tfmini", "nose_bolt")),
                  out_dir / "step_nose.png", elev=-25, azim=-135, zoom=0.9,
                  title="Sensor nose: OAK-1 on dampers, rangefinder (from below)")
    render.render(sel(("hub_bottom_plate", "landing_leg", "leg_foot", "battery_tray", "sensor_nose"),
                      ("battery", "tray_bolt", "leg_bolt")),
                  out_dir / "step_legs_battery.png", elev=-20, azim=-60, zoom=0.95,
                  title="Landing legs, battery tray and battery (from below)")
    return report
