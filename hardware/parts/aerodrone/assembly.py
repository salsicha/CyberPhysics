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

The same checks for the launcher configuration (the foam dart launcher of
hardware/parts/turret/_launcher.py under the hub pointing down, the battery
on the deck above the stack, the tall legs), cocked and primed, plus: the
battery position on the deck for a centred CG with a full and an empty clip
and the bore's offset from the CG; the clearances of the launcher to the
airframe; a dart's path to the ground; the muzzle's ground clearance and
the tip-over angle on the feet; the compass's distance from the battery; the HaLowLink 2's antenna (on the
top deck in both configurations) against the arms, propellers, GPS mast
and launcher;
and the flight impact (take-off mass, thrust-to-weight, hover throttle and
flight time from the motor and propeller test data).

``figures()`` renders the assembly-instruction images.
"""
import json
import math
from pathlib import Path

from hardware.parts.aerodrone import _frame as F
from hardware.parts.aerodrone._footprints import HaLowLink2, PololuD36V28F5
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
    ("leg_tube", "tall_leg_foot", "TPU foot pressed onto the leg tube"),
    ("launcher_dart_detent", "launcher_breech_down", "TPU detent ring pressed into the barrel seat"),
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
    "wiring_rc_telemetry": (125.0, "estimate: wiring, XT60, RC receiver, USB-UART, cables (no SiK radio: MAVLink over HaLow)"),
    "halow": (HaLowLink2.MASS_G, "Morse Micro HaLowLink 2 product brief (in its case)"),
    "halow_antenna": (HaLowLink2.ANTENNA_MASS_G, "estimate (the brief gives 66 g for the stand and the antenna)"),
    "halow_regulator": (PololuD36V28F5.MASS_G, "estimate (Pololu D36V28F5, weight not stated)"),
    "halow_cables": (25.0, "estimate: Ethernet patch cable 0.15 m, USB-C power lead, fuse holder, SMA extension and "
                     "bulkhead"),
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
    add("gps", cad.cyl(HolybroM10GPS.V2[0], HolybroM10GPS.V2[1], (F.MAST_COLUMN_XY[0], F.MAST_COLUMN_XY[1], seat)),
        PURCHASED_MASS["gps"][0])
    for k, (x, y) in enumerate(F.mast_bolts_body()):
        add(f"mast_bolt_{k}", _screw((x, y, F.TOP_Z[1] + 6.0), (0, 0, -1), 12.0, "M3"), PURCHASED_MASS["screw"][0])

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

    # top deck and the HaLowLink 2 on it, its antenna, its 5 V regulator in the hub
    from hardware.parts.aerodrone import halow_cradle as HC
    from hardware.parts.aerodrone import top_deck as TD
    for k, (x, y) in enumerate(F.DECK_POSTS):
        add(f"deck_bolt_{k}", _screw((x, y, F.DECK_Z[1] - TD.CBORE[1]), (0, 0, -1), 50.0, "M3"), PURCHASED_MASS["screw"][0])
    zc = F.DECK_Z[1] + F.HALOW_BASE_T - HC.CBORE[1]
    for k, (x, y) in enumerate(F.HALOW_CRADLE_SCREWS):
        add(f"halow_screw_{k}", _screw((x, y, zc), (0, 0, -1), 5.0, "M3"), PURCHASED_MASS["screw"][0])
    ux0, ux1, uy0, uy1, uz0, uz1 = HC.unit_box()
    add("halow_unit", cad.box(ux0, ux1, uy0, uy1, uz0, uz1), PURCHASED_MASS["halow"][0])
    ax, ay, az0, az1 = HC.antenna_axis()
    add("halow_antenna", cad.cyl(HaLowLink2.ANTENNA[1], az1 - az0, (ax, ay, az0)), PURCHASED_MASS["halow_antenna"][0])
    rx0, rx1, ry0, ry1 = F.HALOW_REGULATOR_XY
    add("halow_regulator", cad.box(rx0, rx1, ry0, ry1, F.BOT_Z[1] + 1.0, F.BOT_Z[1] + 1.0 + PololuD36V28F5.SIZE[2]),
        PURCHASED_MASS["halow_regulator"][0])
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
        halow_cradle,
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
        top_deck,
    )
    from hardware.parts.common import cad
    out = []
    for mod in (hub_bottom_plate, hub_top_plate, stack_tray, gps_mast, sensor_nose, oak1_stage, oak1_cradle, battery_tray,
                top_deck, halow_cradle):
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
    for key in ("wiring_rc_telemetry", "halow_cables"):
        masses.append(PURCHASED_MASS[key][0])
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
    base_ok = not overlaps and not in_view and not blocked and not prop_hits and tray_ok
    lr = launcher_check(entries, base_mass_g=auw)
    report.update(lr)
    report["passed"] = bool(base_ok and lr["launcher_passed"])
    return report


# --------------------------------------------------------------------- foam dart launcher configuration

REPO = Path(__file__).resolve().parents[3]
TURRET_MANIFEST = REPO / "hardware" / "parts" / "turret" / "stl" / "manifest.json"
LAUNCHER_STATES = {"cocked": {"u": 0.0, "cocked": True}, "primed": {"u": None, "cocked": False}}
CLIP_FULL = 5                      # five darts in the clip and one chambered: six, a full clip
CG_TOLERANCE_MM = 3.0              # CG (and the bore) within this radius of the hub centre
MIN_LAUNCHER_GAP_MM = 3.0          # launcher (any state) to the airframe, except its own mounts
DART_PATH_D = 14.0
COMPASS_MIN_MM = 150.0
# Removed for the launcher configuration (the hanger takes the tray's inserts,
# the tall legs the landing legs').
NOT_WITH_LAUNCHER = ("battery_tray", "landing_leg", "leg_foot", "tray_bolt")
LAUNCHER_MOUNT = ("launcher_hanger", "launcher_cam_guide", "cam_guide_bolt", "keel_bolt", "hanger_bolt")
PAYLOAD_MASS_G = {
    "servo": (55.0, "Waveshare ST3215 (hardware/parts/turret/_footprints.py)"),
    "launcher_hardware": (14.0, "turret assembly check: screws, pins, O-rings of the launcher"),
    "servo_adapter": (15.0, "estimate: Waveshare Bus Servo Adapter (A), 42 x 33 mm board"),
    "regulator": (8.0, "estimate: Pololu D36V50F12, 25 x 25 mm board (weight not stated)"),
    "launcher_wiring": (25.0, "estimate: 12 V leads (20 AWG), fuse holder, servo bus and USB cables"),
}
# Motor and propeller test data: T-Motor AIR2216 II KV920 with the T1045
# propeller at 16 V (Air Gear 450 II datasheet,
# https://cdn.robotshop.com/media/T/Tmo/RB-Tmo-267/pdf/ligpower_airgear_450ii_combo_set_multi_rotor_uav_p_datasheet.pdf):
# (throttle %, thrust g, current A). Holybro publishes no thrust table for its
# 2216 KV920 (the BOM's motor, the same 2216/920 KV class); its X500 V2 page
# gives "Payload 1500 g (without battery, 70% throttle)".
THRUST_TABLE = ((30, 210, 1.44), (35, 259, 1.87), (40, 309, 2.29), (45, 373, 2.86), (50, 447, 3.60),
                (55, 536, 4.53), (60, 628, 5.61), (65, 729, 6.78), (70, 814, 7.92), (75, 906, 9.20),
                (80, 993, 10.59), (85, 1087, 12.11), (90, 1191, 13.81), (95, 1289, 15.68), (100, 1332, 16.37))
PROP_THRUST_LIMIT_G = 1200.0       # T1045 "thrust limitation 1.2 kg" (same datasheet)
TABLE_V = 16.0
PACK_MAH = 5000.0
PACK_USABLE = 0.8
AVIONICS_A = 1.1                   # 5 V rail 2.8 A typical (14 W) from the pack at ~15 V and 90 %, launcher servos idle


def turret_masses():
    """Estimated masses (g) of the turret's printed parts (its committed manifest)."""
    if not TURRET_MANIFEST.exists():
        return {}
    return {e["id"]: e.get("estimate", {}).get("mass_g", 0.0) for e in json.loads(TURRET_MANIFEST.read_text())["parts"]}


def launcher_models(state="cocked", darts=CLIP_FULL, chambered=True):
    """The launcher core in the body frame, pointing down: [(name, turret
    part id or None, shape)]. The aerodrone's breech (launcher_breech_down)
    replaces the turret's; the barrel sits DETENT_T further forward, on the
    detent ring."""
    from hardware.parts.common import cad
    from hardware.parts.turret import _launcher as L
    from hardware.parts.turret import assembly as turret
    st = dict(LAUNCHER_STATES[state])
    if st["u"] is None:
        st["u"] = L.ACT_PRIME
    out = []
    for n, stem, s in L.models(darts_in_clip=darts, chambered=chambered, st3215=turret.st3215, screw=_screw, **st):
        if n == "launcher_breech":
            continue
        if n == "launcher_barrel":
            s = cad.translate(s, (F.DETENT_T, 0.0, 0.0))
        out.append((n, stem, L.place(s, F.LAUNCHER_ORIGIN, F.LAUNCHER_ROT)))
    return out


def launcher_printed():
    """[(name, part id, shape)] of the aerodrone's launcher-configuration parts in place."""
    from hardware.parts.aerodrone import (
        launcher_breech_down,
        launcher_cam_guide,
        launcher_dart_detent,
        launcher_hanger,
        tall_leg_foot,
        tall_leg_socket,
    )
    out = []
    for mod in (launcher_breech_down, launcher_dart_detent, launcher_cam_guide, launcher_hanger):
        stem = mod.__name__.rsplit(".", 1)[1]
        out.append((stem, stem, mod.build()))
    sock, foot = tall_leg_socket.build(), tall_leg_foot.build()
    for i, a in enumerate(F.ARM_ANGLES):
        out.append((f"tall_leg_socket_{i}", "tall_leg_socket", _arm_place(sock, a)))
        out.append((f"tall_leg_foot_{i}", "tall_leg_foot", _arm_place(foot, a)))
    return out


def deck_battery(x_center, y_center=0.0):
    """The pack across the aircraft on the battery deck."""
    from hardware.parts.common import cad
    sx, sy, sz = LipoEnvelope4S.NOMINAL_SIZE
    z0 = F.DECK_Z[1]
    return cad.box(x_center - sy / 2, x_center + sy / 2, y_center - sx / 2, y_center + sx / 2, z0, z0 + sz)


def launcher_purchased():
    """[(name, shape, mass_g)] of the purchased parts the launcher
    configuration adds (not the launcher core, not the battery)."""
    from hardware.parts.aerodrone import launcher_cam_guide as CG
    from hardware.parts.aerodrone import tall_leg_socket as TS
    from hardware.parts.common import cad
    from hardware.parts.turret._footprints import WaveshareBusServoAdapterA as A
    out = []
    scr = PURCHASED_MASS["screw"][0]
    z_f = F.BOT_Z[0] - F.HANGER_FLANGE_T
    for k, (x, y) in enumerate(F.BATTERY_TRAY_INSERTS):
        out.append((f"hanger_bolt_{k}", _screw((x, y, z_f), (0, 0, 1), 10.0, "M3"), scr))
    for k, (xl, yl) in enumerate(CG.keel_screws_local()):
        _, y, z = F.launcher_to_body((xl, yl, 0.0))
        out.append((f"keel_bolt_{k}", _screw((F.KEEL_X[1], y, z), (-1, 0, 0), 8.0, "M3"), scr))
    for k, (xl, yl) in enumerate(F.BREECH_UNDER_SCREWS):
        p = F.launcher_to_body((xl, yl, CG.PAD_Z[0] + CG.SCREW_CBORE[1]))
        out.append((f"cam_guide_bolt_{k}", _screw(p, (-1, 0, 0), 10.0, "M3"), scr))
    od, idd = F.TALL_LEG_TUBE
    length = TS.tube_length()
    tube = TS.along(cad.cyl(od, length, (0, 0, TS.STOP_S)).cut(cad.cyl(idd, length + 2, (0, 0, TS.STOP_S - 1))))
    tube_mass = math.pi / 4 * (od ** 2 - idd ** 2) * length / 1000 * CARBON_DENSITY
    pinch = [TS.along(_screw((n, -TS.SLAB_HALF_T, TS.PINCH_S), (0, 1, 0), 20.0, "M3")) for n in (TS.PINCH_N, -TS.PINCH_N)]
    for i, a in enumerate(F.ARM_ANGLES):
        out.append((f"leg_tube_{i}", _arm_place(tube, a), tube_mass))
        for k, s in enumerate(pinch):
            out.append((f"leg_pinch_{i}_{k}", _arm_place(s, a), scr + 0.5))
    z = F.BOT_Z[1]
    x0, x1, y0, y1 = F.SERVO_ADAPTER_XY
    out.append(("servo_adapter", cad.box(x0, x1, y0, y1, z + 1.0, z + 1.0 + 1.6 + A.TOP_PARTS_H),
                PAYLOAD_MASS_G["servo_adapter"][0]))
    x0, x1, y0, y1 = F.REGULATOR_XY
    out.append(("regulator_12v", cad.box(x0, x1, y0, y1, z + 1.0, z + 10.5), PAYLOAD_MASS_G["regulator"][0]))
    return out


def _launcher_mass_items(entries, models, darts_on=True):
    """[(name, mass g, shape)] of the launcher core and its mounts' masses."""
    tm = turret_masses()
    m = {stem: e.get("estimate", {}).get("mass_g", 0.0) for stem, e in (entries or {}).items()}
    items = []
    for n, stem, s in models:
        if stem and stem not in ("breech", "muzzle_cap"):
            items.append((n, tm.get(f"launcher_{stem}", 0.0), s))
        elif n in ("act_servo_case", "trig_servo_case"):
            items.append((n, PAYLOAD_MASS_G["servo"][0], s))
        elif n == "launcher_spring":
            from hardware.parts.turret import _launcher as L
            items.append((n, L.SPRING["mass_g"], s))
        elif n == "launcher_barrel":
            from hardware.parts.turret import _launcher as L
            items.append((n, L.BARREL["mass_g"], s))
        elif n.startswith("launcher_dart") and darts_on:
            from hardware.parts.turret import _launcher as L
            items.append((n, L.DART_MASS_G, s))
    return items, m


def launcher_configuration(entries, state="cocked", darts=CLIP_FULL, chambered=True, battery_xy=None, base=None):
    """Bodies and masses of the launcher configuration. Returns a dict:
    'bodies' [(name, shape)], 'new' (names the configuration adds),
    'items' [(name, mass g, centroid)], 'battery_xy', 'models'. ``base``:
    (printed(), _purchased(), launcher_printed(), launcher_purchased()) if
    already built."""
    entries = entries or {}
    base = base or (printed(), _purchased(), launcher_printed(), launcher_purchased())
    m = {stem: e.get("estimate", {}).get("mass_g", 0.0) for stem, e in entries.items()}
    bodies, items, new = [], [], set()

    def keep(n):
        return not n.startswith(NOT_WITH_LAUNCHER)

    for name, stem, shape in base[0]:
        if keep(name):
            bodies.append((name, shape))
            items.append((name, m.get(stem, 0.0), shape.Center().toTuple()))
    for name, shape, mass in base[1]:
        if keep(name):
            bodies.append((name, shape))
            if mass:
                items.append((name, mass, shape.Center().toTuple()))
    for key in ("wiring_rc_telemetry", "halow_cables"):
        items.append((key, PURCHASED_MASS[key][0], (0.0, 0.0, 0.0)))
    for name, stem, shape in base[2]:
        bodies.append((name, shape))
        new.add(name)
        items.append((name, m.get(stem, 0.0), shape.Center().toTuple()))
    for name, shape, mass in base[3]:
        bodies.append((name, shape))
        new.add(name)
        items.append((name, mass, shape.Center().toTuple()))
    items.append(("launcher_wiring", PAYLOAD_MASS_G["launcher_wiring"][0], (0.0, 0.0, F.BOT_Z[1])))
    models = launcher_models(state, darts, chambered)
    for name, stem, shape in models:
        bodies.append((name, shape))
        new.add(name)
    core, _ = _launcher_mass_items(entries, models)
    items += [(n, w, s.Center().toTuple()) for n, w, s in core]
    breech = next(s for n, s in bodies if n == "launcher_breech_down")
    items.append(("launcher_hardware", PAYLOAD_MASS_G["launcher_hardware"][0], breech.Center().toTuple()))
    if battery_xy is None:
        mx = sum(w * c[0] for _, w, c in items)
        my = sum(w * c[1] for _, w, c in items)
        battery_xy = (-mx / LipoEnvelope4S.MASS_G, -my / LipoEnvelope4S.MASS_G)
    pack = deck_battery(*battery_xy)
    bodies.append(("battery", pack))
    new.add("battery")
    items.append(("battery", LipoEnvelope4S.MASS_G, pack.Center().toTuple()))
    return {"bodies": bodies, "new": new, "items": items, "battery_xy": battery_xy, "models": models}


def centre_of_mass(items):
    total = sum(w for _, w, _ in items)
    return total, [sum(w * c[k] for _, w, c in items) / total for k in range(3)]


def _gap(b1, b2):
    return max(b1.xmin - b2.xmax, b2.xmin - b1.xmax, b1.ymin - b2.ymax, b2.ymin - b1.ymax,
               b1.zmin - b2.zmax, b2.zmin - b1.zmax)


def min_distance(group_a, group_b, limit=60.0):
    """Smallest distance between two lists of (name, shape), with the pair."""
    best = (math.inf, None)
    boxes_b = [(n, s, s.BoundingBox()) for n, s in group_b]
    for n1, s1 in group_a:
        b1 = s1.BoundingBox()
        for n2, s2, b2 in boxes_b:
            g = _gap(b1, b2)
            if g > min(best[0], limit):
                continue
            d = s1.distance(s2)
            if d < best[0]:
                best = (d, f"{n1} / {n2}")
    return best


def halow_clearances(bodies):
    """The HaLow antenna's distance to the arms, the propeller discs, the
    GPS mast and puck and the launcher, as text."""
    from hardware.parts.common import cad
    ant = [(n, s) for n, s in bodies if n == "halow_antenna"]
    groups = (("arm tubes", ("arm_tube",)), ("GPS mast and puck", ("gps",)),
              ("launcher", ("launcher_", "act_servo", "trig_servo")), ("battery", ("battery",)))
    out = []
    for label, prefixes in groups:
        d, _ = min_distance(ant, [(n, s) for n, s in bodies if n.startswith(prefixes)], limit=1e6)
        out.append(f"{label} {d:.0f}")
    discs = [(f"disc_{i}", _arm_place(cad.cyl(Propeller1045.DIAMETER, 12.0, (0, 0, F.PROP_Z - 3.0)), a, F.MOTOR_R))
             for i, a in enumerate(F.ARM_ANGLES)]
    d, _ = min_distance(ant, discs, limit=1e6)
    out.append(f"propeller discs {d:.0f}")
    return "vertical, from the HaLowLink's SMA on the top deck: " + ", ".join(out)


def interpolate_hover(thrust_g):
    """(throttle %, current A) per motor at 16 V for this thrust, from THRUST_TABLE."""
    pts = THRUST_TABLE
    for (t0, g0, a0), (t1, g1, a1) in zip(pts, pts[1:]):
        if g0 <= thrust_g <= g1:
            f = (thrust_g - g0) / (g1 - g0)
            return t0 + f * (t1 - t0), a0 + f * (a1 - a0)
    return (None, None)


def flight_numbers(mass_g):
    """Thrust-to-weight, hover throttle and current, flight time (16 V table)."""
    per_motor = mass_g / 4
    thr, amps = interpolate_hover(per_motor)
    tw_table = 4 * THRUST_TABLE[-1][1] / mass_g
    tw_prop = 4 * PROP_THRUST_LIMIT_G / mass_g
    total_a = 4 * amps + AVIONICS_A
    minutes = PACK_MAH / 1000 * PACK_USABLE / total_a * 60
    return {"tw_table": tw_table, "tw_prop": tw_prop, "hover_pct": thr, "hover_a": total_a, "minutes": minutes,
            "hover_pct_14v8": thr * TABLE_V / 14.8}


def launcher_check(entries, base_mass_g=None):
    """The launcher configuration's checks: a report dict and a pass flag."""
    from hardware.parts.common import cad
    from hardware.parts.turret import _launcher as L
    report = {}
    entries = entries or {}
    base = (printed(), _purchased(), launcher_printed(), launcher_purchased())
    full = launcher_configuration(entries, base=base)
    bx, by = full["battery_xy"]
    mass_full, cg_full = centre_of_mass(full["items"])
    mass_empty, cg_empty = centre_of_mass([i for i in full["items"] if not i[0].startswith("launcher_dart")])
    core_items, _ = _launcher_mass_items(entries, full["models"])
    breech_g = entries.get("launcher_breech_down", {}).get("estimate", {}).get("mass_g", 0.0)
    core_stems = {n: stem for n, stem, _ in full["models"]}
    printed_g = sum(w for n, w, _ in core_items if core_stems.get(n)) + breech_g
    core = sum(w for _, w, _ in core_items) + PAYLOAD_MASS_G["launcher_hardware"][0] + breech_g

    # battery on the deck: between the strap slots and clear of the GPS mast
    from hardware.parts.aerodrone import top_deck as BD
    half = LipoEnvelope4S.NOMINAL_SIZE[1] / 2
    mast_front = F.MAST_COLUMN_XY[0] + 9.0     # the mast column (18 mm)
    lo = max(BD.STRAP_X[0] + BD.STRAP_SLOT[0] / 2, mast_front + 2.0) + half
    hi = BD.STRAP_X[1] - BD.STRAP_SLOT[0] / 2 - half
    on_deck = lo <= bx <= hi and abs(by) <= 10.0
    report["launcher_battery_on_deck_mm"] = (f"pack centre at x {bx:.1f}, y {by:.1f} for a centred CG with a full clip "
                                             f"(the straps and the GPS mast allow x {lo:.1f}..{hi:.1f})"
                                             + ("" if on_deck else ": OUT OF RANGE"))
    off_full = math.hypot(cg_full[0], cg_full[1])
    off_empty = math.hypot(cg_empty[0], cg_empty[1])
    report["launcher_cg_mm"] = (f"full clip ({CLIP_FULL + 1} darts): ({cg_full[0]:.1f}, {cg_full[1]:.1f}, {cg_full[2]:.1f}); "
                                f"empty clip: ({cg_empty[0]:.1f}, {cg_empty[1]:.1f}, {cg_empty[2]:.1f}), battery not moved; "
                                f"tolerance {CG_TOLERANCE_MM:g} mm from the hub centre")
    report["launcher_bore_to_cg_mm"] = (f"{off_full:.2f} (full clip), {off_empty:.2f} (empty): the bore is the vertical "
                                        "line through the hub centre")
    cg_ok = on_deck and off_full <= CG_TOLERANCE_MM and off_empty <= CG_TOLERANCE_MM

    # interference: everything the configuration adds against everything,
    # except the launcher core against itself (the turret's check covers it)
    core_names = {n for n, _, _ in full["models"]}
    overlaps, allowed_n = [], 0
    states = {"cocked": full}
    states["primed"] = launcher_configuration(entries, state="primed", battery_xy=(bx, by), base=base)
    for label, conf in states.items():
        bodies = conf["bodies"]
        boxes = [(n, s, s.BoundingBox()) for n, s in bodies]
        for i, (n1, s1, b1) in enumerate(boxes):
            for n2, s2, b2 in boxes[i + 1:]:
                if n1 not in conf["new"] and n2 not in conf["new"]:
                    continue
                if n1 in core_names and n2 in core_names:
                    continue
                if not _bbox_overlap(b1, b2):
                    continue
                v = s1.intersect(s2).Volume()
                if v > OVERLAP_TOLERANCE_MM3:
                    if _allowed(n1, n2):
                        allowed_n += 1
                    else:
                        overlaps.append(f"{label}: {n1} / {n2}: {v:.1f} mm3")
    report["launcher_bodies"] = len(full["bodies"])
    report["launcher_interferences"] = overlaps or "none (cocked with a full clip, primed)"

    # camera view, rangefinder beam, propeller discs, against what the configuration adds
    lens = lens_point()
    fov = _frustum(lens, LuxonisOAK1.VFOV_DEG / 2 + FOV_MARGIN_DEG, LuxonisOAK1.HFOV_DEG / 2 + FOV_MARGIN_DEG, 1500.0)
    rx = F.RANGEFINDER_X
    beam = cad.cone(20.0, 20.0 + 2 * 1000 * math.tan(math.radians(2.0)), 1000.0,
                    (rx, 0, F.NOSE_Z[0] - BenewakeTFminiS.HEIGHT - 0.5), (0, 0, -1))
    discs = [(i, _arm_place(cad.cyl(Propeller1045.DIAMETER + 10.0, 12.0, (0, 0, F.PROP_Z - 3.0)), a, F.MOTOR_R))
             for i, a in enumerate(F.ARM_ANGLES)]
    in_view, blocked, prop_hits = [], [], []
    for label, conf in states.items():
        new = [(n, s) for n, s in conf["bodies"] if n in conf["new"]]
        for n, s in new:
            b = s.BoundingBox()
            if _bbox_overlap(fov.BoundingBox(), b) and s.intersect(fov).Volume() > OVERLAP_TOLERANCE_MM3:
                in_view.append(f"{label}: {n}")
            if _bbox_overlap(beam.BoundingBox(), b) and s.intersect(beam).Volume() > OVERLAP_TOLERANCE_MM3:
                blocked.append(f"{label}: {n}")
            for i, d in discs:
                if _bbox_overlap(d.BoundingBox(), b) and s.intersect(d).Volume() > OVERLAP_TOLERANCE_MM3:
                    prop_hits.append(f"prop {i}: {n}")
    report["launcher_camera_view_clear"] = in_view or (
        f"yes: launcher, hanger, tall legs and deck out of the OAK-1's 69 x 55 deg view + {FOV_MARGIN_DEG:g} deg, "
        "cocked and primed")
    report["launcher_rangefinder_beam_clear"] = blocked or "yes"
    report["halow_antenna_clearance_mm"] = halow_clearances(full["bodies"])
    report["launcher_propeller_clearance"] = prop_hits or "yes"

    # the muzzle's view margin: how far the barrel's end is behind the view's rear edge
    muzzle_z = F.launcher_muzzle_z()
    depth = lens[2] - muzzle_z
    edge = lens[0] - depth * math.tan(math.radians(LuxonisOAK1.VFOV_DEG / 2 + FOV_MARGIN_DEG))
    report["launcher_view_margin_mm"] = (f"the view's rear edge (with the margin) passes {edge - L.BARREL['od'] / 2:.0f} mm "
                                         f"in front of the barrel at the muzzle ({depth:.0f} mm below the lens)")

    # clearances: the launcher (cocked and primed) to the airframe, except its own mounts
    gaps = []
    for label, conf in states.items():
        lau = [(n, s) for n, s in conf["bodies"] if n in core_names or n in ("launcher_breech_down", "launcher_dart_detent")]
        rest = [(n, s) for n, s in conf["bodies"] if not (n in core_names or n.startswith(LAUNCHER_MOUNT)
                                                         or n in ("launcher_breech_down", "launcher_dart_detent"))]
        d, pair = min_distance(lau, rest)
        gaps.append((d, pair, label))
    d, pair, label = min(gaps, key=lambda g: g[0])
    report["launcher_clearance_mm"] = (f"{d:.1f} at the closest ({pair}, {label}): the launcher clears the airframe, legs, "
                                       "deck and sensors in both states")
    gap_ok = d >= MIN_LAUNCHER_GAP_MM
    hanger = [(n, s) for n, s in full["bodies"] if n.startswith(("launcher_hanger", "launcher_cam_guide"))]
    others = [(n, s) for n, s in full["bodies"] if not (n in core_names or n.startswith(LAUNCHER_MOUNT)
                                                        or n in ("launcher_breech_down", "launcher_dart_detent")
                                                        or n.startswith(("hub_bottom_plate",)))]
    dh, ph = min_distance(hanger, others)
    report["launcher_mount_clearance_mm"] = f"{dh:.1f} from the hanger and cam guide to the rest ({ph})"
    gap_ok = gap_ok and dh >= 1.0

    # a dart's path from the muzzle to the ground
    ground = F.launcher_ground_z()
    path = cad.cyl(DART_PATH_D, muzzle_z - ground - 1.0, (0.0, 0.0, muzzle_z - 1.0), (0, 0, -1))
    hits = [n for n, s in full["bodies"] if _bbox_overlap(path.BoundingBox(), s.BoundingBox())
            and s.intersect(path).Volume() > OVERLAP_TOLERANCE_MM3]
    report["launcher_dart_path"] = hits or (f"clear: a {DART_PATH_D:g} mm path from the muzzle straight down to the "
                                            "ground meets nothing")

    # landing: the lowest point that is not a foot, and the tip-over angle
    low = min(((s.BoundingBox().zmin, n) for n, s in full["bodies"] if not n.startswith(("tall_leg_foot", "leg_tube"))))
    feet_r = F.TALL_LEG_FOOT_R / math.sqrt(2)
    tip = math.degrees(math.atan2(feet_r, cg_full[2] - ground))
    report["launcher_ground_clearance_mm"] = (f"muzzle {muzzle_z - ground:.0f} above the ground on the feet (lowest point "
                                              f"{low[1]} at {low[0] - ground:.0f}); legs {F.BOT_Z[0] - ground:.0f} mm under "
                                              f"the hub, feet at (+-{feet_r:.0f}, +-{feet_r:.0f}); tip-over {tip:.0f} deg "
                                              f"(CG {cg_full[2] - ground:.0f} mm above the ground)")
    land_ok = low[0] - ground >= F.LAUNCHER_GROUND_CLEARANCE - 0.5 and not hits

    # compass: the GPS puck against the battery on the deck
    gps = (F.MAST_COLUMN_XY[0], F.MAST_COLUMN_XY[1], F.TOP_Z[1] + F.MAST_HEIGHT + HolybroM10GPS.V2[1] / 2)
    pack_c = next(c for n, _, c in full["items"] if n == "battery")
    dist = math.dist(gps, pack_c)
    report["launcher_compass_to_battery_mm"] = (f"{dist:.0f} from the GPS/compass to the battery's centre (at least "
                                                f"{COMPASS_MIN_MM:g}; run ArduPilot's compass-motor calibration)")
    compass_ok = dist >= COMPASS_MIN_MM

    # masses and flight impact
    report["launcher_mass_g"] = (f"launcher {core:.0f} with a full clip (printed {printed_g:.0f}, two servos "
                                 f"{2 * PAYLOAD_MASS_G['servo'][0]:g}, spring, barrel, hardware, darts); the configuration "
                                 f"adds {mass_full - (base_mass_g or mass_full):+.0f} g in all (hanger, cam guide, deck, "
                                 "tall legs, servo adapter, regulator, wiring; less the tray and landing legs)")
    report["launcher_takeoff_mass_g"] = f"{mass_full:.0f} with a full clip, {mass_empty:.0f} empty"
    lines = []
    for label, mass in (("without the launcher", base_mass_g), ("with the launcher, full clip", mass_full)):
        if not mass:
            continue
        f = flight_numbers(mass)
        lines.append(f"{label} {mass:.0f} g: thrust-to-weight {f['tw_table']:.2f} (table maximum, 1332 g per motor) or "
                     f"{f['tw_prop']:.2f} (T1045 rated 1.2 kg), hover {f['hover_pct']:.0f} % throttle at 16 V "
                     f"(about {f['hover_pct_14v8']:.0f} % at 14.8 V), {f['hover_a']:.1f} A, about {f['minutes']:.1f} min "
                     f"on {PACK_MAH:.0f} mAh to {100 * (1 - PACK_USABLE):.0f} % left")
    report["flight_impact"] = lines
    report["launcher_passed"] = bool(cg_ok and not overlaps and not in_view and not blocked and not prop_hits and gap_ok
                                     and land_ok and compass_ok)
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
                "leg_bolt": (0, 0, -30), "top_deck": (0, 0, 230), "deck_bolt": (0, 0, 260), "halow_cradle": (0, 0, 260),
                "halow_screw": (0, 0, 280), "halow_unit": (0, 0, 300), "halow_antenna": (0, 0, 260)}
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
    _launcher_figures(out_dir, entries, report, mesh)
    return report


def _launcher_figures(out_dir, entries, report, mesh):
    """The launcher configuration's figures."""
    from hardware.parts.common import render
    entries = entries or {}
    tm = json.loads(TURRET_MANIFEST.read_text())["parts"] if TURRET_MANIFEST.exists() else []
    turret_mat = {e["id"]: e.get("material", "PETG") for e in tm}
    mat = {stem: e.get("material", "PETG") for stem, e in entries.items()}
    conf = launcher_configuration(entries)
    stems = {n: stem for n, stem, _ in printed() + launcher_printed()}
    core = {n: stem for n, stem, _ in conf["models"]}

    def color(n):
        if n in stems:
            return render.MATERIAL_COLORS.get(mat.get(stems[n], "PETG"), "#4c9be8")
        if core.get(n):
            return render.MATERIAL_COLORS.get(turret_mat.get(f"launcher_{core[n]}", "PETG"), "#4c9be8")
        return render.PURCHASED_COLOR

    def items(prefixes, offsets=None, skip=("prop_",)):
        out = []
        offsets = offsets or {}
        for n, s in conf["bodies"]:
            if n.startswith(skip) or not any(n.startswith(pr) for pr in prefixes):
                continue
            m = mesh(f"L:{n}", s)
            off = next((v for k, v in offsets.items() if n.startswith(k)), None)
            out.append((m.copy().apply_translation(off) if off else m, color(n)))
        return out

    every = ("",)
    render.render(items(every), out_dir / "launcher_overview.png", elev=12, azim=-55, zoom=1.1,
                  title="Launcher configuration: launcher under the hub, battery on the deck, tall legs")
    launcher = ("launcher_", "act_servo", "trig_servo", "keel_bolt", "cam_guide_bolt", "hanger_bolt")
    render.render(items(launcher, {"launcher_hanger": (110, 0, 0), "keel_bolt": (125, 0, 0), "hanger_bolt": (110, 0, -10),
                                   "launcher_cam_guide": (55, 0, 0), "cam_guide_bolt": (65, 0, 0),
                                   "launcher_dart_detent": (0, 0, -45), "launcher_barrel": (0, 0, -90),
                                   "launcher_clamp": (0, 0, -90), "launcher_pinch": (0, 0, -90)}),
                  out_dir / "step_launcher.png", elev=8, azim=-70, zoom=1.05,
                  title="Launcher pointing down: cam guide, hanger, detent ring (exploded; foam darts only)")
    render.render(items(("hub_top_plate", "stack_", "pi5", "navigator", "top_deck", "deck_bolt", "battery", "gps",
                         "halow"),
                        {"top_deck": (0, 0, 40), "deck_bolt": (0, 0, 70), "battery": (0, 0, 80), "halow_cradle": (0, 0, 70),
                         "halow_screw": (0, 0, 90), "halow_unit": (0, 0, 110), "halow_antenna": (0, 0, 70)},
                        skip=("prop_", "battery_tray")),
                  out_dir / "step_deck.png", elev=28, azim=-50, zoom=1.0,
                  title="Top deck: HaLowLink 2 and its antenna; the battery with the launcher")
    render.render(items(("hub_bottom_plate", "tall_leg", "leg_tube", "leg_pinch", "leg_bolt", "launcher_hanger")),
                  out_dir / "step_tall_legs.png", elev=8, azim=-60, zoom=1.0,
                  title="Tall legs: socket, carbon tube, TPU foot")
