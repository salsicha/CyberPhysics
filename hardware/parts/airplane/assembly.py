"""Airplane assembly: the printed parts placed in the Skywalker X8's bay and
on its skin with simple datasheet-sized models of the purchased parts, and
the checks run by tools/build_printable_parts.py:

  * interference: no two bodies overlap (screws are modelled at their
    nominal diameter through the clearance or insert holes);
  * camera view: nothing, the belly foam included (modelled at its
    thickest design value), enters the OAK-1's 69 x 55 deg view (plus a
    margin) below the lens;
  * GPS separation from the OAK-1's USB 3 cable route and the tail (ESC,
    motor);
  * centre of gravity: where the battery must sit for the payload and the
    propulsion to balance on the X8's CG mark, assuming the bare airframe
    balances there (Skywalker publishes no empty-airframe CG: balance the
    real aircraft), whether that is on the tray, and how far the pack's
    travel moves the CG;
  * payload mass against the X8 listing's 1000-2000 g.

``figures()`` renders the assembly-instruction images.
"""
import math

from hardware.parts.airplane import _frame as F
from hardware.parts.airplane._footprints import (
    BlueRobotics5V6A,
    BlueRoboticsPowerSense,
    HolybroAirspeedMS4525DO,
    Lipo6SEnvelope,
    RpSmaBulkhead,
    SkywalkerX8,
)
from hardware.parts.common import fasteners
from hardware.parts.common.footprints import (
    BlueRoboticsNavigator,
    HolybroM10GPS,
    LuxonisOAK1,
    RaspberryPi5,
    SiliconeDamperBall,
)

OVERLAP_TOLERANCE_MM3 = 0.5
SCREW_SLACK = 0.02
GPS_MIN_SEPARATION_MM = 150.0
AUW_FOR_TRIM_G = sum(SkywalkerX8.FLYING_WEIGHT_G) / 2

# Masses of purchased parts (g), where they sit, and where the figure comes from.
PURCHASED_MASS = {
    "pi5": (RaspberryPi5.MASS_G, "typical (unverified)"),
    "heatsink": (BlueRoboticsNavigator.HEATSINK_MASS_G, "Blue Robotics"),
    "navigator": (BlueRoboticsNavigator.MASS_G, "Blue Robotics"),
    "oak1": (LuxonisOAK1.MASS_G, "Luxonis datasheet"),
    "gps": (HolybroM10GPS.MASS_G, "Holybro"),
    "power_sense": (BlueRoboticsPowerSense.MASS_G, "estimate"),
    "supply_5v": (BlueRobotics5V6A.MASS_G, "estimate"),
    "servo_bec": (20.0, "estimate"),
    "airspeed": (HolybroAirspeedMS4525DO.MASS_G, "Holybro"),
    "pitot": (HolybroAirspeedMS4525DO.PT60_MASS_G, "estimate"),
    "rc_receiver": (10.0, "estimate"),
    "telemetry_radio": (30.0, "estimate"),
    "rp_sma": (15.0, "estimate"),
    "damper": (2.0, "estimate"),
    "battery": (Lipo6SEnvelope.MASS_G, "design envelope"),
    "wiring": (100.0, "estimate: power leads, XT90/XT60, USB 3 cable, servo extensions, tubing"),
    "motor": (250.0, "estimate for a 4120-class 550 KV motor"),
    "esc": (80.0, "estimate for a 60-80 A ESC"),
    "prop": (35.0, "estimate for a 12 x 6 in folding prop"),
    "servos": (2 * SkywalkerX8.SERVO_CLASS_G, "X8 listing: 2 x 17 g"),
    "screw": (1.0, "estimate per M3 screw"),
}
# Point masses without geometry (the tail and the wings): (x, y, z) mm.
POINT_MASSES = {
    "motor": (F.MOTOR_X + 20.0, 0.0, 30.0),
    "prop": (F.MOTOR_X, 0.0, 30.0),
    "esc": (F.MOTOR_X + 60.0, 0.0, 15.0),
    "servos": (-150.0, 0.0, 15.0),
    "wiring": (-60.0, 0.0, 15.0),
}
MODULE_HEIGHTS = {"rc_receiver": 12.0, "power_sense": BlueRoboticsPowerSense.SIZE[2], "supply_5v": BlueRobotics5V6A.SIZE[2],
                  "servo_bec": 10.0, "airspeed": HolybroAirspeedMS4525DO.MODULE[2], "telemetry_radio": 12.0}


# --------------------------------------------------------------------- placement

def _screw(axis_point, direction, length, size, slack=SCREW_SLACK):
    """Socket head screw: head outside ``axis_point``, shank along ``direction``."""
    from hardware.parts.common import cad
    d = fasteners.MAJOR[size] - slack
    dx, dy, dz = direction
    x, y, z = axis_point
    hd, hh = fasteners.SHCS_HEAD[size]
    return cad.cyl(d, length, (x, y, z), direction).fuse(
        cad.cyl(hd, hh, (x - dx * hh, y - dy * hh, z - dz * hh), direction))


def belly(thickness=SkywalkerX8.BELLY_T_MAX):
    """The bay floor foam (thickest design value) with the camera cut-out."""
    from hardware.parts.common import cad
    lx, ly, _ = F.lens_point()
    hx, hy = F.cutout_half()
    floor = cad.box(-260.0, 300.0, -SkywalkerX8.BAY_FLOOR_WIDTH / 2 - 10, SkywalkerX8.BAY_FLOOR_WIDTH / 2 + 10,
                    -thickness, 0.0)
    return floor.cut(cad.box(lx - hx, lx + hx, ly - hy, ly + hy, -thickness - 1, 1))


def battery(x_center):
    from hardware.parts.common import cad
    sx, sy, sz = Lipo6SEnvelope.NOMINAL_SIZE
    return cad.box(x_center - sx / 2, x_center + sx / 2, -sy / 2, sy / 2, F.BATTERY_FACE_Z + 1.0,
                   F.BATTERY_FACE_Z + 1.0 + sz)


def _purchased():
    """[(name, shape, mass_g)] for every purchased part with geometry."""
    import cadquery as cq

    from hardware.parts.airplane import antenna_mount, pitot_mount
    from hardware.parts.common import cad
    out = []

    def add(name, shape, mass=0.0):
        out.append((name, shape, mass))

    # Pi 5 + heatsink + Navigator on the stack tray
    px0 = F.PI_CENTER[0] - RaspberryPi5.BOARD[0] / 2
    py0 = F.PI_CENTER[1] - RaspberryPi5.BOARD[1] / 2
    z = F.PI_Z0
    pcb = RaspberryPi5.PCB_THICKNESS
    holes = F.pi_holes_body()
    pi = cad.box(px0, px0 + RaspberryPi5.BOARD[0], py0, py0 + RaspberryPi5.BOARD[1], z, z + pcb)
    for x, y in holes:
        pi = pi.cut(cad.cyl(RaspberryPi5.HOLE_D, pcb + 2, (x, y, z - 1)))
    add("pi5", pi, PURCHASED_MASS["pi5"][0])
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
    add("navigator", cad.box(px0, px0 + BlueRoboticsNavigator.BOARD[0], py0, py0 + BlueRoboticsNavigator.BOARD[1],
                             zn, zn + 1.6), PURCHASED_MASS["navigator"][0])
    add("navigator_connectors", cad.box(px0, px0 + BlueRoboticsNavigator.BOARD[0], py0,
                                        py0 + BlueRoboticsNavigator.BOARD[1], zn + 1.6,
                                        zn + 1.6 + BlueRoboticsNavigator.CONNECTOR_HEIGHT))
    head_h = fasteners.SHCS_HEAD["M2.5"][1]
    for k, (x, y) in enumerate((holes[0], holes[3])):
        add(f"pi_screw_{k}", _screw((x, y, F.TRAY_Z0 + head_h), (0, 0, 1), 12.0, "M2.5"))
    for k, (x, y) in enumerate(F.STACK_GROMMETS):
        top = F.TRAY_Z0 + F.TRAY_T + F.GROMMET_STACK[2]
        add(f"grommet_bolt_{k}", _screw((x, y, top), (0, 0, -1), 12.0, "M3"), PURCHASED_MASS["screw"][0])

    # modules on the deck
    for name, ((cx, cy), (sx, sy), _axis) in F.MODULES.items():
        add(f"module_{name}", cad.cbox(sx, sy, MODULE_HEIGHTS[name],
                                       center=(cx, cy, F.DECK_T + MODULE_HEIGHTS[name] / 2 + 0.05)),
            PURCHASED_MASS[name][0])

    # OAK-1, tripod screw, dampers, deck screws
    cx = F.CAMERA_X
    half = LuxonisOAK1.HEIGHT / 2
    oak = cad.box(cx - half, cx + half, -LuxonisOAK1.WIDTH / 2, LuxonisOAK1.WIDTH / 2, F.LENS_Z, F.CAMERA_BACK_Z)
    tx = F.tripod_x()
    oak = oak.cut(cad.cyl(fasteners.THREAD_1_4_20_MAJOR, LuxonisOAK1.TRIPOD_DEPTH + 0.01,
                          (tx, 0, F.CAMERA_BACK_Z - LuxonisOAK1.TRIPOD_DEPTH)))
    add("oak1", oak, PURCHASED_MASS["oak1"][0])
    top = F.CRADLE_Z[1]
    add("tripod_screw", cad.cyl(fasteners.THREAD_1_4_20_MAJOR - SCREW_SLACK, 12.7, (tx, 0, top), (0, 0, -1)).fuse(
        cad.cyl(fasteners.SHCS_HEAD_1_4_20[0], fasteners.SHCS_HEAD_1_4_20[1], (tx, 0, top))), 5.0)
    zc = (F.FRAME_DECK_Z[0] + F.CRADLE_Z[1]) / 2
    for k, (x, y) in enumerate(F.DAMPERS):
        ball = cq.Solid.makeSphere(SiliconeDamperBall.PLATE_SPACING / 2, cad.V(x, y, zc), angleDegrees1=-90, angleDegrees2=90)
        neck = cad.cyl(SiliconeDamperBall.HOLE_D - 0.1, SiliconeDamperBall.PLATE_SPACING + 2 * F.DAMPER_TAB_T,
                       (x, y, F.CRADLE_Z[1] - F.DAMPER_TAB_T))
        add(f"damper_{k}", ball.fuse(neck), PURCHASED_MASS["damper"][0])
    for k, (x, y) in enumerate(F.pillar_xy()):
        add(f"deck_screw_{k}", _screw((x, y, F.FRAME_DECK_Z[1]), (0, 0, -1), 10.0, "M3"), PURCHASED_MASS["screw"][0])

    # GPS on the wing, pitot on the nose, RP-SMA bulkhead in the antenna mount
    from hardware.parts.airplane import gps_mount
    seat = F.WING_SKIN_Z + gps_mount.HEIGHT
    add("gps", cad.cyl(HolybroM10GPS.V2[0], HolybroM10GPS.V2[1], (F.GPS_XY[0], F.GPS_XY[1], seat)),
        PURCHASED_MASS["gps"][0])
    zf = F.PITOT_Z + pitot_mount.PAD[2] + pitot_mount.FLOOR_T
    base_len = HolybroAirspeedMS4525DO.PT60_OVERALL[0] - HolybroAirspeedMS4525DO.PT60_TUBE_LENGTH
    b = HolybroAirspeedMS4525DO.PT60_OVERALL[1]
    pitot = cad.box(F.PITOT_X - base_len, F.PITOT_X, -b / 2, b / 2, zf, zf + HolybroAirspeedMS4525DO.PT60_OVERALL[2]).fuse(
        cad.cyl(5.0, HolybroAirspeedMS4525DO.PT60_TUBE_LENGTH, (F.PITOT_X, 0, zf + 15.0), (1, 0, 0)))
    add("pitot", pitot, PURCHASED_MASS["pitot"][0])
    ax, ay = F.ANTENNA_XY
    zt = F.TOP_SKIN_Z + antenna_mount.PAD[2] + antenna_mount.BRIDGE_H
    add("rp_sma", cad.cyl(RpSmaBulkhead.THREAD_MAJOR - 0.05, 14.0, (ax, ay, zt - 6.0)).fuse(
        cad.cyl(10.0, 40.0, (ax, ay, zt + antenna_mount.TOP_T + 2.0))), PURCHASED_MASS["rp_sma"][0])
    add("belly_foam", belly())
    return out


def printed():
    """[(name, part id, shape)] for every printed piece in place (the bonded
    cable clips go wherever the harness runs and are not placed)."""
    from hardware.parts.airplane import (
        antenna_mount,
        avionics_deck,
        battery_tray,
        camera_deck,
        camera_frame,
        gps_mount,
        oak1_cradle,
        pitot_mount,
        stack_grommet,
        stack_tray,
    )
    from hardware.parts.common import cad
    out = []
    for mod in (avionics_deck, stack_tray, camera_frame, camera_deck, oak1_cradle, battery_tray, gps_mount,
                pitot_mount, antenna_mount):
        stem = mod.__name__.rsplit(".", 1)[1]
        out.append((stem, stem, mod.build()))
    g = stack_grommet.build()
    for k, (x, y) in enumerate(F.STACK_GROMMETS):
        out.append((f"stack_grommet_lower_{k}", "stack_grommet", cad.translate(g, (x, y, F.BOSS_TOP))))
        upper = cad.rotate(g, "X", 180)
        top = F.TRAY_Z0 + F.TRAY_T + F.GROMMET_STACK[2]
        out.append((f"stack_grommet_upper_{k}", "stack_grommet", cad.translate(upper, (x, y, top))))
    return out


# --------------------------------------------------------------------- checks

def _bbox_overlap(a, b, tol=0.01):
    return not (a.xmax < b.xmin - tol or b.xmax < a.xmin - tol or a.ymax < b.ymin - tol or
                b.ymax < a.ymin - tol or a.zmax < b.zmin - tol or b.zmax < a.zmin - tol)


def _frustum(apex, half_x_deg, half_y_deg, depth, start=0.5):
    """Downward pyramid from ``apex`` (lens centre): image vertical along body x."""
    import cadquery as cq
    x, y, z = apex
    tx, ty = math.tan(math.radians(half_x_deg)), math.tan(math.radians(half_y_deg))
    z0, z1 = z - start, z - depth
    w0 = (2 * (tx * start + F.LENS_APERTURE_R), 2 * (ty * start + F.LENS_APERTURE_R))
    w1 = (2 * (tx * depth + F.LENS_APERTURE_R), 2 * (ty * depth + F.LENS_APERTURE_R))
    return (cq.Workplane("XY", origin=(x, y, z0)).rect(*w0)
            .workplane(offset=z1 - z0).rect(*w1).loft(combine=True).val())


def _segment_distance(p, a, b):
    import numpy as np
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * (b - a))))


def battery_position(masses, centroids):
    """Battery centre x that puts the CG on the X8's CG mark (x = 0)."""
    mx = sum(mi * c[0] for mi, c in zip(masses, centroids))
    return -mx / Lipo6SEnvelope.MASS_G


def check(entries=None):
    """Run the assembly checks; returns a report dict ('passed' is the
    overall result)."""
    entries = entries or {}
    report = {}
    bodies, masses, centroids = [], [], []
    printed_mass = 0.0
    for name, stem, shape in printed():
        bodies.append((name, shape))
        mass = entries.get(stem, {}).get("estimate", {}).get("mass_g", 0.0)
        printed_mass += mass
        masses.append(mass)
        centroids.append(shape.Center().toTuple())
    for name, shape, mass in _purchased():
        bodies.append((name, shape))
        if mass:
            masses.append(mass)
            centroids.append(shape.Center().toTuple())
    for name, xyz in POINT_MASSES.items():
        masses.append(PURCHASED_MASS[name][0])
        centroids.append(xyz)
    clip = entries.get("cable_clip", {})
    if clip:            # bonded along the harness, not placed: count them with the wiring
        printed_mass += clip["estimate"]["mass_g_all"]
        masses.append(clip["estimate"]["mass_g_all"])
        centroids.append(POINT_MASSES["wiring"])
    x_batt = battery_position(masses, centroids)
    bodies.append(("battery", battery(x_batt)))
    half_len = Lipo6SEnvelope.NOMINAL_SIZE[0] / 2
    tray_ok = F.BATTERY_TRAY_X[0] + half_len <= x_batt <= F.BATTERY_TRAY_X[1] - 4.0 - half_len
    travel = (F.BATTERY_TRAY_X[1] - 4.0 - F.BATTERY_TRAY_X[0]) - Lipo6SEnvelope.NOMINAL_SIZE[0]
    payload = (sum(masses) - sum(PURCHASED_MASS[k][0] for k in ("motor", "prop", "esc", "servos"))
               + Lipo6SEnvelope.MASS_G)
    report["battery_centre_ahead_of_cg_mark_mm"] = round(x_batt, 1)
    report["battery_on_tray"] = tray_ok
    report["cg_trim_authority_mm"] = (f"+-{Lipo6SEnvelope.MASS_G * travel / 2 / AUW_FOR_TRIM_G:.0f} mm of CG for the "
                                      f"pack's {travel:.0f} mm travel at {AUW_FOR_TRIM_G:.0f} g")
    report["printed_mass_g"] = round(printed_mass)
    report["payload_mass_g"] = (f"{payload:.0f} (avionics, printed parts, wiring and the 6S pack; the X8 listing "
                                f"allows {SkywalkerX8.PAYLOAD_G[0]:.0f}-{SkywalkerX8.PAYLOAD_G[1]:.0f})")
    payload_ok = payload <= SkywalkerX8.PAYLOAD_G[1]

    boxes = [(n, s, s.BoundingBox()) for n, s in bodies]
    overlaps = []
    for i in range(len(boxes)):
        n1, s1, b1 = boxes[i]
        for j in range(i + 1, len(boxes)):
            n2, s2, b2 = boxes[j]
            if not _bbox_overlap(b1, b2):
                continue
            v = s1.intersect(s2).Volume()
            if v > OVERLAP_TOLERANCE_MM3:
                overlaps.append(f"{n1} / {n2}: {v:.1f} mm3")
    report["bodies"] = len(bodies)
    report["interferences"] = overlaps or "none"

    lens = F.lens_point()
    fov = _frustum(lens, LuxonisOAK1.VFOV_DEG / 2 + F.FOV_MARGIN_DEG, LuxonisOAK1.HFOV_DEG / 2 + F.FOV_MARGIN_DEG, 1500.0)
    fb = fov.BoundingBox()
    in_view = []
    for n, s, b in boxes:
        if n == "oak1" or not _bbox_overlap(fb, b):
            continue
        v = s.intersect(fov).Volume()
        if v > OVERLAP_TOLERANCE_MM3:
            in_view.append(f"{n} ({v:.0f} mm3)")
    report["camera_view_clear"] = in_view or (
        f"yes (69 x 55 deg + {F.FOV_MARGIN_DEG:g} deg margin, through a {SkywalkerX8.BELLY_T_MAX:g} mm belly)")
    report["lens_recess_mm"] = f"{SkywalkerX8.BELLY_T + F.LENS_Z:g} above the belly skin ({SkywalkerX8.BELLY_T:g} mm belly)"

    from hardware.parts.airplane import gps_mount
    gps = (F.GPS_XY[0], F.GPS_XY[1], F.WING_SKIN_Z + gps_mount.HEIGHT + HolybroM10GPS.V2[1] / 2)
    usb_a = (F.CAMERA_X - LuxonisOAK1.HEIGHT / 2 - 10.0, 0.0, (F.LENS_Z + F.CAMERA_BACK_Z) / 2)
    usb_b = (F.PI_CENTER[0] + RaspberryPi5.BOARD[0] / 2, F.PI_CENTER[1], F.PI_Z0 + 8.0)
    d_usb = _segment_distance(gps, usb_a, usb_b)
    d_tail = min(math.dist(gps, POINT_MASSES[k]) for k in ("motor", "esc"))
    d_batt = _segment_distance(gps, (x_batt - half_len, 0, 30), (F.MODULES["power_sense"][0][0], 32.0, 10.0))
    gps_ok = min(d_usb, d_tail, d_batt) >= GPS_MIN_SEPARATION_MM
    report["gps_separation_mm"] = (f"{d_usb:.0f} from the USB 3 cable, {d_batt:.0f} from the battery leads, "
                                   f"{d_tail:.0f} from the ESC/motor (need {GPS_MIN_SEPARATION_MM:g})")
    stack_top = F.PI_Z0 + RaspberryPi5.PCB_THICKNESS + BlueRoboticsNavigator.HEATSINK[2] + 1.6 + \
        BlueRoboticsNavigator.CONNECTOR_HEIGHT
    report["heights_above_floor_mm"] = (f"stack {stack_top:.0f}, camera deck {F.FRAME_DECK_Z[1]:.0f}, battery "
                                        f"{F.BATTERY_FACE_Z + 1 + Lipo6SEnvelope.NOMINAL_SIZE[2]:.0f} "
                                        "(the bay must clear these with the hatch shut)")
    report["passed"] = not overlaps and not in_view and tray_ok and gps_ok and payload_ok
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
    pieces = printed()
    buy = [(n, s) for n, s, _ in _purchased()]
    buy.append(("battery", battery(report["battery_centre_ahead_of_cg_mark_mm"])))
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

    inside = ("avionics_deck", "stack_", "camera_", "oak1_cradle", "battery_tray")
    inside_buy = ("pi5", "navigator", "pi_screw", "grommet_bolt", "module_", "oak1", "tripod_screw", "damper",
                  "deck_screw", "battery")
    render.render(sel(inside, inside_buy + ("belly_foam",)), out_dir / "overview.png", elev=32, azim=-62, zoom=1.05,
                  title="Airplane: printed parts (colour) and purchased parts (grey) on the X8's bay floor")
    render.render(sel(("avionics_deck", "stack_"), ("pi5", "navigator", "pi_screw", "grommet_bolt", "module_"),
                      {"stack_grommet_upper": (0, 0, 25), "stack_tray": (0, 0, 15), "pi5": (0, 0, 30),
                       "navigator": (0, 0, 30), "pi_screw": (0, 0, 10), "grommet_bolt": (0, 0, 40)}),
                  out_dir / "step_deck.png", elev=30, azim=-50, title="Avionics deck, stack tray and modules")
    # camera: the frame on a patch of the belly, the rest lifted off it
    from hardware.parts.common import cad
    lx, ly, _ = F.lens_point()
    hx, hy = F.cutout_half()
    patch = cad.box(-75.0, 75.0, -70.0, 70.0, -SkywalkerX8.BELLY_T, 0.0).cut(
        cad.box(lx - hx, lx + hx, ly - hy, ly + hy, -SkywalkerX8.BELLY_T - 1, 1))
    buy.append(("belly_patch", patch))
    render.render(sel(("camera_", "oak1_cradle"), ("oak1", "tripod_screw", "damper", "deck_screw", "belly_patch"),
                      {"camera_deck": (0, 0, 110), "deck_screw": (0, 0, 140), "damper": (0, 0, 85),
                       "oak1_cradle": (0, 0, 60), "oak1": (0, 0, 60), "tripod_screw": (0, 0, 75)}),
                  out_dir / "step_camera.png", elev=24, azim=-55, zoom=1.0,
                  title="Camera frame on the belly cut-out; cradle with the OAK-1, dampers and deck above it")
    render.render(sel(("battery_tray",), ("battery",), {"battery": (0, 0, 40)}),
                  out_dir / "step_battery.png", elev=30, azim=-60, title="Battery tray and 6S pack")
    # outside parts brought side by side (they sit far apart on the aircraft)
    side = {"gps": (200.0, -250.0, 40.0), "antenna_mount": (0.0, 0.0, 0.0), "rp_sma": (0.0, 0.0, 0.0),
            "pitot": (-500.0, 0.0, 50.0)}
    side["gps_mount"] = side["gps"]
    side["pitot_mount"] = side["pitot"]
    render.render(sel(("gps_mount", "pitot_mount", "antenna_mount"), ("gps", "pitot", "rp_sma"), side),
                  out_dir / "step_outside.png", elev=28, azim=-60, zoom=0.95,
                  title="Outside: pitot (nose), GPS (left wing), antenna mount (fuselage top), side by side")
    return report
