"""Lunar rover assembly: the printed parts placed on the rover with simple
datasheet-sized models of the purchased parts (2040 structure, AK70-10
actuators, battery, electronics, sensors, solar panel), and the checks run
by tools/build_printable_parts.py:

  * interference: no two bodies overlap (screws are not modelled; the
    actuator sits in its cup and the hub on its output face);
  * the wheel radius at the grouser tips is the simulator's 0.18 m, and
    the wheels clear each other, the legs and the body;
  * ground clearance, centre of gravity and static tip-over angles against
    the simulator's steepest slope (21 deg) with a 10 deg margin;
  * wheel torque and speed: the AK70-10's rated torque on the steepest
    slope in loose regolith, the largest payload it allows, and the wheel
    speed at the simulator's top speed and worst slip;
  * sensor views: the D455's depth view and the OAK-1's view are clear of
    the rover; nothing of the rover rises above the Mid-360's lowest ray;
    the lux sensor's sky above 30 deg elevation is clear except for the
    GNSS antenna beside it.

``figures()`` renders the assembly-instruction images.
"""
import math

from hardware.parts.common.footprints import LuxonisOAK1
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover import _wheel as W
from hardware.parts.lunar_rover._footprints import (
    CubeMarsAK70_10,
    GnssGroundPlate,
    LiFePO4Pack24V,
    LivoxMid360,
    RealSenseD455,
    SolarPanel50W,
)

OVERLAP_TOLERANCE_MM3 = 0.5
SLOPE_MARGIN_DEG = 10.0
ROLLING_RESISTANCE = 0.15          # loose regolith simulant, an assumption (packed sand is ~0.05)
G = 9.81
DESIGN_PAYLOAD_KG = 10.0           # carried on the top deck in the checks
WHEEL_GAP_MIN = 20.0
GROUND_CLEARANCE_MIN = 180.0
SKY_ELEVATION_DEG = 30.0
FOV_MARGIN_DEG = 1.0
BATTERY_PAD = 6.0                  # closed-cell foam under the pack: level with the end blocks' bases
# Pairs (name prefixes) that overlap on purpose.
INTENDED = (
    ("actuator_shroud", "actuator_mount", "TPU cap pressed into the cup"),
)

# Masses of purchased parts (g) and where they come from.
PURCHASED_MASS = {
    "actuator": (CubeMarsAK70_10.MASS_G, "CubeMars"),
    "battery": (LiFePO4Pack24V.MASS_G, "design envelope"),
    "solar_panel": (SolarPanel50W.MASS_G, "design envelope"),
    "d455": (RealSenseD455.MASS_G, "RealSense datasheet"),
    "oak1": (LuxonisOAK1.MASS_G, "Luxonis datasheet"),
    "lidar": (LivoxMid360.MASS_G, "Livox manual"),
    "extrusion_2040_per_m": (800.0, "estimate for a 20 x 40 B-type profile"),
    "extrusion_2020_per_m": (450.0, "estimate for a 20 x 20 B-type profile"),
    "mini_pc": (900.0, "estimate"),
    "din_electronics": (1500.0, "estimate: fuses, contactor, DC-DC converters, switch, safety controller"),
    "panels_acp": (3000.0, "estimate: 3 mm aluminium composite floor, sides and lid"),
    "pivots_rods_brackets": (2000.0, "estimate: shafts, flanged bearings, corner brackets, rod ends, pivot plates"),
    "wiring": (1200.0, "estimate: power and CAN harness, connectors"),
    "gnss": (250.0, "estimate: antenna, steel disc, receiver"),
    "misc_sensors": (300.0, "estimate: IMU, temperature probes, e-stop, radio"),
}


# --------------------------------------------------------------------- models

def _wheel_place(shape, x, side):
    """A shape in the wheel-local frame placed at wheel x on side +1/-1."""
    from hardware.parts.common import cad
    if side < 0:
        shape = cad.rotate_z(shape, 180.0)
    return cad.translate(shape, (x, side * F.OUTPUT_Y, F.AXLE_Z))


def actuator_model():
    """The AK70-10 in the wheel-local frame."""
    from hardware.parts.common import cad
    ak = CubeMarsAK70_10
    boss = cad.cyl(ak.FRONT_BOSS[0], ak.FRONT_BOSS[1], (0, -ak.FRONT_BOSS[1], 0), (0, 1, 0))
    body_len = ak.LENGTH - ak.FRONT_BOSS[1] - ak.REAR_STEP[1]
    body = cad.cyl(ak.DIAMETER, body_len, (0, -ak.FRONT_BOSS[1] - body_len, 0), (0, 1, 0))
    rear = cad.cyl(ak.REAR_STEP[0], ak.REAR_STEP[1], (0, -ak.LENGTH, 0), (0, 1, 0))
    recess = cad.cyl(ak.OUTPUT_SPIGOT[0], ak.OUTPUT_SPIGOT[1] + 0.01, (0, -ak.OUTPUT_SPIGOT[1], 0), (0, 1, 0))
    return cad.fuse_all([boss, body, rear]).cut(recess)


def _bar(x0, x1, y0, y1, z0, z1):
    from hardware.parts.common import cad
    return cad.box(x0, x1, y0, y1, z0, z1)


def _structure():
    """[(name, shape, mass_g)] for the 2040/2020 structure and panels."""
    out = []
    per_mm = PURCHASED_MASS["extrusion_2040_per_m"][0] / 1000.0
    per_mm_2020 = PURCHASED_MASS["extrusion_2020_per_m"][0] / 1000.0
    ly0, ly1 = F.LEG_Y - 10.0, F.LEG_Y + 10.0
    for s, tag in ((1, "left"), (-1, "right")):
        y0, y1 = sorted((s * ly0, s * ly1))
        out.append((f"rocker_beam_{tag}", _bar(*F.ROCKER_BEAM_X, y0, y1, *F.ROCKER_BEAM_Z),
                    (F.ROCKER_BEAM_X[1] - F.ROCKER_BEAM_X[0]) * per_mm))
        out.append((f"bogie_beam_{tag}", _bar(*F.BOGIE_BEAM_X, y0, y1, *F.BOGIE_BEAM_Z),
                    (F.BOGIE_BEAM_X[1] - F.BOGIE_BEAM_X[0]) * per_mm))
        for x, top in zip(F.WHEEL_X, (F.ROCKER_BEAM_Z[0], F.BOGIE_BEAM_Z[0], F.BOGIE_BEAM_Z[0])):
            z0 = F.LEG_SOCKET_Z[0]
            out.append((f"leg_{tag}_{x:+.0f}", _bar(x - 20.0, x + 20.0, y0, y1, z0, top), (top - z0) * per_mm))
        # bogie pivot plate (aluminium, outboard of both beams) and pivot shafts
        py0, py1 = sorted((s * (ly1 + 0.0), s * (ly1 + 4.0)))
        out.append((f"bogie_pivot_plate_{tag}", _bar(F.BOGIE_PIVOT[0] - 20.0, F.BOGIE_PIVOT[0] + 20.0, py0, py1,
                                                      F.BOGIE_BEAM_Z[0], F.ROCKER_BEAM_Z[1]), 60.0))
        sy0, sy1 = sorted((s * F.BODY_HALF_W, s * ly0))
        from hardware.parts.common import cad
        out.append((f"rocker_pivot_shaft_{tag}", cad.cyl(12.0, sy1 - sy0, (F.ROCKER_PIVOT[0], sy0, F.ROCKER_PIVOT[1]),
                                                          (0, 1, 0)), 40.0))
    # body: rails, posts, floor, lid
    bx0, bx1 = F.BODY_X
    w = F.BODY_HALF_W
    rw, rh = F.RAIL
    zb0, zb1 = F.BODY_Z
    for zs, zname in (((zb0, zb0 + rh), "bottom"), ((zb1 - rh, zb1), "top")):
        for s, tag in ((1, "left"), (-1, "right")):
            y0, y1 = sorted((s * (w - rw), s * w))
            out.append((f"body_{zname}_rail_{tag}", _bar(bx0, bx1, y0, y1, *zs), (bx1 - bx0) * per_mm))
        for x0, x1, tag in ((bx1 - rw, bx1, "front"), (bx0, bx0 + rw, "rear")):
            out.append((f"body_{zname}_cross_{tag}", _bar(x0, x1, -w + rw, w - rw, *zs), (2 * (w - rw)) * per_mm))
    for x0, x1 in ((bx1 - rw, bx1), (bx0, bx0 + rw)):
        for s in (1, -1):
            y0, y1 = sorted((s * (w - rw), s * w))
            out.append((f"body_post_{x0:+.0f}_{s:+d}", _bar(x0, x1, y0, y1, zb0 + rh, zb1 - rh), (zb1 - zb0 - 2 * rh) * per_mm))
    out.append(("body_floor", _bar(bx0 + rw, bx1 - rw, -w + rw, w - rw, F.FLOOR_Z, F.FLOOR_Z + F.FLOOR_T),
                PURCHASED_MASS["panels_acp"][0] * 0.4))
    out.append(("body_lid", _bar(bx0 + rw, bx1 - rw, -w + rw, w - rw, zb1 - F.FLOOR_T, zb1),
                PURCHASED_MASS["panels_acp"][0] * 0.3))
    # differential: 2020 bar under the body on a pillow block hung from an extra cross rail, vertical links
    from hardware.parts.common import cad
    bar_half = F.LEG_Y + 12.0
    zb = F.DIFF_BAR_Z
    out.append(("diff_bar", _bar(F.DIFF_LEVER_X - 10.0, F.DIFF_LEVER_X + 10.0, -bar_half, bar_half, zb - 10.0, zb + 10.0),
                2 * bar_half * per_mm_2020))
    out.append(("diff_cross_rail", _bar(*F.DIFF_CROSS_RAIL_X, -w + rw, w - rw, zb0, zb0 + rh), 2 * (w - rw) * per_mm))
    out.append(("diff_pillow_block", _bar(F.DIFF_LEVER_X - 20.0, F.DIFF_LEVER_X + 20.0, -16.0, 16.0, zb + 10.0, zb0), 150.0))
    from hardware.parts.lunar_rover import diff_clevis as DC
    for s in (1, -1):
        z0 = zb + 10.0 + DC.PIN_Z
        z1 = F.ROCKER_BEAM_Z[0] - DC.PIN_Z
        out.append((f"diff_link_{s:+d}", cad.cyl(6.0, z1 - z0 - 24.0, (F.DIFF_LEVER_X, s * F.LEG_Y, z0 + 12.0)), 40.0))
    # masts
    for name, (mx, my), length in (("front_mast", F.MAST_XY, F.MAST_LENGTH), ("rear_mast", F.REAR_MAST_XY, F.REAR_MAST_LENGTH)):
        z0 = zb1 + F.MAST_FOOT_FLOOR
        out.append((name, _bar(mx - 10.0, mx + 10.0, my - 10.0, my + 10.0, z0, z0 + length), length * per_mm_2020))
    return out


def mast_top(xy, length):
    return F.BODY_Z[1] + F.MAST_FOOT_FLOOR + length


def _equipment():
    """[(name, shape, mass_g)] for the battery, electronics and sensors."""
    from hardware.parts.common import cad
    from hardware.parts.lunar_rover import gnss_head as GH
    from hardware.parts.lunar_rover import hazcam_mount as HM
    from hardware.parts.lunar_rover import lidar_mount as LM
    from hardware.parts.lunar_rover import terrain_cam_mount as TM
    out = []
    lx, ly, lz = LiFePO4Pack24V.MAX_SIZE
    z0 = F.FLOOR_Z + F.FLOOR_T
    zb = z0 + BATTERY_PAD
    out.append(("battery", _bar(F.BATTERY_X - lx / 2, F.BATTERY_X + lx / 2, -ly / 2, ly / 2, zb, zb + lz),
                LiFePO4Pack24V.MASS_G))
    tz = z0 + F.TRAY_FEET + 5.0
    cx = sum(F.COMPUTE_TRAY_X) / 2
    out.append(("mini_pc", _bar(cx - 60.0, cx + 60.0, -60.0, 60.0, tz, tz + 50.0), PURCHASED_MASS["mini_pc"][0]))
    px = sum(F.POWER_TRAY_X) / 2
    out.append(("din_electronics", _bar(px - 60.0, px + 60.0, -110.0, 110.0, tz + 7.5, tz + 70.0),
                PURCHASED_MASS["din_electronics"][0]))
    # D455 on the hazard camera mount
    (ox, _, oz), xa, za = HM.camera_frame()
    cam = cad.box(0.0, RealSenseD455.DEPTH, -RealSenseD455.WIDTH / 2, RealSenseD455.WIDTH / 2,
                  -RealSenseD455.HEIGHT / 2, RealSenseD455.HEIGHT / 2)
    out.append(("d455", cad.translate(cad.rotate(cam, "Y", F.HAZCAM_PITCH_DEG), (ox, 0.0, oz)), RealSenseD455.MASS_G))
    # OAK-1 under the front: back against the cradle plate, lens down
    x0 = F.BODY_X[1] + 2.0 + TM.FIT
    zt = F.BODY_Z[0] - TM.PLATE_T
    out.append(("oak1", _bar(x0, x0 + LuxonisOAK1.HEIGHT, -LuxonisOAK1.WIDTH / 2, LuxonisOAK1.WIDTH / 2,
                             zt - LuxonisOAK1.DEPTH, zt), LuxonisOAK1.MASS_G))
    # lidar on its aluminium heat-sink plate
    mt = mast_top(F.MAST_XY, F.MAST_LENGTH)
    zp = mt + 2.0 + LM.PLATE[1]
    mx, my = F.MAST_XY
    out.append(("lidar_heatsink", _bar(mx - 50.0, mx + 50.0, my - 50.0, my + 50.0, zp, zp + LM.SINK[1]), 81.0))
    zl = zp + LM.SINK[1]
    sx, sy, sz = LivoxMid360.SIZE
    out.append(("lidar", _bar(mx - sx / 2, mx + sx / 2, my - sy / 2, my + sy / 2, zl, zl + sz), LivoxMid360.MASS_G))
    # GNSS antenna on its steel disc
    gt = mast_top(F.REAR_MAST_XY, F.REAR_MAST_LENGTH) + 2.0 + GH.DISC_T
    gx, gy = F.REAR_MAST_XY
    d, dt = GnssGroundPlate.DISC
    out.append(("gnss_disc", cad.cyl(d, dt, (gx, gy, gt)), 115.0))
    out.append(("gnss_antenna", cad.cyl(64.0, 22.0, (gx, gy, gt + dt)), 150.0))
    # e-stop head and contact block
    from hardware.parts.lunar_rover import estop_housing as EH
    ex = F.BODY_X[0] - (EH.BOX[0] + EH.WALL) / 2
    out.append(("estop_head", cad.cyl(40.0, 22.0, (ex, 0.0, F.BODY_Z[1])), 60.0))
    out.append(("estop_block", cad.cyl(30.0, 40.0, (ex, 0.0, F.BODY_Z[1] - EH.TOP_T - 40.0)), 60.0))
    # solar panel
    pxs, pys = F.PANEL_SIZE
    out.append(("solar_panel", _bar(F.PANEL_XY[0] - pxs / 2, F.PANEL_XY[0] + pxs / 2, -pys / 2, pys / 2,
                                    F.PANEL_Z, F.PANEL_Z + 30.0), SolarPanel50W.MASS_G))
    return out


def printed():
    """[(name, part id, shape)] for every printed piece in place (cable clips
    and grousers' screws are not modelled)."""
    from hardware.parts.common import cad
    from hardware.parts.lunar_rover import (
        actuator_mount,
        actuator_shroud,
        battery_end,
        compute_tray,
        diff_clevis,
        estop_housing,
        gnss_head,
        grouser,
        hazcam_mount,
        lidar_mount,
        mast_foot,
        panel_clamp,
        power_tray,
        terrain_cam_mount,
        wheel_hub,
        wheel_segment,
    )
    out = []
    hub, seg, gro = wheel_hub.build(), wheel_segment.build(), grouser.build()
    mount, shroud = actuator_mount.build(), actuator_shroud.build()
    g_local = cad.translate(cad.rotate(gro, "Y", 90.0), (F.RIM_OUTER_R, F.RIM_U[0], 0.0))
    for name, x, side in F.wheel_positions():
        out.append((f"wheel_hub_{name}", "wheel_hub", _wheel_place(hub, x, side)))
        out.append((f"actuator_mount_{name}", "actuator_mount", _wheel_place(mount, x, side)))
        out.append((f"actuator_shroud_{name}", "actuator_shroud", _wheel_place(shroud, x, side)))
        for k in range(F.SEGMENTS):
            a = k * F.SEGMENT_DEG
            out.append((f"wheel_segment_{name}_{k}", "wheel_segment", _wheel_place(cad.rotate(seg, "Y", -a), x, side)))
            for j, ga in enumerate(W.grouser_angles()):
                out.append((f"grouser_{name}_{k}_{j}", "grouser",
                            _wheel_place(cad.rotate(g_local, "Y", -(a + ga)), x, side)))
    for m in (compute_tray, power_tray, hazcam_mount, terrain_cam_mount, estop_housing):
        stem = m.__name__.rsplit(".", 1)[1]
        out.append((stem, stem, m.build()))
    be = battery_end.build()
    lx = LiFePO4Pack24V.MAX_SIZE[0] / 2
    z0 = F.FLOOR_Z + F.FLOOR_T
    out.append(("battery_end_front", "battery_end", cad.translate(be, (F.BATTERY_X + lx, 0.0, z0))))
    out.append(("battery_end_rear", "battery_end", cad.translate(cad.rotate_z(be, 180.0), (F.BATTERY_X - lx, 0.0, z0))))
    mf = mast_foot.build()
    for name, (mx, my) in (("mast_foot_front", F.MAST_XY), ("mast_foot_rear", F.REAR_MAST_XY)):
        turned = cad.rotate_z(mf, 90.0 if my > 0 else -90.0)        # apron on the rail's outer face
        out.append((name, "mast_foot", cad.translate(turned, (mx, my, F.BODY_Z[1]))))
    out.append(("lidar_mount", "lidar_mount", cad.translate(lidar_mount.build(), (*F.MAST_XY, mast_top(F.MAST_XY, F.MAST_LENGTH)))))
    out.append(("gnss_head", "gnss_head", cad.translate(gnss_head.build(),
                                                         (*F.REAR_MAST_XY, mast_top(F.REAR_MAST_XY, F.REAR_MAST_LENGTH)))))
    pc = panel_clamp.build()
    for k, (x, y) in enumerate(F.PANEL_CLAMPS):
        shape = pc if y > 0 else cad.rotate_z(pc, 180.0)
        out.append((f"panel_clamp_{k}", "panel_clamp", cad.translate(shape, (x, y, F.BODY_Z[1]))))
    dc = diff_clevis.build()
    for s in (1, -1):
        out.append((f"diff_clevis_rocker_{s:+d}", "diff_clevis",
                    cad.translate(cad.rotate(dc, "X", 180.0), (F.DIFF_LEVER_X, s * F.LEG_Y, F.ROCKER_BEAM_Z[0]))))
        out.append((f"diff_clevis_bar_{s:+d}", "diff_clevis",
                    cad.translate(cad.rotate_z(dc, 90.0), (F.DIFF_LEVER_X, s * F.LEG_Y, F.DIFF_BAR_Z + 10.0))))
    return out


def _purchased():
    out = [(n, s) for n, s, _ in _structure()] + [(n, s) for n, s, _ in _equipment()]
    for name, x, side in F.wheel_positions():
        out.append((f"actuator_{name}", _wheel_place(actuator_model(), x, side)))
    return out


# --------------------------------------------------------------------- checks

def _bbox_overlap(a, b, tol=0.01):
    return not (a.xmax < b.xmin - tol or b.xmax < a.xmin - tol or a.ymax < b.ymin - tol or
                b.ymax < a.ymin - tol or a.zmax < b.zmin - tol or b.zmax < a.zmin - tol)


def _wheel_key(name):
    for w, _, _ in F.wheel_positions():
        if name.endswith(w) or f"_{w}_" in name:
            return w
    return None


def _frustum(apex, x_ax, y_ax, z_ax, hfov, vfov, depth=2000.0, start=1.0):
    import cadquery as cq
    th, tv = math.tan(math.radians(hfov / 2)), math.tan(math.radians(vfov / 2))
    pts = []
    for d in (start, depth):
        for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
            pts.append(cq.Vector(*[apex[k] + d * (x_ax[k] + su * th * y_ax[k] + sv * tv * z_ax[k]) for k in range(3)]))
    return cq.Solid.makeLoft([cq.Wire.makePolygon(pts[:4], close=True), cq.Wire.makePolygon(pts[4:], close=True)], True)


def check(entries=None):
    """Run the assembly checks; returns a report dict ('passed' is the
    overall result)."""
    entries = entries or {}
    report = {}
    pieces = printed()
    bought = _purchased()
    bodies = [(n, s) for n, _, s in pieces] + bought
    boxes = [(n, s, s.BoundingBox()) for n, s in bodies]

    # interference between every pair of bodies whose boxes overlap
    overlaps, allowed = [], []
    for i in range(len(boxes)):
        n1, s1, b1 = boxes[i]
        for j in range(i + 1, len(boxes)):
            n2, s2, b2 = boxes[j]
            if not _bbox_overlap(b1, b2):
                continue
            v = s1.intersect(s2).Volume()
            if v > OVERLAP_TOLERANCE_MM3:
                if any((n1.startswith(a) and n2.startswith(b)) or (n1.startswith(b) and n2.startswith(a))
                       for a, b, _ in INTENDED) and _wheel_key(n1) == _wheel_key(n2):
                    allowed.append(n1)
                else:
                    overlaps.append(f"{n1} / {n2}: {v:.1f} mm3")
    report["bodies"] = len(bodies)
    report["interferences"] = overlaps or "none (screws are not modelled on the rover)"
    report["intended_press_fits"] = len(allowed)

    # wheel radius at the grouser tips, and wheel-to-wheel gaps
    tip = 0.0
    for n, s, b in boxes:
        if n.startswith("grouser_front_left"):
            for v in s.Vertices():
                p = v.toTuple()
                tip = max(tip, math.hypot(p[0] - F.WHEEL_X[0], p[2] - F.AXLE_Z))
    report["wheel_radius_mm"] = round(tip, 1)
    radius_ok = abs(tip - F.SIM_WHEEL_RADIUS) < 0.5
    gaps = [abs(F.WHEEL_X[i] - F.WHEEL_X[i + 1]) - 2 * tip for i in range(2)]
    report["wheel_gaps_mm"] = [round(g, 1) for g in gaps]
    gaps_ok = min(gaps) >= WHEEL_GAP_MIN

    # ground clearance under the body (wheels, mounts, actuators excluded)
    under = [(b.zmin, n) for n, s, b in boxes if _wheel_key(n) is None and not n.startswith(("leg_",))]
    zmin, lowest = min(under)
    report["ground_clearance_mm"] = f"{zmin:.0f} ({lowest})"
    clearance_ok = zmin >= GROUND_CLEARANCE_MIN

    # mass and centre of gravity
    masses, cents = [], []
    for n, stem, s in pieces:
        masses.append(entries.get(stem, {}).get("estimate", {}).get("mass_g", 0.0))
        cents.append(s.Center().toTuple())
    for n, s, m in _structure() + _equipment():
        masses.append(m)
        cents.append(s.Center().toTuple())
    for name, x, side in F.wheel_positions():
        masses.append(CubeMarsAK70_10.MASS_G)
        cents.append((x, side * (F.OUTPUT_Y - 30.0), F.AXLE_Z))
    misc = PURCHASED_MASS["wiring"][0] + PURCHASED_MASS["misc_sensors"][0] + PURCHASED_MASS["pivots_rods_brackets"][0]
    masses.append(misc)
    cents.append((0.0, 0.0, 380.0))
    payload = DESIGN_PAYLOAD_KG * 1000.0
    masses.append(payload)
    cents.append((0.0, 0.0, F.PANEL_Z + 60.0))
    m = sum(masses)
    cg = [sum(mi * c[k] for mi, c in zip(masses, cents)) / m for k in range(3)]
    report["mass_kg"] = f"{(m - payload) / 1000:.1f} + {DESIGN_PAYLOAD_KG:g} payload on the deck"
    report["centre_of_gravity_mm"] = [round(v) for v in cg]
    half_track = F.WHEEL_MID_Y
    lateral = math.degrees(math.atan2(half_track - abs(cg[1]), cg[2]))
    fore = math.degrees(math.atan2(F.WHEEL_X[0] - cg[0], cg[2]))
    aft = math.degrees(math.atan2(cg[0] - F.WHEEL_X[2], cg[2]))
    report["tip_over_deg"] = f"sideways {lateral:.0f}, nose down {fore:.0f}, nose up {aft:.0f}"
    stable = min(lateral, fore, aft) >= F.SIM_MAX_SLOPE_DEG + SLOPE_MARGIN_DEG

    # torque and speed
    def wheel_torque(mass_kg, slope_deg):
        a = math.radians(slope_deg)
        return mass_kg * G * (math.sin(a) + ROLLING_RESISTANCE * math.cos(a)) * F.SIM_WHEEL_RADIUS / 1000.0 / 6.0
    t_req = wheel_torque(m / 1000.0, F.SIM_MAX_SLOPE_DEG)
    rated = CubeMarsAK70_10.RATED_TORQUE_NM
    per_kg = wheel_torque(1.0, F.SIM_MAX_SLOPE_DEG)
    max_payload = rated / per_kg - (m - payload) / 1000.0
    report["wheel_torque_nm"] = (f"{t_req:.1f} needed on {F.SIM_MAX_SLOPE_DEG:g} deg in loose regolith "
                                 f"(rolling resistance {ROLLING_RESISTANCE:g}), {rated:g} rated, {CubeMarsAK70_10.PEAK_TORQUE_NM:g} peak")
    report["max_payload_at_rated_torque_kg"] = round(max_payload, 1)
    torque_ok = t_req <= rated
    slip = 0.65
    rpm = F.SIM_MAX_SPEED / (1.0 - slip) / (F.SIM_WHEEL_RADIUS / 1000.0) * 60.0 / (2 * math.pi)
    report["wheel_speed_rpm"] = (f"{rpm:.0f} at {F.SIM_MAX_SPEED:g} m/s and {slip:g} slip; "
                                 f"{CubeMarsAK70_10.RATED_RPM_24V:g} rated at 24 V")
    speed_ok = rpm <= CubeMarsAK70_10.RATED_RPM_24V
    report["battery_wh"] = f"{LiFePO4Pack24V.CAPACITY_WH:g} ({F.SIM_BATTERY_WH:g} in the simulator)"

    # sensor views
    from hardware.parts.lunar_rover import hazcam_mount as HM
    (ox, _, oz), xa, za = HM.camera_frame()
    lens = (ox + RealSenseD455.DEPTH, 0.0, oz)
    hf, vf = RealSenseD455.DEPTH_FOV_DEG
    fov = _frustum(lens, xa, (0.0, 1.0, 0.0), za, hf + 2 * FOV_MARGIN_DEG, vf + 2 * FOV_MARGIN_DEG)
    fb = fov.BoundingBox()
    seen_all = [n for n, s, b in boxes if not n.startswith(("d455", "hazcam")) and _bbox_overlap(fb, b)
                and s.intersect(fov).Volume() > OVERLAP_TOLERANCE_MM3]
    wheels_seen = sorted({_wheel_key(n) for n in seen_all if _wheel_key(n)})
    seen = [n for n in seen_all if not _wheel_key(n)]
    report["hazcam_view_clear"] = seen or (f"yes ({hf:g} x {vf:g} deg depth view, pitched {F.HAZCAM_PITCH_DEG:g} deg down)"
                                           + (f"; the tops of the {' and '.join(wheels_seen)} wheels show in the lower corners: "
                                              "mask them" if wheels_seen else ""))
    oak_lens = (F.BODY_X[1] + 2.0 + LuxonisOAK1.LENS_FROM_USB_END, 0.0, F.BODY_Z[0] - 5.0 - LuxonisOAK1.DEPTH)
    fov2 = _frustum(oak_lens, (0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0),
                    LuxonisOAK1.HFOV_DEG + 2 * FOV_MARGIN_DEG, LuxonisOAK1.VFOV_DEG + 2 * FOV_MARGIN_DEG, depth=oak_lens[2] - 0.5)
    fb2 = fov2.BoundingBox()
    seen2 = [n for n, s, b in boxes if not n.startswith(("oak1", "terrain")) and _bbox_overlap(fb2, b)
             and s.intersect(fov2).Volume() > OVERLAP_TOLERANCE_MM3]
    report["terrain_camera_view_clear"] = seen2 or "yes (69 x 55 deg, straight down)"
    from hardware.parts.lunar_rover import lidar_mount as LM
    mx, my = F.MAST_XY
    lo = (mx, my, mast_top(F.MAST_XY, F.MAST_LENGTH) + 2.0 + LM.PLATE[1] + LM.SINK[1] + LivoxMid360.ORIGIN_HEIGHT)
    low = math.tan(math.radians(LivoxMid360.VFOV_DEG[0]))
    above = []
    for n, s, b in boxes:
        if n.startswith(("lidar", "front_mast")):
            continue
        for v in s.Vertices():
            p = v.toTuple()
            dh = math.hypot(p[0] - lo[0], p[1] - lo[1])
            if dh > 40.0 and p[2] - lo[2] > low * dh:
                above.append(n)
                break
    report["lidar_view_clear"] = above or "yes (nothing above the Mid-360's -7 deg lowest ray)"
    from hardware.parts.lunar_rover import gnss_head as GH
    gx, gy = F.REAR_MAST_XY
    lux = (gx - GH.DISC_D / 2 - GH.TAB[0] / 2 + 2.0, gy, mast_top(F.REAR_MAST_XY, F.REAR_MAST_LENGTH) + 2.0 + GH.DISC_T)
    t30 = math.tan(math.radians(SKY_ELEVATION_DEG))
    shade = []
    for n, s, b in boxes:
        if n.startswith(("gnss", "rear_mast")):
            continue
        for v in s.Vertices():
            p = v.toTuple()
            dh = math.hypot(p[0] - lux[0], p[1] - lux[1])
            if p[2] - lux[2] > t30 * max(dh, 1e-6):
                shade.append(n)
                break
    report["lux_sky_view"] = shade or f"clear above {SKY_ELEVATION_DEG:g} deg (the GNSS antenna beside it excepted)"
    report["passed"] = (not overlaps and radius_ok and gaps_ok and clearance_ok and stable and torque_ok and speed_ok
                        and not seen and not seen2 and not above and not shade)
    return report


# --------------------------------------------------------------------- figures

def _mesh(shape, tol=0.4):
    import numpy as np
    import trimesh
    verts, tris = shape.tessellate(tol, 0.5)
    return trimesh.Trimesh(np.array([v.toTuple() for v in verts]), np.array(tris), process=False)


def figures(out_dir, entries=None):
    """Render the assembly-instruction figures into ``out_dir``."""
    from pathlib import Path

    from hardware.parts.common import cad, render
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    report = check(entries)
    materials = {stem: e.get("material", "ASA") for stem, e in (entries or {}).items()}
    pieces = printed()
    bought = _purchased()
    cache = {}

    def mesh(name, shape):
        if name not in cache:
            cache[name] = _mesh(shape)
        return cache[name]

    def sel(prefixes, purchased_prefixes=(), offsets=None, exclude=()):
        items = []
        offsets = offsets or {}
        for n, stem, s in pieces:
            if any(n.startswith(p) for p in prefixes) and not any(n.startswith(e) for e in exclude):
                mm = mesh(n, s)
                off = next((v for k, v in offsets.items() if n.startswith(k)), None)
                items.append((mm.copy().apply_translation(off) if off else mm,
                              render.MATERIAL_COLORS.get(materials.get(stem, "ASA"))))
        for n, s in bought:
            if any(n.startswith(p) for p in purchased_prefixes) and not any(n.startswith(e) for e in exclude):
                mm = mesh(n, s)
                off = next((v for k, v in offsets.items() if n.startswith(k)), None)
                items.append((mm.copy().apply_translation(off) if off else mm, render.PURCHASED_COLOR))
        return items

    every = ("",)
    render.render(sel(every, every), out_dir / "overview.png", elev=22, azim=-55, zoom=1.15, max_faces=250000,
                  title="Lunar rover: printed (orange ASA, blue PETG, black TPU) and purchased parts (grey)")
    w = "front_left"
    exploded = {f"grouser_{w}": (0, 160, 0), f"wheel_segment_{w}": (0, 110, 0), f"wheel_hub_{w}": (0, 60, 0),
                f"actuator_{w}": (0, -30, 0), f"actuator_shroud_{w}": (0, -110, 0)}
    render.render(sel((f"wheel_hub_{w}", f"wheel_segment_{w}", f"grouser_{w}", f"actuator_mount_{w}",
                       f"actuator_shroud_{w}"), (f"actuator_{w}", "leg_left_+400"), exploded),
                  out_dir / "step_wheel.png", elev=15, azim=-25, zoom=1.0, max_faces=150000,
                  title="Wheel module, exploded: shroud, actuator, mount, hub, segments, grousers")
    render.render(sel(("actuator_mount", "diff_clevis"), ("rocker_", "bogie_", "leg_", "body_", "diff_", "actuator_"),
                      exclude=("body_lid", "body_top")),
                  out_dir / "step_structure.png", elev=25, azim=-60, zoom=1.1,
                  title="Structure: body frame, rockers, bogies, legs, differential")
    render.render(sel(("compute_tray", "power_tray", "battery_end", "estop"),
                      ("body_bottom", "body_floor", "battery", "mini_pc", "din_", "estop_", "body_post")),
                  out_dir / "step_body.png", elev=40, azim=-60, zoom=1.0,
                  title="Body bay: battery, compute and power trays, e-stop")
    render.render(sel(("hazcam", "terrain_cam", "mast_foot", "lidar_mount", "gnss_head", "panel_clamp"),
                      ("d455", "oak1", "front_mast", "rear_mast", "lidar", "gnss", "body_top", "body_post",
                       "body_bottom_cross_front")),
                  out_dir / "step_sensors.png", elev=20, azim=-50, zoom=1.0,
                  title="Sensors: hazard camera, terrain camera, lidar mast, GNSS and lux head, panel posts")
    del cad
    return report
