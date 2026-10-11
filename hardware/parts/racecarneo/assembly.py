"""RACECAR Neo assembly: the printed parts in place on the car with simple
datasheet-sized models of the purchased parts and screws, and the checks
run by tools/build_printable_parts.py:

  * interference: no two bodies overlap (screws are modelled at their
    nominal diameter through their holes), except declared press fits;
  * lidar scan plane: every body that crosses the C1's scan plane, the
    angular sector it hides and whether the lidar can measure it (a body
    nearer than the C1's 50 mm blind range returns nothing; anything
    farther shows up in /scan, and inside Nav2's 0.22 m robot radius it
    would stop the car through the collision monitor);
  * camera view: nothing enters the RealSense's widest view (D455 colour,
    90 x 65 deg, plus a margin);
  * sensor positions against the URDF, the Jetson's height under the
    scan plane, and the added mass.

``figures()`` renders the assembly-instruction images and a top view of
the scan plane.
"""
import math

from hardware.parts.racecarneo import _checks as C
from hardware.parts.racecarneo import _frame as F
from hardware.parts.racecarneo._footprints import (
    AdafruitBNO085,
    JetsonOrinNanoDevKit,
    RealSenseD435i,
    RealSenseD455,
    RplidarC1,
    SchneiderXB5AS8442,
    Vesc6MkVI,
)

FOV_MARGIN_DEG = 2.0
FOV_DEPTH = 3000.0
SCAN_REACH = 1500.0

PRESS_FITS = (
    ("rim_clip", "deck_", "clip grips the deck rim"),
    ("post_", "post_adapter", "adapter sleeve clamps the body post"),
)

# Purchased masses (g) for the added-mass estimate.
PURCHASED_MASS = {
    "rplidar_c1": RplidarC1.MASS_G,
    "realsense": RealSenseD435i.MASS_G,
    "jetson": JetsonOrinNanoDevKit.MASS_G,
    "imu_board": AdafruitBNO085.MASS_G,
    "vesc": Vesc6MkVI.MASS_G,
    "estop": 80.0,                     # estimate, XB5 head + contact block
}
OTHER_MASS_G = 250.0                   # estimate: converters, contactor, fuse, cables, screws


# --------------------------------------------------------------------- placement

def turned(shape, x_axis):
    """The right-hand copy of a left-hand part: turned 180 deg about the
    vertical line x = x_axis, y = 0."""
    from hardware.parts.common import cad
    return cad.rotate(shape, "Z", 180.0, about=(x_axis, 0.0, 0.0))


def printed():
    """[(name, part id, shape)] for every printed piece in place."""
    from hardware.parts.common import cad
    from hardware.parts.racecarneo import (
        camera_arm,
        camera_cradle,
        deck_front,
        deck_grommet,
        deck_rear,
        estop_tower,
        imu_mount,
        jetson_tray,
        lidar_mast,
        post_adapter_front,
        post_adapter_rear,
        power_tray,
        rim_clip,
        vesc_tray,
    )
    out = []
    for mod in (deck_front, deck_rear, lidar_mast, camera_arm, camera_cradle, jetson_tray, imu_mount, vesc_tray,
                estop_tower):
        stem = mod.__name__.rsplit(".", 1)[1]
        out.append((stem, stem, mod.build()))
    for mod, which in ((post_adapter_front, "front"), (post_adapter_rear, "rear")):
        stem = mod.__name__.rsplit(".", 1)[1]
        s = mod.build()
        x = F.MOUNTS[which][0][0]
        out.append((f"{stem}_left", stem, s))
        out.append((f"{stem}_right", stem, turned(s, x)))
    pt = power_tray.build()
    out.append(("power_tray_left", "power_tray", pt))
    out.append(("power_tray_right", "power_tray", turned(pt, F.POWER_CENTERS[0][0])))
    g = deck_grommet.build()
    for k, (x, y) in enumerate(F.MOUNTS["front"] + F.MOUNTS["rear"]):
        out.append((f"deck_grommet_lower_{k}", "deck_grommet", cad.translate(g, (x, y, F.DECK_Z[0] - F.GROMMET_FLANGE_T))))
        out.append((f"deck_grommet_upper_{k}", "deck_grommet",
                    cad.translate(cad.rotate(g, "X", 180), (x, y, F.DECK_Z[1] + F.GROMMET_FLANGE_T))))
    clip = rim_clip.build()
    for x in F.RIM_CLIP_X:
        out.append((f"rim_clip_left_{x:g}", "rim_clip", cad.translate(clip, (x, 0, 0))))
        out.append((f"rim_clip_right_{x:g}", "rim_clip", turned(cad.translate(clip, (x, 0, 0)), x)))
    return out


def camera_link(camera=RealSenseD435i):
    """Where the cradle puts camera_link for this camera (level pitch)."""
    screw_y = F.SCREW_Y[1] if camera is RealSenseD435i else F.SCREW_Y[0]
    x = F.SCREW_X + camera.SCREW_FROM_FRONT - camera.LINK_FROM_FRONT
    return (x, screw_y + camera.LEFT_IMAGER_FROM_SCREW, F.CRADLE_SEAT_Z + camera.LINK_HEIGHT)


def realsense_box(camera=RealSenseD435i):
    from hardware.parts.common import cad
    x_front = F.SCREW_X + camera.SCREW_FROM_FRONT
    screw_y = F.SCREW_Y[1] if camera is RealSenseD435i else F.SCREW_Y[0]
    z0 = F.CRADLE_SEAT_Z
    return cad.box(x_front - camera.DEPTH, x_front, screw_y - camera.WIDTH / 2, screw_y + camera.WIDTH / 2,
                   z0, z0 + camera.HEIGHT)


def imu_board_z():
    from hardware.parts.racecarneo import imu_mount
    return F.DECK_Z[1] + imu_mount.PLATE[2] + imu_mount.BOSS_H


def jetson_top_z():
    J = JetsonOrinNanoDevKit
    return F.jetson_pcb_z() + J.PCB_T + J.FAN_TOP_ABOVE_PCB


def purchased():
    """[(name, shape, mass_g)] for the purchased parts and screws in the model."""
    from hardware.parts.common import cad
    out = []

    def add(name, shape, mass=0.0):
        out.append((name, shape, mass))

    # RPLIDAR C1: square base, round optical enclosure, M2.5 screws from below.
    lx, ly = F.LIDAR_XY
    z = F.LIDAR_BOTTOM_Z
    h = RplidarC1.BASE / 2
    c1 = cad.box(lx - h, lx + h, ly - h, ly + h, z, z + RplidarC1.BASE_HEIGHT).fuse(
        cad.cyl(RplidarC1.ENCLOSURE_D, RplidarC1.HEIGHT - RplidarC1.BASE_HEIGHT + 0.01,
                (lx, ly, z + RplidarC1.BASE_HEIGHT - 0.01)))
    for x, y in F.lidar_holes():
        c1 = c1.cut(cad.cyl(2.5, RplidarC1.HOLE_DEPTH + 0.01, (x, y, z - 0.01)))
        add(f"lidar_screw_{x:g}_{y:g}", C.screw((x, y, z - F.SEAT_T), (0, 0, 1), 8.0, "M2.5"))
    add("rplidar_c1", c1, PURCHASED_MASS["rplidar_c1"])

    # RealSense D435i on the cradle, its 1/4"-20 screw, pivot and lock screws.
    cam = realsense_box()
    sy = F.SCREW_Y[1]
    cam = cam.cut(cad.cyl(6.35, RealSenseD435i.SCREW_DEPTH + 0.01, (F.SCREW_X, sy, F.CRADLE_SEAT_Z - 0.01)))
    add("realsense", cam, PURCHASED_MASS["realsense"])
    seat_bot = F.CRADLE_SEAT_Z - F.CRADLE_T
    add("camera_screw", cad.cyl(6.35 - C.SCREW_SLACK, 9.5, (F.SCREW_X, sy, seat_bot)).fuse(
        cad.cyl(11.0, 3.3, (F.SCREW_X, sy, seat_bot - 3.3))))
    from hardware.parts.racecarneo import camera_cradle
    (ya, _), (_, yb) = camera_cradle.cheek_y()
    for x in (F.PIVOT[0], F.PIVOT[0] - F.LOCK_R):
        add(f"cradle_screw_{x:g}", C.screw((x, ya, F.PIVOT[1]), (0, 1, 0), 30.0, "M3"))

    # Jetson carrier, its bottom parts, the module/heatsink and I/O envelopes.
    J = JetsonOrinNanoDevKit
    jx0, jx1 = F.JETSON_X
    jy0, jy1 = F.JETSON_Y
    zp = F.jetson_pcb_z()
    pcb = cad.box(jx0, jx1, jy0, jy1, zp, zp + J.PCB_T)
    for x, y in F.jetson_holes():
        pcb = pcb.cut(cad.cyl(J.HOLE_D, 4, (x, y, zp - 1)))
        add(f"jetson_screw_{x:g}_{y:g}", C.screw((x, y, zp + J.PCB_T), (0, 0, -1), 6.0, "M2.5"))
    add("jetson_carrier", pcb, PURCHASED_MASS["jetson"])
    add("jetson_bottom_parts", cad.box(jx0 + 22, jx1 - 22, jy0 + 6, jy1 - 6, zp - J.BOTTOM_PARTS_H, zp))
    top = zp + J.PCB_T
    add("jetson_heatsink", cad.box(jx0 + 8, jx0 + 50, jy0 + 18, jy1 - 22, top, top + J.FAN_TOP_ABOVE_PCB))
    add("jetson_io", cad.box(jx1 - 14, jx1 + 1.5, jy0 + 1, jy1 - 6, top, top + J.TOP_PARTS_H).fuse(
        cad.box(jx1 - 21, jx1 - 13, jy0 + 70, jy0 + 85, top, top + J.TOP_PARTS_H)))  # RJ45 is deeper
    add("jetson_header", cad.box(jx0 + 6, jx1 - 26, jy1 - 6, jy1 - 1, top, top + 8.5))

    # IMU breakout
    from hardware.parts.racecarneo import imu_mount
    bx, by = AdafruitBNO085.BOARD
    zi = imu_board_z()
    sx, sy2 = AdafruitBNO085.HOLE_SPACING[0] / 2, AdafruitBNO085.HOLE_SPACING[1] / 2
    board = cad.box(-bx / 2, bx / 2, -by / 2, by / 2, zi, zi + 1.6)
    for dx in (sx, -sx):
        for dy in (sy2, -sy2):
            board = board.cut(cad.cyl(AdafruitBNO085.HOLE_D, 4, (dx, dy, zi - 1)))
    add("imu_board", board, PURCHASED_MASS["imu_board"])
    for dx in (sx, -sx):
        for dy in (sy2, -sy2):
            add(f"imu_screw_{dx:g}_{dy:g}", C.screw((dx, dy, zi + 1.6), (0, 0, -1), 6.0, "M2.5"))

    # VESC
    from hardware.parts.racecarneo import vesc_tray
    vx, vy = F.VESC_CENTER
    zv = F.DECK_Z[1] + vesc_tray.PLATE_T
    add("vesc", cad.cbox(*Vesc6MkVI.SIZE, center=(vx, vy, zv + Vesc6MkVI.SIZE[2] / 2)), PURCHASED_MASS["vesc"])

    # E-stop head over the tower panel, body inside the tower.
    ex, ey = F.ESTOP_XY
    zt = F.DECK_Z[1] + F.ESTOP_TOWER_H
    E = SchneiderXB5AS8442
    add("estop_head", cad.cyl(E.MUSHROOM_D, E.HEAD_H, (ex, ey, zt)), PURCHASED_MASS["estop"])
    add("estop_body", cad.cyl(E.BODY_D, E.BODY_H - 4.0, (ex, ey, zt - E.BODY_H)))

    # Screws: deck mounts, mast, arm foot, item screws into deck inserts.
    for k, (x, y) in enumerate(F.MOUNTS["front"] + F.MOUNTS["rear"]):
        add(f"deck_screw_{k}", C.screw((x, y, F.DECK_Z[1] + F.GROMMET_FLANGE_T + 0.5), (0, 0, -1), 16.0, "M3"))
    for x, y in F.MAST_SCREWS:
        add(f"mast_screw_{x:g}_{y:g}", C.screw((x, y, F.DECK_Z[1] + F.MAST_FLANGE_T), (0, 0, -1), 16.0, "M3"))
    for x, y in F.ARM_FOOT_SCREWS:
        add(f"arm_screw_{x:g}_{y:g}", C.screw((x, y, F.DECK_Z[1] + F.MAST_FLANGE_T + F.ARM_FOOT_T), (0, 0, -1), 25.0, "M3"))
    for x, y in F.LAP_SCREWS:
        add(f"lap_screw_{x:g}_{y:g}", C.screw((x, y, F.DECK_Z[1]), (0, 0, -1), 12.0, "M3"))
    from hardware.parts.racecarneo import estop_tower, power_tray
    for x, y in F.JETSON_TRAY_INSERTS:
        add(f"tray_screw_{x:g}_{y:g}", C.screw((x, y, F.DECK_Z[1] + F.JETSON_TRAY_T), (0, 0, -1), 6.0, "M3"))
    for x, y in F.IMU_INSERTS:
        add(f"imu_mount_screw_{y:g}", C.screw((x, y, F.DECK_Z[1] + imu_mount.PLATE[2]), (0, 0, -1), 6.0, "M3"))
    for x, y in F.VESC_INSERTS:
        add(f"vesc_tray_screw_{x:g}_{y:g}", C.screw((x, y, F.DECK_Z[1] + vesc_tray.PLATE_T), (0, 0, -1), 6.0, "M3"))
    for x, y in F.POWER_INSERTS:
        add(f"power_screw_{x:g}_{y:g}", C.screw((x, y, F.DECK_Z[1] + power_tray.T), (0, 0, -1), 6.0, "M3"))
    for x, y in F.ESTOP_INSERTS:
        add(f"estop_screw_{x:g}_{y:g}", C.screw((x, y, F.DECK_Z[1] + estop_tower.EAR[2]), (0, 0, -1), 8.0, "M3"))

    # Chassis: the URDF's base_link envelope and the body posts the adapters clamp.
    add("chassis_envelope", cad.box(-180.0, 180.0, -110.0, 110.0, 0.0, 100.0))
    for which in ("front", "rear"):
        for k, (x, y) in enumerate(F.POSTS[which]):
            add(f"post_{which}_{k}", cad.cyl(F.POST_D, F.SEAT_Z - 12.0 - 100.1, (x, y, 100.1)))
    return out


# --------------------------------------------------------------------- checks

def scan_plane(bodies, lidar_xy=F.LIDAR_XY, scan_z=F.SCAN_Z, skip=("rplidar_c1",)):
    """Bodies crossing the scan plane: list of dicts (name, sector, near,
    far, blind: inside the blind range, shadowed: behind a blind body)."""
    found = []
    for n, s in bodies:
        if any(n.startswith(p) for p in skip):
            continue
        sec = C.scan_plane_section(s, lidar_xy, scan_z, F.SCAN_BAND, SCAN_REACH)
        if sec:
            sec["name"] = n
            sec["blind"] = sec["near_mm"] < RplidarC1.MIN_RANGE - F.BLIND_MARGIN
            found.append(sec)
    blind = [s for s in found if s["blind"]]
    for s in found:
        s["shadowed"] = (not s["blind"]) and any(C.shadowed(b, s) for b in blind)
    return found


def in_footprint(sec, lidar_xy=F.LIDAR_XY):
    """Is the nearest point of a measured body inside Nav2's robot radius?"""
    a = math.radians((sec["from_deg"] + sec["to_deg"]) / 2.0)
    x = lidar_xy[0] + sec["near_mm"] * math.cos(a)
    y = lidar_xy[1] + sec["near_mm"] * math.sin(a)
    return math.hypot(x, y) < F.NAV2_ROBOT_RADIUS


def camera_fov(apex, pitch_deg=0.0):
    """The widest RealSense view (D455 colour) plus the margin, from ``apex``
    looking forward, pitched down by ``pitch_deg``."""
    p = math.radians(pitch_deg)
    fwd = (math.cos(p), 0.0, -math.sin(p))
    up = (math.sin(p), 0.0, math.cos(p))
    h, v = RealSenseD455.COLOR_FOV_DEG
    return C.frustum(apex, fwd, up, h / 2 + FOV_MARGIN_DEG, v / 2 + FOV_MARGIN_DEG, FOV_DEPTH)


def check(entries=None):
    """Run the assembly checks; returns a report dict ('passed' is the
    overall result)."""
    entries = entries or {}
    report = {}
    bodies, masses = [], []
    for name, stem, shape in printed():
        bodies.append((name, shape))
        masses.append(entries.get(stem, {}).get("estimate", {}).get("mass_g", 0.0))
    for name, shape, mass in purchased():
        bodies.append((name, shape))
        masses.append(mass)

    overlaps, allowed = C.interferences(bodies, PRESS_FITS)
    report["bodies"] = len(bodies)
    report["interferences"] = overlaps or "none"
    report["intended_press_fits"] = len(allowed)

    secs = scan_plane(bodies)
    visible = [s for s in secs if not s["blind"] and not s["shadowed"]]
    report["scan_plane_blind_sectors"] = [
        f"{s['name']}: {C.sector_text(s)}, {s['near_mm']:.0f}-{s['far_mm']:.0f} mm from the lidar axis "
        f"(inside the {RplidarC1.MIN_RANGE:g} mm blind range: no returns)" for s in secs if s["blind"]] or "none"
    report["scan_plane_in_the_fin_shadow"] = [
        f"{s['name']}: {C.sector_text(s)}, {s['near_mm']:.0f} mm from the lidar axis" for s in secs if s["shadowed"]] or "none"
    report["scan_plane_measured"] = [
        f"{s['name']}: {C.sector_text(s)} at {s['near_mm']:.0f} mm"
        + (" (inside Nav2's robot radius)" if in_footprint(s) else "") for s in visible] or "none"
    report["fin_shadow_deg"] = f"+-{F.fin_shadow_deg():.1f} about the rear (180 deg)"

    fov_hits = []
    for pitch in (-F.PITCH_RANGE_DEG, 0.0, F.PITCH_RANGE_DEG):
        lens = camera_link()
        fov = camera_fov(lens, pitch)
        for hit in C.in_view(bodies, fov, skip=("realsense",)):
            fov_hits.append(f"pitch {pitch:+g}: {hit}")
    report["camera_view_clear"] = fov_hits or (
        f"yes (D455 colour {RealSenseD455.COLOR_FOV_DEG[0]:g} x {RealSenseD455.COLOR_FOV_DEG[1]:g} deg + "
        f"{FOV_MARGIN_DEG:g} deg margin, at pitch -{F.PITCH_RANGE_DEG:g}, 0, +{F.PITCH_RANGE_DEG:g} deg)")

    cl = camera_link()
    cl455 = camera_link(RealSenseD455)
    report["lidar_link_mm"] = (f"({F.LIDAR_XY[0]:g}, {F.LIDAR_XY[1]:g}, {F.LIDAR_BOTTOM_Z + RplidarC1.SCAN_HEIGHT:.1f}); "
                               "URDF (80, 0, 180)")
    report["camera_link_mm"] = (f"D435i ({cl[0]:.1f}, {cl[1]:.1f}, {cl[2]:.1f}), D455 ({cl455[0]:.1f}, {cl455[1]:.1f}, "
                                f"{cl455[2]:.1f}); URDF (180, 0, 220)")
    report["imu_link_mm"] = f"({F.IMU_XY[0]:g}, {F.IMU_XY[1]:g}, {imu_board_z() + 1.6:.0f}); URDF (0, 0, 80)"
    jt = jetson_top_z()
    report["jetson_fan_top_mm"] = f"{jt:.1f} ({F.SCAN_Z - jt:.1f} mm under the scan plane)"
    report["added_mass_g"] = round(sum(masses) + OTHER_MASS_G)

    pos_ok = (abs(cl[0] - F.CAMERA_LINK[0]) < 1.0 and abs(cl[1]) < 0.5 and abs(cl[2] - F.CAMERA_LINK[2]) < 1.0
              and abs(F.LIDAR_BOTTOM_Z + RplidarC1.SCAN_HEIGHT - F.SCAN_Z) < 0.1)
    report["passed"] = (not overlaps and not visible and not fov_hits and pos_ok
                        and jt < F.SCAN_KEEP_OUT_Z)
    return report


# --------------------------------------------------------------------- figures

def scan_figure(path, secs, lidar_xy=F.LIDAR_XY, title="Lidar scan plane (top view)", extra=()):
    """Top view of the scan plane: the deck, the lidar, its blind range,
    Nav2's robot radius and every section through the plane."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle, Wedge

    from hardware.parts.common import render
    fig, ax = plt.subplots(figsize=(8.0, 5.6), dpi=100)
    lx, ly = lidar_xy
    ax.add_patch(Rectangle((F.DECK_X[0], -F.DECK_HALF_W), F.DECK_X[1] - F.DECK_X[0], 2 * F.DECK_HALF_W,
                           fc="#eef2f7", ec="#8899aa", lw=0.8, label="deck"))
    ax.add_patch(Circle((0, 0), F.NAV2_ROBOT_RADIUS, fill=False, ls="--", ec="#888888", lw=0.8,
                        label="Nav2 robot radius 0.22 m"))
    ax.add_patch(Circle((lx, ly), RplidarC1.MIN_RANGE, fc="#fff3c4", ec="#c9a227", lw=0.8,
                        label="C1 blind range 50 mm"))
    for s in secs:
        r = 450.0
        col = "#d62728" if not s["blind"] and not s["shadowed"] else ("#7f7f7f" if s["shadowed"] else "#e8833a")
        ax.add_patch(Wedge((lx, ly), r, s["from_deg"], s["from_deg"] + s["width_deg"], width=r - s["near_mm"],
                           fc=col, alpha=0.18 if s["blind"] else 0.35, ec="none"))
    for label, xy, col in extra:
        ax.add_patch(Circle(xy, 4.0, fc=col, ec="none"))
    ax.add_patch(Circle((lx, ly), RplidarC1.ENCLOSURE_D / 2, fc="#b9bec6", ec="#555555", lw=0.8, label="RPLIDAR C1"))
    ax.plot([], [], color="#e8833a", lw=6, alpha=0.5, label="hidden, inside the blind range (no returns)")
    ax.plot([], [], color="#7f7f7f", lw=6, alpha=0.5, label="in that shadow")
    ax.plot([], [], color="#d62728", lw=6, alpha=0.9, label="measured by the lidar")
    ax.set_xlim(-300, 330)
    ax.set_ylim(-220, 220)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm, forward)")
    ax.set_ylabel("y (mm, left)")
    ax.grid(True, lw=0.3, color="#dddddd")
    ax.legend(loc="lower left", fontsize=7, framealpha=0.9)
    ax.set_title(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=100, facecolor="white")
    plt.close(fig)
    render._compress_png(path)


def figures(out_dir, entries=None):
    """Render the assembly-instruction figures into ``out_dir``."""
    from pathlib import Path

    from hardware.parts.common import render
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    report = check(entries)
    materials = {stem: e.get("material", "PETG") for stem, e in (entries or {}).items()}
    pieces = printed()
    buy = [(n, s) for n, s, _ in purchased() if not n.startswith(("chassis_envelope",))]
    meshes = {}

    def mesh(name, shape):
        if name not in meshes:
            meshes[name] = C.mesh(shape)
        return meshes[name]

    def color(stem):
        return render.MATERIAL_COLORS.get(materials.get(stem, "ASA"), "#e8833a")

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
    render.render(sel(every, every), out_dir / "overview.png", zoom=1.1,
                  title="RACECAR Neo deck: printed parts (colour), purchased parts (grey)")
    exploded = {"deck_front": (0, 0, 40), "lidar_mast": (0, 0, 80), "mast_screw": (0, 0, 80), "rplidar": (0, 0, 120),
                "lidar_screw": (0, 0, 100), "camera_arm": (0, 0, 160), "arm_screw": (0, 0, 160),
                "camera_cradle": (40, 0, 200), "cradle_screw": (40, 0, 200), "realsense": (40, 0, 240),
                "camera_screw": (40, 0, 220), "jetson_tray": (0, 0, 60), "tray_screw": (0, 0, 60),
                "jetson_": (0, 0, 100), "deck_grommet_upper": (0, 0, 30), "deck_screw": (0, 0, 60),
                "post_adapter": (0, 0, -40), "deck_grommet_lower": (0, 0, -20), "imu_": (0, 0, 40),
                "vesc": (0, 0, 50), "estop": (0, 0, 60), "power_": (0, 0, 40)}
    render.render(sel(every, every, exploded), out_dir / "exploded.png", elev=20, azim=-60, zoom=0.95,
                  title="Exploded view")
    render.render(sel(("deck_", "post_adapter", "rim_clip"), ("deck_screw", "lap_screw", "post_")),
                  out_dir / "step_deck.png", elev=30, azim=-55, zoom=1.0,
                  title="Deck halves on the post adapters, TPU grommets, rim clips")
    render.render(sel(("lidar_mast", "camera_arm", "camera_cradle"),
                      ("rplidar", "lidar_screw", "mast_screw", "arm_screw", "realsense", "camera_screw", "cradle_screw")),
                  out_dir / "step_sensors.png", elev=18, azim=-35, zoom=1.0,
                  title="Lidar mast, camera arm and RealSense cradle")
    render.render(sel(("jetson_tray", "imu_mount", "vesc_tray", "power_tray", "estop_tower"),
                      ("jetson_", "tray_screw", "imu_", "vesc", "estop", "power_screw")),
                  out_dir / "step_electronics.png", elev=35, azim=-60, zoom=1.0,
                  title="Jetson tray, IMU, power trays, VESC tray and e-stop tower")
    bodies = [(n, s) for n, _, s in pieces] + [(n, s) for n, s, _ in purchased()]
    scan_figure(out_dir / "scan_plane.png", scan_plane(bodies),
                title="Lidar scan plane: the camera arm fin hides the rear sector, nothing is measured")
    return report
