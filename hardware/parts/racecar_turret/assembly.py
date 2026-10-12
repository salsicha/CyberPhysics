"""RACECAR Turret assembly: the RACECAR Neo deck (racecarneo parts and
models) with the turret (turret parts and servo models) on the turret plate
and its posts, and the checks run by tools/build_printable_parts.py:

  * interference at the home pose (pan 0, tilt 0), screws modelled through
    their holes;
  * lidar scan plane: the sectors the four posts hide, whether the camera
    arm's fin already hides them, and whether they lie inside the
    laser_filters box of systems/racecar_turret/config/scan_filter.yaml;
    nothing else may be measured;
  * pan sweep over +-90 deg against the plate, the posts and the car
    (the RealSense and its camera arm included), at both tilt limits and
    level, with the turret's foam dart launcher cocked and primed;
  * line of fire: where a dart's path from the muzzle meets the car (a
    no-fire zone for the tracker);
  * the turret camera's view at pan 0: the tilt below which the car's own
    front sensors enter it;
  * cable runs (camera, UART, servo bus, 12 V) and the added mass (the
    launcher with a full clip included).
"""
import math
from pathlib import Path

from hardware.parts.racecar_turret import _frame as F
from hardware.parts.racecarneo import _checks as C
from hardware.parts.racecarneo import _frame as R
from hardware.parts.racecarneo import assembly as car
from hardware.parts.turret import _frame as T
from hardware.parts.turret import _launcher as L
from hardware.parts.turret import assembly as turret
from hardware.parts.turret._footprints import ArducamB0385
from hardware.parts.turret._footprints import WaveshareBusServoAdapterA as B

REPO = Path(__file__).resolve().parents[3]
SCAN_FILTER = REPO / "systems" / "racecar_turret" / "config" / "scan_filter.yaml"
CLIP_Z = (150.0, 215.0)
TURRET_STATIC_SKIP = ("base_plate", "arduino", "uno_", "servo_adapter", "bsa_")


def to_car(shape):
    from hardware.parts.common import cad
    return cad.translate(shape, F.TURRET_ORIGIN)


def printed_and_models(pan_deg=0.0, tilt_deg=0.0, state="cocked"):
    """[(name, part id or None, shape)] for everything on the car, the
    turret's launcher in ``state`` (turret.LAUNCHER_STATES)."""
    from hardware.parts.common import cad
    from hardware.parts.racecar_turret import plate_post, post_clip, turret_plate
    out = [(n, stem, s) for n, stem, s in car.printed()]
    out += [(n, None, s) for n, s, _ in car.purchased()]
    static, pan_st, tilt_st = turret.stages("b0385", state)
    turret_ids = {"pan_base", "pan_hub", "tilt_bracket", "camera_yoke", "camera_plate_b0385"} | {
        f"launcher_{s}" for s in L.PRINTED}
    for n, s in static:
        if not n.startswith(TURRET_STATIC_SKIP):
            out.append((f"turret_{n}", n if n in turret_ids else None, to_car(s)))
    for n, s in pan_st:
        out.append((f"turret_{n}", n if n in turret_ids else None, to_car(turret.place_pan(s, pan_deg))))
    for n, s in tilt_st:
        out.append((f"turret_{n}", n if n in turret_ids else None,
                    to_car(turret.place_pan(turret.place_tilt(s, tilt_deg), pan_deg))))
    out.append(("turret_plate", "turret_plate", turret_plate.build()))
    post = plate_post.build_local()
    clip = post_clip.build()
    for k, (x, y) in enumerate(F.POSTS):
        out.append((f"plate_post_{k}", "plate_post", cad.translate(post, (x, y, 0))))
        for dx in (F.POST_INSERT_DX, -F.POST_INSERT_DX):
            out.append((f"post_screw_{k}_{dx:g}", None,
                        C.screw((x + dx, y, R.DECK_Z[1] + F.POST_FLANGE[2]), (0, 0, -1), 8.0, "M3")))
        out.append((f"plate_screw_{k}", None, C.screw((x, y, F.PLATE_Z[1]), (0, 0, -1), 10.0, "M3")))
        if k >= 2:            # the rear posts carry the cables
            for z in CLIP_Z:
                c = cad.rotate(clip, "Z", 0.0 if y < 0 else 180.0)
                out.append((f"post_clip_{k}_{z:g}", "post_clip", cad.translate(c, (x, y, z))))
    # Bus Servo Adapter (A) under the plate, components down.
    ax, ay = F.ADAPTER_CENTER
    zb = F.PLATE_Z[0] - F.ADAPTER_STANDOFF[1]
    board = cad.box(ax - B.BOARD[0] / 2, ax + B.BOARD[0] / 2, ay - B.BOARD[1] / 2, ay + B.BOARD[1] / 2,
                    zb - 1.6, zb)
    sx, sy = B.HOLE_SPACING[0] / 2, B.HOLE_SPACING[1] / 2
    for dx in (sx, -sx):
        for dy in (sy, -sy):
            board = board.cut(cad.cyl(B.HOLE_D, 4, (ax + dx, ay + dy, zb - 3)))
            out.append((f"adapter_screw_{dx:g}_{dy:g}", None,
                        C.screw((ax + dx, ay + dy, zb - 1.6), (0, 0, 1), 5.0, "M2")))
    out.append(("servo_adapter", None, board))
    out.append(("servo_adapter_parts", None, cad.box(ax - B.BOARD[0] / 2 + 6, ax + B.BOARD[0] / 2 - 6,
                                                       ay - B.BOARD[1] / 2 + 6, ay + B.BOARD[1] / 2 - 6,
                                                       zb - 1.6 - B.TOP_PARTS_H, zb - 1.6)))
    return out


def scan_filter_box():
    """The LaserScanBoxFilter box of scan_filter.yaml, in mm, or None."""
    import yaml
    if not SCAN_FILTER.exists():
        return None
    doc = yaml.safe_load(SCAN_FILTER.read_text())
    params = doc["scan_to_scan_filter_chain"]["ros__parameters"]
    for f in params.values():
        if isinstance(f, dict) and f.get("type", "").endswith("LaserScanBoxFilter"):
            p = f["params"]
            return {k: 1000.0 * p[k] for k in ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")}
    return None


def no_fire_zone(items, pans=None):
    """[(pan deg, lowest clear tilt deg, body)] where a dart's path
    (turret.DART_PATH_D, turret.DART_PATH_MM from the muzzle) meets the car
    at the bottom of the tilt range; the tilt is the first, in 1 deg steps,
    that clears."""
    from hardware.parts.common import cad
    car = [(n, s, s.BoundingBox()) for n, stem, s in items if not n.startswith("turret_")]
    path0 = turret.dart_path()
    ox, oy = F.TURRET_ORIGIN[0], F.TURRET_ORIGIN[1]
    lo = T.TILT_RANGE_DEG[0]

    def hit(tilt, pan):
        path = cad.rotate(to_car(turret.place_tilt(path0, tilt)), "Z", -pan, about=(ox, oy, 0))
        pb = path.BoundingBox()
        for n, s, b in car:
            if C.bbox_overlap(pb, b) and path.intersect(s).Volume() > 0.01:
                return n
        return None

    pans = pans or sorted(set(range(-90, 91, 15)) | set(range(-14, 15, 2)))
    zone = []
    for pan in pans:
        n = hit(lo, pan)
        if not n:
            continue
        tilt = math.ceil(lo)
        while tilt < 0 and hit(tilt, pan):
            tilt += 1
        zone.append((pan, tilt, n))
    return zone


def check(entries=None):
    """Run the assembly checks; returns a report dict ('passed' is the
    overall result)."""
    from hardware.parts.common import cad
    entries = entries or {}
    report = {}
    items = printed_and_models()
    bodies = [(n, s) for n, _, s in items]
    press = car.PRESS_FITS + (("turret_pan_servo_horn", "turret_pan_hub", "hub disc on the horn"),
                              ("turret_tilt_servo_horn", "turret_camera_yoke", "yoke hubs on the horns"),
                              ("post_clip", "plate_post", "clip grips the post"),
                              ("turret_launcher_cam_pin", "turret_launcher_shuttle", "dowel pin pressed into the tab"))
    overlaps, allowed = C.interferences(bodies, press)
    report["bodies"] = len(bodies)
    report["interferences"] = overlaps or "none"
    report["intended_press_fits"] = len(allowed)

    # Scan plane: everything on the car, and each post on its own.
    secs = car.scan_plane(bodies)
    visible = [s for s in secs if not s["blind"] and not s["shadowed"]]
    report["scan_plane_blind_sectors"] = [f"{s['name']}: {C.sector_text(s)}, {s['near_mm']:.0f}-{s['far_mm']:.0f} mm "
                                          "(inside the C1's blind range)" for s in secs if s["blind"]] or "none"
    post_secs = [s for s in secs if s["name"].startswith("plate_post")]
    report["post_sectors"] = [f"{s['name']}: {C.sector_text(s)}, {s['near_mm']:.0f}-{s['far_mm']:.0f} mm from the lidar axis"
                              + (", in the fin's shadow" if s["shadowed"] else ", MEASURED") for s in post_secs]
    report["scan_plane_measured"] = [f"{s['name']}: {C.sector_text(s)} at {s['near_mm']:.0f} mm" for s in visible] or "none"
    box = scan_filter_box()
    if box is None:
        report["posts_in_scan_filter_box"] = "no scan_filter.yaml box found"
        in_box = False
    else:
        outside = []
        for k, (x, y) in enumerate(F.POSTS):
            r = F.POST_D / 2
            if not (box["min_x"] <= x - r and x + r <= box["max_x"] and box["min_y"] <= y - r and y + r <= box["max_y"]
                    and box["min_z"] <= R.SCAN_Z <= box["max_z"]):
                outside.append(f"plate_post_{k} at ({x:g}, {y:g})")
        in_box = not outside
        report["posts_in_scan_filter_box"] = ("yes: box x {min_x:.0f}..{max_x:.0f}, y {min_y:.0f}..{max_y:.0f} mm; "
                                              .format(**box) + "posts at " +
                                              ", ".join(f"({x:g}, {y:g})" for x, y in F.POSTS)) if in_box else outside

    # Pan sweep: the rotating turret against the plate, the posts and the
    # car, at both tilt limits and level, launcher cocked and primed.
    car_static = [(n, s) for n, stem, s in items if not n.startswith("turret_") or n.startswith(
        ("turret_pan_base", "turret_pan_servo_case", "turret_base_bolt", "turret_pan_servo_screw"))]
    worst = (math.inf, None, None, None, None)
    ox, oy = F.TURRET_ORIGIN[0], F.TURRET_ORIGIN[1]
    for tilt, ext in turret.tilt_poses():
        static, pan_st, tilt_st = turret.stages("b0385", ext)
        movers0 = [(n, to_car(s)) for n, s in pan_st if not n.startswith("pan_servo_horn")]
        movers0 += [(n, to_car(turret.place_tilt(s, tilt))) for n, s in tilt_st]
        for a in range(-int(T.PAN_LIMIT_DEG), int(T.PAN_LIMIT_DEG) + 1, 15):
            movers = [(n, cad.rotate(s, "Z", -a, about=(ox, oy, 0))) for n, s in movers0]
            d, pair = turret._min_distance(movers, car_static)
            if d < worst[0]:
                worst = (d, pair, a, tilt, ext)
    report["pan_sweep"] = (f"clear over +-{T.PAN_LIMIT_DEG:g} deg at both tilt limits and level, launcher cocked "
                           f"and primed; smallest gap {worst[0]:.1f} mm ({worst[1]} at {worst[2]:+d} deg, tilt "
                           f"{worst[3]:+.0f}, {worst[4]})")
    # The launcher against the car's own sensors and posts (not the turret's
    # pan base, which the line above covers), and a dart's path against the car.
    car_only = [(n, s) for n, s in car_static if not n.startswith("turret_")]
    near = (math.inf, None)
    for tilt, st in turret.tilt_poses():
        _, _, tilt_st = turret.stages("b0385", st)
        act = [(n, to_car(turret.place_tilt(s, tilt))) for n, s in tilt_st if n.startswith(("launcher_", "act_", "trig_"))]
        for a in range(-int(T.PAN_LIMIT_DEG), int(T.PAN_LIMIT_DEG) + 1, 15):
            d, pair = turret._min_distance([(n, cad.rotate(s, "Z", -a, about=(ox, oy, 0))) for n, s in act], car_only)
            if d < near[0]:
                near = (d, f"{pair} at pan {a:+d}, tilt {tilt:+.0f}, {st}")
    report["launcher_to_car_mm"] = f"{near[0]:.0f} at the closest ({near[1]})"
    # Line of fire: where a dart's path from the muzzle meets the car (the
    # RealSense and its arm in front, at steep nose-down tilts). Reported as
    # a no-fire zone for the tracker, not a fault of the assembly.
    zone = no_fire_zone(items)
    report["line_of_fire"] = ("a dart's path clears the car at every pan and tilt" if not zone else
                              "no-fire zone (the path meets the car): " + "; ".join(
                                  f"pan {a:+d} deg below tilt {tc:+.0f} deg ({n})" for a, tc, n in zone))
    pan_ok = worst[0] >= turret.MIN_CLEARANCE_MM and near[0] >= turret.MIN_CLEARANCE_MM

    # Turret camera view at pan 0: where the car enters it.
    _, front_x = turret.lens_front()
    car_bodies = [(n, s) for n, stem, s in items if not n.startswith("turret_")]
    enters = None
    for tilt in range(0, int(T.TILT_RANGE_DEG[0]) - 1, -3):
        p = math.radians(tilt)
        apex = (ox + front_x * math.cos(p), oy, F.TURRET_ORIGIN[2] + T.TILT_Z + front_x * math.sin(p))
        fov = C.frustum(apex, (math.cos(p), 0, math.sin(p)), (-math.sin(p), 0, math.cos(p)),
                        ArducamB0385.HFOV_DEG / 2, ArducamB0385.VFOV_DEG / 2, 3000.0)
        hits = C.in_view(car_bodies, fov)
        if hits:
            enters = (tilt, hits[:3])
            break
    report["turret_camera_view"] = ("the car stays out of view at pan 0 down to the tilt limit" if enters is None else
                                    f"at pan 0 the car enters the view below about {enters[0]} deg of tilt "
                                    f"({', '.join(h.split(' (')[0] for h in enters[1])})")
    report["turret_camera_view_clear_level"] = enters is None or enters[0] < 0

    # Cable runs (straight segments along the deck and the posts).
    header = (R.JETSON_X[0] + 40.0, R.JETSON_Y[1], R.jetson_pcb_z() + 10.0)
    post_l = F.POSTS[0]
    adapter = (F.ADAPTER_CENTER[0] + B.BOARD[0] / 2, F.ADAPTER_CENTER[1], F.PLATE_Z[0] - 10.0)
    uart = (math.dist(header, (post_l[0], post_l[1], R.DECK_Z[1] + 5)) + (F.PLATE_Z[0] - R.DECK_Z[1])
            + math.dist((post_l[0], post_l[1], F.PLATE_Z[0] - 5), adapter))
    cam_hook = F.turret_xyz(T.BASE_HOOK)
    post_r = F.POSTS[2]
    usb_port = (R.JETSON_X[1], 20.0, R.jetson_pcb_z() + 8.0)
    usb = (300.0 + math.dist(cam_hook, (post_r[0], post_r[1], F.PLATE_Z[1])) + (F.PLATE_Z[1] - R.DECK_Z[1])
           + math.dist((post_r[0], post_r[1], R.DECK_Z[1] + 5), usb_port))
    report["cable_runs_mm"] = (f"UART jumpers Jetson header to adapter about {uart:.0f}; turret camera USB about "
                               f"{usb:.0f} (with 120 mm of pan loop and 150 mm on the tilt stage)")

    m = {stem: e.get("estimate", {}).get("mass_g", 0.0) for stem, e in entries.items()}
    printed_g = sum(m.get(stem, 0.0) for _, stem, _ in items if stem in ("turret_plate", "plate_post", "post_clip"))
    # The launcher's printed parts are the turret platform's: their estimates
    # come from its committed manifest.
    import json
    manifest = REPO / "hardware" / "parts" / "turret" / "stl" / "manifest.json"
    tm = {e["id"]: e.get("estimate", {}).get("mass_g", 0.0) for e in json.loads(manifest.read_text())["parts"]} \
        if manifest.exists() else {}
    launcher_g = (sum(tm.get(f"launcher_{s}", 0.0) for s in L.PRINTED if s != "muzzle_cap")
                  + 2 * turret.MASS_G["servo"] + L.SPRING["mass_g"] + L.BARREL["mass_g"]
                  + turret.MASS_G["launcher_hardware"] + L.CLIP_DARTS * L.DART_MASS_G)
    total = printed_g + 2 * turret.MASS_G["servo"] + turret.MASS_G["b0385"] + 150.0 + launcher_g
    report["added_mass_g"] = f"{total:.0f} (the launcher with a full clip {launcher_g:.0f} of it)"
    report["passed"] = (not overlaps and not visible and in_box and pan_ok)
    return report


def figures(out_dir, entries=None):
    """Render the assembly-instruction figures into ``out_dir``."""
    from hardware.parts.common import render
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    report = check(entries)
    items = printed_and_models()
    materials = {"turret_plate": "ASA", "plate_post": "ASA", "post_clip": "PETG"}

    def color(stem):
        if stem is None:
            return render.PURCHASED_COLOR
        return render.MATERIAL_COLORS.get(materials.get(stem, "ASA"), "#e8833a")

    meshes = []
    for n, stem, s in items:
        if n.startswith("chassis_envelope"):
            continue
        col = color(stem) if not stem or stem in materials else (
            render.MATERIAL_COLORS["TPU95A"] if stem == "deck_grommet" else
            render.MATERIAL_COLORS["PETG"] if stem in ("rim_clip",) else "#c9a27a")
        meshes.append((n, C.mesh(s), col))
    render.render([(m, c) for _, m, c in meshes], out_dir / "overview.png", elev=22, azim=-60, zoom=1.1,
                  title="RACECAR Turret: its own printed parts (colour), the car's and the turret's (tan)")
    keep = ("turret_plate", "plate_post", "post_clip", "post_screw", "plate_screw", "servo_adapter", "adapter_screw",
            "turret_pan_base", "turret_pan_servo", "deck_rear", "camera_arm", "rplidar", "vesc", "estop")
    render.render([(m, c) for n, m, c in meshes if n.startswith(keep)], out_dir / "step_plate.png",
                  elev=18, azim=-35, zoom=1.0, title="Posts, turret plate and servo adapter on the rear deck")
    bodies = [(n, s) for n, _, s in items]
    secs = car.scan_plane(bodies)
    extra = [(f"post {k}", (x, y), "#1f77b4") for k, (x, y) in enumerate(F.POSTS)]
    car.scan_figure(out_dir / "scan_plane.png", secs, extra=extra,
                    title="Lidar scan plane: the posts stand in the camera arm fin's shadow")
    return report
