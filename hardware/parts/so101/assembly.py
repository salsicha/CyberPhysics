"""SO-101 station assembly: the printed station parts placed with simple
datasheet-sized models of the purchased parts, and the checks run by
tools/build_printable_parts.py:

  * interference: no two bodies overlap (screws modelled at their nominal
    diameter through their holes), with the camera cradle swept through its
    whole pitch range against the camera head;
  * camera view: from the scene's camera pose, with the D435i's real colour
    field of view (69.4 x 42.5 deg, narrower vertically than the simulated
    640 x 480 camera's 54.5 deg), every bin and object of the scene is in
    the image, the camera sees no part of the station, and the pitch the
    head should be trimmed to for the best margin;
  * reach: the camera, head and post are out of the arm's reach;
  * the station board carries everything.

The arm's own printed parts are TheRobotStudio's proven design and are not
modelled here, except the base, which is placed from the vendored mesh for
the figures. ``figures()`` renders the assembly-instruction images.
"""
import itertools
import math

from hardware.parts.common import fasteners
from hardware.parts.so101 import _station as S
from hardware.parts.so101._footprints import Extrusion2020, RealSenseD435i, SO101Base, StationBoard

OVERLAP_TOLERANCE_MM3 = 0.5
SCREW_SLACK = 0.02
VIEW_MARGIN = 0.95             # every task item within 95 % of the half-image, both ways
BOARD_EDGE_BACK = 250.0        # arm's pan axis from the board's back edge (scene: 255 mm from the bench edge)
BOARD_EDGE_RIGHT = 640.0       # pan axis from the board's right edge
# Reach of the arm from the shoulder-lift axis, from systems/so101/urdf/so101.urdf:
# upper arm 140, forearm 130, wrist 55, gripper 52 + jaw 72.5 mm.
SHOULDER_LIFT_Z = 65.0 + 45.0
ARM_REACH = 140.0 + 130.0 + 55.0 + 52.0 + 72.5
ARM_COLOR = "#f2c14e"
PITCH_SWEEP = (S.PITCH_RANGE_DEG[0], S.PITCH_DEG, 48.0, 51.0, 54.0, S.PITCH_RANGE_DEG[1])

# Pairs (name prefixes) that overlap on purpose.
INTENDED = (
    ("camera_screw", "d435i", "M3 x 6 in the camera's thread, 2.5 mm"),
)
SLOT_DEPTH = 6.0               # modelled slot in the post (opening 6 mm); the clips' keys sit 1.5 mm in it


# --------------------------------------------------------------------- models

def _screw(axis_point, direction, length, size, head=True, slack=SCREW_SLACK):
    """Socket head screw: head outside ``axis_point`` (against
    ``direction``), shank ``length`` along ``direction``."""
    from hardware.parts.common import cad
    d = fasteners.MAJOR[size] - slack
    n = math.sqrt(sum(c * c for c in direction))
    dx, dy, dz = (c / n for c in direction)
    x, y, z = axis_point
    body = cad.cyl(d, length, (x, y, z), (dx, dy, dz))
    if head:
        hd, hh = fasteners.SHCS_HEAD[size]
        body = body.fuse(cad.cyl(hd, hh, (x - dx * hh, y - dy * hh, z - dz * hh), (dx, dy, dz)))
    return body


def camera_case():
    """The D435i case in the cradle frame: rounded ends seen from the front."""
    import cadquery as cq
    w, h, d = RealSenseD435i.WIDTH, RealSenseD435i.HEIGHT, RealSenseD435i.DEPTH
    return (cq.Workplane("YZ").slot2D(w, h, 0).extrude(d)).val()


def _cradle_purchased(pitch_deg=None):
    """Camera and the cradle's screws in the station frame."""
    out = [("d435i", S.place_cradle(camera_case(), pitch_deg))]
    for v in (-RealSenseD435i.BACK_M3_SPACING / 2, RealSenseD435i.BACK_M3_SPACING / 2):
        out.append((f"camera_screw_{v:+.0f}", S.place_cradle(_screw((-S.PLATE_T, v, 0.0), (1, 0, 0), 6.0, "M3"), pitch_deg)))
    pu = S.PIVOT_BACK
    half = S.CLEVIS_GAP / 2 + 5.0
    for name, w in (("pivot_screw", 0.0), ("lock_screw", -S.LOCK_R)):
        out.append((name, S.place_cradle(_screw((-pu, half, w), (0, -1, 0), 20.0, "M3"), pitch_deg)))
    return out


def _purchased():
    """[(name, shape)] for every purchased part in the model (camera at the
    scene pitch)."""
    from hardware.parts.common import cad
    out = []
    b = SO101Base
    out.append(("arm_base_wings", cad.box(b.FOOTPRINT_X[0], b.FOOTPRINT_X[1], b.FOOTPRINT_Y[0], b.FOOTPRINT_Y[1],
                                         b.UNDERSIDE_Z, b.UNDERSIDE_Z + b.WING_THICKNESS).cut(
        cad.fuse_all([cad.cyl(b.HOLE_D, 20, (x, y, b.UNDERSIDE_Z - 1)) for x, y in b.HOLES]))))
    top = b.UNDERSIDE_Z + b.WING_THICKNESS
    for k, (x, y) in enumerate(b.HOLES):
        out.append((f"arm_bolt_{k}", _screw((x, y, top), (0, 0, -1), S.ARM_BOLT_LENGTH, "M4")))
    px, py = S.POST_XY
    a = Extrusion2020.SIZE / 2
    post = cad.box(px - a, px + a, py - a, py + a, S.POST_BOTTOM, S.POST_TOP)
    slots = []
    zmid, length = (S.POST_BOTTOM + S.POST_TOP) / 2, S.POST_TOP - S.POST_BOTTOM + 2
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        depth = SLOT_DEPTH + 0.01
        centre = (px + dx * (a - SLOT_DEPTH / 2 + 0.005), py + dy * (a - SLOT_DEPTH / 2 + 0.005), zmid)
        size = (depth if dx else Extrusion2020.SLOT_W, depth if dy else Extrusion2020.SLOT_W, length)
        slots.append(cad.cbox(*size, center=centre))
    out.append(("post", cad.cut_all(post, slots)))
    from hardware.parts.so101 import camera_head as H
    from hardware.parts.so101 import post_foot as PF
    for name, z, wall in (("head_tnut_screw", S.POST_TOP - H.TNUT_SCREW_Z, H.SLEEVE_WALL),
                          ("foot_tnut_screw", S.FOOT_FLOOR + PF.TNUT_SCREW_H, PF.SOCKET_WALL)):
        for dx, dy in ((1, 0), (0, 1)):
            start = (px + dx * (S.POST_FIT / 2 + wall), py + dy * (S.POST_FIT / 2 + wall), z)
            out.append((f"{name}_{dx}{dy}", _screw(start, (-dx, -dy, 0), wall + 0.2 - 0.01, "M5", head=False)))
    out += _cradle_purchased()
    bx0 = -BOARD_EDGE_BACK
    by0 = -BOARD_EDGE_RIGHT
    out.append(("station_board", cad.box(bx0, bx0 + StationBoard.SIZE[1], by0, by0 + StationBoard.SIZE[0],
                                         S.TABLE_Z - StationBoard.THICKNESS, S.TABLE_Z)))
    return out


CLIP_Z = (120.0, 260.0, 400.0)     # cable clips on the post, above the board


def printed():
    """[(name, part id, shape)] for every printed piece in place."""
    from hardware.parts.common import cad
    from hardware.parts.so101 import (
        block_green,
        block_red,
        cable_clip,
        calib_board_foot,
        camera_cradle,
        camera_head,
        cylinder_blue,
        occluder_box,
        post_foot,
        station_base_plate,
        task_bin,
    )
    out = [(m.__name__.rsplit(".", 1)[1],) * 2 + (m.build(),)
           for m in (station_base_plate, camera_cradle, camera_head, post_foot)]
    px, py = S.POST_XY
    clip = cable_clip.build()
    for k, z in enumerate(CLIP_Z):
        # on the post's -y face (away from the arm), the hook outward, cable along z
        c = cad.rotate(clip, "X", 90)          # length along z, hook away from the post (local z -> -y)
        c = cad.translate(c, (px - cable_clip.SCREW_X, py - Extrusion2020.SIZE / 2, S.TABLE_Z + z))   # key in the slot
        out.append((f"cable_clip_{k}", "cable_clip", c))
    bin_shape = task_bin.build()
    for name in ("left_bin", "right_bin"):
        (cx, cy, _), size = S.scene_box(name)
        out.append((name, "task_bin", cad.translate(bin_shape, (cx, cy, S.TABLE_Z))))
    for mod, name in ((block_red, "red_block_target"), (block_green, "green_block_distractor"),
                      (cylinder_blue, "blue_cylinder_distractor"), (occluder_box, "occluder_box")):
        (cx, cy, _), _ = S.scene_box(name)
        yaw = math.degrees(S.SCENE[name].get("rpy", [0, 0, 0])[2])
        stem = mod.__name__.rsplit(".", 1)[1]
        out.append((stem, stem, cad.translate(cad.rotate_z(mod.build(), yaw), (cx, cy, S.TABLE_Z))))
    cb = S.world_to_station(S.SCENE["calibration_board"]["xyz"])
    foot = calib_board_foot.build()
    for k, dx in enumerate((-100.0, 100.0)):
        out.append((f"calib_board_foot_{k}", "calib_board_foot", cad.translate(foot, (cb[0] + dx, cb[1], S.TABLE_Z))))
    return out


# --------------------------------------------------------------------- checks

def _bbox_overlap(a, b, tol=0.01):
    return not (a.xmax < b.xmin - tol or b.xmax < a.xmin - tol or a.ymax < b.ymin - tol or
                b.ymax < a.ymin - tol or a.zmax < b.zmin - tol or b.zmax < a.zmin - tol)


def _intended(n1, n2):
    for a, b, why in INTENDED:
        if (n1.startswith(a) and n2.startswith(b)) or (n1.startswith(b) and n2.startswith(a)):
            return why
    return None


def _overlaps(bodies, skip=(), allowed=None):
    boxes = [(n, s, s.BoundingBox()) for n, s in bodies]
    hits = []
    for i in range(len(boxes)):
        n1, s1, b1 = boxes[i]
        for j in range(i + 1, len(boxes)):
            n2, s2, b2 = boxes[j]
            if (n1, n2) in skip or (n2, n1) in skip or not _bbox_overlap(b1, b2):
                continue
            v = s1.intersect(s2).Volume()
            if v > OVERLAP_TOLERANCE_MM3:
                why = _intended(n1, n2)
                if why and allowed is not None:
                    allowed.append(f"{n1}/{n2} ({why})")
                elif not why:
                    hits.append(f"{n1} / {n2}: {v:.1f} mm3")
    return hits


def view_coordinates(points, pitch_deg):
    """Normalised image coordinates (u, v; |u|, |v| <= 1 is in the image)
    and depth of station points seen by the D435i colour camera trimmed to
    ``pitch_deg``."""
    p = S.rgb_centre(pitch_deg)
    x, y, z = S.cam_axes(pitch_deg)
    th = math.tan(math.radians(RealSenseD435i.COLOR_FOV_DEG[0] / 2))
    tv = math.tan(math.radians(RealSenseD435i.COLOR_FOV_DEG[1] / 2))
    out = []
    for q in points:
        d = [q[k] - p[k] for k in range(3)]
        depth = sum(d[k] * x[k] for k in range(3))
        out.append((sum(d[k] * y[k] for k in range(3)) / depth / th,
                    sum(d[k] * z[k] for k in range(3)) / depth / tv, depth))
    return out


def task_view(pitch_deg):
    """{item: worst normalised coordinate of its corners} and the nearest depth."""
    worst, nearest = {}, float("inf")
    for name in S.TASK_ITEMS:
        (cx, cy, cz), (sx, sy, sz) = S.scene_box(name)
        corners = [(cx + ax * sx / 2, cy + ay * sy / 2, cz + az * sz / 2)
                   for ax, ay, az in itertools.product((-1, 1), repeat=3)]
        coords = view_coordinates(corners, pitch_deg)
        worst[name] = max(max(abs(u), abs(v)) for u, v, _ in coords)
        nearest = min(nearest, min(dep for _, _, dep in coords))
    return worst, nearest


def best_pitch():
    """Pitch in the head's range with the most margin around the task area."""
    lo, hi = S.PITCH_RANGE_DEG
    candidates = [lo + 0.5 + 0.25 * k for k in range(int((hi - lo - 1.0) / 0.25) + 1)]
    return min(candidates, key=lambda p: max(task_view(p)[0].values()))


def _frustum(pitch_deg, depth=1500.0, start=1.0):
    import cadquery as cq
    p = S.rgb_centre(pitch_deg)
    x, y, z = S.cam_axes(pitch_deg)
    th = math.tan(math.radians(RealSenseD435i.COLOR_FOV_DEG[0] / 2))
    tv = math.tan(math.radians(RealSenseD435i.COLOR_FOV_DEG[1] / 2))
    pts = []
    for d in (start, depth):
        for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
            pts.append(tuple(p[k] + d * (x[k] + su * th * y[k] + sv * tv * z[k]) for k in range(3)))
    near = cq.Wire.makePolygon([cq.Vector(*q) for q in pts[:4]], close=True)
    far = cq.Wire.makePolygon([cq.Vector(*q) for q in pts[4:]], close=True)
    return cq.Solid.makeLoft([near, far], True)


def _distance_to_box(point, bb):
    d = [max(lo - c, 0.0, c - hi) for c, lo, hi in zip(point, (bb.xmin, bb.ymin, bb.zmin), (bb.xmax, bb.ymax, bb.zmax))]
    return math.sqrt(sum(v * v for v in d))


def check(entries=None):
    """Run the assembly checks; returns a report dict ('passed' is the
    overall result)."""
    report = {}
    pieces = printed()
    bought = _purchased()
    bodies = [(n, s) for n, _, s in pieces] + bought
    allowed = []
    overlaps = _overlaps(bodies, allowed=allowed)
    report["bodies"] = len(bodies)
    report["interferences"] = overlaps or "none"
    report["intended_press_fits"] = len(allowed)

    # cradle and camera swept through the pitch range against the head
    from hardware.parts.so101 import camera_cradle, camera_head
    head = camera_head.build()
    cradle0 = camera_cradle.build()
    sweep_hits = []
    for pitch in PITCH_SWEEP:
        moving = [("camera_cradle", _turn(cradle0, pitch))] + _cradle_purchased(pitch)
        sweep_hits += [f"{h} at {pitch:g} deg" for h in _overlaps([("camera_head", head)] + moving,
                                                                  skip=_cradle_pairs(moving))
                       if "camera_head" in h]
    report["pitch_sweep"] = (sweep_hits or
                             f"clear from {PITCH_SWEEP[0]:g} to {PITCH_SWEEP[-1]:g} deg")

    worst, nearest = task_view(S.PITCH_DEG)
    out = [f"{n} ({w:.2f})" for n, w in worst.items() if w > 1.0]
    report["task_area_in_view_at_scene_pitch"] = (
        f"yes at {S.PITCH_DEG:.1f} deg (worst {max(worst.values()):.2f})" if not out else
        f"no at {S.PITCH_DEG:.1f} deg: {', '.join(out)} partly outside the D435i's 42.5 deg vertical view")
    bp = best_pitch()
    bworst, bnear = task_view(bp)
    report["recommended_pitch_deg"] = bp
    view_ok = max(bworst.values()) <= VIEW_MARGIN
    report["task_area_in_view_at_recommended_pitch"] = (
        f"{'yes' if view_ok else '**no**'} (worst {max(bworst.values()):.2f} of the half-image; "
        f"nearest item {bnear:.0f} mm, min-Z {RealSenseD435i.MIN_Z_MM:g} mm)")
    near_ok = bnear > RealSenseD435i.MIN_Z_MM and nearest > RealSenseD435i.MIN_Z_MM

    fov_hits = []
    for pitch in (S.PITCH_DEG, bp):
        fov = _frustum(pitch)
        fb = fov.BoundingBox()
        for n, s in bodies:
            if n.startswith(("d435i", "camera_screw", "station_board")) or n in S.TASK_ITEMS:
                continue
            if any(n.startswith(t) for t in ("task_bin", "block_", "cylinder_", "occluder", "calib", "arm_",
                                             "station_base_plate")):
                continue
            if _bbox_overlap(fb, s.BoundingBox()) and s.intersect(fov).Volume() > OVERLAP_TOLERANCE_MM3:
                fov_hits.append(f"{n} at {pitch:.1f} deg")
    report["camera_view_clear_of_station"] = fov_hits or "yes (cradle, head, post, foot and clips out of the image)"

    shoulder = (0.0, 0.0, SHOULDER_LIFT_Z)
    reach = {n: _distance_to_box(shoulder, s.BoundingBox()) for n, s in bodies
             if n.startswith(("camera_", "d435i", "post", "cable_clip"))}
    closest = min(reach, key=reach.get)
    reach_ok = reach[closest] > ARM_REACH
    report["out_of_arm_reach"] = (f"{'yes' if reach_ok else '**no**'} (nearest: {closest}, "
                                  f"{reach[closest]:.0f} mm from the shoulder-lift axis; reach {ARM_REACH:.0f} mm)")

    board = dict(bought)["station_board"].BoundingBox()
    off_board = [n for n, s in bodies if n != "station_board" and
                 (s.BoundingBox().xmin < board.xmin or s.BoundingBox().xmax > board.xmax or
                  s.BoundingBox().ymin < board.ymin or s.BoundingBox().ymax > board.ymax)]
    report["on_the_station_board"] = (off_board or
                                      f"yes ({StationBoard.SIZE[0]:g} x {StationBoard.SIZE[1]:g} mm board, arm "
                                      f"{BOARD_EDGE_BACK:g} mm from its back edge, {BOARD_EDGE_RIGHT:g} mm from its right edge)")
    report["camera_rgb_at_recommended_pitch_mm"] = [round(v, 1) for v in S.rgb_centre(bp)]
    report["passed"] = (not overlaps and not sweep_hits and view_ok and near_ok and not fov_hits and reach_ok
                        and not off_board)
    return report


def _turn(shape, pitch_deg):
    from hardware.parts.common import cad
    if abs(pitch_deg - S.PITCH_DEG) < 1e-9:
        return shape
    return cad.rotate(shape, S.cam_axes()[1], pitch_deg - S.PITCH_DEG, about=S.PIVOT)


def _cradle_pairs(moving):
    """Pairs within the moving cradle group (checked at the scene pitch)."""
    names = [n for n, _ in moving]
    return {(a, b) for a in names for b in names}


# --------------------------------------------------------------------- figures

def _mesh(shape, tol=0.25):
    import numpy as np
    import trimesh
    verts, tris = shape.tessellate(tol, 0.4)
    return trimesh.Trimesh(np.array([v.toTuple() for v in verts]), np.array(tris), process=False)


def arm_base_mesh():
    """The vendored SO-101 base placed in the station frame (for figures):
    its mesh x, y, z are the station's y, z, x (see _footprints.SO101Base)."""
    import numpy as np
    import trimesh

    from hardware.parts.so101 import base_so101
    m = trimesh.load(str(_parts_dir() / base_so101.PART["vendored"]["file"]), force="mesh")
    t = np.array([[0, 0, 1, -45.2], [1, 0, 0, 0], [0, 1, 0, SO101Base.UNDERSIDE_Z], [0, 0, 0, 1]], dtype=float)
    return m.copy().apply_transform(t)


def _parts_dir():
    from pathlib import Path
    return Path(__file__).resolve().parent


def figures(out_dir, entries=None):
    """Render the assembly-instruction figures into ``out_dir``."""
    from pathlib import Path

    from hardware.parts.common import render
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    report = check(entries)
    materials = {stem: e.get("material", "PETG") for stem, e in (entries or {}).items()}
    pieces = printed()
    bought = [(n, s) for n, s in _purchased() if n != "station_board"]
    cache = {}

    def mesh(name, shape):
        if name not in cache:
            cache[name] = _mesh(shape)
        return cache[name]

    def sel(prefixes, purchased_prefixes=(), arm=False, colours=None, extra=()):
        items = []
        for n, stem, s in pieces:
            if any(n.startswith(p) for p in prefixes):
                c = (colours or {}).get(n) or render.MATERIAL_COLORS.get(materials.get(stem, "PETG"))
                items.append((mesh(n, s), c))
        for n, s in bought:
            if any(n.startswith(p) for p in purchased_prefixes) and not (arm and n == "arm_base_wings"):
                items.append((mesh(n, s), render.PURCHASED_COLOR))
        for n, s in extra:
            items.append((mesh(n, s), render.PURCHASED_COLOR))
        if arm:
            items.append((arm_base_mesh(), ARM_COLOR))
        return items

    every = ("",)
    render.render(sel(every, every, arm=True), out_dir / "overview.png", elev=28, azim=-140, zoom=1.2,
                  title="SO-101 station: printed (blue), purchased (grey), upstream arm base (yellow)")
    render.render(sel(("station_base_plate",), ("arm_bolt",), arm=True), out_dir / "step_base.png",
                  elev=30, azim=-130, title="Base plate, arm base and its four M4 bolts")
    from hardware.parts.common import cad
    px, py = S.POST_XY
    a = Extrusion2020.SIZE / 2
    stub = cad.box(px - a, px + a, py - a, py + a, S.POST_TOP - 70.0, S.POST_TOP)
    render.render(sel(("camera_cradle", "camera_head"), ("d435i", "camera_screw", "pivot", "lock", "head_tnut"),
                      extra=[("post_stub", stub)]),
                  out_dir / "step_camera_head.png", elev=12, azim=-35, zoom=1.0,
                  title="Camera head on the post, cradle on its pivot, D435i on the cradle")
    render.render(sel(("post_foot", "cable_clip"), ("post", "foot_tnut")), out_dir / "step_post.png",
                  elev=20, azim=-35, zoom=1.0, title="Post foot, 2020 post and cable clips")
    colours = {"left_bin": "#2f62c8", "right_bin": "#c8372f", "block_red": "#c8372f", "block_green": "#3a9a4a",
               "cylinder_blue": "#2f62c8", "occluder_box": "#8a8a84", "station_base_plate": "#3d3d3d"}
    render.render(sel(("left_bin", "right_bin", "block_", "cylinder_", "occluder", "calib", "station_base_plate"),
                      (), arm=True, colours=colours),
                  out_dir / "step_task.png", elev=40, azim=-150, title="Task fixtures at their scene positions")
    return report
