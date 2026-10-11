"""Assembly-check helpers shared by the RACECAR platforms (helper, not a
part): body placement, pairwise interference, the 2D lidar's scan plane and
camera fields of view. Imported by racecarneo/assembly.py and
racecar_turret/assembly.py; CadQuery is imported inside the functions.
"""
import math

OVERLAP_TOLERANCE_MM3 = 0.5
SCREW_SLACK = 0.02          # screw bodies are this much under nominal diameter


def screw(axis_point, direction, length, size, head=True, slack=SCREW_SLACK):
    """Socket head screw: head outside ``axis_point`` (opposite to
    ``direction``), shank ``length`` along ``direction``."""
    from hardware.parts.common import cad, fasteners
    d = fasteners.MAJOR[size] - slack
    dx, dy, dz = direction
    x, y, z = axis_point
    body = cad.cyl(d, length, (x, y, z), direction)
    if head:
        hd, hh = fasteners.SHCS_HEAD[size]
        body = body.fuse(cad.cyl(hd, hh, (x - dx * hh, y - dy * hh, z - dz * hh), direction))
    return body


def bbox_overlap(a, b, tol=0.01):
    return not (a.xmax < b.xmin - tol or b.xmax < a.xmin - tol or a.ymax < b.ymin - tol or
                b.ymax < a.ymin - tol or a.zmax < b.zmin - tol or b.zmax < a.zmin - tol)


def interferences(bodies, allowed_pairs=()):
    """Pairwise overlaps of [(name, shape)]. ``allowed_pairs`` is a list of
    (prefix_a, prefix_b, why) for intended press fits. Returns
    (overlaps, allowed) as lists of strings."""
    def allowed(n1, n2):
        for a, b, why in allowed_pairs:
            if (n1.startswith(a) and n2.startswith(b)) or (n1.startswith(b) and n2.startswith(a)):
                return why
        return None

    boxes = [(n, s, s.BoundingBox()) for n, s in bodies]
    overlaps, ok = [], []
    for i in range(len(boxes)):
        n1, s1, b1 = boxes[i]
        for j in range(i + 1, len(boxes)):
            n2, s2, b2 = boxes[j]
            if not bbox_overlap(b1, b2):
                continue
            v = s1.intersect(s2).Volume()
            if v > OVERLAP_TOLERANCE_MM3:
                why = allowed(n1, n2)
                if why:
                    ok.append(f"{n1}/{n2} {v:.0f} mm3 ({why})")
                else:
                    overlaps.append(f"{n1} / {n2}: {v:.1f} mm3")
    return overlaps, ok


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _seg_dist(p, a, b):
    """Distance from 2D point p to segment ab."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


def scan_plane_section(shape, centre, height, band, reach):
    """Where ``shape`` crosses the lidar's scan plane: the part of it within
    +-``band`` of ``height`` and within ``reach`` of ``centre`` (x, y).
    Returns None, or a dict with the angular sector it covers (degrees,
    counter-clockwise from +x, as seen from the lidar) and its nearest and
    farthest distance from the lidar axis (mm)."""
    from hardware.parts.common import cad
    cx, cy = centre
    slab = cad.cyl(2 * reach, 2 * band, (cx, cy, height - band))
    bb = shape.BoundingBox()
    if bb.zmax < height - band or bb.zmin > height + band:
        return None
    cut = shape.intersect(slab)
    if cut.Volume() < 1e-3:
        return None
    verts, tris = cut.tessellate(0.05, 0.1)
    pts = [(v.x - cx, v.y - cy) for v in verts]
    if not pts:
        return None
    # Angles relative to the direction of the section's centroid, so a
    # sector that straddles +-180 deg is handled.
    mx = sum(p[0] for p in pts) / len(pts)
    my = sum(p[1] for p in pts) / len(pts)
    ref = math.degrees(math.atan2(my, mx))
    rel = [_wrap(math.degrees(math.atan2(y, x)) - ref) for x, y in pts]
    a0, a1 = ref + min(rel), ref + max(rel)
    near = min(math.hypot(*p) for p in pts)
    for t in tris:
        for i, j in ((0, 1), (1, 2), (2, 0)):
            near = min(near, _seg_dist((0.0, 0.0), pts[t[i]], pts[t[j]]))
    far = max(math.hypot(*p) for p in pts)
    return {"from_deg": _wrap(a0), "to_deg": _wrap(a1), "width_deg": a1 - a0,
            "near_mm": near, "far_mm": far}


def sector_text(s):
    return f"{s['from_deg']:.1f} to {s['to_deg']:.1f} deg ({s['width_deg']:.1f} deg wide)"


def shadowed(inner, outer):
    """True when sector ``outer`` lies inside sector ``inner`` and behind it
    (the nearer body casts a shadow over the farther one)."""
    if outer["near_mm"] < inner["near_mm"]:
        return False
    a = _wrap(outer["from_deg"] - inner["from_deg"])
    b = _wrap(outer["to_deg"] - inner["from_deg"])
    return 0.0 <= a <= inner["width_deg"] and 0.0 <= b <= inner["width_deg"] and a <= b


def frustum(apex, forward, up, half_h_deg, half_v_deg, depth, start=0.5):
    """Pyramid of a camera's view: apex at the lens, ``forward`` the optical
    axis, ``up`` the image up direction (both unit 3-vectors), from
    ``start`` to ``depth`` mm."""
    import cadquery as cq
    f = cq.Vector(*forward).normalized()
    u = cq.Vector(*up).normalized()
    r = f.cross(u)                     # image right
    th, tv = math.tan(math.radians(half_h_deg)), math.tan(math.radians(half_v_deg))
    a = cq.Vector(*apex)

    def rect(d):
        c = a + f * d
        return [c + r * (sx * th * d) + u * (sy * tv * d) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]

    near, far = rect(start), rect(depth)
    faces = [cq.Face.makeFromWires(cq.Wire.makePolygon(near + [near[0]])),
             cq.Face.makeFromWires(cq.Wire.makePolygon(far + [far[0]]))]
    for i in range(4):
        j = (i + 1) % 4
        faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon([near[i], near[j], far[j], far[i], near[i]])))
    return cq.Solid.makeSolid(cq.Shell.makeShell(faces)).fix()


def in_view(bodies, fov, skip=()):
    """Bodies that enter the solid ``fov``: list of 'name (mm3)'."""
    fb = fov.BoundingBox()
    out = []
    for n, s in bodies:
        if any(n.startswith(p) for p in skip):
            continue
        if not bbox_overlap(fb, s.BoundingBox()):
            continue
        v = s.intersect(fov).Volume()
        if v > OVERLAP_TOLERANCE_MM3:
            out.append(f"{n} ({v:.0f} mm3)")
    return out


def mesh(shape, tol=0.25):
    import numpy as np
    import trimesh
    verts, tris = shape.tessellate(tol, 0.4)
    return trimesh.Trimesh(np.array([v.toTuple() for v in verts]), np.array(tris), process=False)
