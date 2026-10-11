"""Boat assembly: the crossbar plate and camera mast on a model of a
BlueBoat crossbar (its 25 mm hole pitch is from Blue Robotics; its 40 x 40 mm
section is an assumption), with the screws, nuts and a camera envelope, and
the checks run by tools/build_printable_parts.py:

  * interference: no two bodies overlap (screws are modelled at their
    nominal diameter through the clearance holes, the crossbar holes and
    the inserts, so a misplaced hole shows up as an overlap);
  * crossbar screws: the M6 x 60 reaches through the nyloc nut for the
    modelled crossbar;
  * the camera's height above the crossbar.

``figures()`` renders the assembly-instruction images.
"""
from hardware.parts.boat import _frame as F
from hardware.parts.boat._footprints import BlueBoat, Iso4762M6, Iso7089M6Washer
from hardware.parts.common import fasteners

OVERLAP_TOLERANCE_MM3 = 0.5
SCREW_SLACK = 0.02
M6_LENGTH = 60.0
NYLOC_M6 = (10.0, 6.0)        # DIN 985: 10 mm across flats, 6 mm high
CAMERA_ENVELOPE = (60.0, 45.0, 45.0)   # a small IP67 camera or action camera (assumption)
BALL_HEAD = (30.0, 45.0)      # diameter, height (assumption)


def _screw(axis_point, direction, length, d, head):
    from hardware.parts.common import cad
    dx, dy, dz = direction
    x, y, z = axis_point
    hd, hh = head
    return cad.cyl(d - SCREW_SLACK, length, (x, y, z), direction).fuse(
        cad.cyl(hd, hh, (x - dx * hh, y - dy * hh, z - dz * hh), direction))


def crossbar():
    """The crossbar as a hollow-free box with 8 mm holes every 25 mm on the
    top and bottom faces (through)."""
    from hardware.parts.common import cad
    w, h = F.BAR_W, F.BAR_H
    bar = cad.box(-w / 2, w / 2, -300.0, 300.0, -h, 0.0)
    holes = [cad.cyl(BlueBoat.CROSSBAR_HOLE_D, h + 2, (0.0, k * F.PITCH, -h - 1)) for k in range(-11, 12)]
    return cad.cut_all(bar, holes)


def _purchased():
    from hardware.parts.common import cad
    out = [("crossbar", crossbar())]
    t = F.PLATE[2]
    head = Iso4762M6.HEAD
    for k, y in enumerate(F.PLATE_BOLTS_Y):
        seat = t - 4.0 + Iso7089M6Washer.H
        out.append((f"m6_screw_{k}", _screw((0.0, y, seat), (0, 0, -1), M6_LENGTH, 6.0, head)))
        out.append((f"m6_washer_top_{k}", cad.cyl(Iso7089M6Washer.D2, Iso7089M6Washer.H, (0.0, y, t - 4.0)).cut(
            cad.cyl(Iso7089M6Washer.D1, 4, (0.0, y, t - 5.0)))))
        zb = -F.BAR_H - Iso7089M6Washer.H
        out.append((f"m6_washer_bottom_{k}", cad.cyl(Iso7089M6Washer.D2, Iso7089M6Washer.H, (0.0, y, zb)).cut(
            cad.cyl(Iso7089M6Washer.D1, 4, (0.0, y, zb - 1)))))
        out.append((f"m6_nut_{k}", cad.cyl(NYLOC_M6[0], NYLOC_M6[1], (0.0, y, zb - NYLOC_M6[1])).cut(
            cad.cyl(6.0, NYLOC_M6[1] + 2, (0.0, y, zb - NYLOC_M6[1] - 1)))))
    z_fl = t + F.MAST_FLANGE[2]
    for k, (x, y) in enumerate(F.MAST_SCREWS):
        out.append((f"m4_screw_{k}", _screw((x, y, z_fl), (0, 0, -1), 12.0, fasteners.MAJOR["M4"],
                                            fasteners.SHCS_HEAD["M4"])))
    top = t + F.MAST_HEIGHT
    x, y = F.MAST_XY
    out.append(("stud", cad.cyl(6.3, 25.4, (x, y, top - 12.0))))
    out.append(("ball_head", cad.cyl(BALL_HEAD[0], BALL_HEAD[1], (x, y, top + 0.5)).cut(
        cad.cyl(6.5, 14.0, (x, y, top)))))          # its 1/4"-20 female base
    cx, cy, cz = CAMERA_ENVELOPE
    out.append(("camera", cad.box(x - cx / 2, x + cx / 2, y - cy / 2, y + cy / 2, top + 0.5 + BALL_HEAD[1],
                                  top + 0.5 + BALL_HEAD[1] + cz)))
    return out


def printed():
    from hardware.parts.boat import camera_mast, crossbar_plate
    return [("crossbar_plate", "crossbar_plate", crossbar_plate.build()),
            ("camera_mast", "camera_mast", camera_mast.build())]


def _bbox_overlap(a, b, tol=0.01):
    return not (a.xmax < b.xmin - tol or b.xmax < a.xmin - tol or a.ymax < b.ymin - tol or
                b.ymax < a.ymin - tol or a.zmax < b.zmin - tol or b.zmax < a.zmin - tol)


def check(entries=None):
    report = {}
    bodies = [(n, s) for n, _stem, s in printed()] + _purchased()
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
    t = F.PLATE[2]
    stack = (t - 4.0) + 2 * Iso7089M6Washer.H + F.BAR_H + NYLOC_M6[1]
    reach = M6_LENGTH - stack
    report["crossbar_screws"] = (f"M6 x {M6_LENGTH:g} passes {reach:.1f} mm through the nyloc nut on a "
                                 f"{F.BAR_H:g} mm crossbar (assumed height: measure yours)")
    report["camera_height_mm"] = f"{t + F.MAST_HEIGHT + BALL_HEAD[1]:.0f} from the crossbar top to the top of the ball head"
    report["passed"] = not overlaps and reach >= 1.0
    return report


def _mesh(shape, tol=0.25):
    import numpy as np
    import trimesh
    verts, tris = shape.tessellate(tol, 0.4)
    return trimesh.Trimesh(np.array([v.toTuple() for v in verts]), np.array(tris), process=False)


def figures(out_dir, entries=None):
    from pathlib import Path

    from hardware.parts.common import render
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    report = check(entries)
    materials = {stem: e.get("material", "ASA") for stem, e in (entries or {}).items()}
    pieces = printed()
    buy = _purchased()

    def items(offsets=None, skip=()):
        offsets = offsets or {}
        out = []
        for n, stem, s in pieces:
            m = _mesh(s)
            off = next((v for k, v in offsets.items() if n.startswith(k)), None)
            out.append((m.apply_translation(off) if off else m,
                        render.MATERIAL_COLORS.get(materials.get(stem, "ASA"), "#e8833a")))
        for n, s in buy:
            if any(n.startswith(p) for p in skip):
                continue
            m = _mesh(s)
            off = next((v for k, v in offsets.items() if n.startswith(k)), None)
            out.append((m.apply_translation(off) if off else m, render.PURCHASED_COLOR))
        return out

    render.render(items(), out_dir / "overview.png", elev=22, azim=-55, zoom=1.05,
                  title="Boat: crossbar plate and camera mast (colour) on a BlueBoat crossbar (grey)")
    render.render(items({"crossbar_plate": (0, 0, 40), "m6_screw": (0, 0, 80), "m6_washer_top": (0, 0, 60),
                         "camera_mast": (0, 0, 110), "m4_screw": (0, 0, 150), "stud": (0, 0, 170),
                         "ball_head": (0, 0, 190), "camera": (0, 0, 210), "m6_nut": (0, 0, -40),
                         "m6_washer_bottom": (0, 0, -25)}),
                  out_dir / "exploded.png", elev=15, azim=-60, zoom=0.95, title="Exploded view")
    return report
