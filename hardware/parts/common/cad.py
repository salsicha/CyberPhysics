"""Small CadQuery helpers shared by the part modules.

Import this inside ``build()`` (it imports CadQuery). Everything works on
``cq.Shape`` objects in millimetres; the frame of each helper is the frame
of the coordinates passed in.
"""
import math

import cadquery as cq

from . import fasteners


def V(x, y=None, z=None):
    if y is None:
        x, y, z = x
    return cq.Vector(float(x), float(y), float(z))


def box(x0, x1, y0, y1, z0, z1):
    """Axis-aligned box from its extents."""
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cbox(sx, sy, sz, center=(0, 0, 0)):
    """Axis-aligned box from its size and centre."""
    cx, cy, cz = center
    return box(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, cz - sz / 2, cz + sz / 2)


def cyl(d, h, base=(0, 0, 0), axis=(0, 0, 1)):
    """Cylinder of diameter d and length h from ``base`` along ``axis``."""
    return cq.Solid.makeCylinder(d / 2.0, h, V(base), V(axis).normalized())


def cone(d0, d1, h, base=(0, 0, 0), axis=(0, 0, 1)):
    return cq.Solid.makeCone(d0 / 2.0, d1 / 2.0, h, V(base), V(axis).normalized())


def rounded_plate(sx, sy, t, r, center=(0, 0, 0)):
    """Plate sx x sy x t (z from center z) with vertical corner radius r."""
    cx, cy, cz = center
    wp = cq.Workplane("XY").workplane(offset=cz).center(cx, cy).rect(sx, sy).extrude(t)
    if r > 0:
        wp = wp.edges("|Z").fillet(r)
    return wp.val()


def polygon_prism(points, z0, z1):
    """Prism of a closed XY polygon between z0 and z1."""
    wp = cq.Workplane("XY").workplane(offset=z0).polyline(points).close().extrude(z1 - z0)
    return wp.val()


def profile_xz(points, y0, y1):
    """Prism of a closed polygon given in (x, z), extruded from y0 to y1."""
    wp = (cq.Workplane("XZ", origin=(0, y1, 0)).polyline(points).close()
          .extrude(y1 - y0))
    return wp.val()


def slot(x0, y0, x1, y1, width, z0, z1):
    """Stadium slot between two centre points, through z0..z1."""
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    angle = math.degrees(math.atan2(dy, dx))
    wp = (cq.Workplane("XY").workplane(offset=z0).center((x0 + x1) / 2, (y0 + y1) / 2)
          .slot2D(length + width, width, angle).extrude(z1 - z0))
    return wp.val()


def insert_hole(size, x, y, z, direction=-1):
    """Heat-set insert hole (CNC Kitchen D2, depth L + 1) entering the face
    at height z and going down (direction=-1) or up (+1)."""
    ins = fasteners.INSERTS[size]
    depth = ins.hole_depth
    z0 = z - depth if direction < 0 else z
    return cyl(ins.hole, depth + 0.01, (x, y, z0 - (0.01 if direction < 0 else 0)))


def insert_hole_axis(size, entry, direction):
    """Heat-set insert hole entering at point ``entry`` along ``direction``
    (pointing into the material)."""
    ins = fasteners.INSERTS[size]
    d = V(direction).normalized()
    start = V(entry) - d * 0.01
    return cq.Solid.makeCylinder(ins.hole / 2.0, ins.hole_depth + 0.01, start, d)


def clearance_hole(size, x, y, z0, z1):
    """ISO 273 medium clearance hole along z."""
    d = fasteners.CLEARANCE[size]
    return cyl(d, z1 - z0, (x, y, z0))


def fuse_all(shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.clean()


def cut_all(shape, cutters):
    if not cutters:
        return shape
    tool = cutters[0]
    for c in cutters[1:]:
        tool = tool.fuse(c)
    return shape.cut(tool).clean()


def rotate_z(shape, deg):
    return shape.rotate(V(0, 0, 0), V(0, 0, 1), deg)


def rotate(shape, axis, deg, about=(0, 0, 0)):
    axes = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}
    a = axes[axis] if isinstance(axis, str) else axis
    o = V(about)
    return shape.rotate(o, o + V(a), deg)


def translate(shape, xyz):
    return shape.translate(V(xyz))


def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


def single_solid(shape):
    """The one solid of ``shape``; raises if there are none or several."""
    solids = shape.Solids() if hasattr(shape, "Solids") else []
    if len(solids) != 1:
        raise ValueError(f"expected one solid, got {len(solids)}")
    return solids[0]
