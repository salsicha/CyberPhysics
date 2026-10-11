"""Shared wheel geometry for the lunar rover (helper, not a part).

Wheel-local frame: y along the axle, outward (u), z up, x forward; origin
on the axle at the actuator's output face. Angles of wheel features are
measured about the axle, in the x-z plane, from +x toward +z.
"""
import math

from hardware.parts.lunar_rover import _frame as F


def sector(r0, r1, y0, y1, a0_deg, a1_deg):
    """Annular sector about the y axis: radii r0..r1, axial y0..y1, angles
    a0..a1 (degrees, from +x toward +z)."""
    import cadquery as cq

    from hardware.parts.common import cad
    ring = cad.cyl(2 * r1, y1 - y0, (0, y0, 0), (0, 1, 0))
    if r0 > 0:
        ring = ring.cut(cad.cyl(2 * r0, y1 - y0 + 2, (0, y0 - 1, 0), (0, 1, 0)))
    big = 2.0 * r1 + 10.0
    pts = [(0.0, 0.0)]
    n = max(2, int(abs(a1_deg - a0_deg) / 5) + 1)
    for k in range(n + 1):
        a = math.radians(a0_deg + (a1_deg - a0_deg) * k / n)
        pts.append((big * math.cos(a), big * math.sin(a)))
    wedge = (cq.Workplane("XZ", origin=(0, y1 + 1, 0)).polyline(pts).close().extrude(y1 - y0 + 2)).val()
    return ring.intersect(wedge)


def polar_xz(r, a_deg, y):
    a = math.radians(a_deg)
    return (r * math.cos(a), y, r * math.sin(a))


def grouser_angles():
    """Grouser angles of the segment centred on 0 deg."""
    half = F.SEGMENT_DEG / 2
    n = F.GROUSERS_PER_SEGMENT
    return [-half + F.SEGMENT_DEG * (k + 0.5) / n for k in range(n)]


GROUSER_BOLT_Y = (34.0, 94.0)            # along the axle, both grousers
END_BOLTS = ((158.0, 40.0), (158.0, 90.0))   # (r, y) of the bolts through the end walls
