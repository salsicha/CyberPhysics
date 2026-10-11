"""Shared builder for the printed task objects (helper, not a part)."""
from hardware.parts.so101 import _station as S

CHAMFER = 1.0


def box_object(name):
    """A scene box object with chamfered edges, standing on z = 0."""
    import cadquery as cq
    sx, sy, sz = S.scene_box(name)[1]
    return (cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, False)).edges().chamfer(CHAMFER)).val()


def scene_mass_g(name):
    return 1000.0 * S.SCENE[name]["mass_kg"]
