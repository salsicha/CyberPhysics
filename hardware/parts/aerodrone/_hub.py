"""Geometry shared by the two hub plates (helper, not a part)."""
import math

from hardware.parts.aerodrone import _frame as F


def arm(shape, angle):
    """Place a shape built along +x (the arm axis) on the arm at ``angle``."""
    from hardware.parts.common import cad
    return cad.rotate_z(shape, angle)


def outline(z0, z1):
    """Central 150 mm square plus an ear along each arm."""
    from hardware.parts.common import cad
    parts = [cad.box(-F.PLATE_HALF, F.PLATE_HALF, -F.PLATE_HALF, F.PLATE_HALF, z0, z1)]
    for a in F.ARM_ANGLES:
        ear = cad.box(0.0, F.EAR_LENGTH, -F.EAR_WIDTH / 2, F.EAR_WIDTH / 2, z0, z1)
        parts.append(arm(ear, a))
    plate = cad.fuse_all(parts)
    # Round the ear tips a little so they do not snag.
    return plate


def blocks(z0, z1):
    """The four clamp half-blocks between z0 and z1, without the bores."""
    from hardware.parts.common import cad
    out = []
    for a in F.ARM_ANGLES:
        b = cad.box(F.BLOCK_R[0], F.BLOCK_R[1], -F.BLOCK_W / 2, F.BLOCK_W / 2, z0, z1)
        out.append(arm(b, a))
    return out


def bores():
    """Tube bores: from the stop at TUBE_R0 out past the block end."""
    from hardware.parts.common import cad
    out = []
    for a in F.ARM_ANGLES:
        c = cad.cyl(F.TUBE_BORE, F.EAR_LENGTH + 5 - F.TUBE_R0, (F.TUBE_R0, 0, 0), (1, 0, 0))
        out.append(arm(c, a))
    return out


def clamp_bolt_xy():
    """Body (x, y) of the 16 clamp bolts."""
    pts = []
    for a in F.ARM_ANGLES:
        ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
        for r in F.CLAMP_BOLT_R:
            for lat in (F.CLAMP_BOLT_LAT, -F.CLAMP_BOLT_LAT):
                pts.append((r * ca - lat * sa, r * sa + lat * ca))
    return pts


def leg_bolt_xy():
    """Body (x, y) of the eight landing-leg inserts (on the arm axes)."""
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
            for a in F.ARM_ANGLES for r in F.LEG_BOLT_R]
