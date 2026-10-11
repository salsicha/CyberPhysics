"""Post clip: snaps onto a plate post and holds the cables (camera USB,
servo bus, 12 V and UART leads) along it, away from the lidar."""
import math

from hardware.parts.common import printing
from hardware.parts.racecar_turret import _frame as F

WIDTH = 8.0
GRIP_ID = F.POST_D - 0.3
WALL = 2.2
OPENING_DEG = 100.0
RING_ID = 9.0
RING_WALL = 1.8
RING_OPENING_DEG = 70.0

PART = {
    "name": "Post cable clip",
    "quantity": 4,
    "material": "PETG",
    "material_reason": "flexes to snap on without cracking",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat: the clip profile on the plate", wall_loops=4, infill_pct=100),
    "mates_with": [
        {"part": "plate_post (printed)", "dimensions": f"{GRIP_ID:.1f} mm grip on the {F.POST_D:g} mm post",
         "source": "hardware/parts/racecar_turret/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": [
        "Snap onto the posts' thick sections (above or below the waist), ring facing inboard; one or two per post that "
        "carries cables.",
    ],
}


def _c_ring(cx, cy, ri, ro, opening_deg, toward_deg, z0, h):
    import cadquery as cq
    half = math.radians(opening_deg / 2)
    t = math.radians(toward_deg)
    a0, a1 = t + half, t - half + 2 * math.pi
    am = (a0 + a1) / 2

    def p(r, a):
        return (cx + r * math.cos(a), cy + r * math.sin(a))

    return (cq.Workplane("XY", origin=(0, 0, z0))
            .moveTo(*p(ro, a0)).threePointArc(p(ro, am), p(ro, a1))
            .lineTo(*p(ri, a1)).threePointArc(p(ri, am), p(ri, a0))
            .close().extrude(h)).val()


def build():
    gri = GRIP_ID / 2
    grip = _c_ring(0, 0, gri, gri + WALL, OPENING_DEG, 180.0, 0, WIDTH)
    rr = RING_ID / 2
    cxr = gri + WALL + rr - 0.5
    ring = _c_ring(cxr, 0, rr, rr + RING_WALL, RING_OPENING_DEG, 0.0, 0, WIDTH)
    return grip.fuse(ring).clean()
