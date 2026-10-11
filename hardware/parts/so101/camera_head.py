"""Camera head: sits on the top of the 2020 camera post and carries the
camera cradle on a pivot, with an arc slot to lock the pitch (38-57 deg).
The tongue points along the scene camera's heading (83 deg left of the
arm's x axis); the sleeve is square to the station."""
import math

from hardware.parts.common import fasteners, printing
from hardware.parts.so101 import _station as S
from hardware.parts.so101._footprints import Extrusion2020

SLEEVE_WALL = 5.0
TNUT_SCREW_Z = 18.0                  # below the post top
# Tongue outline in the head frame (x along the heading, z up; pivot at 0, 0)
TONGUE_OUTLINE = ((-20.0, S.HEAD_CAP_TOP - S.PIVOT[2] - 0.5), (-5.0, S.HEAD_CAP_TOP - S.PIVOT[2] - 0.5),
                  (-5.0, -22.0), (6.0, -3.0), (5.0, 4.0), (-6.0, 7.0), (-19.0, 0.0), (-20.0, -8.0))
PIVOT_BOSS_D = 14.0
FOOT = (-20.0, -5.0, 9.0, 6.0)        # tongue foot: x from, x to, half-width, height above the cap

PART = {
    "name": "Camera head (post top, pitch pivot)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "indoors; PETG's toughness for the thin tongue",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "upright: the open end of the sleeve on the plate, tongue up", wall_loops=4,
                               infill_pct=40),
    "mates_with": [
        {"part": "2020 aluminium extrusion (camera post)",
         "dimensions": f"{S.POST_FIT:g} mm square sleeve, {S.SLEEVE_DEPTH:g} mm deep, two M5 into drop-in T-nuts "
                       "(slot 6); optional M5 into the tapped 4.2 mm centre hole",
         "source": Extrusion2020.SOURCE, "verified": True},
        {"part": "camera_cradle (printed)", "dimensions": f"{S.TONGUE_T} mm tongue in its {S.CLEVIS_GAP} mm clevis",
         "source": "hardware/parts/so101/camera_cradle.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 10 button head screw + M5 drop-in T-nut (slot 6)", "qty": 2, "use": "head to the camera post"},
    ],
    "fit_test": False,
    "notes": [
        "The pivot and lock screws are listed on camera_cradle.",
        "Optional: tap the post's centre hole M5 and add an M5 x 12 through the cap.",
    ],
}


def pivot_slot_arc():
    """(start, end) angles, degrees, of the lock screw's arc around the
    pivot in the head frame, measured from -z toward -x."""
    return S.PITCH_RANGE_DEG


def build():
    from hardware.parts.common import cad
    px, py = S.POST_XY
    z_top, z_bot = S.HEAD_CAP_TOP, S.POST_TOP - S.SLEEVE_DEPTH
    half = S.POST_FIT / 2 + SLEEVE_WALL
    sleeve = cad.box(px - half, px + half, py - half, py + half, z_bot, z_top)
    bore = cad.box(px - S.POST_FIT / 2, px + S.POST_FIT / 2, py - S.POST_FIT / 2, py + S.POST_FIT / 2,
                   z_bot - 1, S.POST_TOP)

    # tongue and its foot, drawn in the head frame, then placed
    t2 = S.TONGUE_T / 2
    tongue = cad.profile_xz(list(TONGUE_OUTLINE), -t2, t2)
    tongue = tongue.fuse(cad.cyl(PIVOT_BOSS_D, S.TONGUE_T, (0, -t2, 0), (0, 1, 0)))
    cap_z = S.HEAD_CAP_TOP - S.PIVOT[2]
    tongue = tongue.fuse(cad.box(FOOT[0], FOOT[1], -FOOT[2], FOOT[2], cap_z - 0.5, cap_z + FOOT[3]))
    a0, a1 = (math.radians(a) for a in pivot_slot_arc())
    r, w = S.LOCK_R, fasteners.CLEARANCE["M3"]
    ends = [(-r * math.sin(a), -r * math.cos(a)) for a in (a0, a1)]
    holes = [cad.cyl(w, S.TONGUE_T + 2, (0, -t2 - 1, 0), (0, 1, 0)),
             cad.profile_xz(_arc_band(r, w, a0, a1), -t2 - 1, t2 + 1)]
    holes += [cad.cyl(w, S.TONGUE_T + 2, (x, -t2 - 1, z), (0, 1, 0)) for x, z in ends]
    tongue = cad.cut_all(tongue, holes)
    head = cad.fuse_all([sleeve, S.place_head(tongue)])
    cutters = [bore]
    z = S.POST_TOP - TNUT_SCREW_Z
    for dx, dy in ((1, 0), (0, 1)):
        start = (px + dx * (half + 1), py + dy * (half + 1), z)
        cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], SLEEVE_WALL + 2, start, (-dx, -dy, 0)))
    cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], S.HEAD_CAP_T + 2, (px, py, S.POST_TOP - 1)))
    return cad.cut_all(head, cutters)


def _arc_band(r, w, a0, a1, n=12):
    """Closed (x, z) outline of an arc slot of width w on radius r between
    angles a0 and a1 (radians, from -z toward -x)."""
    outer = [(-(r + w / 2) * math.sin(a0 + (a1 - a0) * k / n), -(r + w / 2) * math.cos(a0 + (a1 - a0) * k / n))
             for k in range(n + 1)]
    inner = [(-(r - w / 2) * math.sin(a1 - (a1 - a0) * k / n), -(r - w / 2) * math.cos(a1 - (a1 - a0) * k / n))
             for k in range(n + 1)]
    return outer + inner
