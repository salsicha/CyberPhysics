"""Plate post: one of the four posts that carry the turret plate 108 mm
above the RACECAR deck. It narrows to 8 mm where it crosses the lidar's
scan plane (mid-height, where a post bent by the plate swaying carries the
least moment); all four are inside the rear sector the camera arm's fin
already hides from the lidar."""
from hardware.parts.common import printing
from hardware.parts.racecar_turret import _frame as F
from hardware.parts.racecarneo import _frame as R

PART = {
    "name": "Plate post",
    "quantity": 4,
    "material": "ASA",
    "material_reason": "outdoors in the sun; carries the turret",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: the flange on the plate", wall_loops=5, infill_pct=60),
    "mates_with": [
        {"part": "racecarneo deck_rear (printed)", "dimensions": f"two M3 x 3 inserts {2 * F.POST_INSERT_DX:g} mm apart "
                                                                  "along x at each post position",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
        {"part": "turret_plate (printed)", "dimensions": "M3 insert in the top end",
         "source": "hardware/parts/racecar_turret/_frame.py", "verified": True},
    ],
    "inserts": [{"size": "M3", "qty": 1, "where": "top end (plate screw)"}],
    "fasteners": [{"item": "M3 x 8 socket head cap screw", "qty": 2, "use": "plate post flange to the deck inserts"}],
    "fit_test": False,
    "notes": [
        "Print upright with five walls: the waist is the thinnest section, but the posts are fixed at both ends, so the "
        "bending moment from the plate swaying is smallest there.",
    ],
}


def build_local():
    """One post on the deck at the origin (x, y = 0), flange along x."""
    import cadquery as cq

    from hardware.parts.common import cad
    z0 = R.DECK_Z[1]
    z1 = F.PLATE_Z[0]
    fl, fw, ft = F.POST_FLANGE
    w0, w1 = F.POST_WAIST_Z
    r, rw = F.POST_D / 2, F.POST_WAIST_D / 2
    taper = r - rw                                  # 45 deg
    profile = [(0, z0 + ft - 0.01), (r, z0 + ft - 0.01), (r, w0 - taper), (rw, w0), (rw, w1), (r, w1 + taper),
               (r, z1), (0, z1)]
    column = cq.Workplane("XZ").polyline(profile).close().revolve(360, (0, 0, 0), (0, 1, 0)).val()
    flange = cad.rounded_plate(fl, fw, ft, 3.0, center=(0, 0, z0))
    post = flange.fuse(column).clean()
    cutters = [cad.insert_hole("M3", 0, 0, z1)]
    for dx in (F.POST_INSERT_DX, -F.POST_INSERT_DX):
        cutters.append(cad.clearance_hole("M3", dx, 0, z0 - 1, z0 + ft + 1))
    return cad.cut_all(post, cutters)


def build():
    from hardware.parts.common import cad
    x, y = F.POSTS[0]
    return cad.translate(build_local(), (x, y, 0))
