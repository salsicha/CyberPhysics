"""Post adapter geometry shared by the front and rear adapters (helper, not
a part): a sleeve clamped on a chassis body post, an arm to the deck
mounting point and a seat with an M3 insert for the deck screw."""
from hardware.parts.racecarneo import _frame as F

ARM_T = 10.0
ARM_W = 12.0
SLEEVE_OD = 13.0
BORE_CLEARANCE = 0.3
SEAT_D = 14.0
SLIT = 1.5
EAR_T = 6.0
EAR_OUT = 12.5                      # ears reach this far from the post axis
PINCH_R = 9.5                       # pinch screw distance from the post axis
PINCH_Z_FROM_BOTTOM = 4.0
EAR_TOP_FROM_ARM = 2.0              # the ears' 45 deg top ends this far under the arm (an overhang-free print)


def build(which):
    """The left adapter of the pair ``which`` ('front' or 'rear') in the
    vehicle frame; the right one is the same part turned 180 deg."""
    from hardware.parts.common import cad
    (px, py), (mx, my) = F.POSTS[which][0], F.MOUNTS[which][0]
    top = F.SEAT_Z
    z_arm = top - ARM_T
    z_sleeve = top - F.SLEEVE_LEN
    parts = [cad.cyl(SLEEVE_OD, F.SLEEVE_LEN, (px, py, z_sleeve)),
             cad.cyl(SEAT_D, ARM_T, (mx, my, z_arm)),
             cad.slot(px, py, mx, my, ARM_W, z_arm, top)]
    # Pinch ears on the side of the sleeve away from the arm; their top is a
    # 45 deg chamfer, so the part prints upside down without supports.
    sgn = -1.0 if my >= py else 1.0
    r0 = 0.0
    z_top = z_arm - EAR_TOP_FROM_ARM
    full = z_top - (EAR_OUT - SLEEVE_OD / 2)
    profile = [(r0, z_sleeve), (EAR_OUT, z_sleeve), (EAR_OUT, full), (SLEEVE_OD / 2, z_top), (r0, z_top)]
    ear = cad.profile_xz([(u, z) for u, z in profile], -EAR_T - SLIT / 2, EAR_T + SLIT / 2)
    # profile_xz builds in x-z: turn it so u runs along +-y from the post.
    ear = cad.rotate(ear, "Z", 90.0 if sgn > 0 else -90.0)
    parts.append(cad.translate(ear, (px, py, 0)))
    body = cad.fuse_all(parts)
    bore_top = z_arm - 2.0 if (px, py) == (mx, my) else top + 1
    pinch_y = py + sgn * PINCH_R
    cutters = [
        cad.cyl(F.POST_D + 2 * BORE_CLEARANCE, bore_top - z_sleeve + 1, (px, py, z_sleeve - 1)),
        cad.insert_hole("M3", mx, my, top),
        # slit through the ears and the sleeve wall, up to under the arm
        cad.box(px - SLIT / 2, px + SLIT / 2, *sorted((py, py + sgn * (EAR_OUT + 1))), z_sleeve - 1, z_arm - 1.0),
        cad.cyl(3.4, 2 * EAR_T + SLIT + 2, (px - EAR_T - SLIT / 2 - 1, pinch_y, z_sleeve + PINCH_Z_FROM_BOTTOM), (1, 0, 0)),
    ]
    return cad.cut_all(body, cutters)


def part(which):
    from hardware.parts.common import printing
    (px, py), (mx, my) = F.POSTS[which][0], F.MOUNTS[which][0]
    return {
        "name": f"Post adapter, {which} (chassis body post to deck)",
        "quantity": 2,
        "material": "ASA",
        "material_reason": "outdoors, over the warm chassis; carries the deck",
        "material_fallback": None,
        "flight_critical": False,
        "print": printing.settings("ASA", "upside down: the seat face on the plate, sleeve up",
                                   wall_loops=5, infill_pct=50, rotate=[("X", 180)]),
        "mates_with": [
            {"part": "Traxxas Slash 4x4 body post",
             "dimensions": f"round post dia. {F.POST_D:g} mm at ({px:g}, +-{abs(py):g}) mm; the sleeve clamps anywhere "
                           f"along {F.SLEEVE_LEN:g} mm",
             "source": "not published by Traxxas: measure the posts on your chassis and set POSTS and POST_D in "
                       "hardware/parts/racecarneo/_frame.py", "verified": False},
            {"part": "deck halves through deck_grommet (printed)",
             "dimensions": f"M3 insert at the deck mounting point ({mx:g}, +-{abs(my):g}) mm",
             "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
        ],
        "inserts": [{"size": "M3", "qty": 1, "where": "seat, from the top (deck screw)"}],
        "fasteners": [
            {"item": "M3 x 20 socket head cap screw", "qty": 1, "use": f"post adapter pinch clamp ({which})"},
            {"item": "M3 nyloc nut", "qty": 1, "use": f"post adapter pinch clamp ({which})"},
        ],
        "fit_test": True,
        "notes": [
            "Slide the sleeve over the body post, set the deck height (deck top 132 mm above the ground with the car on its "
            "wheels, which puts the lidar's scan plane at the URDF's 180 mm), then tighten the pinch screw.",
            "If your chassis has Traxxas' clipless latching body mounts (2020 and later Slash 4x4 kits) instead of round "
            "posts, this adapter does not fit: measure the mounts and adapt the sleeve.",
        ],
    }
