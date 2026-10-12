"""Geometry shared by the two deck halves (helper, not a part)."""
from hardware.parts.racecarneo import _frame as F

INSERT = "M3-short"
TIE_SLOT = (2.6, 5.0)           # zip-tie slot (x, y)
TIE_PAIR_GAP = 9.0              # slot centres of a pair, across the cable
# Zip-tie pairs (x, y), each pair across a cable running along x.
TIE_PAIRS = ((-100.0, 60.0), (28.0, 58.0), (100.0, 50.0),
             (-100.0, -60.0), (28.0, -58.0), (100.0, -50.0))
LIGHTENING = (                  # (x0, x1, y0, y1) windows
    (140.0, 172.0, -16.0, 16.0),      # under the Jetson: air
    (-122.0, -72.0, -12.0, 12.0),     # under the VESC: air
    (55.0, 95.0, 38.0, 56.0), (55.0, 95.0, -56.0, -38.0),
)


def through_holes():
    return F.MAST_SCREWS + F.ARM_FOOT_SCREWS + F.LAP_SCREWS


def insert_holes():
    return (F.JETSON_TRAY_INSERTS + F.IMU_INSERTS + F.VESC_INSERTS + F.POWER_INSERTS
            + F.ESTOP_INSERTS + F.turret_post_inserts() + F.HALOW_SPACERS + F.ANTENNA_INSERTS)


def mount_holes():
    return F.MOUNTS["front"] + F.MOUNTS["rear"]


def half(x0, x1, lap):
    """One deck half between x0 and x1 (the lap included). ``lap`` is
    'top' (front half: upper layer over JOINT_X) or 'bottom' (rear half)."""
    from hardware.parts.common import cad, fasteners
    z0, z1 = F.DECK_Z
    w = F.DECK_HALF_W
    r = F.DECK_CORNER_R
    jx0, jx1 = F.JOINT_X
    # Full outline, rounded at the deck's own ends only.
    full = cad.rounded_plate(F.DECK_X[1] - F.DECK_X[0], 2 * w, z1 - z0, r,
                             center=((F.DECK_X[0] + F.DECK_X[1]) / 2, 0.0, z0))
    plate = full.intersect(cad.box(x0, x1, -w - 1, w + 1, z0 - 1, z1 + 1))
    rim_x0, rim_x1 = max(x0, F.DECK_X[0] + r), min(x1, F.DECK_X[1] - r)
    if lap == "top":
        # Front half: only the upper layer over the lap.
        plate = plate.cut(cad.box(jx0 - 1, jx1, -w - 1, w + 1, z0 - 1, F.LAP_Z))
        rim_x0 = jx0
    else:
        # Rear half: only the lower layer over the lap.
        plate = plate.cut(cad.box(jx0, jx1 + 1, -w - 1, w + 1, F.LAP_Z, z1 + 1))
        rim_x1 = jx0
    rims = [cad.box(rim_x0, rim_x1, *sorted((s * (w - F.RIM_W), s * w)), z1 - 0.01, z1 + F.RIM_H)
            for s in (1, -1)] if rim_x1 > rim_x0 else []
    body = cad.fuse_all([plate] + rims)

    def inside(x, y, pad=0.0):
        return x0 + pad <= x <= x1 - pad

    cutters = []
    for x, y in through_holes():
        if inside(x, y):
            cutters.append(cad.clearance_hole("M3", x, y, z0 - 1, z1 + F.RIM_H + 1))
    ins = fasteners.INSERTS[INSERT]
    for x, y in insert_holes():
        if inside(x, y):
            cutters.append(cad.cyl(ins.hole, z1 - z0 + 2, (x, y, z0 - 1)))
    for x, y in mount_holes():
        if inside(x, y):
            cutters.append(cad.cyl(F.GROMMET_HOLE, z1 - z0 + 2, (x, y, z0 - 1)))
    for x, y in TIE_PAIRS:
        if inside(x, y, 5.0):
            for dy in (-TIE_PAIR_GAP / 2, TIE_PAIR_GAP / 2):
                cutters.append(cad.cbox(TIE_SLOT[0], TIE_SLOT[1], 20, center=(x, y + dy, z0)))
    for wx0, wx1, wy0, wy1 in LIGHTENING:
        if x0 + 3 <= wx0 and wx1 <= x1 - 3:
            cutters.append(cad.rounded_plate(wx1 - wx0, wy1 - wy0, z1 - z0 + 2, 5.0,
                                             center=((wx0 + wx1) / 2, (wy0 + wy1) / 2, z0 - 1)))
    (wx0, wy0), (wx1, wy1) = F.WIRE_WINDOW
    if x0 + 3 <= wx0 and wx1 <= x1 - 3:
        cutters.append(cad.slot((wx0 + wx1) / 2, wy0 + 7, (wx0 + wx1) / 2, wy1 - 7, wx1 - wx0, z0 - 1, z1 + 1))
    return cad.cut_all(body, cutters)
