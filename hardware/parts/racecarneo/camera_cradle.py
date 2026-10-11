"""Camera cradle: holds the RealSense D435i or D455 by its 1/4"-20 thread
and pivots on the camera arm's tongue for pitch (+-10 deg, locked by a
second screw in an arc slot). The screw slot puts the camera's left imager
(the realsense-ros camera_link) on the car's centre line, as the URDF
has it."""
import math

from hardware.parts.common import fasteners, printing
from hardware.parts.racecarneo import _frame as F
from hardware.parts.racecarneo._footprints import RealSenseD435i, RealSenseD455

CHEEK_BOTTOM = F.ARM_FRONT_Z[0]      # 188, level with the tongue's underside
SLOT_W = 3.4

PART = {
    "name": "Camera cradle (RealSense D435i / D455, pitch adjust)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "touches the RealSense housing (it runs warm) and sits in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upside down: the camera seat on the plate, cheeks up",
                               wall_loops=4, infill_pct=40, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "RealSense D435i",
         "dimensions": f"1/4\"-20 in the bottom; left imager {RealSenseD435i.LEFT_IMAGER_FROM_SCREW:g} mm from the screw "
                       f"centre line; {RealSenseD435i.WIDTH:g} x {RealSenseD435i.HEIGHT:g} x {RealSenseD435i.DEPTH:g} mm",
         "source": RealSenseD435i.SOURCE, "verified": True},
        {"part": "RealSense D435i 1/4\"-20 position along the depth",
         "dimensions": f"{RealSenseD435i.SCREW_FROM_FRONT:g} mm behind the front face (realsense-ros, not the datasheet)",
         "source": RealSenseD435i.SOURCE, "verified": False},
        {"part": "RealSense D455",
         "dimensions": f"1/4\"-20 {RealSenseD455.SCREW_FROM_FRONT:g} mm behind the front face, left imager "
                       f"{RealSenseD455.LEFT_IMAGER_FROM_SCREW:g} mm from it; USB-C on the bottom "
                       f"{RealSenseD455.USB_FROM_SCREW:g} mm right of it",
         "source": RealSenseD455.SOURCE, "verified": True},
        {"part": "camera_arm (printed)", "dimensions": "12 mm tongue, M3 pivot and lock 11 mm behind it",
         "source": "hardware/parts/racecarneo/camera_arm.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "1/4\"-20 UNC x 3/8\" button head screw", "qty": 1,
         "use": "RealSense to the cradle (6 mm of thread in the camera; the D455 allows 9 mm)"},
        {"item": "M3 x 30 socket head cap screw", "qty": 2, "use": "camera cradle pivot and pitch lock, through the arm's tongue"},
        {"item": "M3 nyloc nut", "qty": 2, "use": "camera cradle pivot and pitch lock"},
        {"item": "M3 washer", "qty": 4, "use": "camera cradle pivot and pitch lock, both sides"},
    ],
    "fit_test": True,
    "notes": [
        "D435i: screw at the slot mark 17.5 mm right of the centre line; D455: at the far end, 47.5 mm right. "
        "Either way camera_link lands on the car's centre line.",
        "The D455 seat sits 2 mm low for the URDF (its imager is 14.5 mm above its base, the D435i's 12.5 mm): "
        "set camera_joint z to 0.222 m, or measure.",
        "Pitch: level for the URDF (rpy 0). Tighten the pivot nut only enough to hold, then lock with the second screw.",
        "D455: use a USB-C cable with a right-angle plug; its socket is on the bottom face.",
    ],
}


def _arc_slot(cx, cz, radius, a0, a1, width, y0, y1):
    """Arc slot about (cx, cz) in the x-z plane between angles a0..a1 (deg,
    from +x toward +z), through y0..y1."""
    import cadquery as cq
    ro, ri, h = radius + width / 2, radius - width / 2, width / 2

    def p(r, a, tangent=0.0):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        return (cx + r * c - tangent * s, cz + r * s + tangent * c)

    am = (a0 + a1) / 2
    wp = (cq.Workplane("XZ", origin=(0, y1, 0))
          .moveTo(*p(ro, a0))
          .threePointArc(p(ro, am), p(ro, a1))
          .threePointArc(p(radius, a1, h), p(ri, a1))
          .threePointArc(p(ri, am), p(ri, a0))
          .threePointArc(p(radius, a0, -h), p(ro, a0))
          .close().extrude(y1 - y0))
    return wp.val()


def cheek_y():
    """(y0, y1) of the two cheeks either side of the arm's tongue."""
    g, t = F.CHEEK_GAP, F.CHEEK_T
    ty0, ty1 = F.TONGUE_Y
    return ((ty0 - g - t, ty0 - g), (ty1 + g, ty1 + g + t))


def build():
    from hardware.parts.common import cad
    x0, x1 = F.CRADLE_X
    y0, y1 = F.CRADLE_Y
    top = F.CRADLE_SEAT_Z
    bot = top - F.CRADLE_T
    px, pz = F.PIVOT
    plate = cad.rounded_plate(x1 - x0, y1 - y0, F.CRADLE_T, 3.0, center=((x0 + x1) / 2, (y0 + y1) / 2, bot))
    lock_x = px - F.LOCK_R
    cheeks = []
    for ya, yb in cheek_y():
        outline = [(lock_x - 4.5, bot + 0.01), (x1 - 2.0, bot + 0.01), (x1 - 2.0, pz - 2.0),
                   (px + 4.0, CHEEK_BOTTOM), (lock_x - 2.0, CHEEK_BOTTOM), (lock_x - 4.5, pz - 2.0)]
        cheeks.append(cad.profile_xz(outline, ya, yb))
    body = cad.fuse_all([plate] + cheeks)

    cutters = []
    sy0, sy1 = F.SCREW_Y
    cutters.append(cad.slot(F.SCREW_X, sy0, F.SCREW_X, sy1, fasteners.CLEARANCE_1_4_20, bot - 1, top + 1))
    ya_lo = min(c[0] for c in cheek_y()) - 1
    ya_hi = max(c[1] for c in cheek_y()) + 1
    cutters.append(cad.cyl(3.4, ya_hi - ya_lo, (px, ya_lo, pz), (0, 1, 0)))
    r = F.PITCH_RANGE_DEG
    for ya, yb in cheek_y():
        cutters.append(_arc_slot(px, pz, F.LOCK_R, 180 - r, 180 + r, SLOT_W, ya - 0.5, yb + 0.5))
    return cad.cut_all(body, cutters)
