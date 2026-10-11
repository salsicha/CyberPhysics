"""Camera arm: carries the RealSense 100 mm ahead of and 40 mm above the
lidar (the URDF's camera_link) without putting anything in the lidar's
scan plane where the lidar can see it. Its fin rises behind the C1, 31-44
mm from the lidar axis (inside the C1's 50 mm blind range), a deep boom
passes over the lidar, and a tongue in front of it carries the camera
cradle's pitch pivot."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F
from hardware.parts.racecarneo._footprints import RplidarC1

TIE_SLOT = (2.6, 5.0)                       # zip-tie slots through the arm (x, z)
TIE_SLOTS = ((62.0, 230.0), (118.0, 230.0), (140.0, 230.0), (42.5, 172.0))

PART = {
    "name": "Camera arm (RealSense over the lidar)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on top of the car in the sun; carries the camera, so it must not creep",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings(
        "ASA", "on its side: the flat +y face (the tongue side) on the plate",
        wall_loops=5, infill_pct=40, rotate=[("X", -90)]),
    "mates_with": [
        {"part": "Slamtec RPLIDAR C1 (clearance)",
         "dimensions": f"fin {F.LIDAR_XY[0] - F.FIN_X[1]:g}-{F.LIDAR_XY[0] - F.FIN_X[0]:g} mm behind the lidar axis, "
                       f"inside its {RplidarC1.MIN_RANGE:g} mm blind range; boom {F.BOOM_Z[0] - F.LIDAR_BOTTOM_Z - RplidarC1.HEIGHT:.1f} mm "
                       "over its top",
         "source": RplidarC1.SOURCE, "verified": True},
        {"part": "lidar_mast (printed)", "dimensions": "foot on the mast flange, two M3 through flange and deck",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
        {"part": "camera_cradle (printed)", "dimensions": "M3 pivot and M3 lock through the tongue",
         "source": "hardware/parts/racecarneo/camera_cradle.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 25 socket head cap screw", "qty": 2, "use": "camera arm foot through the lidar mast flange and the deck"},
        {"item": "M3 nyloc nut", "qty": 2, "use": "under the deck, camera arm foot screws"},
        {"item": "M3 washer", "qty": 4, "use": "camera arm foot screws, both ends"},
        {"item": "zip tie 2.5 mm", "qty": 4, "use": "RealSense USB cable along the camera arm"},
    ],
    "fit_test": False,
    "notes": [
        "The arm is the only printed part in the lidar's scan plane: it hides a rear sector of about +-23 deg "
        "(the assembly check reports it) and returns no ranges because it is inside the C1's 50 mm blind range.",
        "Run the RealSense cable down the arm's ties and the fin, never in front of or beside the lidar.",
    ],
}


def profile():
    """The arm's outline in (x, z)."""
    fx0, fx1 = F.FIN_X
    foot0, foot_top = F.ARM_FOOT_X[0], F.DECK_Z[1] + F.MAST_FLANGE_T + F.ARM_FOOT_T
    base = F.DECK_Z[1] + F.MAST_FLANGE_T
    b0, b1 = F.BOOM_Z
    t0, t1 = F.ARM_FRONT_Z
    lidar_front = F.LIDAR_XY[0] + RplidarC1.BASE / 2 + 4.0       # 111.8
    return [
        (foot0, base), (fx1, base),                 # foot underside on the flange
        (fx1, b0),                                  # fin front face, up behind the C1
        (lidar_front, b0),                          # boom underside over the C1
        (lidar_front, t0),                          # drop in front of the C1
        (F.TONGUE_X[1], t0), (F.TONGUE_X[1], t1),   # tongue
        (F.BOOM_FRONT_X, t1), (F.BOOM_FRONT_X, b1),  # boom front face
        (fx0, b1),                                  # boom top
        (fx0, foot_top), (foot0, foot_top),         # fin back face, foot top
    ]


def build():
    from hardware.parts.common import cad
    w = F.ARM_HALF_W
    arm = cad.profile_xz(profile(), -w, w)
    # The tongue is 12 mm thick, flush with the +y face; the cradle's
    # cheeks straddle it.
    arm = arm.cut(cad.box(F.TONGUE_X[0], F.TONGUE_X[1] + 1, -w - 1, F.TONGUE_Y[0], F.ARM_FRONT_Z[0] - 1,
                          F.ARM_FRONT_Z[1] + 1))
    px, pz = F.PIVOT
    cutters = [
        cad.cyl(3.4, 2 * w + 2, (px, -w - 1, pz), (0, 1, 0)),
        cad.cyl(3.4, 2 * w + 2, (px - F.LOCK_R, -w - 1, pz), (0, 1, 0)),
    ]
    for x, y in F.ARM_FOOT_SCREWS:
        cutters.append(cad.clearance_hole("M3", x, y, F.DECK_Z[1], F.BOOM_Z[1]))
    for x, z in TIE_SLOTS:
        cutters.append(cad.cbox(TIE_SLOT[0], 2 * w + 2, TIE_SLOT[1], center=(x, 0, z)))
    # Lightening windows in the boom web, clear of the edges.
    cutters.append(cad.profile_xz([(60.0, 204.0), (100.0, 204.0), (100.0, 222.0), (60.0, 222.0)], -w - 1, w + 1))
    return cad.cut_all(arm, cutters)
