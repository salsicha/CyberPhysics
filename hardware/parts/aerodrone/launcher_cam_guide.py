"""Launcher cam guide (aerodrone): a pad screwed to the downward breech's
underside, with the shelf the turret's pedestal gives the cam plate and a
lip over the plate's outer edge. On the turret gravity holds the cam plate
on the pedestal's shelf; pointing down, gravity runs along the rack, so the
plate is guided on both faces here. The hanger's keel bolts to the pad's
front face."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.turret import _launcher as L

PAD_Z = (-F.KEEL_X[0], L.BREECH_Z0)       # launcher z_L: from the keel's back face to the breech's underside
SHELF_TOP = L.CAM_Z[0] - 0.2               # 0.2 mm under the cam plate, as the turret's shelf
LIP_Z = (L.CAM_Z[1] + 0.3, L.CAM_Z[1] + 2.8)
WALL_Y = (-F.CAM_GUIDE_WING_Y[1], -F.CAM_GUIDE_WING_Y[1] + 2.3)     # launcher y_L of the outer wall
LIP_Y1 = L.CAM_PLATE_Y[0] + 2.5             # the lip covers the plate's outer 2.5 mm
SCREW_CBORE = (6.5, 3.2)

PART = {
    "name": "Launcher cam guide (pad on the downward breech)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "the PETG cam plate slides on it (PETG on ASA wears least); outdoors",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "the keel face (front) on the plate, shelf and lip up",
                               wall_loops=4, infill_pct=30, rotate=[("Y", 90)]),
    "mates_with": [
        {"part": "launcher_breech_down (printed)",
         "dimensions": f"four M3 x 10 into its underside inserts at (x_L, y_L) = {F.BREECH_UNDER_SCREWS}",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
        {"part": "turret launcher_cam (printed cam plate)",
         "dimensions": f"shelf {L.CAM_Z[0] - SHELF_TOP:.1f} mm under it, lip {LIP_Z[0] - L.CAM_Z[1]:.1f} mm over it, over "
                       f"x_L {L.CAM_SHELF_X[0]:g}..{L.CAM_SHELF_X[1]:g} mm",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
        {"part": "launcher_hanger (printed)",
         "dimensions": "keel against the front face, four M3 x 8 into short inserts",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
    ],
    "inserts": [{"size": "M3-short", "qty": 4, "where": "front face, keel screws"}],
    "fasteners": [{"item": "M3 x 10 socket head cap screw", "qty": 4,
                   "use": "cam guide to the breech's underside inserts (heads in counterbores)"}],
    "fit_test": True,
    "notes": ["Screw it to the breech before the hanger: the upper two screws are under the keel.",
              "With the rack in place, the cam plate must slide its 39 mm on the shelf and under the lip without "
              "catching; ease the lip's lead-in with a file if it does."],
}


def keel_screws_local():
    """(x_L, y_L) of the keel screws (inserts in the pad's front face)."""
    return [(x, -y) for x in F.KEEL_PAD_SCREWS_XL for y in F.KEEL_PAD_SCREWS_Y]


def build_local():
    """The cam guide in the launcher frame."""
    import cadquery as cq

    from hardware.parts.common import cad
    x0, x1 = F.CAM_GUIDE_XL
    yp0, yp1 = -F.CAM_GUIDE_Y[1], -F.CAM_GUIDE_Y[0]          # launcher y_L of the pad
    z0, z1 = PAD_Z
    sx0, sx1 = L.CAM_SHELF_X
    wy0, wy1 = WALL_Y
    parts = [
        cad.box(x0, x1, yp0, yp1, z0, z1),
        # bridge from the pad to the shelf, under the cam plate's level
        cad.box(sx0, sx1, wy0, yp0 + 0.01, z0, SHELF_TOP),
        # outer wall, up past the plate
        cad.box(sx0, sx1, wy0, wy1, z0, LIP_Z[1]),
    ]
    # lip over the plate's outer edge, with a 45 deg lead-in where the plate
    # comes back under it
    lx0, lx1 = F.CAM_LIP_XL
    lip = (cq.Workplane("XY", origin=(0, 0, LIP_Z[0]))
           .polyline([(lx0, wy1 - 0.01), (lx0, wy1 + 0.5), (lx0 + 3.0, LIP_Y1), (lx1, LIP_Y1), (lx1, wy1 - 0.01)])
           .close().extrude(LIP_Z[1] - LIP_Z[0]).val())
    parts.append(lip)
    body = cad.fuse_all(parts)
    cutters = []
    for x, y in F.BREECH_UNDER_SCREWS:
        cutters.append(cad.cyl(3.4, z1 - z0 + 2, (x, y, z0 - 1)))
        cutters.append(cad.cyl(SCREW_CBORE[0], SCREW_CBORE[1] + 0.01, (x, y, z0 - 0.01)))
    for x, y in keel_screws_local():
        cutters.append(cad.insert_hole("M3-short", x, y, z0, direction=1))
    return cad.cut_all(body, cutters)


def build():
    return L.place(build_local(), F.LAUNCHER_ORIGIN, F.LAUNCHER_ROT)
