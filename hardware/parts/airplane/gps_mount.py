"""GPS mount: a low pedestal bonded to the top of the left wing, 250 mm
out, away from the OAK-1's USB 3 cable, the battery leads, and the ESC and
motor at the tail; the puck sits in a lip on top, arrow forward."""
from hardware.parts.airplane import _frame as F
from hardware.parts.airplane._footprints import SkywalkerX8
from hardware.parts.common import printing
from hardware.parts.common.footprints import HolybroM10GPS

BASE = (64.0, 64.0)
BASE_T = 3.0
COLUMN_D = 38.0
SEAT_D = 58.0
SEAT_T = 3.0
HEIGHT = 20.0                 # base underside to the puck seat: clears a curved skin
LIP_H = 3.0
LIP_WALL = 2.0
PUCK_CLEARANCE = 0.4
CABLE_NOTCH_W = 10.0
CABLE_HOLE_D = 9.0
CABLE_HOLE_X = -25.0          # through the base, behind the column

PART = {
    "name": "GPS mount",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on the wing in full sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: base on the plate (the 45 deg cone under the seat needs no supports)",
                               wall_loops=4, infill_pct=25),
    "mates_with": [
        {"part": "Holybro M10/M9N GPS",
         "dimensions": f"puck dia. {HolybroM10GPS.V2[0]:g} mm (V2; V1 {HolybroM10GPS.V1[0]:g} mm) in a "
                       f"{max(HolybroM10GPS.V1[0], HolybroM10GPS.V2[0]) + 2 * PUCK_CLEARANCE:.1f} mm lip",
         "source": HolybroM10GPS.SOURCE, "verified": True},
        {"part": "Skywalker X8 wing top skin",
         "dimensions": f"{BASE[0]:g} x {BASE[1]:g} mm base bonded to the foam (the wing's curvature is not published)",
         "source": SkywalkerX8.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "double-sided foam tape (VHB), 50 mm disc", "qty": 1, "use": "GPS puck on the mount seat"},
    ],
    "fit_test": True,
    "notes": [
        "Arrow on the GPS puck toward the nose (or set COMPASS_ORIENT to match); the cable notch faces the tail.",
        "Bond the base with foam-safe epoxy (it fills the gap to the curved wing); the cable runs from the lip notch down to the hole in the base, through a matching hole in the skin and inside the wing to the bay.",
        "Keep it at least 150 mm from the ESC, the motor leads and the OAK-1 USB 3 cable: USB 3 radiates in the GNSS L1 band.",
    ],
}


def build():
    from hardware.parts.common import cad
    x, y = F.GPS_XY
    z0 = F.WING_SKIN_Z
    seat_z = z0 + HEIGHT
    cone_h = (SEAT_D - COLUMN_D) / 2          # 45 degrees
    cone_z = seat_z - SEAT_T - cone_h
    lip_id = max(HolybroM10GPS.V1[0], HolybroM10GPS.V2[0]) + 2 * PUCK_CLEARANCE
    body = cad.fuse_all([
        cad.rounded_plate(BASE[0], BASE[1], BASE_T, 8.0, center=(x, y, z0)),
        cad.cyl(COLUMN_D, cone_z - z0 - BASE_T + 0.02, (x, y, z0 + BASE_T - 0.01)),
        cad.cone(COLUMN_D, SEAT_D, cone_h, (x, y, cone_z)),
        cad.cyl(SEAT_D, SEAT_T, (x, y, seat_z - SEAT_T)),
        cad.cyl(lip_id + 2 * LIP_WALL, LIP_H, (x, y, seat_z - 0.01)),
    ])
    cutters = [cad.cyl(lip_id, LIP_H + 1, (x, y, seat_z)),
               cad.box(x - SEAT_D, x, y - CABLE_NOTCH_W / 2, y + CABLE_NOTCH_W / 2, seat_z, seat_z + LIP_H + 1),
               cad.cyl(CABLE_HOLE_D, BASE_T + 2, (x + CABLE_HOLE_X, y, z0 - 1))]
    return cad.cut_all(body, cutters)
