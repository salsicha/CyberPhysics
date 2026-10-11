"""GPS mast: lifts the GPS/compass puck about 130 mm above the top plate,
away from the power wiring and the USB 3 cable."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.common.footprints import HolybroM10GPS

FLANGE_T = 5.0
COLUMN_D = 18.0
SEAT_D = 58.0
SEAT_T = 3.0
LIP_H = 3.0
LIP_WALL = 2.0
PUCK_CLEARANCE = 0.4
CABLE_NOTCH_W = 10.0

PART = {
    "name": "GPS mast",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "the highest point of the aircraft, in full sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings(
        "ASA", "upright: flange on the plate (the 45 deg cone under the seat needs no supports)",
        wall_loops=4, infill_pct=25, brim="outer brim only, 5 mm"),
    "mates_with": [
        {"part": "Holybro M10/M9N GPS",
         "dimensions": f"puck dia. {HolybroM10GPS.V2[0]:g} mm (V2; V1 {HolybroM10GPS.V1[0]:g} mm) in a "
                       f"{max(HolybroM10GPS.V1[0], HolybroM10GPS.V2[0]) + 2 * PUCK_CLEARANCE:.1f} mm lip",
         "source": HolybroM10GPS.SOURCE, "verified": True},
        {"part": "hub_top_plate (printed)", "dimensions": "three M3 inserts on a 13.5 mm radius",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 10 socket head cap screw", "qty": 3, "use": "GPS mast to top plate"},
        {"item": "double-sided foam tape (VHB), 50 mm disc", "qty": 1, "use": "GPS puck on the mast seat"},
    ],
    "fit_test": False,
    "notes": [
        "Arrow on the GPS puck toward the nose (or set COMPASS_ORIENT to match).",
        "The cable notch in the lip faces the nose; run the cable down the mast with two cable ties.",
    ],
}


def build():
    from hardware.parts.common import cad
    x, y = F.MAST_XY
    z0 = F.TOP_Z[1]
    seat_z = z0 + F.MAST_HEIGHT
    cone_h = (SEAT_D - COLUMN_D) / 2          # 45 degrees
    cone_z = seat_z - SEAT_T - cone_h
    lip_id = max(HolybroM10GPS.V1[0], HolybroM10GPS.V2[0]) + 2 * PUCK_CLEARANCE
    body = cad.fuse_all([
        cad.cyl(F.MAST_FLANGE_D, FLANGE_T, (x, y, z0)),
        cad.cyl(COLUMN_D, cone_z - z0 - FLANGE_T + 0.01, (x, y, z0 + FLANGE_T - 0.01)),
        cad.cone(COLUMN_D, SEAT_D, cone_h, (x, y, cone_z)),
        cad.cyl(SEAT_D, SEAT_T, (x, y, seat_z - SEAT_T)),
        cad.cyl(lip_id + 2 * LIP_WALL, LIP_H, (x, y, seat_z - 0.01)),
    ])
    cutters = [cad.cyl(lip_id, LIP_H + 1, (x, y, seat_z)),
               cad.box(x, x + SEAT_D, -CABLE_NOTCH_W / 2, CABLE_NOTCH_W / 2, seat_z, seat_z + LIP_H + 1)]
    for bx, by in F.mast_bolts_body():
        cutters.append(cad.clearance_hole("M3", bx, by, z0 - 1, z0 + FLANGE_T + 1))
    return cad.cut_all(body, cutters)
