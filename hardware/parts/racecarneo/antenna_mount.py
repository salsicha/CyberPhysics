"""Antenna mount: stands the HaLowLink 2's sub-GHz antenna vertically at the
rear-right corner of the deck. An SMA bulkhead jack (the antenna end of an
RG316 pigtail from the HaLowLink) goes through the wall sideways, a
right-angle SMA adapter on it points up, and the half-wave dipole screws on
top. The antenna stands in the camera arm fin's shadow (the lidar does not
see it), behind the RealSense, clear of the e-stop, and its tip under the
RACECAR Turret's launcher at full nose-up tilt (the racecar_turret assembly
check)."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F
from hardware.parts.racecarneo._footprints import LinxAnt916CwHw as A
from hardware.parts.racecarneo._footprints import SmaBulkheadAndElbow as S

BASE_T = 3.0
BASE_Y = (F.ANTENNA_PANEL_Y[0], -32.0)
WALL_HALF_W = 6.5
WALL_TOP_ABOVE_AXIS = 10.0
GUSSET = (1.7, 10.0)                 # thickness, height above the base
SCREW_EDGE = 3.0                     # base edge beyond the outer screws

PART = {
    "name": "Antenna mount (HaLowLink 2, rear-right corner)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on top of the car in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: the base on the plate, the wall up", wall_loops=4, infill_pct=40),
    "mates_with": [
        {"part": "SMA bulkhead jack (RG316 pigtail) and right-angle SMA adapter",
         "dimensions": f"{S.PANEL_HOLE:g} mm hole for the 1/4-36 thread in a "
                       f"{F.ANTENNA_PANEL_Y[1] - F.ANTENNA_PANEL_Y[0]:g} mm panel (most bulkhead jacks take up to "
                       f"about 4-5 mm); the antenna's axis {S.ELBOW_AXIS_FROM_PANEL:g} mm out from the panel",
         "source": S.SOURCE, "verified": False},
        {"part": "Linx ANT-916-CW-HW-SMA antenna (clearance)",
         "dimensions": f"{A.LENGTH:g} mm long, {A.DIAMETER:g} mm diameter, vertical",
         "source": A.SOURCE, "verified": True},
        {"part": "deck_rear (printed)", "dimensions": "two M3 x 3 inserts", "source": "hardware/parts/racecarneo/_frame.py",
         "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 6 socket head cap screw", "qty": 2, "use": "antenna mount to the rear deck inserts"},
    ],
    "fit_test": True,
    "notes": [
        ("Tighten the bulkhead's nut with the right-angle adapter's jack pointing straight up, then screw the antenna "
         "on finger tight."),
        ("The antenna's tip stays under the RACECAR Turret's launcher only with an antenna no longer than the "
         f"{A.LENGTH:g} mm one modelled: do not fit the HaLowLink's own, longer antenna on the turret car."),
    ],
}


def build():
    from hardware.parts.common import cad
    ax, _ = F.ANTENNA_XY
    az = F.ANTENNA_AXIS_Z
    z0 = F.DECK_Z[1]
    (sx0, _), (sx1, _) = sorted(F.ANTENNA_INSERTS)
    py0, py1 = F.ANTENNA_PANEL_Y
    base = cad.box(sx0 - SCREW_EDGE, sx1 + SCREW_EDGE, BASE_Y[0], BASE_Y[1], z0, z0 + BASE_T)
    wall = cad.box(ax - WALL_HALF_W, ax + WALL_HALF_W, py0, py1, z0, az + WALL_TOP_ABOVE_AXIS)
    # Ribs behind the wall, either side of the bulkhead's inner hex.
    gussets = [cad.box(*sorted((s * WALL_HALF_W, s * (WALL_HALF_W - GUSSET[0]))), py1 - 0.01,
                       py1 + S.INNER_HEX_T - 0.25, z0 + BASE_T - 0.01, z0 + BASE_T + GUSSET[1])
               for s in (1, -1)]
    gussets = [cad.translate(g, (ax, 0, 0)) for g in gussets]
    body = cad.fuse_all([base, wall] + gussets)
    cutters = [cad.cyl(S.PANEL_HOLE, py1 - py0 + 2, (ax, py0 - 1, az), (0, 1, 0))]
    for x, y in F.ANTENNA_INSERTS:
        cutters.append(cad.clearance_hole("M3", x, y, z0 - 1, z0 + BASE_T + 1))
    return cad.cut_all(body, cutters)
