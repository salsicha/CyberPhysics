"""Hub top plate: upper half of the four arm clamps; carries the Pi 5 +
Navigator stack tray and the GPS mast."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.common.footprints import CarbonTube16

PART = {
    "name": "Hub top plate",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "airframe exterior in direct sun; clamps the arms",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings(
        "ASA", "upside down: flat top face on the plate, clamp blocks up",
        wall_loops=4, infill_pct=25, top_shell_layers=4, bottom_shell_layers=4,
        brim="outer brim only, 5 mm", rotate=[("X", 180)]),
    "mates_with": [
        {"part": "16 mm carbon arm tube (x4)",
         "dimensions": f"tube OD {CarbonTube16.OD} mm in a {F.TUBE_BORE} mm half bore",
         "source": CarbonTube16.SOURCE, "verified": True},
        {"part": "hub_bottom_plate, stack_tray, gps_mast (printed)",
         "dimensions": "bolt and insert positions from _frame.py",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
    ],
    "inserts": [
        {"size": "M3", "qty": 4, "where": "top face, stack tray grommet bolts"},
        {"size": "M3", "qty": 3, "where": "top face, GPS mast flange"},
    ],
    "fasteners": [
        {"item": "M3 x 25 socket head cap screw", "qty": 16,
         "use": "arm clamps, through the top plate into the bottom plate inserts"},
        {"item": "M3 washer", "qty": 16, "use": "under the arm clamp screw heads"},
    ],
    "fit_test": False,
    "notes": [
        "Tighten the 16 clamp screws evenly in a cross pattern until the blocks almost touch (about 1 mm gap); snug, do not crush.",
    ],
}


def build():
    from hardware.parts.aerodrone import _hub
    from hardware.parts.common import cad

    z0, z1 = F.TOP_Z
    body = cad.fuse_all(
        [_hub.outline(z0, z1)] + _hub.blocks(F.CLAMP_GAP / 2, z0)
        + [cad.cyl(F.POST_BOSS_D, F.POST_BOSS_H, (x, y, z0 - F.POST_BOSS_H))
           for x, y in F.STACK_GROMMETS + F.mast_bolts_body()])

    cutters = list(_hub.bores())
    for x, y in _hub.clamp_bolt_xy():
        cutters.append(cad.clearance_hole("M3", x, y, F.CLAMP_GAP / 2 - 1, z1 + 1))
    for x, y in F.STACK_GROMMETS + F.mast_bolts_body():
        cutters.append(cad.insert_hole("M3", x, y, z1, direction=-1))
    # Cables from the hub up to the stack, under the tray window.
    cutters.append(cad.slot(-5.0, 0.0, 25.0, 0.0, 14.0, z0 - 1, z1 + 1))
    for x, y in ((-5.0, 46.0), (-5.0, -46.0)):
        cutters.append(cad.cyl(20.0, F.PLATE_T + 2, (x, y, z0 - 1)))
    return cad.cut_all(body, cutters)
