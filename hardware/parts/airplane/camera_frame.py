"""Camera frame: bonded to the bay floor around the belly cut-out (its
flange is the cutting template); four pillars carry the camera deck, from
which the OAK-1 cradle hangs on four silicone damper balls."""
from hardware.parts.airplane import _frame as F
from hardware.parts.airplane._footprints import SkywalkerX8
from hardware.parts.common import printing
from hardware.parts.common.footprints import LuxonisOAK1

CORNER_R = 6.0

PART = {
    "name": "Camera frame (belly cut-out template and pillars)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "bonded to the belly skin, which heats in the sun; ASA keeps its shape",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flange (bonding face) on the plate, pillars up", wall_loops=4, infill_pct=30),
    "mates_with": [
        {"part": "Skywalker X8 belly (bay floor)",
         "dimensions": f"{2 * F.FRAME_HALF[0]:g} x {2 * F.FRAME_HALF[1]:g} mm flange bonded around the cut-out; "
                       f"the cut-out keeps the view clear for floors up to {SkywalkerX8.BELLY_T_MAX:g} mm thick",
         "source": SkywalkerX8.SOURCE, "verified": False},
        {"part": "Luxonis OAK-1",
         "dimensions": f"{LuxonisOAK1.VFOV_DEG:g} deg (along x) by {LuxonisOAK1.HFOV_DEG:g} deg (across) view, "
                       f"+{F.FOV_MARGIN_DEG:g} deg, through the cut-out from the lens {F.LENS_Z:g} mm above the floor",
         "source": LuxonisOAK1.SOURCE, "verified": True},
        {"part": "camera_deck (printed)", "dimensions": "four M3 inserts in the pillar tops",
         "source": "hardware/parts/airplane/_frame.py", "verified": True},
    ],
    "inserts": [{"size": "M3", "qty": 4, "where": "pillar tops (camera deck screws)"}],
    "fasteners": [],
    "fit_test": True,
    "notes": [
        "Cut the belly foam along the inside of the flange with a sharp blade, then bond the flange around the hole (foam-safe epoxy), the long side of the opening across the fuselage.",
        "The lens ends up recessed (belly thickness + 5 mm) above the belly skin, so belly landings do not touch it; clean grass and dust off the lens after each landing.",
    ],
}


def build():
    from hardware.parts.common import cad
    cx = F.CAMERA_X
    hx, hy = F.FRAME_HALF
    lx, ly, _ = F.lens_point()
    chx, chy = F.cutout_half()
    flange = cad.rounded_plate(2 * hx, 2 * hy, F.FLANGE_T, CORNER_R, center=(cx, 0.0, 0.0))
    d0 = F.FRAME_DECK_Z[0]
    p = F.PILLAR
    pillars = [cad.cbox(p, p, d0 - F.FLANGE_T + 0.01, center=(x, y, (F.FLANGE_T - 0.01 + d0) / 2))
               for x, y in F.pillar_xy()]
    body = cad.fuse_all([flange] + pillars)
    cutters = [cad.box(lx - chx, lx + chx, ly - chy, ly + chy, -1, F.FLANGE_T + 1)]
    cutters += [cad.insert_hole("M3", x, y, d0) for x, y in F.pillar_xy()]
    return cad.cut_all(body, cutters)
