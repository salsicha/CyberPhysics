"""Battery tray under the hub: the 4S pack hangs below it on two straps,
which pass through the strap slots; slide the pack fore or aft to balance."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import fasteners, printing
from hardware.parts.common.footprints import LipoEnvelope4S

POST_D = 10.0
STRAP_SLOT = (22.0, 3.5)          # for 20 mm hook-and-loop straps
STRAP_X = (-66.0, 10.0)
STRAP_Y = 27.5
RIB_Y = (12.0, -12.0)
RIB_T = 3.0
RIB_H = 5.0
WINDOWS = ((-92.0, -72.0), (-48.0, -2.0), (20.0, 42.0))
WINDOW_HALF_W = 8.0

PART = {
    "name": "Battery tray",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "shaded under the hub; the pack reaches about 60 C, well under PETG's limit",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("PETG", "flat battery face on the plate, posts and ribs up",
                               wall_loops=3, infill_pct=30),
    "mates_with": [
        {"part": "4S LiPo pack",
         "dimensions": f"up to {LipoEnvelope4S.MAX_SIZE[0]:g} x {LipoEnvelope4S.MAX_SIZE[1]:g} x {LipoEnvelope4S.MAX_SIZE[2]:g} mm "
                       "between the strap slots",
         "source": LipoEnvelope4S.SOURCE, "verified": True},
        {"part": "hub_bottom_plate (printed)", "dimensions": "four M3 inserts at x = -45/35 mm, y = +-25 mm",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 14 socket head cap screw", "qty": 4, "use": "battery tray to bottom plate (heads recessed in the posts)"},
        {"item": "battery strap, 20 mm hook-and-loop, 250-300 mm, with rubberised face", "qty": 2, "use": "battery to tray"},
        {"item": "anti-slip foam pad, 150 x 50 mm", "qty": 1, "use": "between the battery and the tray"},
    ],
    "fit_test": False,
    "notes": [
        "A lost battery is a crash: use two straps, each through its own slot pair, and check them before every flight.",
        "The pack position sets the centre of gravity; see the assembly steps.",
    ],
}


def build():
    from hardware.parts.common import cad
    x0, x1 = F.BATTERY_TRAY_X
    z_top = F.BOT_Z[0] - F.TRAY_POST_H        # -29: tray top face
    z_bot = z_top - F.BATTERY_TRAY_T          # -32: battery face
    w = F.BATTERY_TRAY_HALF_W
    parts = [cad.rounded_plate(x1 - x0, 2 * w, F.BATTERY_TRAY_T, 6.0, center=((x0 + x1) / 2, 0.0, z_bot))]
    parts += [cad.box(x0 + 2, x1 - 2, y - RIB_T / 2, y + RIB_T / 2, z_top - 0.01, z_top + RIB_H) for y in RIB_Y]
    parts += [cad.cyl(POST_D, F.TRAY_POST_H + 0.01, (x, y, z_top - 0.01)) for x, y in F.BATTERY_TRAY_INSERTS]
    body = cad.fuse_all(parts)

    head_d, head_h = fasteners.SHCS_HEAD["M3"]
    cutters = []
    for x, y in F.BATTERY_TRAY_INSERTS:
        cutters.append(cad.clearance_hole("M3", x, y, z_bot - 1, F.BOT_Z[0] + 1))
        cutters.append(cad.cyl(head_d + 1.0, head_h + 0.5 + 1, (x, y, z_bot - 1)))
    for x in STRAP_X:
        for y in (STRAP_Y, -STRAP_Y):
            cutters.append(cad.cbox(STRAP_SLOT[0], STRAP_SLOT[1], 10, center=(x, y, z_bot)))
    for xa, xb in WINDOWS:
        cutters.append(cad.box(xa, xb, -WINDOW_HALF_W, WINDOW_HALF_W, z_bot - 1, z_top + 1))
    return cad.cut_all(body, cutters)
