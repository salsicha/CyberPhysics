"""Battery tray: bonded to the bay floor ahead of the camera. The 6S pack
lies between two rails under two straps, which pass through any two of the
five strap tunnels in each rail; slide the pack fore or aft to set the
centre of gravity, then strap it."""
from hardware.parts.airplane import _frame as F
from hardware.parts.airplane._footprints import Lipo6SEnvelope, SkywalkerX8
from hardware.parts.common import printing

STRAP_X = (75.0, 120.0, 165.0, 210.0, 255.0)
STRAP_TUNNEL = (22.0, 3.5)       # along x, height: 20 mm hook-and-loop strap
WINDOWS = ((87.0, 108.0), (132.0, 153.0), (177.0, 198.0), (222.0, 243.0))
WINDOW_HALF_W = 14.0
STOP_T = 4.0

PART = {
    "name": "Battery tray",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "tough in a nose-in landing; the pack stays under 60 C, well inside PETG's range",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("PETG", "flat underside (the bonding face) on the plate, rails up; the strap tunnels bridge",
                               wall_loops=3, infill_pct=25, brim="outer brim only, 5 mm"),
    "mates_with": [
        {"part": "6S LiPo pack",
         "dimensions": f"up to {Lipo6SEnvelope.MAX_SIZE[1]:g} mm wide between rails {2 * F.BATTERY_HALF_W:g} mm apart, "
                       f"{F.BATTERY_TRAY_X[1] - F.BATTERY_TRAY_X[0]:g} mm of travel for packs up to {Lipo6SEnvelope.MAX_SIZE[0]:g} mm long",
         "source": Lipo6SEnvelope.SOURCE, "verified": True},
        {"part": "Skywalker X8 payload-bay floor ahead of the CG",
         "dimensions": f"{F.BATTERY_TRAY_X[1] - F.BATTERY_TRAY_X[0]:g} x {2 * F.BATTERY_TRAY_HALF_W:g} mm bonded face",
         "source": SkywalkerX8.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "battery strap, 20 mm hook-and-loop, 300-350 mm, with rubberised face", "qty": 2, "use": "battery to tray"},
        {"item": "anti-slip foam pad, 160 x 50 mm", "qty": 1, "use": "between the battery and the tray"},
    ],
    "fit_test": True,
    "notes": [
        "A loose battery moves the CG and ends the flight: two straps, each through its own pair of tunnels at least 15 mm inside the pack's ends, checked before every flight.",
        "The front stop takes the pack in a nose-in landing; the battery lead leaves at the rear, toward the power sense module.",
    ],
}


def build():
    from hardware.parts.common import cad
    x0, x1 = F.BATTERY_TRAY_X
    w = F.BATTERY_TRAY_HALF_W
    t = F.BATTERY_TRAY_T
    plate = cad.rounded_plate(x1 - x0, 2 * w, t, 6.0, center=((x0 + x1) / 2, 0.0, 0.0))
    yi = F.BATTERY_HALF_W
    rails = [cad.box(x0 + 2, x1 - 0.01, ya, yb, t - 0.01, t + F.BATTERY_RAIL_H)
             for ya, yb in ((yi, yi + F.BATTERY_RAIL_T), (-yi - F.BATTERY_RAIL_T, -yi))]
    stop = cad.box(x1 - STOP_T, x1, -yi - 0.01, yi + 0.01, t - 0.01, t + F.BATTERY_RAIL_H)
    body = cad.fuse_all([plate, stop] + rails)
    cutters = []
    for x in STRAP_X:
        cutters.append(cad.box(x - STRAP_TUNNEL[0] / 2, x + STRAP_TUNNEL[0] / 2, -w - 1, w + 1, t, t + STRAP_TUNNEL[1]))
    for xa, xb in WINDOWS:
        cutters.append(cad.box(xa, xb, -WINDOW_HALF_W, WINDOW_HALF_W, -1, t + 1))
    return cad.cut_all(body, cutters)
