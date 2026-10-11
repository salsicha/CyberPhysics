"""Sensor nose: bolted under the front of the bottom plate; carries the
OAK-1 camera stage on four silicone damper balls and the downward
Benewake rangefinder."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.common.footprints import (
    BenewakeTFLuna,
    BenewakeTFminiS,
    LuxonisOAK1,
    SiliconeDamperBall,
)

RIB_Y = (14.0, -14.0)
RIB_T = 3.0
RIB_H = 10.0
RIB_X = (F.PLATE_HALF + 1.0, 140.0)
POCKET_D = 13.0
CABLE_NOTCH_X = (95.0, 125.0)
CABLE_NOTCH_DEPTH = 6.0
TRIPOD_ACCESS_D = 9.0

PART = {
    "name": "Sensor nose (OAK-1 damper deck and rangefinder mount)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "airframe exterior in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat underside on the plate, ribs up", wall_loops=4, infill_pct=30),
    "mates_with": [
        {"part": "hub_bottom_plate (printed)", "dimensions": "four M3 inserts at x = 56/70 mm, y = +-30 mm",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
        {"part": "Benewake TFmini-S",
         "dimensions": f"2 x dia. {BenewakeTFminiS.HOLE_D} mm ear holes {BenewakeTFminiS.HOLE_SPACING:g} mm apart, ST2.9 screws",
         "source": BenewakeTFminiS.SOURCE, "verified": False},
        {"part": "Benewake TF-Luna (alternative)",
         "dimensions": f"2 x dia. {BenewakeTFLuna.HOLE_D} mm holes {BenewakeTFLuna.HOLE_SPACING:g} mm apart, M2 screws (same slots)",
         "source": BenewakeTFLuna.SOURCE, "verified": False},
        {"part": "silicone gimbal damper balls (x4)",
         "dimensions": f"dia. {SiliconeDamperBall.HOLE_D} mm holes in {SiliconeDamperBall.PLATE_THICKNESS:g} mm tabs, "
                       f"{SiliconeDamperBall.PLATE_SPACING:g} mm between the tabs",
         "source": SiliconeDamperBall.SOURCE, "verified": False},
        {"part": "Luxonis OAK-1 (through the camera stage)",
         "dimensions": f"tripod screw access over the 1/4\"-20 thread {LuxonisOAK1.TRIPOD_FROM_USB_END:g} mm from the USB-C end",
         "source": LuxonisOAK1.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 10 socket head cap screw", "qty": 4, "use": "sensor nose to bottom plate"},
        {"item": "ST2.9 x 9.5 pan head self-tapping screw (TFmini-S) or M2 x 12 + nut (TF-Luna)", "qty": 2,
         "use": "rangefinder ears, from above"},
        {"item": "silicone gimbal damper ball, 6.5 mm neck", "qty": 4, "use": "camera stage isolation"},
    ],
    "fit_test": True,
    "notes": [
        "Rangefinder lenses point straight down, long side across the nose, connector toward the camera.",
        "The damper ball necks snap into the 6.5 mm holes from below; print this part first to test the fit.",
    ],
}


def build():
    from hardware.parts.common import cad
    x0, x1 = F.NOSE_X
    z0, z1 = F.NOSE_Z
    w = F.NOSE_HALF_W
    plate = cad.rounded_plate(x1 - x0, 2 * w, z1 - z0, 6.0, center=((x0 + x1) / 2, 0.0, z0))
    ribs = [cad.box(RIB_X[0], RIB_X[1], y - RIB_T / 2, y + RIB_T / 2, z1 - 0.01, z1 + RIB_H) for y in RIB_Y]
    body = cad.fuse_all([plate] + ribs)

    cutters = [cad.clearance_hole("M3", x, y, z0 - 1, z1 + 1) for x, y in F.NOSE_INSERTS]
    tab_top = z0 + F.DAMPER_TAB_T
    for x, y in F.DAMPERS:
        cutters.append(cad.cyl(SiliconeDamperBall.HOLE_D, z1 - z0 + 2, (x, y, z0 - 1)))
        cutters.append(cad.cyl(POCKET_D, z1 - tab_top + 1, (x, y, tab_top)))
    for sign in (1, -1):
        y_in = sign * (w - CABLE_NOTCH_DEPTH)
        cutters.append(cad.box(CABLE_NOTCH_X[0], CABLE_NOTCH_X[1], min(y_in, sign * (w + 1)),
                               max(y_in, sign * (w + 1)), z0 - 1, z1 + 1))
        y_a, y_b = sign * BenewakeTFLuna.HOLE_SPACING / 2, sign * BenewakeTFminiS.HOLE_SPACING / 2
        cutters.append(cad.slot(F.RANGEFINDER_X, y_a, F.RANGEFINDER_X, y_b, 3.2, z0 - 1, z1 + 1))
    tripod_x = F.CAMERA_X - LuxonisOAK1.HEIGHT / 2 + LuxonisOAK1.TRIPOD_FROM_USB_END
    cutters.append(cad.cyl(TRIPOD_ACCESS_D, z1 - z0 + 2, (tripod_x, 0.0, z0 - 1)))
    return cad.cut_all(body, cutters)
