"""Tray for the Raspberry Pi 5 + Blue Robotics Navigator stack, on four
TPU grommet pairs on the avionics deck."""
from hardware.parts.airplane import _frame as F
from hardware.parts.common import fasteners, printing
from hardware.parts.common.footprints import BlueRoboticsNavigator, RaspberryPi5

HALF_LEN = 55.5
HALF_W = 29.5
BOSS_D = 7.0
WINDOWS = (((-30.0, 12.0), 16.0), ((24.0, 40.0), 16.0))   # (x range rel. Pi centre, half width)

PART = {
    "name": "Stack tray (Pi 5 + Navigator)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "carries the Pi 5 and its heatsink in a closed foam bay",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("ASA", "flat underside on the plate, bosses up", wall_loops=3, infill_pct=30),
    "mates_with": [
        {"part": "Raspberry Pi 5",
         "dimensions": f"{RaspberryPi5.HOLE_SPACING[0]:g} x {RaspberryPi5.HOLE_SPACING[1]:g} mm holes, "
                       f"{RaspberryPi5.HOLE_INSET:g} mm from the edges, dia. {RaspberryPi5.HOLE_D} mm",
         "source": RaspberryPi5.SOURCE, "verified": True},
        {"part": "Blue Robotics Navigator heatsink (M2.5 threads)",
         "dimensions": f"same {BlueRoboticsNavigator.HOLE_SPACING[0]:g} x {BlueRoboticsNavigator.HOLE_SPACING[1]:g} mm pattern",
         "source": BlueRoboticsNavigator.SOURCE, "verified": True},
        {"part": "Pi 5 underside clearance",
         "dimensions": f"{F.PI_BOSS_H:g} mm bosses for the parts under the PCB (microSD socket)",
         "source": "not on the Raspberry Pi 5 drawing", "verified": False},
        {"part": "stack_grommet (printed TPU)", "dimensions": f"{F.GROMMET_HOLE} mm holes, {F.TRAY_T:g} mm tray",
         "source": "hardware/parts/airplane/stack_grommet.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 12 socket head cap screw", "qty": 4,
         "use": "stack tray to the avionics deck, through a pair of stack grommets"},
        {"item": "M3 washer", "qty": 4, "use": "on the upper stack grommets"},
        {"item": "M2.5 x 12 socket head cap screw", "qty": 2,
         "use": "from under the tray through the Pi into the Navigator heatsink (the two holes the Navigator guide's standoff uses)"},
    ],
    "fit_test": True,
    "notes": [
        "Replaces the Raspberry Pi mounting standoff of the Navigator kit; heatsink and Navigator stack as in Blue Robotics' Navigator Hardware Setup.",
        "USB/Ethernet end of the Pi toward the nose (the camera's USB 3 cable plugs in there), microSD toward the tail.",
    ],
}


def build():
    from hardware.parts.common import cad
    cx, cy = F.PI_CENTER
    z0 = F.TRAY_Z0
    z1 = z0 + F.TRAY_T
    tray = cad.rounded_plate(2 * HALF_LEN, 2 * HALF_W, F.TRAY_T, 5.0, center=(cx, cy, z0))
    holes = F.pi_holes_body()
    tray = cad.fuse_all([tray] + [cad.cyl(BOSS_D, F.PI_BOSS_H, (x, y, z1)) for x, y in holes])
    head_d, head_h = fasteners.SHCS_HEAD["M2.5"]
    cutters = []
    for x, y in holes:
        cutters.append(cad.clearance_hole("M2.5", x, y, z0 - 1, z1 + F.PI_BOSS_H + 1))
        cutters.append(cad.cyl(head_d + 1.0, head_h, (x, y, z0 - 0.01)))
    for x, y in F.STACK_GROMMETS:
        cutters.append(cad.cyl(F.GROMMET_HOLE, F.TRAY_T + 2, (x, y, z0 - 1)))
    for (xa, xb), hw in WINDOWS:
        cutters.append(cad.slot(cx + xa + hw / 2, cy, cx + xb - hw / 2, cy, 2 * hw, z0 - 1, z1 + 1)
                       if xb - xa > 2 * hw else cad.box(cx + xa, cx + xb, cy - hw, cy + hw, z0 - 1, z1 + 1))
    return cad.cut_all(tray, cutters)
