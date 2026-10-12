"""HaLowLink cradle: holds the Morse Micro HaLowLink 2 (the car's end of the
long-range Wi-Fi HaLow link) upright behind the camera arm's fin, its
connector side (SMA, Ethernet, USB-C) facing the rear. A U-channel on a
floor that bridges the IMU, on four printed spacers into the rear deck; a
hook-and-loop strap over the unit's top holds it down. Everything above the
deck here is inside the fin's shadow, so the lidar sees none of it."""
from hardware.parts.common import printing
from hardware.parts.racecarneo import _frame as F
from hardware.parts.racecarneo._footprints import HaLowLink2 as H

CLEAR = 0.3
WALL_T = 3.0
WALL_H = 40.0                        # the unit's lower 40 of its 88 mm
FRONT_STOP = (3.0, 20.0)             # thickness, height
REAR_STOP = (2.3, 3.0)               # low: the USB-C plug is just above it
FLOOR_FRONT_X = 32.0                 # the arms to the front spacers end here
ARM_W = 8.0
EAR_R = 4.0                          # floor around each spacer screw
STRAP_SLOT = (22.0, 3.5)             # 20 mm strap, along x and across
STRAP_X = (-15.0, 7.0)
WINDOW = (-30.0, 18.0, 8.0, 8.0)     # wall window: x0, x1, margin to the floor and to the top


def inner_half_w():
    return F.HALOW_HALF_T + CLEAR


def outer_half_w():
    return inner_half_w() + WALL_T


PART = {
    "name": "HaLowLink 2 cradle (behind the camera arm)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on top of the car in the sun, and the HaLowLink warms it",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: the floor on the plate, walls up", wall_loops=3, infill_pct=30),
    "mates_with": [
        {"part": "Morse Micro HaLowLink 2",
         "dimensions": f"{H.SIZE[2]:g} mm thick, {H.SIZE[1]:g} mm wide, {H.SIZE[0]:g} mm tall, standing in a "
                       f"{2 * inner_half_w():.1f} mm channel",
         "source": H.SOURCE, "verified": True},
        {"part": "HaLowLink 2 USB-C port (clearance)",
         "dimensions": f"about {H.SIZE[0] - H.USBC_FROM_TOP:g} mm above the unit's bottom, over the {REAR_STOP[1]:g} mm "
                       "rear stop (scaled from the brief's drawing, not dimensioned)",
         "source": H.SOURCE, "verified": False},
        {"part": "halow_spacer (printed) and deck_rear inserts",
         "dimensions": f"four M3 through the floor and the spacers into the deck at {F.HALOW_SPACERS}",
         "source": "hardware/parts/racecarneo/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 25 socket head cap screw", "qty": 4,
         "use": "HaLowLink cradle through the spacers into the rear deck inserts"},
        {"item": "hook-and-loop strap, 20 mm, 400 mm", "qty": 1,
         "use": "HaLowLink 2 into its cradle, over the unit's top and under the floor"},
    ],
    "fit_test": True,
    "notes": [
        ("The unit stands with its connector side (SMA, WAN, LAN, USB-C) to the rear; pair it with the ground "
         "station's unit before it goes in (hardware/ground_station.md)."),
        "The floor bridges the IMU 3 mm above its connectors: lift the cradle off (four screws) to reach the IMU.",
    ],
}


def build():
    from hardware.parts.common import cad
    x0, x1 = F.HALOW_X
    z0, z1 = F.HALOW_FLOOR_Z
    hi, ho = inner_half_w(), outer_half_w()
    rear = x0 - CLEAR - REAR_STOP[0]
    front = x1 + CLEAR + FRONT_STOP[0]
    parts = [cad.box(rear, FLOOR_FRONT_X, -ho, ho, z0, z1)]
    # Arms out to the front spacers, ears round the rear ones, strap lands.
    for sx, sy in F.HALOW_SPACERS:
        if sx > 0:
            parts.append(cad.box(sx - ARM_W / 2, sx + ARM_W / 2, *sorted((0.0, sy)), z0, z1))
        parts.append(cad.cyl(2 * EAR_R, z1 - z0, (sx, sy, z0)))
        if sx < 0:
            parts.append(cad.box(sx - EAR_R, sx + EAR_R, *sorted((0.0, sy)), z0, z1))
    strap_out = ho + 1.0 + STRAP_SLOT[1] + 2.5
    parts.append(cad.rounded_plate(STRAP_X[1] - STRAP_X[0] + 4.0, 2 * strap_out, z1 - z0, 3.0,
                                   center=((STRAP_X[0] + STRAP_X[1]) / 2, 0.0, z0)))
    # The channel: two walls, the front stop full width, the low rear stop.
    for s in (1, -1):
        parts.append(cad.box(x0 - CLEAR, x1 + CLEAR, *sorted((s * hi, s * ho)), z1 - 0.01, z1 + WALL_H))
    parts.append(cad.box(x1 + CLEAR, front, -ho, ho, z1 - 0.01, z1 + FRONT_STOP[1]))
    parts.append(cad.box(rear, x0 - CLEAR, -ho, ho, z1 - 0.01, z1 + REAR_STOP[1]))
    body = cad.fuse_all(parts)

    cutters = []
    for sx, sy in F.HALOW_SPACERS:
        cutters.append(cad.clearance_hole("M3", sx, sy, z0 - 1, z1 + 1))
    for s in (1, -1):
        y = s * (ho + 1.0 + STRAP_SLOT[1] / 2)
        cutters.append(cad.cbox(STRAP_SLOT[0], STRAP_SLOT[1], 10.0, center=((STRAP_X[0] + STRAP_X[1]) / 2, y, z0)))
        wx0, wx1, m0, m1 = WINDOW
        cutters.append(cad.box(wx0, wx1, *sorted((s * (hi - 1), s * (ho + 1))), z1 + m0, z1 + WALL_H - m1))
    return cad.cut_all(body, cutters)
