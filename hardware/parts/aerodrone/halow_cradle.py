"""HaLowLink 2 cradle on the top deck's front: holds the Wi-Fi HaLow unit
standing on its side (the connector side, with the SMA, Ethernet and USB-C,
up), in its case, and on an arm forward of it the antenna's stay: an SMA
extension from the unit to a bulkhead jack on the stay, the antenna
standing straight up from it, clear of the arms, the propellers, the GPS
mast (about 130 mm) and, under the hub, the foam dart launcher."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.aerodrone._footprints import HaLowLink2 as H
from hardware.parts.common import fasteners, printing

CLEAR = 0.3
WALL = 2.4
SIDE_H = 30.0                       # front and rear walls, up the unit's 68 mm
END_H = 20.0
ARM_HALF_W = 7.0                    # the arm to the antenna's stay, round HALOW_ANTENNA_XY's y
STAY = (14.0, 16.0, 32.0, 5.0)      # x, y, height above the arm, top plate thickness
CBORE = (6.5, 3.2)


def unit_box():
    """The unit's envelope in place (x0, x1, y0, y1, z0, z1)."""
    tall, wide, deep = H.SIZE
    z0 = F.DECK_Z[1] + F.HALOW_BASE_T
    return (F.HALOW_X[0], F.HALOW_X[1], -tall / 2, tall / 2, z0, z0 + wide)


def stay_top():
    """z of the stay's top face (the bulkhead's nut and the antenna on it)."""
    return F.DECK_Z[1] + F.HALOW_BASE_T + STAY[2]


def antenna_axis():
    """(x, y, z0, z1) of the antenna, from the bulkhead on the stay."""
    ax, ay = F.HALOW_ANTENNA_XY
    length, _ = H.ANTENNA
    z0 = stay_top() + 3.0
    return (ax, ay, z0, z0 + length)


PART = {
    "name": "HaLowLink 2 cradle (on the top deck)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on top of the aircraft in full sun, next to a router that runs warm",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat: the base on the plate, walls and antenna stay up", wall_loops=3,
                               infill_pct=30),
    "mates_with": [
        {"part": "Morse Micro HaLowLink 2 (in its case)",
         "dimensions": f"{H.SIZE[0]:g} x {H.SIZE[1]:g} x {H.SIZE[2]:g} mm, {CLEAR} mm clearance each side",
         "source": H.SOURCE, "verified": True},
        {"part": "SMA bulkhead jack (the extension's far end), the HaLowLink 2's antenna on it",
         "dimensions": f"{H.BULKHEAD_HOLE:g} mm hole for the 1/4-36 UNS thread, vertical; the unit's SMA "
                       f"{H.SMA_FROM_TOP:g} mm from its end (scaled from the brief)",
         "source": H.SOURCE, "verified": False},
        {"part": "top_deck (printed)", "dimensions": f"five M3 x 5 into its short inserts at {F.HALOW_CRADLE_SCREWS}",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 5 socket head cap screw", "qty": len(F.HALOW_CRADLE_SCREWS),
         "use": "HaLowLink cradle to the top deck (heads recessed)"},
        {"item": "hook-and-loop strap, 20 mm, 300 mm", "qty": 1, "use": "HaLowLink 2 round the cradle's walls"},
        {"item": "zip tie 2.5 mm", "qty": 1, "use": "SMA extension cable to the cradle's arm"},
    ],
    "fit_test": True,
    "notes": ["Check the unit's fit and the bulkhead jack's fit in the stay before relying on them.",
              "Keep the case on: Morse Micro's brief gives no bare-board option; the unit runs from 0 to 40 C."],
}


def build():
    from hardware.parts.common import cad
    x0, x1, y0, y1, z0, _ = unit_box()
    bx0, bx1 = x0 - CLEAR - WALL, x1 + CLEAR + WALL
    by = y1 + CLEAR + WALL
    zb = F.DECK_Z[1]
    ax, ay = F.HALOW_ANTENNA_XY
    sx, sy, sh, st = STAY
    parts = [
        cad.rounded_plate(bx1 - bx0, 2 * by, z0 - zb, 3.0, center=((bx0 + bx1) / 2, 0.0, zb)),
        cad.box(bx0, x0 - CLEAR, -by, by, z0 - 0.01, z0 + SIDE_H),
        cad.box(x1 + CLEAR, bx1, -by, by, z0 - 0.01, z0 + SIDE_H),
        cad.box(bx0, bx1, y1 + CLEAR, by, z0 - 0.01, z0 + END_H),
        cad.box(bx0, bx1, -by, -(y1 + CLEAR), z0 - 0.01, z0 + END_H),
        # the arm forward to the antenna's stay, and the stay: a hood with the
        # bulkhead hole in its top, open toward the unit for the cable
        cad.box(bx1 - 0.01, ax + sx / 2, ay - ARM_HALF_W, ay + ARM_HALF_W, zb, z0),
        cad.box(ax - sx / 2, ax + sx / 2, ay - sy / 2, ay + sy / 2, zb, stay_top()),
    ]
    body = cad.fuse_all(parts)
    head = fasteners.SHCS_HEAD["M3"]
    cutters = [cad.box(ax - sx / 2 - 1, ax + sx / 2 - 2.5, ay - sy / 2 + 2.5, ay + sy / 2 - 2.5, z0 - 0.01,
                       stay_top() - st),
               cad.cyl(H.BULKHEAD_HOLE, st + 2, (ax, ay, stay_top() - st - 1))]
    for x, y in F.HALOW_CRADLE_SCREWS:
        cutters.append(cad.clearance_hole("M3", x, y, zb - 1, z0 + 1))
        cutters.append(cad.cyl(max(CBORE[0], head[0] + 1.0), CBORE[1] + 0.01, (x, y, z0 - CBORE[1])))
    return cad.cut_all(body, cutters)
