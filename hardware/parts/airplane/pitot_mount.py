"""Pitot mount: a U-channel bonded on the nose that holds the Holybro PT60
pitot's base with one zip tie, the probe pointing forward into clean air
ahead of the nose (the X8's propeller is a pusher, so there is no prop wash
here)."""
from hardware.parts.airplane import _frame as F
from hardware.parts.airplane._footprints import HolybroAirspeedMS4525DO, SkywalkerX8
from hardware.parts.common import printing

POCKET = 30.6                 # square: takes the 28 x 30 mm base either way up
POCKET_LEN = 17.0
WALL = 3.0
FLOOR_T = 5.0
WALL_H = 20.0                 # above the floor: the tie goes over the top of the base
PAD = (60.0, 46.0, 3.0)       # bonding pad, x and y, thickness
TUNNEL = (3.4, 1.8)
TUBE_SLOT = (14.0, 9.0)       # rear wall slot for the two silicone tubes (width, height)

PART = {
    "name": "Pitot mount",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on the nose in full sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: bonding pad on the plate, channel open to the top", wall_loops=4, infill_pct=30),
    "mates_with": [
        {"part": "Holybro PT60 pitot tube",
         "dimensions": f"base about {HolybroAirspeedMS4525DO.PT60_OVERALL[1]:g} x {HolybroAirspeedMS4525DO.PT60_OVERALL[2]:g} mm "
                       f"in a {POCKET:g} mm square channel, {HolybroAirspeedMS4525DO.PT60_TUBE_LENGTH:g} mm probe ahead of it",
         "source": HolybroAirspeedMS4525DO.SOURCE, "verified": False},
        {"part": "Skywalker X8 nose",
         "dimensions": f"{PAD[0]:g} x {PAD[1]:g} mm pad bonded on the nose, centre line, the probe level with the "
                       "fuselage datum",
         "source": SkywalkerX8.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "zip tie 3.6 mm, 200 mm", "qty": 1, "use": "pitot base in the pitot mount, through the floor tunnel"},
    ],
    "fit_test": True,
    "notes": [
        "Probe parallel to the fuselage datum (the floor of the bay) and on the centre line; a probe tilted more than about 10 deg reads low.",
        "Route the two silicone tubes back to the airspeed module without kinks; the tube from the tip (total pressure) goes to the sensor's port marked for it.",
    ],
}


def build():
    from hardware.parts.common import cad
    x1 = F.PITOT_X                      # front face of the mount
    z0 = F.PITOT_Z                      # pad underside on the nose skin (position estimated)
    zf = z0 + PAD[2] + FLOOR_T          # channel floor
    half_in = POCKET / 2
    half_out = half_in + WALL
    x0 = x1 - POCKET_LEN - WALL
    pad = cad.rounded_plate(PAD[0], PAD[1], PAD[2], 6.0, center=(x1 - PAD[0] / 2, 0.0, z0))
    block = cad.box(x0, x1, -half_out, half_out, z0 + PAD[2] - 0.01, zf + WALL_H)
    body = cad.fuse_all([pad, block])
    cutters = [
        cad.box(x0 + WALL, x1 + 1, -half_in, half_in, zf, zf + WALL_H + 1),
        cad.box(x0 - 1, x0 + WALL + 0.01, -TUBE_SLOT[0] / 2, TUBE_SLOT[0] / 2, zf, zf + TUBE_SLOT[1]),
        # tie tunnel across the floor, under the pitot base
        cad.box(x1 - POCKET_LEN / 2 - TUNNEL[0] / 2, x1 - POCKET_LEN / 2 + TUNNEL[0] / 2, -half_out - 1, half_out + 1,
                zf - 1.6 - TUNNEL[1], zf - 1.6),
    ]
    return cad.cut_all(body, cutters)
