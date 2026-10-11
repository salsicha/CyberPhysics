"""GNSS and lux head: on the top of the 200 mm rear mast, above the solar
panel: a 100 mm steel disc under the magnetic-base GNSS antenna (its
ground plane; GNSS is for Earth testing only), and a pocket on a rear tab
for the VEML7700 lux sensor (/lunar_rover/solar/illumination), under a
clear cover, looking straight up."""
from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover._footprints import AdafruitVEML7700, Extrusion2020, GnssGroundPlate

FIT = 0.4
WALL = 6.0
SOCKET_H = 30.0
DISC_T = 4.0
DISC_D = 104.0               # the 100 mm steel disc sits on it, centred by eye, on foam tape
CONE_TOP_R = 48.0            # cone from part way up the socket to the disc
CONE_START = -12.0           # the socket is bare below this: the T-nut screws go through it there
TAB = (34.0, 32.0, 10.0)     # lux tab toward -x: reach beyond the disc, width, thickness
POCKET = (27.0, 19.5, 5.5)   # VEML7700 board pocket (board up to 26 x 18 mm)
COVER = (32.0, 25.0, 2.0)    # clear acrylic cover recess over the pocket

PART = {
    "name": "GNSS and lux head (rear mast)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "outdoors at the top of the mast, in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upside down: the disc's top face on the plate, cone and socket up",
                               wall_loops=4, infill_pct=30, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "steel disc (antenna ground plane)", "dimensions": "{:g} x {:g} mm on the 104 mm top".format(*GnssGroundPlate.DISC),
         "source": GnssGroundPlate.SOURCE, "verified": True},
        {"part": "Adafruit VEML7700 lux sensor board", "dimensions": "pocket for a board up to 26 x 18 x 5 mm",
         "source": AdafruitVEML7700.SOURCE, "verified": False},
        {"part": "2020 mast", "dimensions": f"{Extrusion2020.SIZE + FIT:g} mm socket, two M5 into drop-in T-nuts",
         "source": Extrusion2020.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 10 button head screw + M5 drop-in T-nut (slot 6)", "qty": 2, "use": "head to the mast"},
        {"item": "double-sided foam tape (VHB), 100 mm disc", "qty": 1, "use": "steel disc on the head"},
        {"item": "clear acrylic cover, 32 x 25 x 2 mm, and clear silicone", "qty": 1, "use": "over the lux sensor pocket"},
    ],
    "fit_test": True,
    "notes": ["Antenna arrow forward; its cable down the mast in two cable clips.",
              "The lux sensor sits beside the antenna, toward the rear: the antenna shades it only for sun below about 40 deg ahead of the rover. Calibrate its reading through the cover against a lux meter."],
}


def build():
    """Local frame: the mast's top end at z = 0, its axis on z."""
    import cadquery as cq

    from hardware.parts.common import cad
    s = Extrusion2020.SIZE + FIT
    so = s / 2 + WALL
    socket = cad.box(-so, so, -so, so, -SOCKET_H, 2.0)
    cone = (cq.Workplane("XY", origin=(0, 0, CONE_START)).rect(2 * so, 2 * so)
            .workplane(offset=2.0 - CONE_START).circle(CONE_TOP_R).loft(combine=True)).val()
    disc = cad.cyl(DISC_D, DISC_T, (0, 0, 2.0))
    tx, tw, tt = TAB
    z_top = 2.0 + DISC_T
    tab = cad.box(-DISC_D / 2 - tx, -DISC_D / 2 + 6.0, -tw / 2, tw / 2, z_top - tt, z_top)
    body = cad.fuse_all([socket, cone, disc, tab])
    cutters = [cad.box(-s / 2, s / 2, -s / 2, s / 2, -SOCKET_H - 1, 0.0)]
    pc = -DISC_D / 2 - tx / 2 + 2.0
    cutters.append(cad.cbox(POCKET[0], POCKET[1], POCKET[2] + 0.01, center=(pc, 0.0, z_top - COVER[2] - POCKET[2] / 2)))
    cutters.append(cad.cbox(COVER[0], COVER[1], COVER[2] + 0.01, center=(pc, 0.0, z_top - COVER[2] / 2)))
    cutters.append(cad.cyl(6.0, tt + 2, (pc + POCKET[0] / 2 - 4.0, 0.0, z_top - tt - 1)))   # cable down
    z = (CONE_START - SOCKET_H) / 2
    cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], WALL + 2, (so + 1, 0.0, z), (-1, 0, 0)))
    cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], WALL + 2, (0.0, so + 1, z), (0, -1, 0)))
    return cad.cut_all(body, cutters)
