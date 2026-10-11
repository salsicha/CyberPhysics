"""Cable clip: a small pad bonded to the foam with a zip-tie bridge; holds
the OAK-1 USB cable, the servo leads and the pitot tubes along the bay
walls, away from the GPS and from anything that moves."""
from hardware.parts.common import printing

PAD = (22.0, 16.0, 2.0)
BRIDGE = (8.0, 10.0)          # along the tie, across it
TUNNEL = (4.0, 2.0)           # width, height: 3.6 mm ties
ROOF = 1.6

PART = {
    "name": "Cable clip (bonded)",
    "quantity": 8,
    "material": "PETG",
    "material_reason": "inside the bay; PETG bonds well with epoxy and does not crack when the tie is pulled",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "pad on the plate, bridge up", wall_loops=3, infill_pct=100),
    "mates_with": [
        {"part": "EPO foam (any flat or gently curved spot)", "dimensions": f"{PAD[0]:g} x {PAD[1]:g} mm bonded pad",
         "source": "requirement of this design", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "zip tie 3.6 mm, 150 mm", "qty": 1, "use": "cables to the cable clip"},
    ],
    "fit_test": False,
    "notes": [
        "Bond with foam-safe epoxy or hot glue; at least one clip within 50 mm of the camera frame so the USB cable cannot pull the camera off its dampers (leave a loose loop between the clip and the camera).",
    ],
}


def build():
    from hardware.parts.common import cad
    pad = cad.rounded_plate(PAD[0], PAD[1], PAD[2], 3.0, center=(0, 0, 0))
    h = TUNNEL[1] + ROOF
    bridge = cad.cbox(BRIDGE[0], BRIDGE[1], h + 0.01, center=(0, 0, PAD[2] + h / 2 - 0.005))
    body = pad.fuse(bridge).clean()
    tunnel = cad.cbox(BRIDGE[0] + 2, TUNNEL[0], TUNNEL[1] + 0.01, center=(0, 0, PAD[2] + (TUNNEL[1] + 0.01) / 2))
    return body.cut(tunnel).clean()
