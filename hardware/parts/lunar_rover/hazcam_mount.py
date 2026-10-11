"""Hazard camera mount: holds the RealSense D455 by the two M4 threads in
its back on the front of the body, pitched 20 deg down; its stereo pair is
the front hazard camera pair."""
import math

from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover._footprints import Extrusion2040, RealSenseD455

PLATE = (110.0, 26.0, 4.5)       # camera plate: width (y), height, thickness (M4 x 8 goes 3.5 mm in)
FOOT = (8.0, 110.0)              # foot on the rail's front face: thickness, width (y)
ARM_Y = (25.0, 35.0)             # the two arms, either side of the centre line (inside the screws)
SLOT = 1.0                       # M4 holes slotted +-1 mm up and down: the pair's height is not dimensioned

PART = {
    "name": "Hazard camera mount (RealSense D455)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "outdoors on the front of the rover, in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "on its side: the profile flat on the plate", wall_loops=5, infill_pct=40,
                               rotate=[("X", 90)]),
    "mates_with": [
        {"part": "RealSense D455", "dimensions": f"two M4 on the back {RealSenseD455.BACK_M4_SPACING:g} mm apart, "
                                                 f"max insertion {RealSenseD455.BACK_M4_MAX_INSERTION:g} mm",
         "source": RealSenseD455.SOURCE, "verified": True},
        {"part": "RealSense D455 M4 pair height", "dimensions": f"assumed on the case's centre line; slotted +-{SLOT} mm",
         "source": RealSenseD455.SOURCE, "verified": False},
        {"part": "2040 front top cross rail", "dimensions": "four M5 into drop-in T-nuts in the front face's two slots",
         "source": Extrusion2040.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M4 x 8 socket head cap screw", "qty": 2, "use": "D455 to the plate (3.5 mm into the camera; Intel's max 4 mm, 0.4 N m)"},
        {"item": "M5 x 12 button head screw + M5 drop-in T-nut (slot 6)", "qty": 4, "use": "mount to the front top cross rail"},
    ],
    "fit_test": True,
    "notes": ["Never use screws longer than M4 x 8 in the camera: Intel allows 4 mm of thread."],
}


def camera_frame():
    """(origin of the camera's back-face centre, x out of the lens, z up of the camera) in the rover frame."""
    p = math.radians(F.HAZCAM_PITCH_DEG)
    x_ax = (math.cos(p), 0.0, -math.sin(p))
    z_ax = (math.sin(p), 0.0, math.cos(p))
    origin = (F.BODY_X[1] + F.HAZCAM_FORWARD, 0.0, F.HAZCAM_Z)
    return origin, x_ax, z_ax


def build():
    """In the rover frame: the foot on the rail's front face (x = 300), the
    camera plate pitched 20 deg down ahead of it."""
    from hardware.parts.common import cad
    x_face = F.BODY_X[1]
    z_rail = (F.BODY_Z[1] - F.RAIL[1], F.BODY_Z[1])
    w = FOOT[1] / 2
    foot = cad.box(x_face, x_face + FOOT[0], -w, w, z_rail[0], z_rail[1])
    (ox, _, oz), xa, za = camera_frame()
    pw, ph, pt = PLATE
    # plate in the camera frame: behind the camera's back face
    plate = cad.box(-pt, 0.0, -pw / 2, pw / 2, -ph / 2, ph / 2)
    plate = cad.translate(cad.rotate(plate, "Y", F.HAZCAM_PITCH_DEG), (ox, 0.0, oz))
    # two arms from the foot to the plate's back, either side of the screws
    arms = []
    for y0 in (-ARM_Y[1], ARM_Y[0]):
        pts = [(x_face + 0.5, z_rail[0]), (x_face + 0.5, z_rail[1]),
               (ox - pt * xa[0] + ph / 2 * za[0], oz - pt * xa[2] + ph / 2 * za[2]),
               (ox - pt * xa[0] - ph / 2 * za[0], oz - pt * xa[2] - ph / 2 * za[2])]
        arms.append(cad.profile_xz(pts, y0, y0 + ARM_Y[1] - ARM_Y[0]))
    body = cad.fuse_all([foot, plate] + arms)
    cutters = []
    for y in (-RealSenseD455.BACK_M4_SPACING / 2, RealSenseD455.BACK_M4_SPACING / 2):
        s = cad.box(-pt - 1, 1.0, y - fasteners.CLEARANCE["M4"] / 2, y + fasteners.CLEARANCE["M4"] / 2, -SLOT, SLOT)
        s = s.fuse(cad.cyl(fasteners.CLEARANCE["M4"], pt + 2, (-pt - 1, y, -SLOT), (1, 0, 0)))
        s = s.fuse(cad.cyl(fasteners.CLEARANCE["M4"], pt + 2, (-pt - 1, y, SLOT), (1, 0, 0)))
        cutters.append(cad.translate(cad.rotate(s, "Y", F.HAZCAM_PITCH_DEG), (ox, 0.0, oz)))
    for z in (z_rail[0] + 10.0, z_rail[0] + 30.0):
        for y in (-w + 18.0, w - 18.0):
            cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], FOOT[0] + 2, (x_face - 1, y, z), (1, 0, 0)))
    return cad.cut_all(body, cutters)
