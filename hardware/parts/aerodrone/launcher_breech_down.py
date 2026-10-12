"""Launcher breech for the aerodrone (toy power, foam darts only): the
turret's breech core (hardware/parts/turret/_launcher.py, breech()) without
the turret's pedestal, with four M3 inserts underneath for the cam guide
(BREECH_UNDER_SCREWS: in the rear plate and the front wall). Every other launcher part is the turret's (printed from
hardware/parts/turret/stl/). Built in place: the bore vertical, muzzle
down."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.turret import _launcher as L

PART = {
    "name": "Launcher breech, downward (no pedestal; aerodrone)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "carries the launcher under the hub, outdoors in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: the underside (the cam guide's face) on the plate, channel open upward",
                               wall_loops=4, infill_pct=25, rotate=[("Y", 90)]),
    "mates_with": [
        {"part": "K&S Precision Metals 8140 brass tube (barrel)",
         "dimensions": f"17/32 in OD ({L.BARREL['od']} mm) in a {L.BARREL['od'] + 0.2:.2f} mm seat, ID 0.503 in on the "
                       f"bore; on the aerodrone its rear face stops on the {F.DETENT_T:g} mm dart detent ring",
         "source": "ksmetals.com, round brass tube 17/32 in x 0.014 in wall, SKU 8140 (OD/ID +-0.002 in)",
         "verified": True},
        {"part": "turret launcher_tube, launcher_shuttle, launcher_clip, launcher_barrel_clamp (printed)",
         "dimensions": "the turret breech's channel, sockets and inserts, unchanged",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
        {"part": "launcher_cam_guide (printed)",
         "dimensions": f"four M3 inserts underneath at (x_L, y_L) = {F.BREECH_UNDER_SCREWS}",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
    ],
    "inserts": [{"size": "M3", "qty": 2, "where": "rear plate, lower flange screws"},
                {"size": "M3-short", "qty": 2, "where": "rear plate, upper flange screws"},
                {"size": "M3", "qty": 4, "where": "wall bosses, cover screws, from the top"},
                {"size": "M3", "qty": 2, "where": "front wall, barrel clamp screws"},
                {"size": "M3", "qty": 4, "where": "underside, cam guide screws"}],
    "fasteners": [],
    "fit_test": True,
    "notes": ["Same fit checks as the turret's breech: the shuttle must slide its 17 mm in the channel with the cover "
              "screwed on, without play or a tight spot (sand the shuttle, not the channel).",
              "Use this breech, not the turret's: the turret's has the pedestal for the camera yoke."],
}


def build():
    return L.place(L.breech(under_screws=F.BREECH_UNDER_SCREWS), F.LAUNCHER_ORIGIN, F.LAUNCHER_ROT)
