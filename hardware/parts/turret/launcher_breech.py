"""Launcher breech: the core of the foam dart launcher (toy power, foam
darts only). The sliding breech block's channel, the plunger tube's socket
at the rear, the barrel's seat at the front; on the turret a pedestal
under it stands on the camera yoke's pad (hardware/parts/turret/_launcher.py)."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret import _launcher as L
from hardware.parts.turret._footprints import ST3215 as S  # noqa: F401

PART = {
    "name": "Launcher breech (with the pedestal on the camera yoke)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "carries the launcher on the tilt stage; outdoors on the RACECAR Turret",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: the pedestal's underside on the plate, channel open upward",
                               wall_loops=4, infill_pct=25),
    "mates_with": [
        {"part": "K&S Precision Metals 8140 brass tube (barrel)",
         "dimensions": f"17/32 in OD ({L.BARREL['od']} mm) in a {L.BARREL['od'] + 0.2:.2f} mm seat, ID 0.503 in on the "
                       "bore; its rear face flush with the chamber's front face",
         "source": "ksmetals.com, round brass tube 17/32 in x 0.014 in wall, SKU 8140 (OD/ID +-0.002 in)",
         "verified": True},
        {"part": "launcher_tube (printed)", "dimensions": f"outlet spigot {L.SPIGOT_D} mm with a 10 x 2 O-ring in a "
                                                           f"{L.SPIGOT_D + 0.3:.1f} mm socket; four M3 inserts for its flange",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
        {"part": "launcher_shuttle, launcher_clip, launcher_barrel_clamp (printed)",
         "dimensions": f"channel {L.CHANNEL_Y[1] - L.CHANNEL_Y[0]:.1f} x {L.CHAMBER[1] - L.CHAMBER[0]:g} mm, "
                       f"{L.SHUTTLE_CLEAR} mm clearance; four M3 inserts in the wall bosses; two in the front wall",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
        {"part": "camera_yoke (printed)", "dimensions": "two M3 from below the yoke's pad into the pedestal's inserts",
         "source": "hardware/parts/turret/_frame.py", "verified": True},
    ],
    "inserts": [{"size": "M3", "qty": 2, "where": "rear plate, lower flange screws"},
                {"size": "M3-short", "qty": 2, "where": "rear plate, upper flange screws"},
                {"size": "M3", "qty": 4, "where": "wall bosses, cover screws, from the top"},
                {"size": "M3", "qty": 2, "where": "front wall, barrel clamp screws"},
                {"size": "M3", "qty": 2, "where": "pedestal underside, yoke pad screws"}],
    "fasteners": [],
    "fit_test": True,
    "notes": ["Print it with the shuttle and the clip first: the shuttle must slide its 17 mm in the channel with the "
              "cover screwed on, without play or a tight spot (sand the shuttle, not the channel)."],
}


def build():
    return F.launcher_to_turret(L.breech(F.PEDESTAL))
