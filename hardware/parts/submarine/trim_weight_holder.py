"""Trim weight holder: a spigot under the bottom panel that carries up to
nine M16 stainless washers (about 11 g each, 10 g in sea water), held on by
the M5 screw through its centre; finer steps than Blue Robotics' 200 g
ballast weights for trimming pitch and roll after adding payload."""
from hardware.parts.common import printing
from hardware.parts.submarine import _frame as F
from hardware.parts.submarine._footprints import BlueROV2, Iso7089M16Washer

PART = {
    "name": "Trim weight holder",
    "quantity": 4,
    "material": "ASA",
    "material_reason": "under water: nearly neutral in sea water; printed solid so no air is trapped",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flange (panel face) on the plate, spigot up", wall_loops=5, infill_pct=100,
                               top_shell_layers=5, bottom_shell_layers=5, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "M16 stainless washers (ISO 7089)",
         "dimensions": f"{F.TRIM_SPIGOT[0]:.1f} mm spigot in their {Iso7089M16Washer.D1:g} mm holes, "
                       f"{F.TRIM_SPIGOT[1]:g} mm long: up to {F.TRIM_WASHERS_MAX} washers",
         "source": Iso7089M16Washer.SOURCE, "verified": True},
        {"part": "BlueROV2 bottom panel",
         "dimensions": f"{BlueROV2.BOTTOM_PANEL_T} mm HDPE, one M5 hole drilled through it at the spigot centre",
         "source": BlueROV2.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 55 socket head cap screw, A4 (316) stainless", "qty": 1, "use": "trim weight holder and its washers through the bottom panel"},
        {"item": "M5 x 20 mm fender (penny) washer, A4 (316) stainless", "qty": 1, "use": "retains the trim washers"},
        {"item": "M5 washer, A4 (316) stainless", "qty": 1, "use": "trim weight holder nut, on top of the bottom panel"},
        {"item": "M5 nyloc nut, A4 (316) stainless", "qty": 1, "use": "trim weight holder, on top of the bottom panel"},
        {"item": "M16 washer (ISO 7089), A4 (316) stainless", "qty": 5,
         "use": "trim weight, as needed (0-9 per holder; buy 20 to start)"},
    ],
    "fit_test": False,
    "notes": [
        "Fit spacers (more washers, or a short length of 16 mm tube) so the stack fills the spigot: the screw clamps the stack against the flange.",
        "Trim in fresh water first, then add about 2.5 % of the ROV's displacement for sea water if you move between them.",
    ],
}


def build():
    """In its own frame: spigot axis on z, the panel face at z = 0."""
    from hardware.parts.common import cad
    fd, ft = F.TRIM_FLANGE
    sd, sl = F.TRIM_SPIGOT
    body = cad.fuse_all([cad.cyl(fd, ft, (0, 0, -ft)), cad.cyl(sd, sl + 0.01, (0, 0, -ft - sl))])
    return body.cut(cad.cyl(F.M5_CLEARANCE, ft + sl + 2, (0, 0, -ft - sl - 1))).clean()
