"""Pan hub: the block on the pan servo's horn that carries the tilt bracket.
Its four key shafts reach the horn screws; four M3 inserts in its top face
take the tilt bracket."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret._footprints import ST3215 as S

PART = {
    "name": "Pan hub (on the pan horn)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "next to the pan servo, which runs warm under load; the RACECAR Turret's is in the sun",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upside down: the column's top face on the plate, horn disc up",
                               wall_loops=4, infill_pct=50, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "Waveshare ST3215 horn (pan servo)",
         "dimensions": f"4 x {S.HORN_THREAD} on a {S.HORN_HOLE_CIRCLE:g} mm circle on a dia. {S.HORN_D} mm horn; "
                       f"M3 x 5 through the {F.HUB_T:g} mm disc engage its {S.HORN_FLANGE_T} mm flange",
         "source": S.SOURCE, "verified": True},
        {"part": "Waveshare ST3215 horn thread", "dimensions": "M3 tapped (Feetech's horn drawing; Waveshare's drawing "
                                                                "does not state it)",
         "source": S.SOURCE, "verified": False},
        {"part": "tilt_bracket (printed)", "dimensions": "four M3 inserts in the top face",
         "source": "hardware/parts/turret/_frame.py", "verified": True},
    ],
    "inserts": [{"size": "M3", "qty": 4, "where": "column top face (tilt bracket screws)"}],
    "fasteners": [{"item": "M3 x 5 socket head cap screw", "qty": 4,
                   "use": "pan hub to the pan servo's horn (no longer: the tip must stay inside the horn flange)"}],
    "fit_test": True,
    "notes": [
        "Centre the pan servo (step 2048) and press its horn on with a hole pointing forward, then screw the hub on "
        "with the column's long side toward the rear.",
    ],
}


def build():
    from hardware.parts.common import cad
    z0 = F.PAN_HORN_Z
    z1 = z0 + F.HUB_T
    x0, x1, hw = F.COLUMN
    hub = cad.cyl(F.HUB_D, F.HUB_T + 0.01, (0, 0, z0))
    column = cad.box(x0, x1, -hw, hw, z1, F.COLUMN_TOP)
    body = hub.fuse(column).clean()
    cutters = []
    for x, y in F.horn_holes(F.PAN_HORN_HOLES_DEG):
        cutters.append(cad.clearance_hole("M3", x, y, z0 - 1, z1 + 1))
        if x + F.KEY_SHAFT_D / 2 < x1:         # under the column: a shaft for the hex key
            cutters.append(cad.cyl(F.KEY_SHAFT_D, F.COLUMN_TOP - z1 + 1, (x, y, z1)))
    for x, y in F.JOINT_INSERTS:
        cutters.append(cad.insert_hole("M3", x, y, F.COLUMN_TOP))
    return cad.cut_all(body, cutters)
