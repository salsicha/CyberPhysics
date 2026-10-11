"""Hub bottom plate: lower half of the four arm clamps, and the base the
landing legs, battery tray and sensor nose bolt to."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.common.footprints import CarbonTube16

PART = {
    "name": "Hub bottom plate",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "airframe exterior in the sun; carries every arm, the legs and the battery",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings(
        "ASA", "flat underside on the plate, clamp blocks up",
        wall_loops=4, infill_pct=25, top_shell_layers=4, bottom_shell_layers=4,
        brim="outer brim only, 5 mm"),
    "mates_with": [
        {"part": "16 mm carbon arm tube (x4)",
         "dimensions": f"tube OD {CarbonTube16.OD} mm in a {F.TUBE_BORE} mm half bore, tube end on the stop at r = {F.TUBE_R0} mm",
         "source": CarbonTube16.SOURCE, "verified": True},
        {"part": "hub_top_plate (printed)", "dimensions": "clamp blocks 1 mm apart when tightened",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
    ],
    "inserts": [
        {"size": "M3", "qty": 16, "where": "clamp blocks, from the top face (arm clamp bolts)"},
        {"size": "M3", "qty": 8, "where": "underside of the ears, two per arm (landing legs)"},
        {"size": "M3", "qty": 4, "where": "underside, rear and middle bosses (battery tray)"},
        {"size": "M3", "qty": 4, "where": "underside, front bosses (sensor nose)"},
    ],
    "fasteners": [
        {"item": "zip tie 2.5 mm", "qty": 4, "use": "power sense module and 5 V BEC to the four centre slots"},
    ],
    "fit_test": False,
    "notes": [
        "Press the 32 M3 x 5.7 inserts before assembly: clamp-block inserts from the top, the rest from the underside.",
        "The bores print as open half-channels; clear any stringing so the tube seats on the stop.",
    ],
}


def build():
    from hardware.parts.aerodrone import _hub
    from hardware.parts.common import cad

    z0, z1 = F.BOT_Z
    plate = _hub.outline(z0, z1)
    parts = [plate] + _hub.blocks(z1, -F.CLAMP_GAP / 2)
    for x, y in F.BATTERY_TRAY_INSERTS + F.NOSE_INSERTS:
        parts.append(cad.cyl(F.POST_BOSS_D, F.POST_BOSS_H, (x, y, z1)))
    body = cad.fuse_all(parts)

    cutters = list(_hub.bores())
    for x, y in _hub.clamp_bolt_xy():
        cutters.append(cad.insert_hole("M3", x, y, -F.CLAMP_GAP / 2, direction=-1))
    for x, y in _hub.leg_bolt_xy() + list(F.BATTERY_TRAY_INSERTS + F.NOSE_INSERTS):
        cutters.append(cad.insert_hole("M3", x, y, z0, direction=+1))
    # Battery lead up into the hub, lightening holes, zip-tie slots.
    cutters.append(cad.slot(-63.0, -10.0, -63.0, 10.0, 10.0, z0 - 1, z1 + 1))
    for x, y in ((-5.0, 46.0), (-5.0, -46.0), (30.0, 0.0), (-30.0, 0.0)):
        cutters.append(cad.cyl(20.0, F.PLATE_T + 2, (x, y, z0 - 1)))
    for x in (-12.0, 12.0):
        for y in (16.0, -16.0):
            cutters.append(cad.slot(x - 2.5, y, x + 2.5, y, 3.0, z0 - 1, z1 + 1))
    return cad.cut_all(body, cutters)
