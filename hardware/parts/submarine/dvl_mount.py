"""DVL mount: a slotted collar that holds the Water Linked DVL A50 by its
66 mm housing, squeezed by a stainless hose clamp, back plate up against a
flange that is through-bolted under the BlueROV2's bottom panel; beams
down, cable out through the widest slot."""
import math

from hardware.parts.common import printing
from hardware.parts.submarine import _frame as F
from hardware.parts.submarine._footprints import BlueROV2, WaterLinkedDVLA50

WALL = 3.0
FINGER_SLOTS = 6                  # including the cable slot
SLOT_W = 2.0
SLOT_TOP = 3.0                    # the fingers start this far below the flange
CABLE_SLOT_W = WaterLinkedDVLA50.CABLE_D + 2.2
CLAMP_BAND = (10.0, 0.8)          # groove for a 9 mm hose-clamp band: width, depth
DRAIN_D = 10.0

PART = {
    "name": "DVL mount (Water Linked A50 collar)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": ("under water: ASA (about 1.05 g/cm3) is nearly neutral in sea water, so the ROV's trim "
                        "hardly changes; printed solid so no air is trapped"),
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upside down: the flange's panel face on the plate, collar up",
                               wall_loops=5, infill_pct=100, top_shell_layers=5, bottom_shell_layers=5,
                               rotate=[("X", 180)]),
    "mates_with": [
        {"part": "Water Linked DVL A50",
         "dimensions": f"dia. {WaterLinkedDVLA50.DIAMETER:g} mm housing in a {WaterLinkedDVLA50.DIAMETER + 2 * F.DVL_FIT:.1f} mm "
                       f"slotted collar {F.DVL_RING_H:g} mm tall, back plate on the flange, {WaterLinkedDVLA50.HEIGHT:g} mm tall housing",
         "source": WaterLinkedDVLA50.SOURCE, "verified": True},
        {"part": "Water Linked DVL A50 cable exit",
         "dimensions": f"{CABLE_SLOT_W:.1f} mm slot the full height of the collar for the "
                       f"{WaterLinkedDVLA50.CABLE_D} mm cable (where it leaves the housing is not dimensioned)",
         "source": WaterLinkedDVLA50.SOURCE, "verified": False},
        {"part": "BlueROV2 bottom panel",
         "dimensions": f"{BlueROV2.BOTTOM_PANEL_T} mm (1/2 in) HDPE; four M5 holes drilled through it 80 x 80 mm "
                       "apart with the flange as the template",
         "source": BlueROV2.SOURCE, "verified": True},
        {"part": "BlueROV2 bottom panel: clear area under its centre",
         "dimensions": f"{F.DVL_FLANGE[0]:g} x {F.DVL_FLANGE[1]:g} mm, clear of the ballast weights",
         "source": BlueROV2.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 30 socket head cap screw, A4 (316) stainless", "qty": 4, "use": "DVL mount through the bottom panel"},
        {"item": "M5 washer, A4 (316) stainless", "qty": 8, "use": "under the DVL mount screw heads and nuts"},
        {"item": "M5 nyloc nut, A4 (316) stainless", "qty": 4, "use": "DVL mount, on top of the bottom panel"},
        {"item": "hose clamp, 9 mm band, 60-80 mm range, W5 (316) stainless", "qty": 1, "use": "DVL collar"},
    ],
    "fit_test": True,
    "notes": [
        "Print solid (100 % infill): sparse infill traps air that floods slowly or is crushed at depth, and the ROV's trim drifts.",
        "No brass inserts under water: 316 screws, nuts and clamp only; rinse in fresh water after every salt-water dive.",
        "Tighten the hose clamp until the DVL cannot turn by hand, no more: the PEEK housing does not need force.",
        "Set the DVL's mounting rotation in the Water Linked DVL BlueOS extension to match the cable slot's direction (aft here).",
    ],
}


def build():
    from hardware.parts.common import cad
    cx, cy = F.DVL_XY
    fx, fy, ft = F.DVL_FLANGE
    r_in = WaterLinkedDVLA50.DIAMETER / 2 + F.DVL_FIT
    r_out = r_in + WALL
    z_ring = F.DVL_BACK_Z - F.DVL_RING_H
    flange = cad.rounded_plate(fx, fy, ft, 8.0, center=(cx, cy, F.DVL_BACK_Z))
    ring = cad.cyl(2 * r_out, F.DVL_RING_H + 0.01, (cx, cy, z_ring))
    body = flange.fuse(ring).clean()
    z_slot = F.DVL_BACK_Z - SLOT_TOP
    cutters = [cad.cyl(2 * r_in, F.DVL_RING_H + 1, (cx, cy, z_ring - 1)),
               cad.cyl(DRAIN_D, ft + 2, (cx, cy, F.DVL_BACK_Z - 1))]
    # band groove near the bottom of the collar
    band = cad.cyl(2 * r_out + 2, CLAMP_BAND[0], (cx, cy, z_ring + 2.0)).cut(
        cad.cyl(2 * (r_out - CLAMP_BAND[1]), CLAMP_BAND[0] + 2, (cx, cy, z_ring + 1.0)))
    cutters.append(band)
    for k in range(FINGER_SLOTS):
        a = 180.0 + k * 360.0 / FINGER_SLOTS          # k = 0 is the cable slot, aft
        w = CABLE_SLOT_W if k == 0 else SLOT_W
        top = F.DVL_BACK_Z - 0.01 if k == 0 else z_slot
        slot = cad.box(r_in - 1, r_out + 2, -w / 2, w / 2, z_ring - 1, top)
        cutters.append(cad.translate(cad.rotate_z(slot, a), (cx, cy, 0)))
    for x, y in F.DVL_BOLTS:
        cutters.append(cad.cyl(F.M5_CLEARANCE, ft + 2, (x, y, F.DVL_BACK_Z - 1)))
    assert math.isclose(z_ring, F.DVL_BACK_Z - F.DVL_RING_H)
    return cad.cut_all(body, cutters)
