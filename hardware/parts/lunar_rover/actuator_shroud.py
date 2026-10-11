"""Actuator shroud: a TPU cap pressed into the back of the actuator mount,
keeping regolith simulant off the AK70-10's driver board and connectors;
the actuator's cable leaves through a slot at the bottom."""
from hardware.parts.common import printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover import actuator_mount as M

PLUG_DEPTH = 6.0
PRESS = 0.15                  # interference on the plug's sides
CAP_T = 3.0
SLOT = (14.0, 9.0)            # cable slot at the bottom: width, height

PART = {
    "name": "Actuator shroud (TPU dust cap)",
    "quantity": 6,
    "material": "TPU95A",
    "material_reason": "flexible: presses into the cup and pulls out for service",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("TPU95A", "cap face on the plate, plug up", wall_loops=3, infill_pct=30,
                               rotate=[("X", -90)]),
    "mates_with": [
        {"part": "actuator_mount (printed)", "dimensions": f"teardrop bore {F.CUP_BORE:g} mm, {PRESS} mm press fit",
         "source": "hardware/parts/lunar_rover/actuator_mount.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [{"item": "zip tie 3.6 mm", "qty": 1, "use": "actuator cable to the leg, below the shroud slot"}],
    "fit_test": False,
    "notes": ["Seal the cable slot with a wrap of tape around the cable for regolith runs."],
}


def build():
    from hardware.parts.common import cad
    y0 = F.CUP_REAR_U
    cap = cad.cyl(2 * M.CUP_R, CAP_T, (0, y0 - CAP_T, 0), (0, 1, 0))
    plug = M.teardrop(F.CUP_BORE + 2 * PRESS, y0, y0 + PLUG_DEPTH, roof=M.BORE_ROOF + PRESS)
    hollow = M.teardrop(F.CUP_BORE - 6.0, y0 - 0.01, y0 + PLUG_DEPTH + 1, roof=M.BORE_ROOF - 3.0)
    body = cap.fuse(plug.cut(hollow)).clean()
    slot = cad.box(-SLOT[0] / 2, SLOT[0] / 2, y0 - CAP_T - 1, y0 + PLUG_DEPTH + 1,
                   -M.CUP_R - 1, -M.CUP_R + SLOT[1] + 4.0)
    return body.cut(slot).clean()
