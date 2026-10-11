"""Grouser: a bolt-on tread bar across the rim (two per segment, 16 per
wheel). Separate so that worn grousers are replaced without reprinting
the wheel, and so the segments print without supports."""
from hardware.parts.common import printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover import _wheel as W

HEIGHT = F.GROUSER_TIP_R - F.RIM_OUTER_R      # 12 mm
LENGTH = F.RIM_U[1] - F.RIM_U[0]               # 120 mm, the rim's width

PART = {
    "name": "Grouser (tread bar)",
    "quantity": 6 * F.SEGMENTS * F.GROUSERS_PER_SEGMENT,
    "material": "ASA",
    "material_reason": "outdoors, abrasive regolith simulant; replaceable wear part",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "base on the plate", wall_loops=4, infill_pct=60),
    "mates_with": [
        {"part": "wheel_segment rim (printed)", "dimensions": f"flat base on the {F.RIM_OUTER_R:g} mm rim; two M3 inserts",
         "source": "hardware/parts/lunar_rover/wheel_segment.py", "verified": True},
    ],
    "inserts": [{"size": "M3", "qty": 2, "where": "base, from below"}],
    "fasteners": [{"item": "M3 x 8 socket head cap screw", "qty": 2,
                   "use": "from inside the rim into the grouser (3 mm rim, 5 mm into the 5.7 mm insert)"}],
    "fit_test": False,
    "notes": [f"Tip at {F.GROUSER_TIP_R:g} mm from the axle: the simulator's 0.18 m wheel radius."],
}


def build():
    from hardware.parts.common import cad
    b, t = F.GROUSER_BASE_W / 2, F.GROUSER_TIP_W / 2
    bar = cad.profile_xz([(-b, 0.0), (b, 0.0), (t, HEIGHT), (-t, HEIGHT)], 0.0, LENGTH)
    holes = [cad.insert_hole("M3", 0.0, y - F.RIM_U[0], 0.0, direction=1) for y in W.GROUSER_BOLT_Y]
    return cad.cut_all(bar, holes)
