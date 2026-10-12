"""TPU foot on the end of each tall leg's carbon tube (launcher
configuration): a socket pushed over the tube and a flat sole."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.aerodrone import tall_leg_socket as S
from hardware.parts.common import printing

SOLE = (28.0, 22.0, 7.0)            # along the arm, across it, thickness (outside the OAK-1's view)
SOCKET_DEPTH = 16.0
BOTTOM_T = 9.0                      # TPU under the tube's end, along the axis
OD = 20.0
PRESS = printing.PRESS_FIT_MM

PART = {
    "name": "Tall leg foot (TPU, launcher configuration)",
    "quantity": 4,
    "material": "TPU95A",
    "material_reason": "soft pad that takes the landing shock and grips the ground",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("TPU95A", "flat sole on the plate, socket leaning out", wall_loops=3, infill_pct=40),
    "mates_with": [
        {"part": f"carbon fibre tube {F.TALL_LEG_TUBE[0]:g} mm OD",
         "dimensions": f"socket {F.TALL_LEG_TUBE[0] + 2 * PRESS:.1f} mm (press fit), {SOCKET_DEPTH:g} mm deep",
         "source": "design requirement (purchased tube)", "verified": True},
    ],
    "inserts": [],
    "fasteners": [],
    "fit_test": False,
    "notes": ["Push onto the tube's end; a drop of CA glue if it works loose."],
}


def tube_end_s():
    """Distance along the leg axis from its top point to the tube's lower end."""
    return S.axis()[3] - BOTTOM_T


def build():
    from hardware.parts.common import cad
    ground = F.launcher_ground_z()
    s_end = tube_end_s()
    body = S.along(cad.cyl(OD, SOCKET_DEPTH + BOTTOM_T + 8.0, (0, 0, s_end - SOCKET_DEPTH)))
    rc = F.TALL_LEG_FOOT_R
    sole = cad.rounded_plate(SOLE[0], SOLE[1], SOLE[2], 5.0, center=(rc, 0.0, ground))
    foot = body.fuse(sole).clean()
    cutters = [cad.box(rc - 60, rc + 60, -40, 40, ground - 40, ground),
               S.along(cad.cyl(F.TALL_LEG_TUBE[0] + 2 * PRESS, SOCKET_DEPTH + 1, (0, 0, s_end - SOCKET_DEPTH - 1)))]
    return cad.cut_all(foot, cutters)
