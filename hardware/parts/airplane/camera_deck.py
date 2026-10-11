"""Camera deck: screwed onto the camera frame's pillars; the OAK-1 cradle
hangs from it on four silicone damper balls."""
from hardware.parts.airplane import _frame as F
from hardware.parts.common import fasteners, printing
from hardware.parts.common.footprints import SiliconeDamperBall

WINDOW_HALF = (15.0, 30.0)     # over the camera: tripod screw access, cable room
POCKET_D = 13.0
CORNER_R = 6.0

PART = {
    "name": "Camera deck (OAK-1 damper deck)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "sits over the warm OAK-1 inside the bay; ASA keeps its shape",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "flat underside on the plate", wall_loops=4, infill_pct=30),
    "mates_with": [
        {"part": "silicone gimbal damper balls (x4)",
         "dimensions": f"dia. {SiliconeDamperBall.HOLE_D} mm holes in {SiliconeDamperBall.PLATE_THICKNESS:g} mm tabs, "
                       f"{SiliconeDamperBall.PLATE_SPACING:g} mm between the tabs",
         "source": SiliconeDamperBall.SOURCE, "verified": False},
        {"part": "camera_frame (printed)", "dimensions": "four M3 screws into the pillar inserts",
         "source": "hardware/parts/airplane/_frame.py", "verified": True},
        {"part": "oak1_cradle (printed)", "dimensions": "damper positions, 11 mm ball spacing",
         "source": "hardware/parts/airplane/_frame.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 10 socket head cap screw", "qty": 4, "use": "camera deck to the frame pillars"},
        {"item": "silicone gimbal damper ball, 6.5 mm neck", "qty": 4, "use": "camera cradle isolation"},
    ],
    "fit_test": True,
    "notes": [
        "Snap the balls into the deck from below, the cradle (with the camera on it) onto the balls, then screw the deck onto the pillars.",
    ],
}


def build():
    from hardware.parts.common import cad
    cx = F.CAMERA_X
    hx, hy = F.FRAME_HALF
    d0, d1 = F.FRAME_DECK_Z
    deck = cad.rounded_plate(2 * hx, 2 * hy, d1 - d0, CORNER_R, center=(cx, 0.0, d0))
    tab_top = d0 + F.DAMPER_TAB_T
    cutters = [cad.box(cx - WINDOW_HALF[0], cx + WINDOW_HALF[0], -WINDOW_HALF[1], WINDOW_HALF[1], d0 - 1, d1 + 1)]
    for x, y in F.DAMPERS:
        cutters.append(cad.cyl(SiliconeDamperBall.HOLE_D, d1 - d0 + 2, (x, y, d0 - 1)))
        cutters.append(cad.cyl(POCKET_D, d1 - tab_top + 1, (x, y, tab_top)))
    for x, y in F.pillar_xy():
        cutters.append(cad.clearance_hole("M3", x, y, d0 - 1, d1 + 1))
    assert fasteners.SHCS_HEAD["M3"][0] < F.PILLAR
    return cad.cut_all(deck, cutters)
