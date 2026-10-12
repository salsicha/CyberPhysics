"""Top deck above the Pi 5 + Navigator stack, on four posts on inserts in
the top plate. It carries the HaLowLink 2's cradle at its front in both
configurations and, with the foam dart launcher under the hub, the 4S pack
behind it, across the aircraft: two strap slot pairs, a notch round the
GPS mast. Slide the pack fore or aft (and sideways) on the deck to put the
centre of gravity on the hub centre."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import fasteners, printing
from hardware.parts.common.footprints import LipoEnvelope4S

STRAP_SLOT = (3.5, 22.0)            # along x, along y: 20 mm hook-and-loop straps
STRAP_X = (-72.0, 0.0)              # behind and in front of the pack (the rear pair beside the mast column)
STRAP_Y = 61.0                      # near the pack's ends, outside the posts: the straps run under the deck clear of them
RIB_T = 3.0
RIB_H = 6.0                         # under the deck: along the posts (y = +-42) and across behind the stack (x = -40)
MAST_CLEAR_D = 24.0
SCREW = "M3 x 50 socket head cap screw"
CBORE = (6.5, 5.0)

PART = {
    "name": "Top deck (HaLowLink 2; the battery with the launcher)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on top of the aircraft in full sun, under a pack that reaches about 60 C",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("ASA", "upside down: the deck's top face on the plate, posts up",
                               wall_loops=3, infill_pct=20, rotate=[("X", 180)]),
    "mates_with": [
        {"part": "hub_top_plate (printed)", "dimensions": f"four M3 inserts at {F.DECK_POSTS}",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
        {"part": "4S LiPo pack, across the aircraft",
         "dimensions": f"up to {LipoEnvelope4S.MAX_SIZE[0]:g} x {LipoEnvelope4S.MAX_SIZE[1]:g} x "
                       f"{LipoEnvelope4S.MAX_SIZE[2]:g} mm, its {LipoEnvelope4S.MAX_SIZE[1]:g} mm side between the "
                       "strap slots", "source": LipoEnvelope4S.SOURCE, "verified": True},
        {"part": "Navigator connector envelope (plugged leads)",
         "dimensions": f"deck underside {F.DECK_Z[0]:g} mm above the arm plane, 4.8 mm over the 15 mm budget",
         "source": "hardware/parts/common/footprints.py (BlueRoboticsNavigator.CONNECTOR_HEIGHT, unverified)",
         "verified": False},
        {"part": "halow_cradle (printed)", "dimensions": f"M3 short inserts at {F.HALOW_CRADLE_SCREWS}",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
    ],
    "inserts": [{"size": "M3-short", "qty": len(F.HALOW_CRADLE_SCREWS),
                 "where": "top face, HaLowLink cradle screws, pressed flush (through holes)"}],
    "fasteners": [
        {"item": SCREW, "qty": 4, "use": "top deck posts to the top plate inserts (heads recessed in the deck)"},
    ],
    "fit_test": True,
    "notes": ["With the launcher: a lost battery is a crash; two straps, each through its own slot pair, checked before "
              "every flight (the tray's two straps and anti-slip pad move to the deck).",
              "Check before printing that the plugged Navigator leads stay under the deck (the connector height is "
              "a budget, not a drawing).",
              "The battery lead runs down beside the stack, through the top plate's side window, to the power module."],
}


def build():
    from hardware.parts.common import cad
    x0, x1 = F.DECK_X
    z0, z1 = F.DECK_Z
    w = F.DECK_HALF_W
    xm, xt = F.DECK_MID_X, F.DECK_TONGUE_X
    ty0, ty1 = F.DECK_TONGUE_Y
    deck = cad.fuse_all([
        cad.rounded_plate(xm - x0, 2 * w, z1 - z0, 3.0, center=((x0 + xm) / 2, 0.0, z0)),
        cad.rounded_plate(xt - xm + 6.0, 2 * F.DECK_MID_HALF_W, z1 - z0, 3.0, center=((xm - 6.0 + xt) / 2, 0.0, z0)),
        cad.rounded_plate(x1 - xt + 6.0, ty1 - ty0, z1 - z0, 3.0, center=((xt - 6.0 + x1) / 2, (ty0 + ty1) / 2, z0)),
    ])
    parts = [deck]
    for x, y in F.DECK_POSTS:
        parts.append(cad.cyl(F.DECK_POST_D, z0 - F.TOP_Z[1] + 0.01, (x, y, F.TOP_Z[1])))
    # stiffening ribs under the deck, clear of the stack
    for y in (F.DECK_POSTS[0][1], F.DECK_POSTS[1][1]):
        parts.append(cad.box(x0 + 4, F.DECK_TONGUE_X - 4, y - RIB_T / 2, y + RIB_T / 2, z0 - RIB_H, z0 + 0.01))
    ry = abs(F.DECK_POSTS[0][1]) + 4.0
    parts.append(cad.box(F.DECK_POSTS[0][0] - RIB_T / 2, F.DECK_POSTS[0][0] + RIB_T / 2, -ry, ry, z0 - RIB_H, z0 + 0.01))
    body = cad.fuse_all(parts)
    cutters = [cad.cyl(MAST_CLEAR_D, z1 - z0 + RIB_H + 2, (F.MAST_COLUMN_XY[0], F.MAST_COLUMN_XY[1], z0 - RIB_H - 1))]
    head_d = fasteners.SHCS_HEAD["M3"][0]
    for x, y in F.DECK_POSTS:
        cutters.append(cad.clearance_hole("M3", x, y, F.TOP_Z[1] - 1, z1 + 1))
        cutters.append(cad.cyl(max(CBORE[0], head_d + 1.0), CBORE[1] + 0.01, (x, y, z1 - CBORE[1])))
    for x in STRAP_X:
        for y in (STRAP_Y, -STRAP_Y):
            cutters.append(cad.cbox(STRAP_SLOT[0], STRAP_SLOT[1], 30, center=(x, y, z0)))
    for x, y in F.HALOW_CRADLE_SCREWS:
        cutters.append(cad.cyl(fasteners.INSERTS["M3-short"].hole, z1 - z0 + 2, (x, y, z0 - 1)))
    return cad.cut_all(body, cutters)
