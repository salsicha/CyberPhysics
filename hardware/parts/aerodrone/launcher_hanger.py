"""Launcher hanger (aerodrone): carries the foam dart launcher under the hub,
its bore on the vertical axis through the hub centre. A flange under the
bottom plate on the four battery-tray inserts (the battery rides on the
deck with the launcher fitted), a keel down the launcher's front (toward
the camera, which it stays behind) to the cam guide on the breech, a
tapered rib for stiffness, a saddle on the plunger tube, and tie slots for
the servo cables. Toy power, foam darts only."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.turret import _launcher as L

TIE_GROOVE = 1.5                    # over the hub's zip ties (power module, 5 V supply)
SADDLE_GAP = 0.3                    # no clamping: the plunger slides inside the tube
TIE_SLOT = (3.2, 1.8)               # zip tie 2.5 mm
TIE_SLOTS = ((-45.0, (-29.0, -22.0)), (-45.0, (6.0, 12.5)), (-120.0, (6.0, 12.5)))   # (z, (y, y)) slot pairs in the web

PART = {
    "name": "Launcher hanger (flange, keel, rib)",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "carries the 0.43 kg launcher under the hub, outdoors in the sun",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("ASA", "upside down: the flange's top face on the plate, keel standing up (the rib's "
                               "slope and the saddle's 45 deg underside need no supports)",
                               wall_loops=4, infill_pct=25, top_shell_layers=5, bottom_shell_layers=5,
                               brim="outer brim only, 5 mm", rotate=[("X", 180)]),
    "mates_with": [
        {"part": "hub_bottom_plate (printed)", "dimensions": "four M3 inserts at x = -45/35 mm, y = +-25 mm (the battery "
                                                             "tray's)", "source": "hardware/parts/aerodrone/_frame.py",
         "verified": True},
        {"part": "launcher_cam_guide (printed)", "dimensions": "keel against its front face, four M3 x 8 into its inserts",
         "source": "hardware/parts/aerodrone/_frame.py", "verified": True},
        {"part": "turret launcher_tube (printed plunger tube)",
         "dimensions": f"saddle for the {L.PT_BORE + 2 * L.PT_WALL:.1f} mm tube, {SADDLE_GAP} mm clear, a zip tie round it",
         "source": "hardware/parts/turret/_launcher.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 10 socket head cap screw", "qty": 4,
         "use": "hanger flange to the bottom plate's battery-tray inserts"},
        {"item": "M3 x 8 socket head cap screw", "qty": 4, "use": "hanger keel to the cam guide's inserts"},
        {"item": "zip tie 2.5 mm", "qty": 4, "use": "plunger tube to the hanger's saddle (1); servo cables to the keel (3)"},
        {"item": "Dyneema or Kevlar cord, 1.5 mm, 300 mm", "qty": 1,
         "use": "safety lanyard: launcher (round the plunger tube, above the saddle) to the hub, slack"},
    ],
    "fit_test": False,
    "notes": ["Flight-critical in the sense that a failure drops a 0.43 kg launcher from the aircraft: check it for cracks "
              "and the eight screws for tightness before every flight, and fit the safety lanyard.",
              "The battery tray stays off while the launcher is fitted; the battery goes on the battery deck."],
}


def _flange_screws():
    return F.BATTERY_TRAY_INSERTS


def build():
    from hardware.parts.aerodrone import launcher_cam_guide as CG
    from hardware.parts.common import cad
    z_top = F.BOT_Z[0]
    z_f = z_top - F.HANGER_FLANGE_T
    fx0, fx1 = F.HANGER_FLANGE_X
    w = F.HANGER_FLANGE_HALF_W
    kx0, kx1 = F.KEEL_X
    ky0, ky1 = F.KEEL_Y
    flange = cad.rounded_plate(fx1 - fx0, 2 * w, F.HANGER_FLANGE_T, 6.0, center=((fx0 + fx1) / 2, 0.0, z_f))
    keel = cad.box(kx0, kx1, ky0, ky1, F.KEEL_BOTTOM_Z, z_f + 0.01)
    (rx_top, _), (rx_bot, rz_bot) = F.RIB_FRONT
    # its top runs out from under the flange at 45 deg (no overhang when printed upside down)
    rib = cad.profile_xz([(kx1 - 0.01, z_f + 0.01), (fx1, z_f + 0.01), (rx_top, z_f - (rx_top - fx1)),
                          (rx_bot, rz_bot), (kx1 - 0.01, rz_bot)], F.RIB_Y[0], F.RIB_Y[1])
    # Saddle on the plunger tube, its upper side at 45 deg toward the keel.
    tube_x = F.launcher_to_body((0.0, 0.0, L.PT_Z))[0]
    tube_r = (L.PT_BORE + 2 * L.PT_WALL) / 2
    zs0 = F.launcher_to_body((F.SADDLE_XL[1], 0, 0))[2]
    zs1 = F.launcher_to_body((F.SADDLE_XL[0], 0, 0))[2]
    front = tube_x + tube_r - 4.0
    sy = 12.0
    saddle = cad.profile_xz([(kx0 + 0.01, zs0), (front, zs0), (front, zs1), (kx0 + 0.01, zs1 + (kx0 - front))],
                            -sy, sy)
    body = cad.fuse_all([flange, keel, rib, saddle])

    cutters = [cad.cyl(tube_r * 2 + 2 * SADDLE_GAP, zs1 - zs0 + 30, (tube_x, 0.0, zs0 - 10))]
    # zip tie round the tube, through the saddle
    zt = (zs0 + zs1) / 2
    cutters.append(cad.box(kx0 - 6.0, kx0 - 3.0, -sy - 1, sy + 1, zt - 2.0, zt + 2.0))
    for x, y in _flange_screws():
        cutters.append(cad.clearance_hole("M3", x, y, z_f - 1, z_top + 1))
    # grooves in the top face for the zip ties of the hub's centre slots
    for x in (-12.0, 12.0):
        cutters.append(cad.box(x - 2.0, x + 2.0, -19.0, 19.0, z_top - TIE_GROOVE, z_top + 1))
    for y in (-16.0, 16.0):
        cutters.append(cad.box(-15.0, 15.0, y - 2.0, y + 2.0, z_top - TIE_GROOVE, z_top + 1))
    hx, hy, hd = F.HANGER_CABLE_HOLE
    cutters.append(cad.cyl(hd, F.HANGER_FLANGE_T + 2, (hx, hy, z_f - 1)))
    # keel to cam guide: through the web, heads on the front face
    for xl, yl in CG.keel_screws_local():
        _, by, bz = F.launcher_to_body((xl, yl, 0.0))
        cutters.append(cad.cyl(3.4, kx1 - kx0 + 2, (kx0 - 1, by, bz), (1, 0, 0)))
    for z, ys in TIE_SLOTS:
        for y in ys:
            cutters.append(cad.box(kx0 - 1, kx1 + 1, y - TIE_SLOT[1] / 2, y + TIE_SLOT[1] / 2, z - TIE_SLOT[0] / 2,
                                   z + TIE_SLOT[0] / 2))
    return cad.cut_all(body, cutters)
