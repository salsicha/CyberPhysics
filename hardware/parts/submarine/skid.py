"""Skid: one of two rails through-bolted under the BlueROV2's bottom panel.
They stand the ROV 40 mm off the deck and the seabed so the DVL under the
panel never takes the vehicle's weight; their ends are ramped so they slide
over edges."""
from hardware.parts.common import printing
from hardware.parts.submarine import _frame as F
from hardware.parts.submarine._footprints import BlueROV2

COUNTERBORE = (11.0, 28.0)        # diameter, depth from the skid's bottom: screw head and M5 washer recessed
RAMP = 20.0                       # 45 deg ramp at each end of the bottom edge
WINDOWS = ((-75.0, -25.0), (25.0, 75.0))
WINDOW_Z = (-30.0, -16.0)

PART = {
    "name": "Skid (DVL guard)",
    "quantity": 2,
    "material": "ASA",
    "material_reason": ("under water and on a sunny deck: ASA is UV-stable and nearly neutral in sea water; "
                        "printed solid so no air is trapped"),
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "on its side: one 240 x 40 mm face on the plate (layers along the rail)",
                               wall_loops=5, infill_pct=100, top_shell_layers=5, bottom_shell_layers=5,
                               rotate=[("X", 90)]),
    "mates_with": [
        {"part": "BlueROV2 bottom panel",
         "dimensions": f"{BlueROV2.BOTTOM_PANEL_T} mm (1/2 in) HDPE; three M5 holes per skid drilled through it with "
                       "the skid as the template",
         "source": BlueROV2.SOURCE, "verified": True},
        {"part": "BlueROV2 bottom panel: room for the skids",
         "dimensions": f"rails {F.SKID[0]:g} mm long at y = +-{abs(F.SKID_Y[0]):g} mm, clear of the ballast weights",
         "source": BlueROV2.SOURCE, "verified": False},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M5 x 35 socket head cap screw, A4 (316) stainless", "qty": 3, "use": "skid through the bottom panel (heads recessed)"},
        {"item": "M5 washer, A4 (316) stainless", "qty": 6, "use": "under the skid screw heads and nuts"},
        {"item": "M5 nyloc nut, A4 (316) stainless", "qty": 3, "use": "skid, on top of the bottom panel"},
    ],
    "fit_test": True,
    "notes": [
        "Lift the ROV by its frame, never by a skid alone; the skids carry it sitting on the deck.",
        "Print solid: see the DVL mount; check the screws for tightness after the first few dives (HDPE and ASA settle).",
    ],
}


def build():
    """In its own frame: the rail's centre line on x, the panel face at z = 0."""
    from hardware.parts.common import cad
    length, width, height = F.SKID
    y = 0.0
    x0, x1 = -length / 2, length / 2
    profile = [(x0, 0.0), (x1, 0.0), (x1, -height + RAMP), (x1 - RAMP, -height), (x0 + RAMP, -height), (x0, -height + RAMP)]
    body = cad.profile_xz(profile, y - width / 2, y + width / 2)
    cutters = []
    for x in F.SKID_BOLTS_X:
        cutters.append(cad.cyl(F.M5_CLEARANCE, height + 2, (x, y, -height - 1)))
        cutters.append(cad.cyl(COUNTERBORE[0], COUNTERBORE[1] + 1, (x, y, -height - 1)))
    for xa, xb in WINDOWS:
        cutters.append(cad.box(xa, xb, y - width, y + width, WINDOW_Z[0], WINDOW_Z[1]))
    return cad.cut_all(body, cutters)
