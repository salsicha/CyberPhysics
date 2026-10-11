"""Camera mast: a 200 mm tube on the crossbar plate with a captive 1/4"-20
nut at the top for a short stud and a mini ball head, so any camera or
all-round light with a tripod thread fits; its cable runs down inside the
tube."""
from hardware.parts.boat import _frame as F
from hardware.parts.boat._footprints import CameraTripodThread
from hardware.parts.common import fasteners, printing

TUBE_OD = 32.0
TUBE_ID = 20.0
TOP_SOLID = 16.0              # solid head of the mast that holds the nut
NUT_AF = 11.11                # 1/4"-20 hex nut, ASME B18.2.2: 7/16 in across flats
NUT_T = 5.56                  # 7/32 in thick
NUT_FIT = 0.3
NUT_Z = 6.0                   # nut pocket bottom, below the top face
CABLE_HOLE_D = 12.0
CABLE_SLOT = (14.0, 26.0)     # width, height at the base, facing aft
GUSSET = 10.0                 # 45 deg fillet cone between the flange and the tube

PART = {
    "name": "Camera mast",
    "quantity": 1,
    "material": "ASA",
    "material_reason": "on deck in sun and salt spray; ASA is UV-stable",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("ASA", "upright: flange on the plate; the nut slot roof bridges 11 mm",
                               wall_loops=4, infill_pct=30, brim="outer brim only, 5 mm"),
    "mates_with": [
        {"part": "crossbar_plate (printed)", "dimensions": "four M4 screws into the 50 mm insert grid",
         "source": "hardware/parts/boat/_frame.py", "verified": True},
        {"part": "1/4\"-20 hex nut and stud (camera thread)",
         "dimensions": f"{NUT_AF} mm across flats, {NUT_T} mm thick, captive in a side slot; "
                       f"{CameraTripodThread.MAJOR} mm stud through the top",
         "source": CameraTripodThread.SOURCE + "; ASME B18.2.2 hex nut 1/4\": 7/16 in AF, 7/32 in thick",
         "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M4 x 12 socket head cap screw, A4 (316) stainless", "qty": 4, "use": "camera mast to the crossbar plate inserts"},
        {"item": "1/4\"-20 UNC hex nut, stainless", "qty": 1, "use": "captive in the mast head"},
        {"item": "1/4\"-20 UNC x 1\" stud (set screw), stainless", "qty": 1,
         "use": "mast head to the ball head (thread locker on the nut side)"},
        {"item": "mini ball head with a 1/4\"-20 female base", "qty": 1, "use": "camera or light on the mast"},
    ],
    "fit_test": False,
    "notes": [
        "Slide the nut into the side slot, screw the stud down through the top into it until it bottoms, then the ball head onto the stud.",
        "The camera cable enters the hole under the head and leaves through the slot at the base (facing aft), then runs along the crossbar to the starboard hatch penetrator; zip-tie it through the crossbar holes.",
    ],
}


def build():
    import math

    from hardware.parts.common import cad
    x, y = F.MAST_XY
    z0 = F.PLATE[2]
    fx, fy, ft = F.MAST_FLANGE
    top = z0 + F.MAST_HEIGHT
    body = cad.fuse_all([
        cad.rounded_plate(fx, fy, ft, 6.0, center=(x, y, z0)),
        cad.cone(TUBE_OD + 2 * GUSSET, TUBE_OD, GUSSET, (x, y, z0 + ft - 0.01)),
        cad.cyl(TUBE_OD, top - z0 - ft + 0.01, (x, y, z0 + ft - 0.01)),
    ])
    head_d, _ = fasteners.SHCS_HEAD["M4"]
    cutters = [cad.cyl(TUBE_ID, top - TOP_SOLID - z0 + 1, (x, y, z0 - 1))]
    for sx, sy in F.MAST_SCREWS:
        cutters.append(cad.clearance_hole("M4", sx, sy, z0 - 1, z0 + ft + 1))
    # stud hole, hex nut pocket and the side slot it slides in through (from the rear)
    cutters.append(cad.cyl(CameraTripodThread.MAJOR + 0.4, TOP_SOLID + 1, (x, y, top - TOP_SOLID)))
    af = NUT_AF + 2 * NUT_FIT
    r = af / math.sqrt(3)
    hexagon = [(x + r * math.cos(math.radians(a)), y + r * math.sin(math.radians(a))) for a in range(0, 360, 60)]
    nz0 = top - NUT_Z - NUT_T - NUT_FIT
    cutters.append(cad.polygon_prism(hexagon, nz0, top - NUT_Z))
    cutters.append(cad.box(x - TUBE_OD, x, y - af / 2, y + af / 2, nz0, top - NUT_Z))
    cutters.append(cad.cyl(CABLE_HOLE_D, TUBE_OD, (x, y, top - TOP_SOLID - CABLE_HOLE_D / 2 - 4), (-1, 0, 0)))
    cutters.append(cad.box(x - TUBE_OD, x, y - CABLE_SLOT[0] / 2, y + CABLE_SLOT[0] / 2,
                           z0 + ft + GUSSET, z0 + ft + GUSSET + CABLE_SLOT[1]))
    assert head_d < fx / 2
    return cad.cut_all(body, cutters)
