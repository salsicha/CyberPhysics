"""Camera cradle: holds the RealSense D435i by the two M3 threads in its
back and pivots on the camera head (clevis around the head's tongue), so
the pitch can be trimmed between 38 and 57 deg and locked."""
from hardware.parts.common import fasteners, printing
from hardware.parts.so101 import _station as S
from hardware.parts.so101._footprints import RealSenseD435i

PLATE_V = 40.0            # plate half-width (camera's left-right)
PLATE_W = 12.0            # plate half-height
EAR_T = 5.0
SLOT = 0.8                # M3 holes slotted +-0.8 mm up/down: the M3 pair's height is not dimensioned
NUT_POCKET_DEPTH = 3.0
EAR_OUTLINE = ((-1.0, 11.0), (-1.0, -12.0), (-18.0, -20.5), (-27.0, -19.5),
               (-30.0, -8.0), (-30.0, 2.0), (-26.0, 6.0), (-13.0, 10.0))   # (u, w)

PART = {
    "name": "Camera cradle (RealSense D435i)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "indoors; stiff and does not creep under the camera screws",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "camera face on the plate, ears up", wall_loops=4, infill_pct=40,
                               rotate=[("Z", -S.YAW_DEG), ("Y", 90.0 - S.PITCH_DEG)]),
    "mates_with": [
        {"part": "RealSense D435i",
         "dimensions": f"two M3 on the back {RealSenseD435i.BACK_M3_SPACING:g} mm apart, max insertion "
                       f"{RealSenseD435i.BACK_M3_MAX_INSERTION:g} mm: {S.PLATE_T} mm plate and M3 x 6",
         "source": RealSenseD435i.SOURCE, "verified": True},
        {"part": "RealSense D435i M3 pair position",
         "dimensions": f"assumed centred on the back; holes slotted +-{SLOT} mm up and down",
         "source": RealSenseD435i.SOURCE, "verified": False},
        {"part": "camera_head (printed)", "dimensions": f"{S.CLEVIS_GAP} mm clevis around the {S.TONGUE_T} mm tongue",
         "source": "hardware/parts/so101/camera_head.py", "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 6 socket head cap screw", "qty": 2,
         "use": f"D435i to the cradle (2.5 mm into the camera; Intel's max 3 mm, {RealSenseD435i.BACK_M3_TORQUE_NM} N m)"},
        {"item": "M3 x 20 socket head cap screw", "qty": 2, "use": "pivot and pitch lock, through the clevis and the tongue"},
        {"item": "M3 nyloc nut", "qty": 2, "use": "pivot and pitch lock, in the hex pockets"},
        {"item": "M3 washer", "qty": 2, "use": "under the pivot and lock screw heads"},
    ],
    "fit_test": True,
    "notes": [
        "Never use screws longer than M3 x 6 in the camera: Intel allows 3 mm of thread.",
        "USB-C on the camera's left end (seen from behind): the cable leaves sideways, clear of the cradle.",
    ],
}


def build():
    import cadquery as cq

    from hardware.parts.common import cad
    t = S.PLATE_T
    plate = (cq.Workplane("YZ", origin=(-t, 0, 0)).rect(2 * PLATE_V, 2 * PLATE_W).extrude(t)
             .edges("|X").fillet(4.0)).val()
    pu, _, _ = S.cradle_pivot_local()
    ears = []
    for side in (1, -1):
        v0 = side * S.CLEVIS_GAP / 2
        v1 = side * (S.CLEVIS_GAP / 2 + EAR_T)
        lo, hi = min(v0, v1), max(v0, v1)
        ear = cad.profile_xz(list(EAR_OUTLINE), lo, hi)
        ear = ear.fuse(cad.cyl(14.0, hi - lo, (pu, lo, 0.0), (0, 1, 0)))
        ear = ear.fuse(cad.cyl(10.0, hi - lo, (pu, lo, -S.LOCK_R), (0, 1, 0)))
        ears.append(ear)
    body = cad.fuse_all([plate] + ears)
    cutters = []
    for v in (-RealSenseD435i.BACK_M3_SPACING / 2, RealSenseD435i.BACK_M3_SPACING / 2):
        cutters.append((cq.Workplane("YZ", origin=(-t - 1, 0, 0)).center(v, 0)
                        .slot2D(2 * SLOT + fasteners.CLEARANCE["M3"], fasteners.CLEARANCE["M3"], 90)
                        .extrude(t + 2)).val())
    width = S.CLEVIS_GAP + 2 * EAR_T + 2
    for w in (0.0, -S.LOCK_R):
        cutters.append(cad.cyl(fasteners.CLEARANCE["M3"], width, (pu, -width / 2, w), (0, 1, 0)))
        s_af, _ = fasteners.NYLOC_NUT["M3"]
        y0 = -(S.CLEVIS_GAP / 2 + EAR_T) - 0.01
        nut = (cq.Workplane("XZ", origin=(0, y0 + NUT_POCKET_DEPTH, 0)).center(pu, w)
               .polygon(6, (s_af + 0.3) / 0.8660254).extrude(NUT_POCKET_DEPTH)).val()
        cutters.append(nut)
    return S.place_cradle(cad.cut_all(body, cutters))
