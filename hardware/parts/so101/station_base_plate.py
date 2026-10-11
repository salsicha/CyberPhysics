"""Station base plate: the 31 mm riser the follower arm's base bolts to,
220 x 220 mm like the scene's arm_base_mount, screwed to the station board.

It sets the arm's height above the table (the scene spawns the arm 31 mm
above the workbench) and the arm's position relative to the camera post,
so the camera extrinsics stay put when the station is moved."""
from hardware.parts.common import fasteners, printing
from hardware.parts.so101 import _station as S
from hardware.parts.so101._footprints import SO101Base, StationBoard

BOSS_D = 12.0
BOARD_BOSS_D = 14.0
BOARD_HOLE = 4.5          # for 4 mm wood screws (or M4 bolts through the board)
BOARD_WELL = (9.0, 8.0)   # head well from the top: diameter, and the floor left under the head
RIB_PITCH = 55.0
TIE_SLOTS = ((-100.0, 12.0), (-100.0, 42.0), (-100.0, -12.0), (-100.0, -42.0))   # two pairs, one per rib cell
TIE_SLOT = (3.0, 6.0)     # along x, along y
# Windows in the deck where nothing stands (rib cells clear of the arm base and the screws)
WINDOWS = ((80.5, -27.5), (80.5, 27.5), (-27.5, 80.5), (27.5, 80.5), (-27.5, -80.5), (27.5, -80.5))
WINDOW = 40.0

PART = {
    "name": "Station base plate (31 mm riser under the follower)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "indoors, stiff enough as a ribbed box; PETG does not creep under the bolt preload like PLA",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "upside down: the top face on the plate, ribs up", wall_loops=3,
                               infill_pct=15, brim="outer brim only, 5 mm", rotate=[("X", 180)]),
    "mates_with": [
        {"part": "SO-101 base (upstream Base_SO101.stl)",
         "dimensions": f"four {SO101Base.HOLE_D:g} mm wing holes at {SO101Base.HOLES} mm (station frame)",
         "source": SO101Base.SOURCE, "verified": False},
        {"part": "station board", "dimensions": "four wood screws on a 190 x 190 mm square",
         "source": StationBoard.SOURCE, "verified": True},
    ],
    "inserts": [{"size": "M4", "qty": 4, "where": "top face, under the arm base's four wing holes"}],
    "fasteners": [
        {"item": "M4 x 25 socket head cap screw", "qty": 4,
         "use": "follower base to the riser inserts (heads on the base's wings, follower only)"},
        {"item": "4 x 25 mm pan-head wood screw (or M4 bolt and washer through the board)", "qty": 4,
         "use": "base plate to the station board"},
        {"item": "zip tie 3.6 mm", "qty": 2, "use": "arm USB-C and power leads to the rear slots"},
    ],
    "fit_test": True,
    "notes": [
        "The arm base's wing holes are measured on the vendored mesh; print this plate and the base first and check the four bolts drop in.",
        "The arm's x axis (toward the bins) is the plate's +x: the zip-tie slots are at the rear (-x).",
    ],
}


def build():
    from hardware.parts.common import cad
    h, z0, z1 = S.PLATE_HALF, S.PLATE_BOTTOM, S.PLATE_TOP
    outer = cad.rounded_plate(2 * h, 2 * h, z1 - z0, 8.0, center=(0, 0, z0))
    inner = cad.rounded_plate(2 * (h - S.PLATE_WALL), 2 * (h - S.PLATE_WALL), z1 - S.PLATE_SKIN - z0 + 0.01,
                              8.0 - S.PLATE_WALL, center=(0, 0, z0 - 0.01))
    body = outer.cut(inner)
    ribs = []
    n = int((h - S.PLATE_WALL) // RIB_PITCH)
    for k in range(-n, n + 1):
        c = k * RIB_PITCH
        ribs.append(cad.box(-h + 1, h - 1, c - S.RIB_T / 2, c + S.RIB_T / 2, z1 - S.RIB_H, z1 - 1))
        ribs.append(cad.box(c - S.RIB_T / 2, c + S.RIB_T / 2, -h + 1, h - 1, z1 - S.RIB_H, z1 - 1))
    bosses = ([cad.cyl(BOSS_D, z1 - z0, (x, y, z0)) for x, y in SO101Base.HOLES] +
              [cad.cyl(BOARD_BOSS_D, z1 - z0, (x, y, z0)) for x, y in S.BOARD_SCREWS])
    body = cad.fuse_all([body] + ribs + bosses)
    ins = fasteners.INSERTS[S.ARM_BOLT]
    cutters = []
    for x, y in SO101Base.HOLES:
        cutters.append(cad.insert_hole(S.ARM_BOLT, x, y, z1))
        cutters.append(cad.cyl(fasteners.CLEARANCE[S.ARM_BOLT], z1 - z0 + 2, (x, y, z0 - 1)))   # bolt tip below the insert
    assert ins.hole_depth < S.ARM_BOLT_LENGTH - SO101Base.WING_THICKNESS + 3.0
    for x, y in S.BOARD_SCREWS:
        cutters.append(cad.cyl(BOARD_HOLE, z1 - z0 + 2, (x, y, z0 - 1)))
        cutters.append(cad.cyl(BOARD_WELL[0], z1 - z0 - BOARD_WELL[1] + 0.01, (x, y, z0 + BOARD_WELL[1])))
    for x, y in TIE_SLOTS:
        cutters.append(cad.cbox(TIE_SLOT[0], TIE_SLOT[1], 2 * S.PLATE_SKIN + 2, center=(x, y, z1)))
    for x, y in WINDOWS:
        cutters.append(cad.rounded_plate(WINDOW, WINDOW, S.PLATE_SKIN + 2, 6.0, center=(x, y, z1 - S.PLATE_SKIN - 1)))
    return cad.cut_all(body, cutters)
