"""Actuator mount: a cup that holds the CubeMars AK70-10 wheel actuator
by the eight M3 threads in its front face and carries it on the bottom of
a 2040 leg of the rocker or bogie. The wheel's load reaches the leg
through the actuator's cross-roller bearing and this cup."""
import math

from hardware.parts.common import fasteners, printing
from hardware.parts.lunar_rover import _frame as F
from hardware.parts.lunar_rover._footprints import CubeMarsAK70_10 as AK
from hardware.parts.lunar_rover._footprints import Extrusion2040

CUP_R = F.CUP_BORE / 2 + F.CUP_WALL
KEEL_HALF = 36.0             # flat bottom, wide enough that the cup's flanks print at <= 45 deg
SOCKET_WALL = 6.0
LEG_FIT = 0.4
SCREW_Z = (72.0, 94.0)       # T-nut screws, above the axle
BORE_ROOF = F.CUP_BORE / 2 + 3.0      # the bore's teardrop roof is cut flat here (a 31 mm bridge)
BOSS_ROOF = (AK.FRONT_BOSS[0] + 1.5) / 2 + 0.2
BOSS_HOLE = AK.FRONT_BOSS[0] + 1.5
CBORE = (6.5, 3.0)

PART = {
    "name": "Actuator mount (AK70-10 cup on the leg)",
    "quantity": 6,
    "material": "ASA",
    "material_reason": "carries the wheel load next to a warm actuator, outdoors; ASA keeps its strength in the heat",
    "material_fallback": None,
    "flight_critical": True,
    "print": printing.settings("ASA", "upright, as on the rover: keel on the plate, leg socket up (teardrop bores, no supports)",
                               wall_loops=6, top_shell_layers=6, bottom_shell_layers=6, infill_pct=50),
    "mates_with": [
        {"part": "CubeMars AK70-10 front face",
         "dimensions": f"{AK.MOUNT_HOLES} x M3 ({AK.MOUNT_THREAD_DEPTH:g} mm deep) on {AK.MOUNT_PCD:g} mm, "
                       f"{AK.DIAMETER:g} mm body, {AK.FRONT_BOSS[0]:g} mm boss",
         "source": AK.SOURCE, "verified": True},
        {"part": "CubeMars AK70-10 hole phase", "dimensions": "the 8 x M3 ring's angle is not dimensioned (holes at k 45 deg from x here)",
         "source": AK.SOURCE, "verified": False},
        {"part": "2040 leg", "dimensions": f"{Extrusion2040.SIZE[0] + LEG_FIT:g} x {Extrusion2040.SIZE[1] + LEG_FIT:g} mm socket, "
                                           "four M5 into drop-in T-nuts in the inboard face's two slots",
         "source": Extrusion2040.SOURCE, "verified": True},
    ],
    "inserts": [],
    "fasteners": [
        {"item": "M3 x 10 socket head cap screw, class 12.9", "qty": AK.MOUNT_HOLES,
         "use": "actuator to the cup's front ring (7 mm into the 10 mm threads), medium thread locker"},
        {"item": "M5 x 10 button head screw + M5 drop-in T-nut (slot 6)", "qty": 4, "use": "cup to the leg"},
    ],
    "fit_test": True,
    "notes": [
        "One part for all six wheels: the right-hand mounts are the same part turned 180 deg.",
        "The actuator slides in from the rear; its cable leaves through the actuator_shroud at the back.",
    ],
}


def teardrop(d, y0, y1, x=0.0, z=0.0, roof=None):
    """Hole of diameter d along y with a 45 deg roof (prints without
    support), cut flat at height ``roof`` above its centre if given."""
    import cadquery as cq
    r = d / 2
    apex = r * math.sqrt(2)
    top = apex if roof is None else min(roof, apex)
    rp = r / math.cos(math.radians(7.5))       # polygon around the circle, never inside it
    pts = [(x + rp * math.cos(math.radians(a)), z + rp * math.sin(math.radians(a))) for a in range(135, 406, 15)]
    half = apex - top
    pts += [(x + half, z + top), (x - half, z + top)] if half > 1e-6 else [(x, z + apex)]
    return (cq.Workplane("XZ", origin=(0, y1, 0)).polyline(pts).close().extrude(y1 - y0)).val()


def build():
    from hardware.parts.common import cad
    y_front = F.ACT_FACE_U + F.MOUNT_RING_T            # -5.2
    y_rear = F.CUP_REAR_U
    cup = cad.cyl(2 * CUP_R, y_front - y_rear, (0, y_rear, 0), (0, 1, 0))
    keel = cad.box(-KEEL_HALF, KEEL_HALF, y_rear, y_front, -CUP_R, -20.0)
    lw, ll = Extrusion2040.SIZE[0] + LEG_FIT, Extrusion2040.SIZE[1] + LEG_FIT
    s_y0 = F.LEG_U - lw / 2 - SOCKET_WALL
    s_x = ll / 2 + SOCKET_WALL
    z_top = F.LEG_SOCKET_Z[1] - F.AXLE_Z
    # the block over the cup runs its full length: it covers the bore's flat roof
    socket = cad.box(-s_x, s_x, min(s_y0, y_rear), y_front, 0.0, z_top)
    body = cad.fuse_all([cup, keel, socket])
    cutters = [teardrop(F.CUP_BORE, y_rear - 1, F.ACT_FACE_U, roof=BORE_ROOF),
               teardrop(BOSS_HOLE, F.ACT_FACE_U - 1, y_front + 1, roof=BOSS_ROOF)]
    floor = F.LEG_SOCKET_Z[0] - F.AXLE_Z
    cutters.append(cad.box(-ll / 2, ll / 2, F.LEG_U - lw / 2, F.LEG_U + lw / 2, floor, z_top + 1))
    for k in range(AK.MOUNT_HOLES):
        a = math.radians(AK.HOLE_PHASE_DEG + 45.0 * k)
        x, z = AK.MOUNT_PCD / 2 * math.cos(a), AK.MOUNT_PCD / 2 * math.sin(a)
        cutters.append(cad.cyl(fasteners.CLEARANCE["M3"], F.MOUNT_RING_T + 2, (x, F.ACT_FACE_U - 1, z), (0, 1, 0)))
        cutters.append(cad.cyl(CBORE[0], CBORE[1] + 0.01, (x, y_front - CBORE[1], z), (0, 1, 0)))
    for z in SCREW_Z:
        for x in (-10.0, 10.0):
            cutters.append(cad.cyl(fasteners.CLEARANCE["M5"], SOCKET_WALL + 2, (x, s_y0 - 1, z), (0, 1, 0)))
    return cad.cut_all(body, cutters)
