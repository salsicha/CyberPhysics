"""SO-101 station layout: the numbers the station's printed parts share
(helper, not a part).

Pure Python. Station frame in millimetres: the repository URDF's base_link
(systems/so101/urdf/so101.urdf): origin on the shoulder-pan axis, 22.5 mm
above the underside of the arm base; x toward the bins, y left, z up.

Everything is derived from the simulator scene,
systems/so101/scenarios/picking_table.json, so the physical station is the
simulated one:

* the arm stands on a 31 mm riser (station_base_plate): the scene spawns
  the arm 31 mm above the workbench top (robot_spawn z 0.431 m, table top
  0.400 m);
* the RealSense D435i's RGB optical centre is at the scene camera's
  position (base_link xyz 0.17, -0.52, 0.4665 m), looking 83 deg left of x
  and pitched 45 deg down (rpy 0, 0.785, 1.449); the camera head pivots so
  the pitch can be trimmed (see assembly.py);
* bins and objects have the scene's sizes and stand on the table.
"""
import json
import math
from pathlib import Path

from hardware.parts.so101._footprints import Extrusion2020, RealSenseD435i, SO101Base

SCENARIO_FILE = Path(__file__).resolve().parents[3] / "systems" / "so101" / "scenarios" / "picking_table.json"
SCENARIO = json.loads(SCENARIO_FILE.read_text())

URDF_BASE_LINK_Z = 22.5          # world_to_base in so101.urdf: base_link above the arm's underside


def _scene_by_name():
    out = {a["name"]: a for a in SCENARIO["static_assets"]}
    out.update({o["id"]: o for o in SCENARIO["objects"]})
    return out


SCENE = _scene_by_name()
SPAWN_M = SCENARIO["robot_spawn"]["xyz"]
BASE_LINK_WORLD_M = (SPAWN_M[0], SPAWN_M[1], SPAWN_M[2] + URDF_BASE_LINK_Z / 1000.0)


def world_to_station(xyz_m):
    """A scene position (m, world frame) in the station frame (mm)."""
    return tuple((v - o) * 1000.0 for v, o in zip(xyz_m, BASE_LINK_WORLD_M))


_bench = SCENE["workbench"]
TABLE_Z = round(world_to_station((0, 0, _bench["xyz"][2] + _bench["size"][2] / 2))[2], 3)   # -53.5
ARM_UNDERSIDE_Z = -URDF_BASE_LINK_Z                                                       # -22.5
assert abs(ARM_UNDERSIDE_Z - SO101Base.UNDERSIDE_Z) < 1e-9

# ----------------------------------------------------------------- riser plate
_mount = SCENE["arm_base_mount"]
PLATE_HALF = _mount["size"][0] * 1000.0 / 2      # 220 x 220 mm footprint, as the scene's mount
PLATE_TOP = ARM_UNDERSIDE_Z                       # the arm stands on it
PLATE_BOTTOM = TABLE_Z                            # 31 mm riser
PLATE_SKIN = 3.0
PLATE_WALL = 3.0
RIB_T = 1.6
RIB_H = 14.0                                      # interior ribs hang from the deck; the wall and bosses reach the table
BOARD_SCREWS = tuple((sx * 95.0, sy * 95.0) for sx in (1, -1) for sy in (1, -1))
ARM_BOLT = "M4"
ARM_BOLT_LENGTH = 25.0                            # through the 16.5 mm wings, 8.5 mm into the insert

# ----------------------------------------------------------------- camera
_cam = SCENARIO["camera"]
CAM_XYZ = world_to_station(_cam["xyz"])           # RGB optical centre (groot_camera_link)
CAM_ROLL, CAM_PITCH, CAM_YAW = _cam["rpy"]        # rad
assert abs(CAM_ROLL) < 1e-9
PITCH_DEG = math.degrees(CAM_PITCH)               # 45 deg down
YAW_DEG = math.degrees(CAM_YAW)                   # 83 deg left of x
PITCH_RANGE_DEG = (38.0, 57.0)                    # the head's lock slot


def cam_axes(pitch_deg=None, yaw_deg=None):
    """(x out of the lens, y left, z up) unit vectors of the camera at this
    pitch and yaw (degrees), in the station frame."""
    p = math.radians(PITCH_DEG if pitch_deg is None else pitch_deg)
    y = math.radians(YAW_DEG if yaw_deg is None else yaw_deg)
    x_ax = (math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p))
    y_ax = (-math.sin(y), math.cos(y), 0.0)
    z_ax = (math.sin(p) * math.cos(y), math.sin(p) * math.sin(y), math.cos(p))
    return x_ax, y_ax, z_ax


def _comb(origin, *pairs):
    """origin + sum of scale * vector."""
    out = list(origin)
    for s, v in pairs:
        for k in range(3):
            out[k] += s * v[k]
    return tuple(out)


# The cradle frame: origin at the centre of the D435i's back face, axes the
# camera's. The RGB centre is 32.5 mm to the camera's left of the case
# centre line and 25 - 4.2 = 20.8 mm in front of the back face.
_X0, _Y0, _Z0 = cam_axes()
BACK_TO_RGB = RealSenseD435i.DEPTH - RealSenseD435i.RGB_BEHIND_FRONT
CRADLE_ORIGIN = _comb(CAM_XYZ, (-BACK_TO_RGB, _X0), (-RealSenseD435i.RGB_Y, _Y0))
PIVOT_BACK = 24.0                       # pivot axis behind the camera's back face (cradle u)
PIVOT = _comb(CRADLE_ORIGIN, (-PIVOT_BACK, _X0))
LOCK_R = 14.0                           # lock screw below the pivot (cradle w = -LOCK_R)
CLEVIS_GAP = 6.0                        # between the cradle's ears, the head's tongue inside
TONGUE_T = 5.6
PLATE_T = 3.5                           # cradle plate behind the camera: M3 x 6 insert 2.5 mm

# Post: 2020 extrusion under the pivot, its faces along the station axes.
POST_BACK = 12.0                         # post axis behind the pivot (head x): clears the camera at 57 deg
_xh = (math.cos(math.radians(YAW_DEG)), math.sin(math.radians(YAW_DEG)))
POST_XY = (PIVOT[0] - POST_BACK * _xh[0], PIVOT[1] - POST_BACK * _xh[1])
HEAD_CAP_TOP = PIVOT[2] - 34.0           # top of the head's sleeve cap
HEAD_CAP_T = 6.0
SLEEVE_DEPTH = 36.0                      # post inside the head's sleeve
POST_TOP = HEAD_CAP_TOP - HEAD_CAP_T
POST_LENGTH = 500.0
POST_BOTTOM = POST_TOP - POST_LENGTH
POST = Extrusion2020.SIZE
POST_FIT = POST + 0.4                    # sleeve and socket bore
FOOT_FLOOR = POST_BOTTOM                 # the post stands on the foot's floor
FOOT_SOCKET_DEPTH = 40.0
FOOT_PLATE_T = 8.0


def rotate_about_pivot(point, delta_pitch_deg):
    """Where a point of the cradle goes when the pitch changes by
    ``delta_pitch_deg`` (the cradle turns about the pivot axis, the
    camera's y)."""
    p = [point[k] - PIVOT[k] for k in range(3)]
    ax = _Y0
    a = math.radians(delta_pitch_deg)
    c, s = math.cos(a), math.sin(a)
    dot = sum(p[k] * ax[k] for k in range(3))
    cross = (ax[1] * p[2] - ax[2] * p[1], ax[2] * p[0] - ax[0] * p[2], ax[0] * p[1] - ax[1] * p[0])
    # Rodrigues; a positive pitch change turns x down, i.e. about +y.
    return tuple(PIVOT[k] + p[k] * c + cross[k] * s + ax[k] * dot * (1 - c) for k in range(3))


def rgb_centre(pitch_deg):
    """The RGB optical centre with the camera trimmed to this pitch."""
    return rotate_about_pivot(CAM_XYZ, pitch_deg - PITCH_DEG)


# ----------------------------------------------------------------- task area
def scene_box(name):
    """(centre, size) of a scene object or bin standing on the table, mm,
    station frame (z from the table top, not the scene's)."""
    e = SCENE[name]
    if e.get("shape") == "cylinder":
        size = (2000.0 * e["radius_m"], 2000.0 * e["radius_m"], 1000.0 * e["height_m"])
    else:
        size = tuple(1000.0 * s for s in e["size"])
    c = world_to_station(e["xyz"])
    return (c[0], c[1], TABLE_Z + size[2] / 2), size


TASK_ITEMS = ("left_bin", "right_bin", "red_block_target", "green_block_distractor",
              "blue_cylinder_distractor", "occluder_box")


# ----------------------------------------------------------------- placement
def place_cradle(shape, pitch_deg=None):
    """A shape drawn in the cradle frame (origin at the centre of the
    D435i's back face at the scene pitch; u out of the lens, v left, w up)
    in the station frame, with the camera trimmed to ``pitch_deg``."""
    from hardware.parts.common import cad
    s = cad.rotate(shape, "Y", PITCH_DEG)
    s = cad.rotate(s, "Z", YAW_DEG)
    s = cad.translate(s, CRADLE_ORIGIN)
    if pitch_deg is not None and abs(pitch_deg - PITCH_DEG) > 1e-9:
        s = cad.rotate(s, _Y0, pitch_deg - PITCH_DEG, about=PIVOT)
    return s


def place_head(shape):
    """A shape drawn in the head frame (origin on the pivot; x along the
    camera's heading, horizontal; y left; z up) in the station frame."""
    from hardware.parts.common import cad
    return cad.translate(cad.rotate(shape, "Z", YAW_DEG), PIVOT)


def cradle_pivot_local():
    """The pivot in the cradle frame (u, v, w)."""
    return (-PIVOT_BACK, 0.0, 0.0)
