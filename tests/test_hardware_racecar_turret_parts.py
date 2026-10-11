"""RACECAR Neo, turret and RACECAR Turret printed parts: their datasheet
footprints and their layout against the URDFs and the lidar scan filter.

Pure Python (no CadQuery). The geometry itself is checked by
tests/test_printable_parts_build.py (committed STLs fresh, assembly checks
passed) in the cad image.
"""
import re
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from hardware.parts.racecar_turret import _frame as RT  # noqa: E402
from hardware.parts.racecarneo import _footprints as RF  # noqa: E402
from hardware.parts.racecarneo import _frame as R  # noqa: E402
from hardware.parts.turret import _footprints as TF  # noqa: E402
from hardware.parts.turret import _frame as T  # noqa: E402

RACECAR_URDF = REPO / "systems" / "racecarneo" / "urdf" / "racecarneo.urdf.xacro"
TURRET_URDF = REPO / "systems" / "racecar_turret" / "urdf" / "racecar_turret.urdf.xacro"
SCAN_FILTER = REPO / "systems" / "racecar_turret" / "config" / "scan_filter.yaml"


@pytest.mark.parametrize("fp", RF.ALL + TF.ALL, ids=lambda fp: fp.__name__)
def test_footprints_cite_sources_and_list_real_attributes(fp):
    assert "http" in fp.SOURCE or "ASME" in fp.SOURCE, fp
    for name in fp.UNVERIFIED:
        assert hasattr(fp, name), (fp.__name__, name)


def _joint_xyz(text, joint):
    """xyz of the first (hardware, not sim_geometry) origin of a joint."""
    m = re.search(rf'<joint name="{joint}".*?<origin xyz="([^"]+)"', text, re.DOTALL)
    return tuple(1000.0 * float(v) for v in m.group(1).split())


def test_racecar_mounts_match_the_urdf():
    text = RACECAR_URDF.read_text()
    assert _joint_xyz(text, "lidar_joint") == (R.LIDAR_XY[0], R.LIDAR_XY[1], R.SCAN_Z)
    assert _joint_xyz(text, "camera_joint") == R.CAMERA_LINK
    assert _joint_xyz(text, "imu_joint")[:2] == R.IMU_XY
    # The lidar mast puts the C1's scan plane on the URDF's lidar height.
    assert R.LIDAR_BOTTOM_Z + RF.RplidarC1.SCAN_HEIGHT == pytest.approx(R.SCAN_Z)
    # The cradle seat puts a D435i's camera_link at the URDF's height.
    assert R.CRADLE_SEAT_Z + RF.RealSenseD435i.LINK_HEIGHT == pytest.approx(R.CAMERA_LINK[2])


def test_camera_arm_fin_is_inside_the_lidar_blind_range():
    lx = R.LIDAR_XY[0]
    far_corner = ((lx - R.FIN_X[0]) ** 2 + R.ARM_HALF_W ** 2) ** 0.5
    assert far_corner < RF.RplidarC1.MIN_RANGE - R.BLIND_MARGIN
    assert lx - R.FIN_X[1] > RF.RplidarC1.BASE / 2      # clear of the C1's base
    assert R.BOOM_Z[0] > R.LIDAR_BOTTOM_Z + RF.RplidarC1.HEIGHT
    assert R.ARM_FRONT_Z[0] > R.SCAN_Z + R.SCAN_BAND


def test_turret_heights_match_the_racecar_turret_urdf():
    text = TURRET_URDF.read_text()
    props = dict(re.findall(r'<xacro:property name="(\w+)" value="([^"]+)"/>', text))
    mount = [1000.0 * float(v) for v in props["turret_mount_xyz"].split()]
    assert (mount[0], mount[1], mount[2]) == (RT.PLATE_CENTER[0], RT.PLATE_CENTER[1], RT.PLATE_Z[0])
    pan = 1000.0 * float(props["pan_axis_height"])
    tilt = 1000.0 * float(props["tilt_axis_height"])
    # turret/base_link is the plate's underside; the turret frame stands on its top.
    assert RT.PLATE_Z[1] - RT.PLATE_Z[0] == T.PLATE_T_URDF
    assert T.TILT_Z + T.PLATE_T_URDF == pytest.approx(pan + tilt)
    assert T.CAMERA_X == pytest.approx(1000.0 * float(props["camera_forward"]))
    assert float(props["pan_limit"]) == pytest.approx(1.5708)


def test_plate_posts_sit_in_the_fin_shadow_and_the_scan_filter_box():
    import math
    shadow = R.fin_shadow_deg()
    for x, y in RT.POSTS:
        dist = math.hypot(R.LIDAR_XY[0] - x, y)
        bearing_from_rear = math.degrees(math.atan2(abs(y), R.LIDAR_XY[0] - x))
        half_width = math.degrees(math.asin(RT.POST_D / 2 / dist))
        assert bearing_from_rear + half_width < shadow, (x, y)
    params = yaml.safe_load(SCAN_FILTER.read_text())["scan_to_scan_filter_chain"]["ros__parameters"]
    box = params["filter1"]["params"]
    assert box["box_frame"] == "base_link"
    r = RT.POST_D / 2
    for x, y in RT.POSTS:
        assert 1000 * box["min_x"] <= x - r and x + r <= 1000 * box["max_x"]
        assert 1000 * box["min_y"] <= y - r and y + r <= 1000 * box["max_y"]
    # The rear deck has the posts' inserts.
    assert len(R.turret_post_inserts()) == 2 * len(RT.POSTS)


def test_jetson_and_deck_items_stay_under_the_scan_plane():
    J = RF.JetsonOrinNanoDevKit
    fan_top = R.jetson_pcb_z() + J.PCB_T + J.FAN_TOP_ABOVE_PCB
    assert fan_top < R.SCAN_KEEP_OUT_Z
    assert R.DECK_Z[1] + R.MAST_FLANGE_T < R.LIDAR_BOTTOM_Z


def test_st3215_and_turret_geometry():
    S = TF.ST3215
    assert (S.LENGTH, S.WIDTH, S.THICKNESS_MAX) == (45.22, 24.72, 35.0)
    assert S.SHAFT_X == 10.11 and S.HORN_FACES == 37.25 and S.HORN_HOLE_CIRCLE == 14.0
    assert TF.WaveshareBusServoAdapterA.HOLE_SPACING == (37.0, 28.0)
    assert TF.ArduinoUnoR4.HOLES == ((13.97, 2.54), (15.24, 50.8), (66.04, 7.62), (66.04, 35.56))
    assert (TF.ArducamB0385.BOARD, TF.ArducamB0385.HOLES_34) == (38.0, 34.0)
    # The tilt servo lies on the bracket floor with the shaft on the tilt axis.
    assert T.TILT_FLOOR_TOP + 0.2 + S.WIDTH / 2 == pytest.approx(T.TILT_Z)
    # The pan servo's horn is above its plugs' room.
    assert T.PAN_BOTTOM_Z - T.BASE_FLANGE_T >= S.PLUG_LENGTH + 4.0
