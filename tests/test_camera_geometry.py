"""Downward-camera geometry shared by DemNav and WildNav (camera_geometry):
sign conventions (ENU world, FLU body, optical camera frame), the point
below the vehicle for known tilts, yaws and heights, attitude lookup at an
image stamp (including simulator-clock stamps) and the fix covariance."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest
from conftest import REPO
from demnav_pkg import camera_geometry as cg
from demnav_pkg.terrain_match import ground_template, terrain_template

FOCAL, CX, CY = 554.7, 319.5, 239.5
K = (FOCAL, FOCAL, CX, CY)


def quaternion(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    """(x, y, z, w) of ZYX Euler angles (FLU body: +roll lowers the right
    side, +pitch lowers the nose; +yaw turns left from east)."""
    roll, pitch, yaw = (math.radians(v) for v in (roll_deg, pitch_deg,
                                                    yaw_deg))
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def ground_point(rotation, pixel, height):
    """(east, north) of the flat-ground point seen at pixel, relative to
    the point below the camera, by explicit ray intersection."""
    ray = rotation @ np.array([(pixel[0] - CX) / FOCAL,
                               (pixel[1] - CY) / FOCAL, 1.0])
    scale = height / -ray[2]
    return ray[0] * scale, ray[1] * scale


def test_package_copies_are_identical():
    demnav = REPO / 'applications/demnav/src/demnav_pkg/demnav_pkg'
    wildnav = REPO / 'applications/wildnav/src/wildnav_pkg/wildnav_pkg'
    assert ((demnav / 'camera_geometry.py').read_text() ==
            (wildnav / 'camera_geometry.py').read_text())


@pytest.mark.parametrize('mount_deg,image_right,image_down', [
    # Gazebo (rgbd_camera pitched +90 deg about body y): image up is the
    # nose, image right the right side.
    (0.0, (0.0, -1.0, 0.0), (-1.0, 0.0, 0.0)),
    # Isaac (USD camera looking down body -z, unrotated): image right is
    # body +x (the nose), image up body +y.
    (90.0, (1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
])
def test_mount_matches_the_simulated_cameras(mount_deg, image_right,
                                             image_down):
    mount = cg.camera_mount(math.radians(mount_deg))
    assert mount[:, 0] == pytest.approx(image_right)
    assert mount[:, 1] == pytest.approx(image_down)
    assert mount[:, 2] == pytest.approx((0.0, 0.0, -1.0))
    assert np.linalg.det(mount) == pytest.approx(1.0)


def test_level_camera_views_north_up_like_the_derotation():
    """Level flight heading east (yaw 0) with the image top toward the
    nose: the image top looks east and image right south."""
    rotation = cg.camera_rotation(quaternion(), 0.0)
    assert rotation @ np.array([0.0, -1.0, 0.0]) == pytest.approx(
        (1.0, 0.0, 0.0))
    assert rotation @ np.array([1.0, 0.0, 0.0]) == pytest.approx(
        (0.0, -1.0, 0.0))
    assert cg.nadir_pixel(rotation, K) == pytest.approx((CX, CY))


@pytest.mark.parametrize('yaw_deg', [0.0, 30.0, 135.0, -100.0])
@pytest.mark.parametrize('mount_deg', [0.0, 90.0])
def test_nose_down_pitch_puts_the_vehicle_ahead_of_the_image_centre(
        yaw_deg, mount_deg):
    height, pitch = 250.0, 10.0
    rotation = cg.camera_rotation(quaternion(pitch_deg=pitch, yaw_deg=yaw_deg),
                                  math.radians(mount_deg))
    east, north = ground_point(rotation, (CX, CY), height)
    behind = height * math.tan(math.radians(pitch))
    yaw = math.radians(yaw_deg)
    assert (east, north) == pytest.approx(
        (-behind * math.cos(yaw), -behind * math.sin(yaw)), abs=1e-6)
    # The nadir pixel sits f * tan(pitch) toward the nose in the image.
    u, v = cg.nadir_pixel(rotation, K)
    shift = FOCAL * math.tan(math.radians(pitch))
    if mount_deg == 0.0:   # image top toward the nose
        assert (u, v) == pytest.approx((CX, CY - shift))
    else:                  # image right toward the nose
        assert (u, v) == pytest.approx((CX + shift, CY))
    assert ground_point(rotation, (u, v), height) == pytest.approx(
        (0.0, 0.0), abs=1e-9)


@pytest.mark.parametrize('yaw_deg', [0.0, 90.0, -45.0])
def test_right_wing_down_roll_puts_the_vehicle_right_of_the_image_centre(
        yaw_deg):
    height, roll = 180.0, 15.0
    rotation = cg.camera_rotation(quaternion(roll_deg=roll, yaw_deg=yaw_deg),
                                  0.0)
    east, north = ground_point(rotation, (CX, CY), height)
    left = height * math.tan(math.radians(roll))
    yaw = math.radians(yaw_deg)
    # Body +y (left) in ENU: yaw + 90 deg.
    assert (east, north) == pytest.approx(
        (left * -math.sin(yaw), left * math.cos(yaw)), abs=1e-6)


def test_full_rotation_not_small_angles():
    """Combined 25 deg roll, -20 deg pitch: the exact offset of the
    image-centre ground point from the vehicle is h * (R z)_xy / -(R z)_z,
    which small-angle sums (h * roll, h * pitch) miss by metres."""
    height = 300.0
    q = quaternion(roll_deg=25.0, pitch_deg=-20.0, yaw_deg=70.0)
    rotation = cg.camera_rotation(q, 0.0)
    axis = cg.quaternion_matrix(*q) @ np.array([0.0, 0.0, -1.0])
    exact = (height * axis[0] / -axis[2], height * axis[1] / -axis[2])
    assert ground_point(rotation, (CX, CY), height) == pytest.approx(exact)
    small = height * math.hypot(math.radians(25.0), math.radians(20.0))
    assert abs(math.hypot(*exact) - small) > 10.0
    assert ground_point(rotation, cg.nadir_pixel(rotation, K),
                        height) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_stabilized_camera_ignores_roll_and_pitch():
    q = quaternion(roll_deg=12.0, pitch_deg=-7.0, yaw_deg=40.0)
    stabilised = cg.camera_rotation(q, 0.0, stabilized=True)
    level = cg.camera_rotation(quaternion(yaw_deg=40.0), 0.0)
    assert stabilised == pytest.approx(level)
    assert cg.nadir_pixel(stabilised, K) == pytest.approx((CX, CY))


def test_ground_projection_matches_ray_intersection():
    q = quaternion(roll_deg=-9.0, pitch_deg=14.0, yaw_deg=-120.0)
    rotation = cg.camera_rotation(q, math.radians(90.0))
    projection = cg.ground_projection(rotation, K, 210.0)
    for pixel in ((0.0, 0.0), (639.0, 0.0), (CX, CY), (100.0, 400.0)):
        mapped = projection @ np.array([pixel[0], pixel[1], 1.0])
        assert mapped[2] > 0.0
        assert (mapped[0] / mapped[2], mapped[1] / mapped[2]) == \
            pytest.approx(ground_point(rotation, pixel, 210.0))


def test_ground_template_anchor_is_the_point_below_the_vehicle():
    """A depth feature 40 m west and 30 m north of the vehicle lands 4 and
    3 cells (10 m DEM) from the template anchor, for a tilted camera."""
    width, height_px, cell, agl = 160, 120, 10.0, 200.0
    focal = width / (2.0 * math.tan(math.radians(69.0) / 2.0))
    intrinsics = (focal, focal, width / 2.0, height_px / 2.0)
    q = quaternion(roll_deg=6.0, pitch_deg=8.0, yaw_deg=25.0)
    rotation = cg.camera_rotation(q, 0.0)
    projection = cg.ground_projection(rotation, intrinsics, agl)
    u, v = np.meshgrid(np.arange(width, dtype=float),
                       np.arange(height_px, dtype=float))
    mapped = np.einsum('ij,jhw->ihw', projection, np.stack((u, v, 1 + 0 * u)))
    east, north = mapped[0] / mapped[2], mapped[1] / mapped[2]
    depth = np.exp(-((east + 40.0) ** 2 + (north - 30.0) ** 2) /
                   (2.0 * 15.0 ** 2)).astype(np.float32)
    # Wholly inside the frame.
    assert max(depth[0].max(), depth[-1].max(),
               depth[:, 0].max(), depth[:, -1].max()) < 0.01
    template, mask, anchor = ground_template(
        depth, np.ones_like(depth, bool), rotation, intrinsics, agl, cell)
    rows, cols = np.mgrid[0:template.shape[0], 0:template.shape[1]]
    weight = np.where(template > 0.3, template, 0.0)
    assert (weight * cols).sum() / weight.sum() - anchor[1] == \
        pytest.approx(-4.0, abs=0.2)
    assert (weight * rows).sum() / weight.sum() - anchor[0] == \
        pytest.approx(-3.0, abs=0.2)
    assert mask.sum() > 0.5 * mask.size


def test_ground_template_refuses_a_view_of_the_horizon():
    rotation = cg.camera_rotation(quaternion(pitch_deg=80.0), 0.0)
    depth = np.ones((120, 160), np.float32)
    assert ground_template(depth, depth > 0, rotation,
                           (115.0, 115.0, 80.0, 60.0), 200.0, 10.0) is None


K_HALF = (FOCAL / 2.0, FOCAL / 2.0, 159.5, 119.5)  # 320 x 240
CAMERA_ALTITUDE = 200.0


def hill_on_a_slope(east, north):
    """Terrain height (m) east/north of the point below the camera: a
    40 m hill 70 m from it on a 1:5 slope."""
    return (0.2 * east - 0.1 * north +
            40.0 * np.exp(-((east - 60.0) ** 2 + (north + 40.0) ** 2) /
                          (2.0 * 30.0 ** 2)))


def ray_cast(rotation):
    """Metric z-depth of every pixel of a 320 x 240 camera at
    CAMERA_ALTITUDE over hill_on_a_slope."""
    fx, fy, cx, cy = K_HALF
    u, v = np.meshgrid(np.arange(320.0), np.arange(240.0))
    rays = np.stack(((u - cx) / fx, (v - cy) / fy, np.ones_like(u)),
                    axis=-1) @ rotation.T
    depth = np.full(u.shape, CAMERA_ALTITUDE)
    for _ in range(100):
        gap = CAMERA_ALTITUDE + depth * rays[..., 2] - hill_on_a_slope(
            depth * rays[..., 0], depth * rays[..., 1])
        depth += 0.7 * gap / np.maximum(-rays[..., 2], 1e-3)
    return depth.astype(np.float32), rays


def template_error(template, mask, anchor, cell=10.0):
    """Template value minus the true height below the camera at the centre
    of every cell in the mask."""
    rows, cols = np.nonzero(mask)
    east, north = (cols - anchor[1]) * cell, -(rows - anchor[0]) * cell
    return template[mask] - (CAMERA_ALTITUDE - hill_on_a_slope(east, north))


@pytest.mark.parametrize('roll,pitch,yaw,mount', [
    (0.0, 0.0, 0.0, 0.0), (12.0, -5.0, 30.0, 90.0), (-8.0, 15.0, -120.0, 0.0)])
def test_terrain_template_puts_each_pixel_on_its_ground_point(
        roll, pitch, yaw, mount):
    """Metric depth places every pixel where it meets the terrain: each
    cell holds the height below the camera at that cell, east/north of the
    template anchor (the point below the camera), whatever the attitude.
    Projected onto flat ground instead, the hill and the slope are
    displaced by metres (relief displacement)."""
    rotation = cg.camera_rotation(quaternion(roll, pitch, yaw),
                                  math.radians(mount))
    depth, rays = ray_cast(rotation)
    template, mask, anchor = terrain_template(
        depth, np.ones(depth.shape, bool), rotation, K_HALF, 10.0)
    assert anchor == (round(anchor[0]), round(anchor[1]))
    assert mask.sum() > 300
    assert np.abs(template_error(template, mask, anchor)).max() < 0.5

    # The flat-ground projection of the same heights below the camera.
    flat = ground_template(depth * -rays[..., 2], np.ones(depth.shape, bool),
                           rotation, K_HALF,
                           CAMERA_ALTITUDE - hill_on_a_slope(0.0, 0.0), 10.0)
    assert np.abs(template_error(*flat)).max() > 5.0


def test_terrain_template_masks_cells_it_does_not_cover_all_around():
    """Cells at the footprint edge or beside missing depth are partly
    covered: they are masked rather than averaged off centre."""
    rotation = cg.camera_rotation(quaternion(9.0, 4.0, 60.0), 0.0)
    depth, _ = ray_cast(rotation)
    valid = np.ones(depth.shape, bool)
    whole = terrain_template(depth, valid, rotation, K_HALF, 10.0)
    valid[90:150, 120:200] = False  # e.g. no depth return
    holed = terrain_template(depth, valid, rotation, K_HALF, 10.0)
    assert holed[2] == whole[2]
    assert whole[1].sum() - holed[1].sum() > 15
    assert np.abs(template_error(*holed)).max() < 0.5
    # The outer ring of the footprint is never in the mask.
    covered = np.zeros(whole[1].shape, bool)
    covered[whole[1]] = True
    assert not covered[[0, -1], :].any() and not covered[:, [0, -1]].any()
    assert terrain_template(depth, np.zeros(depth.shape, bool), rotation,
                            K_HALF, 10.0) is None


def test_roll_pitch_and_heading_follow_zyx_euler():
    q = quaternion(20.0, -10.0, 30.0)
    roll, pitch = cg.roll_pitch_rad(q)
    assert (math.degrees(roll), math.degrees(pitch)) == pytest.approx(
        (20.0, -10.0))
    assert math.degrees(cg.heading_rad(cg.quaternion_matrix(*q))) == \
        pytest.approx(30.0)


def stamp(seconds):
    return round(seconds * 1e9)


def test_attitude_is_interpolated_at_the_image_stamp():
    history = cg.AttitudeHistory()
    history.add(stamp(100.0), quaternion(pitch_deg=0.0), 0.1, 5.0)
    history.add(stamp(100.2), quaternion(pitch_deg=10.0), 0.3, 6.0)
    history.add(stamp(110.0), quaternion(roll_deg=30.0))  # latest: unrelated
    attitude = history.at(stamp(100.05))
    assert attitude.offset_s == 0.0
    assert attitude.quaternion == pytest.approx(quaternion(pitch_deg=2.5))
    assert (attitude.rate, attitude.speed) == (0.3, 6.0)


def test_attitude_falls_back_to_a_near_sample_and_reports_the_gap():
    history = cg.AttitudeHistory(max_offset_s=0.25, max_gap_s=1.0)
    history.add(stamp(100.0), quaternion(pitch_deg=4.0))
    # Newer than every sample: the nearest, with its distance.
    attitude = history.at(stamp(100.1))
    assert attitude.quaternion == pytest.approx(quaternion(pitch_deg=4.0))
    assert attitude.offset_s == pytest.approx(0.1)
    assert history.at(stamp(100.3)) is None
    # Samples 5 s apart are not interpolated across.
    history.add(stamp(105.0), quaternion(pitch_deg=-4.0))
    assert history.at(stamp(102.5)) is None
    assert history.at(stamp(104.9)).offset_s == pytest.approx(0.1)


def test_time_going_backwards_starts_a_new_history():
    history = cg.AttitudeHistory()
    history.add(stamp(100.0), quaternion(pitch_deg=4.0))
    history.add(stamp(50.0), quaternion(roll_deg=4.0))
    assert history.at(stamp(100.0)) is None
    assert history.at(stamp(50.0)).quaternion == pytest.approx(
        quaternion(roll_deg=4.0))


def test_simulator_stamps_map_through_the_clock():
    """Isaac stamps frames in simulator time running at 0.2x real time;
    MAVROS stamps odometry in wall time."""
    history = cg.AttitudeHistory(max_offset_s=0.25)
    wall0 = 1_791_637_000.0
    for i in range(50):
        sim = 300.0 + 0.1 * i
        history.add_clock(stamp(sim), stamp(wall0 + 0.5 * i))
    # Odometry every 0.25 s of wall time (0.05 s of simulator time),
    # pitching 1 deg per wall second.
    for i in range(100):
        wall = wall0 + 0.25 * i
        history.add(stamp(wall), quaternion(pitch_deg=wall - wall0))
    attitude = history.at(stamp(301.0))     # wall0 + 5 s
    assert attitude.offset_s == 0.0
    assert attitude.quaternion == pytest.approx(quaternion(pitch_deg=5.0))
    # Just past the last /clock pair (frames can arrive before /clock):
    # extrapolated at the recorded rate; offsets are on the frame clock.
    attitude = history.at(stamp(304.95))    # wall0 + 24.75 s: last sample
    assert attitude.quaternion == pytest.approx(quaternion(pitch_deg=24.75))
    attitude = history.at(stamp(304.99))    # 0.2 s of wall past it
    assert attitude.offset_s == pytest.approx(0.04)
    # Outside the clock history: unknown.
    assert history.at(stamp(250.0)) is None
    assert cg.AttitudeHistory().at(stamp(301.0)) is None


def test_offsets_count_simulator_time_while_a_clock_runs():
    """Isaac stamps frames in wall time but runs at ~0.2x real time: half a
    wall second from the nearest odometry sample is 0.1 s of flight."""
    wall0 = 1_791_637_000.0
    history = cg.AttitudeHistory(max_offset_s=0.25)
    history.add(stamp(wall0), quaternion(pitch_deg=3.0))
    assert history.at(stamp(wall0 + 0.5)) is None
    for i in range(10):
        history.add_clock(stamp(40.0 + 0.1 * i), stamp(wall0 - 2.0 + 0.5 * i))
    attitude = history.at(stamp(wall0 + 0.5))
    assert attitude.offset_s == pytest.approx(0.1)
    assert attitude.quaternion == pytest.approx(quaternion(pitch_deg=3.0))


def test_stamp_ns():
    assert cg.stamp_ns(NS(sec=12, nanosec=345)) == 12_000_000_345


def test_sigma_combines_matching_attitude_and_timing():
    assert cg.horizontal_sigma(3.0, 250.0, 0.0) == pytest.approx(3.0)
    attitude = math.radians(1.0)
    assert cg.horizontal_sigma(0.0, 250.0, attitude) == pytest.approx(
        250.0 * attitude)
    assert cg.horizontal_sigma(3.0, 250.0, attitude) == pytest.approx(
        math.hypot(3.0, 4.363), abs=1e-3)
    # 0.5 rad/s over a 0.1 s stamp uncertainty is another 0.05 rad, and
    # 10 m/s over it another metre.
    assert cg.horizontal_sigma(
        0.0, 100.0, 0.0, rate_rad_s=0.5, stamp_sigma_s=0.1) == \
        pytest.approx(5.0)
    assert cg.horizontal_sigma(
        0.0, 100.0, 0.0, speed_m_s=10.0, stamp_sigma_s=0.1) == \
        pytest.approx(1.0)
    # The gap to the attitude sample counts like stamp uncertainty.
    assert cg.horizontal_sigma(
        0.0, 100.0, 0.0, rate_rad_s=0.5, attitude_offset_s=0.1) == \
        pytest.approx(5.0)
