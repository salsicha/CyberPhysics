"""Pose of the downward camera at an image's timestamp, and where the
vehicle is relative to what the image shows. ROS-free.

demnav_pkg and wildnav_pkg ship identical copies of this file (each image
installs only its own package); tests/test_camera_geometry.py keeps them
identical.

Frames
------
World ENU and body FLU, as in MAVROS ``local_position/odom`` and the
Aerostack2 platform odometry that relays it (orientation = world <- body).
Camera optical frame: x image right, y image down, z along the view.

The camera looks down the body -z axis with the image top ``mount_yaw``
counter-clockwise from the nose, seen from above: ``camera_mount_yaw_deg``
of systems/aerodrone/config/topics.env (0 = image top toward the nose; the
Isaac camera is mounted at 90). A body-fixed camera (the OAK-1 on the
aircraft, Isaac, Gazebo) tilts with the vehicle, so the image centre is
not the point below the vehicle. A roll/pitch-stabilised camera
(``camera_stabilized``: the airplane's satellite_camera_sim camera)
only turns with the heading.

Time
----
Odometry is stamped by MAVROS in wall time. Gazebo stamps camera frames in
simulator time, so, as in navigation_fusion, a frame stamp outside the
odometry's clock domain is mapped onto wall time through recorded
(simulator, wall) /clock pairs. Time offsets and gaps are measured in
simulator seconds while a /clock runs (the vehicle moves by it: Isaac
stamps frames in wall time but runs well below real time), else in wall
time.
"""
import bisect
import math
import threading
from dataclasses import dataclass

import numpy as np


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_matrix(x, y, z, w):
    """Rotation matrix (world <- body) of an (x, y, z, w) quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def slerp(q0, q1, fraction):
    """Spherical linear interpolation of (x, y, z, w) quaternions."""
    a = np.asarray(q0, dtype=np.float64)
    b = np.asarray(q1, dtype=np.float64)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    dot = float(a @ b)
    if dot < 0.0:
        b, dot = -b, -dot
    if dot > 0.9995:
        q = a + fraction * (b - a)
    else:
        theta = math.acos(dot)
        q = (math.sin((1.0 - fraction) * theta) * a +
             math.sin(fraction * theta) * b) / math.sin(theta)
    return tuple(float(value) for value in q / np.linalg.norm(q))


def heading_rad(rotation):
    """ENU heading (counter-clockwise from east) of the body x axis."""
    return math.atan2(rotation[1, 0], rotation[0, 0])


def roll_pitch_rad(quaternion):
    """ZYX Euler roll and pitch of an (x, y, z, w) quaternion."""
    x, y, z, w = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(sin_pitch)


def camera_mount(mount_yaw_rad):
    """Body <- optical rotation of a nadir camera: optical z along body -z,
    image top (optical -y) mount_yaw counter-clockwise from the nose and
    image right (optical x) 90 deg clockwise of it."""
    s, c = math.sin(mount_yaw_rad), math.cos(mount_yaw_rad)
    # Columns: optical x, y, z in body axes.
    return np.array([[s, -c, 0.0],
                     [-c, -s, 0.0],
                     [0.0, 0.0, -1.0]])


def camera_rotation(quaternion, mount_yaw_rad, stabilized=False):
    """World <- optical rotation for a vehicle attitude (x, y, z, w)."""
    body = quaternion_matrix(*quaternion)
    if stabilized:
        yaw = heading_rad(body)
        c, s = math.cos(yaw), math.sin(yaw)
        body = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return body @ camera_mount(mount_yaw_rad)


def nadir_pixel(rotation, intrinsics):
    """(u, v) of the ray straight down from the camera (the point below
    the vehicle), or None if the camera does not look downward. May lie
    outside the image when the tilt exceeds half the field of view."""
    fx, fy, cx, cy = intrinsics
    down = rotation.T @ np.array([0.0, 0.0, -1.0])
    if down[2] <= 1e-6:
        return None
    return (cx + fx * down[0] / down[2], cy + fy * down[1] / down[2])


def ground_projection(rotation, intrinsics, height_m):
    """Homography taking image pixels (u, v, 1) to (east, north) metres from
    the point below the camera, for flat ground height_m below it. Pixels
    whose third coordinate is not positive look at or above the horizon."""
    fx, fy, cx, cy = intrinsics
    k_inv = np.array([[1.0 / fx, 0.0, -cx / fx],
                      [0.0, 1.0 / fy, -cy / fy],
                      [0.0, 0.0, 1.0]])
    return np.diag([height_m, height_m, -1.0]) @ rotation @ k_inv


def horizontal_sigma(match_sigma_m, height_m, attitude_sigma_rad,
                     rate_rad_s=0.0, speed_m_s=0.0, stamp_sigma_s=0.0,
                     attitude_offset_s=0.0):
    """1-sigma horizontal error (m) of a fix of the point below the vehicle:
    the matching error; the attitude error times the height above ground
    (an error d tilts the ray to that point by d), with the attitude change
    over the stamp uncertainty and over the gap to the odometry sample used;
    and the distance flown over the stamp uncertainty."""
    timing_s = math.hypot(stamp_sigma_s, attitude_offset_s)
    attitude = math.hypot(attitude_sigma_rad, rate_rad_s * timing_s)
    return math.sqrt(match_sigma_m ** 2 + (height_m * attitude) ** 2 +
                     (speed_m_s * stamp_sigma_s) ** 2)


@dataclass(frozen=True)
class Attitude:
    quaternion: tuple   # (x, y, z, w), world <- body
    rate: float         # angular speed, rad/s
    speed: float        # linear speed, m/s
    # Distance from the stamp to the odometry sample used when the stamp
    # could not be interpolated (s, simulator time while a /clock runs);
    # 0 when interpolated.
    offset_s: float


class AttitudeHistory:
    """Vehicle attitude at camera stamps from timestamped odometry.

    A stamp between two samples at most max_gap_s apart is interpolated
    (slerp); otherwise the nearest sample within max_offset_s is used and
    its distance reported. Thread-safe: WildNav matches in a worker."""

    def __init__(self, history_s=30.0, max_offset_s=0.25, max_gap_s=1.0,
                 clock_domain_s=10.0, clock_extrapolation_s=1.0):
        self.history_ns = int(history_s * 1e9)
        self.max_offset_s = float(max_offset_s)
        self.max_gap_s = float(max_gap_s)
        self.clock_domain_ns = int(clock_domain_s * 1e9)
        self.clock_extrapolation_ns = int(clock_extrapolation_s * 1e9)
        self._lock = threading.Lock()
        self._samples = []  # (stamp_ns, quaternion, rate, speed)
        self._clock = []    # (simulator ns, wall ns)

    def add(self, stamp, quaternion, rate=0.0, speed=0.0):
        values = tuple(float(value) for value in quaternion) + (
            float(rate), float(speed))
        if stamp <= 0 or not all(math.isfinite(value) for value in values):
            return
        with self._lock:
            samples = self._samples
            if samples and stamp <= samples[-1][0]:
                if stamp < samples[-1][0]:
                    samples.clear()  # time went backwards: a new timeline
                else:
                    samples.pop()
            samples.append((int(stamp), values[:4], values[4], values[5]))
            cut = bisect.bisect_left(
                [item[0] for item in samples], stamp - self.history_ns)
            if cut:
                del samples[:cut]

    def add_clock(self, sim_ns, wall_ns):
        """Record a (simulator time, wall time on receipt) pair."""
        with self._lock:
            clock = self._clock
            if clock:
                if sim_ns < clock[-1][0]:
                    clock.clear()  # simulator reset
                elif sim_ns - clock[-1][0] < 10_000_000:
                    return
            clock.append((int(sim_ns), int(wall_ns)))
            cut = 0
            while (cut < len(clock) and
                   sim_ns - clock[cut][0] > self.history_ns):
                cut += 1
            if cut:
                del clock[:cut]

    @staticmethod
    def _segment(clock, column, value):
        """The pair of /clock records around value in column (0 simulator,
        1 wall), the first or last pair outside them."""
        index = bisect.bisect_left([item[column] for item in clock], value)
        index = min(max(index, 1), len(clock) - 1)
        (s0, w0), (s1, w1) = clock[index - 1], clock[index]
        if s1 <= s0 or w1 <= w0:
            return None
        return s0, w0, (s1 - s0) / (w1 - w0)

    def _sim_to_wall(self, sim_ns):
        """Wall ns of a simulator stamp within the recorded /clock history
        (or just past its end), else None."""
        clock = self._clock
        if len(clock) < 2 or sim_ns < clock[0][0] or (
                sim_ns > clock[-1][0] + self.clock_extrapolation_ns):
            return None
        segment = self._segment(clock, 0, sim_ns)
        if segment is None:
            return None
        s0, w0, rate = segment
        return int(w0 + (sim_ns - s0) / rate)

    def _rate(self, wall_ns):
        """Simulator seconds per wall second around wall_ns; 1 without
        /clock."""
        if len(self._clock) < 2:
            return 1.0
        segment = self._segment(self._clock, 1, wall_ns)
        return 1.0 if segment is None else segment[2]

    def at(self, stamp):
        """Attitude at a frame stamp (ns), or None."""
        with self._lock:
            samples = list(self._samples)
            if not samples or stamp <= 0:
                return None
            if abs(stamp - samples[-1][0]) > self.clock_domain_ns:
                stamp = self._sim_to_wall(stamp)
                if stamp is None:
                    return None
            rate = self._rate(stamp)
        times = [item[0] for item in samples]
        index = bisect.bisect_left(times, stamp)
        if 0 < index < len(times):
            (t0, q0, r0, v0), (t1, q1, r1, v1) = (
                samples[index - 1], samples[index])
            if (t1 - t0) * rate <= self.max_gap_s * 1e9:
                fraction = (stamp - t0) / (t1 - t0)
                return Attitude(slerp(q0, q1, fraction), max(r0, r1),
                                max(v0, v1), 0.0)
        nearest = min((i for i in (index - 1, index) if 0 <= i < len(times)),
                      key=lambda i: abs(times[i] - stamp))
        offset_s = abs(times[nearest] - stamp) * rate / 1e9
        if offset_s > self.max_offset_s:
            return None
        _, quaternion, angular, speed = samples[nearest]
        return Attitude(quaternion, angular, speed, offset_s)
