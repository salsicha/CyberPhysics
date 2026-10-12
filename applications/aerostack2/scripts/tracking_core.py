"""Aerodrone object tracking core (systems/aerodrone/tracking.md): target
geolocation, the target filter, the two follow modes' set-points, the
command and the status. Plain Python and NumPy; the ROS nodes are
aerodrone_tracker.py and tracking_truth_detector.py beside this file.

Frames
------
The navigation frame is ENU metres about /navigation/origin (the ArduPilot
EKF origin; its altitude is above the WGS-84 ellipsoid), the frame of
/navigation/odometry and of Aerostack2's ``earth``. Body FLU, the optical
frame, the camera mount (``camera_mount_yaw_deg``), roll/pitch-stabilised
cameras and simulator-clock stamps are DemNav's and WildNav's
(demnav_pkg.camera_geometry).

Geolocation
-----------
A detection's box centre is a ray from the camera through the pixel, in the
camera pose at the image stamp: position and attitude interpolated from
/navigation/odometry (PoseHistory), the camera mount, the image's
intrinsics. The ray is marched to the terrain:

- the DemNav DEM (SRTM heights above EGM96 mean sea level, cached as
  ``dem_srtm_<lat>_<lon>_<km>_<res>.npy``), put on the ellipsoid with the
  geoid and expressed about the origin, which knows the ground under the
  target, not only under the aircraft: a 6 m rise between the point below
  the aircraft and a target 30 m away moves a 60 deg-depression fix 3.5 m.
  A fresh rangefinder reading removes the DEM's (and the altitude's) offset
  at the point below the aircraft;
- without a DEM, flat ground at the rangefinder's height below the
  aircraft;
- with neither, flat ground at the origin's height (the home's ground),
  reported in the status.

The aircraft's Benewake rangefinders reach 8-12 m and tracking flies 15-120 m
above ground, so on the aircraft the DEM carries the geolocation and the
rangefinder only corrects it near the ground.

Filter, set-points, limits
--------------------------
ConstantVelocityFilter is a 3D constant-velocity Kalman filter of the
target's position with Mahalanobis gating. The follow modes' set-points
(centered_setpoint, slant_setpoint) aim at the target predicted ``lead_s``
ahead, plus what TrackerCore.adapt_lead learns, which makes up ArduPilot's
lag behind a moving position target;
ReferenceShaper limits how fast the reference moves (the maximum speed) and
how far it runs ahead of the aircraft. Heights are kept within
[min_agl_m, max_agl_m] of the ground below the set-point; the slant
depression is kept where the camera still sees the target
(min_depression_for_fov).

Track handover
--------------
The detector's track ids change when its image-space tracker loses an
object, which a body-fixed camera on a manoeuvring aircraft makes common
(the image jumps by tens of pixels a frame when the attitude changes). A
selected id gone from a frame, at most ``handover_window_s`` after the
target was last placed, is handed over to a new id of the same kind of
object (vehicle, person, two-wheeler) and box size, placed within ``handover_max_m``,
plus half the distance the target covered meanwhile, of its position
predicted along its arc, inside the filter's gate, when it is the only one
of its kind there (TrackerCore.handover). An occluded target
reappears later than that, its prediction too uncertain to tell it from a
look-alike: it is selected again by the operator.
"""
import bisect
import glob
import json
import math
import os
import re
from dataclasses import dataclass, field, replace

import numpy as np
from demnav_pkg.camera_geometry import camera_rotation, quaternion_matrix, slerp
from synthetic_world import latlon_to_local, local_to_latlon

STATES = ('idle', 'acquiring', 'tracking', 'lost', 'error')
# Classes a track may change between when handed over (the detector
# hesitates between car and van); any other class only to itself.
CLASS_KINDS = (frozenset({'car', 'van', 'truck', 'bus'}),
               frozenset({'bicycle', 'motorcycle', 'tricycle'}))
ACTIVE_STATES = ('acquiring', 'tracking', 'lost', 'error')
MODES = ('centered', 'slant')
# The contract's depression range: a nadir camera sees the target only
# within about 30 deg of nadir (tracking.md, "Modes").
MIN_DEPRESSION_DEG = 60.0
MAX_DEPRESSION_DEG = 90.0
# DEM file names written by demnav_pkg.heightmap.HeightMapCache.
DEM_NAME = re.compile(
    r'^dem_(?:srtm_)?(-?\d+\.\d+)_(-?\d+\.\d+)_(\d+\.\d+)_(\d+)\.npy$')


@dataclass(frozen=True)
class TrackerConfig:
    """The tracker's parameters; tracking.env holds the deployed values."""
    min_agl_m: float = 15.0
    max_agl_m: float = 120.0
    max_speed_mps: float = 5.0
    max_vertical_speed_mps: float = 1.5
    max_yaw_rate_dps: float = 30.0
    max_reference_lead_m: float = 15.0
    lead_s: float = 2.5
    lead_adapt_rate: float = 0.2
    max_lead_extra_s: float = 2.5
    lost_timeout_s: float = 3.0
    slant_range_m: float = 60.0
    depression_deg: float = 70.0
    fov_margin_deg: float = 4.5
    pixel_sigma_px: float = 3.0
    attitude_sigma_deg: float = 1.0
    terrain_sigma_m: float = 3.0
    max_range_m: float = 1000.0
    process_accel_mps2: float = 1.5
    process_vertical_accel_mps2: float = 0.3
    initial_speed_sigma_mps: float = 5.0
    gate_chi2: float = 16.0
    max_rejections: int = 5
    require_flying: bool = True
    # Track handover (module doc): how long after the target was last
    # placed, and how far from its predicted position (plus half the
    # distance it covered meanwhile).
    handover_window_s: float = 2.5
    handover_max_m: float = 4.0
    handover_size_ratio: float = 2.0


def same_kind(a, b):
    """Whether a track of class a may be handed over to one of class b."""
    a, b = str(a or '').strip().lower(), str(b or '').strip().lower()
    if not a or not b:
        return False
    return a == b or any(a in kind and b in kind for kind in CLASS_KINDS)


def _number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def wrap_angle(angle):
    """Angle in (-pi, pi]."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def yaw_of(quaternion):
    """ENU heading (counter-clockwise from east) of the body x axis."""
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# --- poses at image stamps ------------------------------------------------
@dataclass(frozen=True)
class Pose:
    position: tuple      # (east, north, up), navigation frame
    quaternion: tuple    # (x, y, z, w), world <- body
    stamp_ns: int        # in the odometry's clock
    offset_s: float      # distance to the sample used when not interpolated


class PoseHistory:
    """Vehicle pose at a camera stamp from timestamped odometry, as
    demnav_pkg.camera_geometry.AttitudeHistory does for the attitude.

    A stamp between two samples at most max_gap_s apart is interpolated
    (linear position, slerp attitude); otherwise the nearest sample within
    max_offset_s is used. Simulators stamp camera frames in simulator time
    and MAVROS stamps odometry in wall time: a stamp outside the odometry's
    clock domain is mapped onto it through recorded (simulator, wall)
    /clock pairs; gaps and offsets are then simulator seconds."""

    def __init__(self, history_s=30.0, max_offset_s=0.25, max_gap_s=1.0,
                 clock_domain_s=10.0, clock_extrapolation_s=1.0):
        self.history_ns = int(history_s * 1e9)
        self.max_offset_s = float(max_offset_s)
        self.max_gap_s = float(max_gap_s)
        self.clock_domain_ns = int(clock_domain_s * 1e9)
        self.clock_extrapolation_ns = int(clock_extrapolation_s * 1e9)
        self._samples = []  # (stamp_ns, position, quaternion)
        self._clock = []    # (simulator ns, wall ns)

    def add(self, stamp_ns, position, quaternion):
        position = tuple(float(value) for value in position)
        quaternion = tuple(float(value) for value in quaternion)
        if stamp_ns <= 0 or not all(math.isfinite(v)
                                    for v in position + quaternion):
            return
        samples = self._samples
        if samples and stamp_ns <= samples[-1][0]:
            if stamp_ns < samples[-1][0]:
                samples.clear()  # time went backwards: a new timeline
            else:
                samples.pop()
        samples.append((int(stamp_ns), position, quaternion))
        cut = bisect.bisect_left([item[0] for item in samples],
                                 stamp_ns - self.history_ns)
        if cut:
            del samples[:cut]

    def add_clock(self, sim_ns, wall_ns):
        clock = self._clock
        if clock:
            if sim_ns < clock[-1][0]:
                clock.clear()  # simulator reset
            elif sim_ns - clock[-1][0] < 10_000_000:
                return
        clock.append((int(sim_ns), int(wall_ns)))
        while clock and sim_ns - clock[0][0] > self.history_ns:
            clock.pop(0)

    def _segment(self, column, value):
        clock = self._clock
        index = bisect.bisect_left([item[column] for item in clock], value)
        index = min(max(index, 1), len(clock) - 1)
        (s0, w0), (s1, w1) = clock[index - 1], clock[index]
        if s1 <= s0 or w1 <= w0:
            return None
        return s0, w0, (s1 - s0) / (w1 - w0)

    def _sim_to_wall(self, sim_ns):
        clock = self._clock
        if len(clock) < 2 or sim_ns < clock[0][0] or (
                sim_ns > clock[-1][0] + self.clock_extrapolation_ns):
            return None
        segment = self._segment(0, sim_ns)
        if segment is None:
            return None
        s0, w0, rate = segment
        return int(w0 + (sim_ns - s0) / rate)

    def _rate(self, wall_ns):
        """Simulator seconds per wall second; 1 without /clock."""
        if len(self._clock) < 2:
            return 1.0
        segment = self._segment(1, wall_ns)
        return 1.0 if segment is None else segment[2]

    def to_odometry_time(self, stamp_ns):
        """The stamp in the odometry's clock, or None."""
        if not self._samples or stamp_ns <= 0:
            return None
        if abs(stamp_ns - self._samples[-1][0]) > self.clock_domain_ns:
            return self._sim_to_wall(stamp_ns)
        return int(stamp_ns)

    def latest(self):
        if not self._samples:
            return None
        stamp, position, quaternion = self._samples[-1]
        return Pose(position, quaternion, stamp, 0.0)

    def at(self, stamp_ns):
        """Pose at a frame stamp (ns), or None."""
        stamp = self.to_odometry_time(stamp_ns)
        if stamp is None:
            return None
        samples = self._samples
        rate = self._rate(stamp)
        times = [item[0] for item in samples]
        index = bisect.bisect_left(times, stamp)
        if 0 < index < len(times):
            (t0, p0, q0), (t1, p1, q1) = samples[index - 1], samples[index]
            if (t1 - t0) * rate <= self.max_gap_s * 1e9:
                fraction = (stamp - t0) / (t1 - t0)
                position = tuple(a + fraction * (b - a) for a, b in zip(p0, p1))
                return Pose(position, slerp(q0, q1, fraction), stamp, 0.0)
        if index < len(times) and times[index] == stamp:
            nearest = index
        else:
            nearest = min((i for i in (index - 1, index) if 0 <= i < len(times)),
                          key=lambda i: abs(times[i] - stamp))
        offset_s = abs(times[nearest] - stamp) * rate / 1e9
        if offset_s > self.max_offset_s:
            return None
        _, position, quaternion = samples[nearest]
        return Pose(position, quaternion, stamp, offset_s)


# --- camera -------------------------------------------------------------
@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @property
    def k(self):
        return (self.fx, self.fy, self.cx, self.cy)


def intrinsics_from_camera_info(info):
    """Intrinsics of a sensor_msgs/CameraInfo, or None when unusable."""
    k = list(info.k)
    if (len(k) < 9 or not all(math.isfinite(value) for value in k) or
            k[0] <= 0.0 or k[4] <= 0.0 or info.width <= 0 or info.height <= 0):
        return None
    return Intrinsics(float(k[0]), float(k[4]), float(k[2]), float(k[5]),
                      int(info.width), int(info.height))


def long_axis_half_fov_rad(intrinsics):
    """Half field of view along the image's long axis."""
    if intrinsics.width >= intrinsics.height:
        return math.atan(0.5 * intrinsics.width / intrinsics.fx)
    return math.atan(0.5 * intrinsics.height / intrinsics.fy)


def min_depression_for_fov(intrinsics, margin_deg):
    """Smallest depression (deg) that keeps a target on the long axis
    margin_deg inside the image edge, never below the contract's 60."""
    if intrinsics is None:
        return MIN_DEPRESSION_DEG
    half = math.degrees(long_axis_half_fov_rad(intrinsics))
    return min(MAX_DEPRESSION_DEG,
               max(MIN_DEPRESSION_DEG, 90.0 - (half - float(margin_deg))))


def camera_world_rotation(quaternion, mount_yaw_rad, stabilized=False):
    """World <- optical rotation of the nadir camera."""
    return camera_rotation(quaternion, mount_yaw_rad, stabilized)


def pixel_ray(rotation, intrinsics, u, v):
    """Unit world direction of the ray through pixel (u, v)."""
    ray = rotation @ np.array([(u - intrinsics.cx) / intrinsics.fx,
                               (v - intrinsics.cy) / intrinsics.fy, 1.0])
    return ray / np.linalg.norm(ray)


def project_point(point, camera_position, rotation, intrinsics):
    """(u, v, depth) of a world point, or None behind the camera."""
    offset = rotation.T @ (np.asarray(point, dtype=float) -
                           np.asarray(camera_position, dtype=float))
    if offset[2] <= 1e-6:
        return None
    return (intrinsics.cx + intrinsics.fx * offset[0] / offset[2],
            intrinsics.cy + intrinsics.fy * offset[1] / offset[2],
            float(offset[2]))


def image_axis_heading(mount_yaw_rad, intrinsics=None):
    """Body heading (rad, counter-clockwise from the nose) of the image's
    long axis: image right (optical x) for a landscape image, image up for
    a portrait one (camera_geometry.camera_mount)."""
    if intrinsics is None or intrinsics.width >= intrinsics.height:
        return mount_yaw_rad - math.pi / 2.0
    return mount_yaw_rad


def long_axis_yaw(bearing_rad, mount_yaw_rad, current_yaw_rad,
                  intrinsics=None):
    """Vehicle yaw that puts a target at bearing_rad (ENU) on the image's
    long axis, the one of the two (either end of the axis) closer to the
    current yaw."""
    axis = image_axis_heading(mount_yaw_rad, intrinsics)
    candidates = (wrap_angle(bearing_rad - axis),
                  wrap_angle(bearing_rad - axis + math.pi))
    return min(candidates,
               key=lambda yaw: abs(wrap_angle(yaw - current_yaw_rad)))


# --- terrain ------------------------------------------------------------
class FlatTerrain:
    """Level ground at one height (navigation-frame up)."""
    resolution_m = 5.0

    def __init__(self, up, source):
        self.up = float(up)
        self.source = source

    def height(self, east, north):
        return self.up


class DemTerrain:
    """A DemNav DEM in the navigation frame: SRTM heights (EGM96 mean sea
    level) plus the geoid height, minus the origin's ellipsoid altitude,
    plus an offset (from the rangefinder). height() is None off the map."""
    source = 'dem'

    def __init__(self, heightmap, origin, geoid=None, offset_m=0.0):
        self.heightmap = heightmap
        self.origin = tuple(float(value) for value in origin)
        self.geoid = geoid
        self.offset_m = float(offset_m)
        self.resolution_m = float(min(heightmap.row_spacing_m,
                                      heightmap.col_spacing_m))
        self._geoid_m = geoid.height(*self.origin[:2]) if geoid else 0.0

    def with_offset(self, offset_m):
        return DemTerrain(self.heightmap, self.origin, self.geoid, offset_m)

    def contains(self, east, north, margin_m=0.0):
        lat, lon = local_to_latlon(east, north, *self.origin[:2])
        return self.heightmap.edge_distance_m(lat, lon) >= margin_m

    def height(self, east, north):
        lat, lon = local_to_latlon(east, north, *self.origin[:2])
        elevation = self.heightmap.elevation_at(lat, lon)
        if not math.isfinite(elevation):
            return None
        # The geoid varies by centimetres across a DEM; its value at the
        # origin serves the whole map.
        return elevation + self._geoid_m - self.origin[2] + self.offset_m


def dem_files(cache_dir):
    """[(path, lat, lon, area_km, resolution_m)] of the SRTM DEMs DemNav
    cached (synthetic DEMs excluded)."""
    found = []
    for path in sorted(glob.glob(os.path.join(cache_dir, 'dem_*.npy'))):
        match = DEM_NAME.match(os.path.basename(path))
        if match:
            lat, lon, area_km, resolution = match.groups()
            found.append((path, float(lat), float(lon), float(area_km),
                          float(resolution)))
    return found


def best_dem(cache_dir, lat, lon):
    """The cached DEM whose map holds (lat, lon) farthest from its edge,
    as (path, lat, lon, area_km, resolution_m), or None."""
    best, best_margin = None, 0.0
    for entry in dem_files(cache_dir):
        _, dem_lat, dem_lon, area_km, _ = entry
        east, north = latlon_to_local(lat, lon, dem_lat, dem_lon)
        margin = 500.0 * area_km - max(abs(east), abs(north))
        if margin > best_margin:
            best, best_margin = entry, margin
    return best


def load_heightmap(entry):
    """demnav_pkg's HeightMapCache for a cached DEM entry (no download:
    the file exists)."""
    from demnav_pkg.heightmap import HeightMapCache
    path, lat, lon, area_km, resolution = entry
    return HeightMapCache(lat, lon, area_km, resolution,
                          cache_dir=os.path.dirname(path))


def intersect_terrain(origin, direction, height_fn, max_range_m=1000.0,
                      min_step_m=0.25, tolerance_m=0.05):
    """First point where the ray origin + t * direction (t >= 0, unit
    direction) meets the ground height_fn(east, north), or None (looking at
    or above the horizon, starting below ground, leaving the map, beyond
    max_range_m).

    Steps half the height above ground along the ray (it cannot step
    through slopes up to 45 deg), down to min_step_m, then bisects."""
    origin = np.asarray(origin, dtype=float)
    direction = np.asarray(direction, dtype=float)
    if direction[2] >= -1e-6:
        return None

    def gap(t):
        point = origin + t * direction
        ground = height_fn(point[0], point[1])
        return None if ground is None else point[2] - ground

    t, above = 0.0, gap(0.0)
    if above is None or above <= 0.0:
        return None
    while t < max_range_m:
        step = max(min_step_m, 0.5 * above)
        t_next = min(t + step, max_range_m)
        below = gap(t_next)
        if below is None:
            return None
        if below <= 0.0:
            low, high = t, t_next
            for _ in range(60):
                middle = 0.5 * (low + high)
                value = gap(middle)
                if value is None:
                    return None
                if value > 0.0:
                    low = middle
                else:
                    high = middle
                if high - low < tolerance_m:
                    break
            return origin + 0.5 * (low + high) * direction
        if t_next >= max_range_m:
            return None
        t, above = t_next, below
    return None


@dataclass(frozen=True)
class Geolocation:
    position: np.ndarray   # (east, north, up)
    covariance: np.ndarray  # 3x3
    range_m: float
    depression_rad: float  # of the ray, below the horizontal


def geolocate(u, v, box_size, pose, intrinsics, height_fn, mount_yaw_rad,
              stabilized, config):
    """Ground position of the pixel (u, v) seen in pose, on the ground
    height_fn(east, north), with its covariance, or None. box_size
    (pixels) sets the box-centre noise."""
    rotation = camera_world_rotation(pose.quaternion, mount_yaw_rad, stabilized)
    direction = pixel_ray(rotation, intrinsics, u, v)
    point = intersect_terrain(pose.position, direction, height_fn,
                              config.max_range_m)
    if point is None:
        return None
    range_m = float(np.linalg.norm(point - np.asarray(pose.position)))
    depression = math.asin(min(1.0, -float(direction[2])))
    sigma_px = math.hypot(config.pixel_sigma_px, 0.1 * max(0.0, min(box_size)))
    angular = math.hypot(sigma_px / min(intrinsics.fx, intrinsics.fy),
                         math.radians(config.attitude_sigma_deg))
    sine = max(math.sin(depression), 0.2)
    horizontal = math.hypot(range_m * angular / sine,
                            config.terrain_sigma_m * math.cos(depression) / sine)
    covariance = np.diag([horizontal ** 2, horizontal ** 2,
                          config.terrain_sigma_m ** 2])
    return Geolocation(point, covariance, range_m, depression)


# --- target filter ------------------------------------------------------
class ConstantVelocityFilter:
    """Kalman filter of the target's (east, north, up) position and
    velocity, white-acceleration process noise; times in seconds of the
    odometry clock."""

    def __init__(self, position, covariance, t, config):
        self.config = config
        self.x = np.zeros(6)
        self.x[:3] = np.asarray(position, dtype=float)
        self.P = np.zeros((6, 6))
        self.P[:3, :3] = np.asarray(covariance, dtype=float)
        speed = config.initial_speed_sigma_mps
        self.P[3:, 3:] = np.diag([speed ** 2, speed ** 2,
                                  (0.2 * speed) ** 2])
        self.t = float(t)
        self.rejections = 0
        # Turn rate (rad/s) of the filtered heading, low-pass: the set-points
        # predict a turning target along its arc (predicted_turning).
        self.turn_rate = 0.0
        self._heading = None

    def _transition(self, dt):
        F = np.eye(6)
        F[:3, 3:] = dt * np.eye(3)
        Q = np.zeros((6, 6))
        for axis, accel in ((0, self.config.process_accel_mps2),
                            (1, self.config.process_accel_mps2),
                            (2, self.config.process_vertical_accel_mps2)):
            q = accel ** 2
            Q[axis, axis] = q * dt ** 3 / 3.0
            Q[axis, axis + 3] = Q[axis + 3, axis] = q * dt ** 2 / 2.0
            Q[axis + 3, axis + 3] = q * dt
        return F, Q

    def predicted(self, t):
        """(state, covariance) at time t, without updating the filter."""
        dt = max(0.0, float(t) - self.t)
        F, Q = self._transition(dt)
        return F @ self.x, F @ self.P @ F.T + Q

    def predict(self, t):
        if t > self.t:
            self.x, self.P = self.predicted(t)
            self.t = float(t)

    def distance(self, position, covariance, t):
        """Squared Mahalanobis distance of a measurement from the position
        predicted to t (what the gate compares with gate_chi2), and the
        horizontal distance in metres from the position predicted along
        the target's arc (predicted_turning)."""
        state, P = self.predicted(t)
        residual = np.asarray(position, dtype=float) - state[:3]
        S = P[:3, :3] + np.asarray(covariance, dtype=float)
        turning = self.predicted_turning(t)
        return (float(residual @ np.linalg.solve(S, residual)),
                float(math.hypot(position[0] - turning[0], position[1] - turning[1])))

    def update(self, position, covariance, t):
        """Fuse a measurement; False when the gate rejects it."""
        self.predict(t)
        H = np.hstack((np.eye(3), np.zeros((3, 3))))
        residual = np.asarray(position, dtype=float) - H @ self.x
        S = H @ self.P @ H.T + np.asarray(covariance, dtype=float)
        distance = float(residual @ np.linalg.solve(S, residual))
        if distance > self.config.gate_chi2:
            self.rejections += 1
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ residual
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ covariance @ K.T
        self.rejections = 0
        self._update_turn_rate(t)
        return True

    def _update_turn_rate(self, t, time_constant_s=1.0, max_rate=1.0):
        if self.speed_mps < 1.0:
            self._heading = None
            self.turn_rate = 0.0
            return
        heading = math.atan2(self.x[4], self.x[3])
        if self._heading is not None and t > self._heading[1]:
            dt = t - self._heading[1]
            rate = wrap_angle(heading - self._heading[0]) / dt
            alpha = dt / (time_constant_s + dt)
            self.turn_rate += alpha * (min(max(rate, -max_rate), max_rate) -
                                       self.turn_rate)
        self._heading = (heading, t)

    def predicted_turning(self, t):
        """Position and velocity at time t, the horizontal motion turning
        at turn_rate (a coordinated turn; constant velocity when it is
        small), the vertical constant velocity."""
        state, _ = self.predicted(t)
        dt = max(0.0, float(t) - self.t)
        omega = self.turn_rate
        if abs(omega) * dt < 1e-3:
            return state
        # At most a quarter turn ahead: past it the arc is a guess.
        omega = math.copysign(min(abs(omega), 0.5 * math.pi / dt), omega)
        vx, vy = self.x[3], self.x[4]
        speed, heading = math.hypot(vx, vy), math.atan2(vy, vx)
        turned = heading + omega * dt
        radius = speed / omega
        state[0] = self.x[0] + radius * (math.sin(turned) - math.sin(heading))
        state[1] = self.x[1] - radius * (math.cos(turned) - math.cos(heading))
        state[3] = speed * math.cos(turned)
        state[4] = speed * math.sin(turned)
        return state

    @property
    def position(self):
        return self.x[:3].copy()

    @property
    def velocity(self):
        return self.x[3:].copy()

    @property
    def speed_mps(self):
        return float(math.hypot(self.x[3], self.x[4]))


# --- set-points ---------------------------------------------------------
@dataclass(frozen=True)
class Setpoint:
    position: tuple    # (east, north, up), navigation frame
    yaw: float         # rad, ENU
    bearing: float     # rad, drone -> target, None in centered mode
    depression_deg: float
    clamped: str       # why the height was limited ('' when not)


def clamp_height(up, ground, config):
    """up limited to [ground + min_agl_m, ground + max_agl_m], and why."""
    low, high = ground + config.min_agl_m, ground + config.max_agl_m
    if up < low:
        return low, f'held at the minimum {config.min_agl_m:g} m above ground'
    if up > high:
        return high, f'held at the maximum {config.max_agl_m:g} m above ground'
    return up, ''


def _ground(ground_fn, east, north, fallback):
    value = ground_fn(east, north) if ground_fn is not None else None
    return fallback if value is None else value


def centered_setpoint(target, altitude_agl_m, yaw, ground_fn, config):
    """Directly above the target at altitude_agl_m above its ground."""
    east, north, up = (float(value) for value in target[:3])
    ground = _ground(ground_fn, east, north, up)
    height, clamped = clamp_height(ground + altitude_agl_m, ground, config)
    return Setpoint((east, north, height), yaw, None, 90.0, clamped)


def slant_setpoint(target, drone, slant_range_m, depression_deg, bearing_hint,
                   mount_yaw_rad, current_yaw, ground_fn, config,
                   intrinsics=None):
    """On the target's bearing from the drone, slant_range_m away at
    depression_deg below the horizontal: R cos(theta) short of it, R
    sin(theta) above it, yawed so the target lies on the image's long axis.
    The bearing of a drone (nearly) above the target is bearing_hint."""
    east, north, up = (float(value) for value in target[:3])
    theta = math.radians(depression_deg)
    standoff = slant_range_m * math.cos(theta)
    d_east, d_north = east - drone[0], north - drone[1]
    if math.hypot(d_east, d_north) > 1.0 or bearing_hint is None:
        bearing = math.atan2(d_north, d_east)
    else:
        bearing = bearing_hint
    position_e = east - standoff * math.cos(bearing)
    position_n = north - standoff * math.sin(bearing)
    ground = _ground(ground_fn, position_e, position_n, up)
    height, clamped = clamp_height(up + slant_range_m * math.sin(theta),
                                   ground, config)
    if standoff < 0.5:
        yaw = current_yaw  # straight above: any yaw sees it at the centre
    else:
        yaw = long_axis_yaw(bearing, mount_yaw_rad, current_yaw, intrinsics)
    return Setpoint((position_e, position_n, height), yaw, bearing,
                    float(depression_deg), clamped)


def geometry(drone, target):
    """(slant range m, depression deg) from the drone to the target."""
    d = np.asarray(target[:3], dtype=float) - np.asarray(drone[:3], dtype=float)
    horizontal = math.hypot(d[0], d[1])
    return (float(np.linalg.norm(d)),
            math.degrees(math.atan2(-d[2], horizontal)))


class ReferenceShaper:
    """The position reference sent to the FollowReference behaviour: it
    moves toward the set-point no faster than the speed limits and stays
    within max_reference_lead_m of the aircraft horizontally (ArduPilot
    flies toward a far position target at its own WPNAV_SPEED)."""

    def __init__(self, config):
        self.config = config
        self.position = None
        self.yaw = None

    def reset(self, position, yaw):
        self.position = np.asarray(position, dtype=float).copy()
        self.yaw = float(yaw)

    def step(self, desired, desired_yaw, drone, dt):
        config = self.config
        if self.position is None:
            self.reset(drone, desired_yaw)
        dt = max(0.0, float(dt))
        desired = np.asarray(desired, dtype=float)
        drone = np.asarray(drone, dtype=float)
        delta = desired[:2] - self.position[:2]
        distance = float(np.linalg.norm(delta))
        step = config.max_speed_mps * dt
        if distance > step:
            self.position[:2] += delta / distance * step
        else:
            self.position[:2] = desired[:2]
        ahead = self.position[:2] - drone[:2]
        lead = float(np.linalg.norm(ahead))
        if lead > config.max_reference_lead_m:
            self.position[:2] = drone[:2] + ahead / lead * config.max_reference_lead_m
        rise = config.max_vertical_speed_mps * dt
        self.position[2] += min(max(desired[2] - self.position[2], -rise), rise)
        turn = math.radians(config.max_yaw_rate_dps) * dt
        self.yaw = wrap_angle(self.yaw + min(max(
            wrap_angle(desired_yaw - self.yaw), -turn), turn))
        return self.position.copy(), self.yaw


# --- commands -----------------------------------------------------------
@dataclass(frozen=True)
class Command:
    action: str
    target_id: str = None
    mode: str = None
    altitude_agl_m: float = None
    slant_range_m: float = None
    depression_deg: float = None


def parse_command(text, config):
    """(Command, '') for a valid /tracking/command JSON string, else
    (None, reason). Optional fields: mode (centered), altitude_agl_m (the
    current height above ground, resolved later), slant_range_m and
    depression_deg (config defaults)."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None, 'not valid JSON'
    if not isinstance(data, dict):
        return None, 'not a JSON object'
    action = data.get('action')
    if action == 'stop':
        return Command('stop'), ''
    if action != 'track':
        return None, f'unknown action {action!r} (track or stop)'
    target_id = data.get('target_id')
    if _number(target_id) and float(target_id).is_integer():
        target_id = str(int(target_id))
    if not isinstance(target_id, str) or not target_id.strip():
        return None, 'target_id must be a non-empty string'
    target_id = target_id.strip()
    mode = data.get('mode', 'centered')
    if mode not in MODES:
        return None, f'unknown mode {mode!r} (centered or slant)'
    if mode == 'centered':
        altitude = data.get('altitude_agl_m')
        if altitude is not None:
            if not _number(altitude):
                return None, 'altitude_agl_m must be a number'
            if not config.min_agl_m <= altitude <= config.max_agl_m:
                return None, (f'altitude_agl_m must be within '
                              f'[{config.min_agl_m:g}, {config.max_agl_m:g}] m')
            altitude = float(altitude)
        return Command('track', target_id, mode, altitude_agl_m=altitude), ''
    slant = data.get('slant_range_m', config.slant_range_m)
    depression = data.get('depression_deg', config.depression_deg)
    if not _number(depression):
        return None, 'depression_deg must be a number'
    if not MIN_DEPRESSION_DEG <= depression <= MAX_DEPRESSION_DEG:
        return None, (f'depression_deg must be within [{MIN_DEPRESSION_DEG:g}, '
                      f'{MAX_DEPRESSION_DEG:g}] deg (nadir camera)')
    if not _number(slant) or slant <= 0.0:
        return None, 'slant_range_m must be a positive number'
    height = slant * math.sin(math.radians(depression))
    if not config.min_agl_m - 1e-6 <= height <= config.max_agl_m + 1e-6:
        low = config.min_agl_m / math.sin(math.radians(depression))
        high = config.max_agl_m / math.sin(math.radians(depression))
        return None, (f'slant_range_m at {depression:g} deg must be within '
                      f'[{low:.1f}, {high:.1f}] m (height above the target '
                      f'{config.min_agl_m:g}-{config.max_agl_m:g} m)')
    return Command('track', target_id, mode, slant_range_m=float(slant),
                   depression_deg=float(depression)), ''


# --- the tracker ----------------------------------------------------------
@dataclass
class Drone:
    """What the tracker knows of the aircraft now."""
    position: tuple            # (east, north, up)
    yaw: float                 # rad
    agl_m: float = None        # height above ground, if known
    flying: bool = True
    busy: str = ''             # a behaviour that must not be pre-empted


@dataclass
class Measurement:
    position: np.ndarray
    covariance: np.ndarray
    t: float                   # seconds, odometry clock


@dataclass
class TrackerCore:
    """The tracker's state machine (tracking.md): commands, measurements,
    timeouts and pre-emption in, actions out. Actions are strings for the
    ROS node: 'preempt' (stop a running GoTo/FollowPath), 'follow' (start
    the FollowReference behaviour), 'hover' (stop it: the behaviour leaves
    the aircraft hovering in place)."""
    config: TrackerConfig = field(default_factory=TrackerConfig)
    state: str = 'idle'
    message: str = 'waiting for a command'
    command: Command = None
    filter: ConstantVelocityFilter = None
    since: float = 0.0
    last_seen: float = None
    failure: str = ''
    bearing: float = None
    yaw_hold: float = None
    following: bool = False
    retry_at: float = 0.0
    fov_min_depression: float = MIN_DEPRESSION_DEG
    notes: tuple = ()
    # Learnt on top of lead_s: ArduPilot's lag behind a moving position
    # target, a property of the autopilot, kept across tracks.
    lead_extra_s: float = 0.0
    adapted_at: float = None
    # The selected track's class and box size (longer side, pixels) in
    # its latest detection, and the id it was handed over from, if it was.
    target_class: str = None
    target_side_px: float = None
    handed_over_from: str = None

    @property
    def active(self):
        return self.state in ACTIVE_STATES

    def _enter(self, state, now, message):
        if state != self.state:
            self.since = now
        self.state = state
        self.message = message

    def _reject(self, reason, now):
        message = f'rejected command: {reason}'
        if self.state in ('idle', 'error') and not self.following:
            self._enter('error', now, message)
        else:
            self.message = message
        return []

    def handle_command(self, text, now, drone=None):
        command, error = parse_command(text, self.config)
        if command is None:
            return self._reject(error, now)
        if command.action == 'stop':
            actions = ['hover'] if self.following else []
            was_active = self.active
            self._clear()
            self._enter('idle', now, 'stopped: hovering' if was_active
                        else 'waiting for a command')
            return actions
        if drone is not None and drone.busy:
            return self._reject(f'{drone.busy} is running', now)
        if self.config.require_flying and drone is not None and not drone.flying:
            return self._reject('the aircraft is not flying', now)
        same_target = (self.command is not None and self.filter is not None and
                       self.command.target_id == command.target_id and
                       self.state == 'tracking')
        if command.mode == 'centered' and command.altitude_agl_m is None:
            agl = drone.agl_m if drone is not None else None
            if agl is not None and math.isfinite(agl):
                command = replace(command, altitude_agl_m=min(
                    max(agl, self.config.min_agl_m), self.config.max_agl_m))
        if drone is not None:
            self.yaw_hold = drone.yaw
        self.command = command
        self.bearing = None
        self.failure = ''
        self.retry_at = 0.0
        if same_target:
            self._enter('tracking', now, f'following {command.target_id} '
                        f'({command.mode})')
            return ['preempt', 'follow']
        # A new target: hover until it is placed.
        actions = ['preempt'] + (['hover'] if self.following else [])
        self.following = False
        self.filter = None
        self.last_seen = None
        self.target_class = None
        self.target_side_px = None
        self.handed_over_from = None
        self._enter('acquiring', now, f'acquiring {command.target_id}')
        return actions

    def _clear(self):
        self.command = None
        self.filter = None
        self.last_seen = None
        self.bearing = None
        self.failure = ''
        self.following = False
        self.target_class = None
        self.target_side_px = None
        self.handed_over_from = None

    def _following_message(self):
        note = (f'; handed over from {self.handed_over_from}'
                if self.handed_over_from else '')
        return f'following {self.command.target_id} ({self.command.mode}){note}'

    def measurement(self, measurement, now):
        """A geolocated detection of the selected target."""
        if not self.active or self.command is None:
            return []
        self.failure = ''
        reacquired = self.state in ('lost', 'acquiring') or (
            self.state == 'error' and self.last_seen is not None and
            now - self.last_seen >= self.config.lost_timeout_s)
        if self.filter is None or reacquired:
            self.filter = ConstantVelocityFilter(
                measurement.position, measurement.covariance, measurement.t,
                self.config)
        elif not self.filter.update(measurement.position,
                                    measurement.covariance, measurement.t):
            if self.filter.rejections < self.config.max_rejections:
                return []
            # The target is consistently elsewhere: start again there.
            self.filter = ConstantVelocityFilter(
                measurement.position, measurement.covariance, measurement.t,
                self.config)
        self.last_seen = now
        if self.state == 'error' and now < self.retry_at:
            return []  # the follow behaviour failed: retry later
        if self.state != 'tracking':
            self._enter('tracking', now, self._following_message())
            return ['follow']
        return []

    def handover_open(self, now):
        """Whether the selected track may be handed over now (module doc):
        the target placed, last at most handover_window_s ago."""
        return bool(self.active and self.command is not None and
                    self.filter is not None and self.last_seen is not None and
                    self.target_class and
                    now - self.last_seen <= self.config.handover_window_s)

    def handover(self, candidates, now):
        """The id to hand the selected target over to, or None.
        candidates: {track id: (Measurement, class, box side px)}, the
        other tracks of a frame without the selected id. The only one of
        the target's kind and box size (longer sides within
        handover_size_ratio) within handover_max_m (plus half the distance
        it covered since it was placed) of its predicted position and
        inside the filter's gate, with the next such one at least 1.5 times
        as far."""
        if not self.handover_open(now):
            return None
        near = []
        ratio = self.config.handover_size_ratio
        for track_id, (measurement, cls, side) in candidates.items():
            if track_id == self.command.target_id or not same_kind(cls, self.target_class):
                continue
            if self.target_side_px and side and not (
                    self.target_side_px / ratio <= side <= self.target_side_px * ratio):
                continue
            chi2, metres = self.filter.distance(measurement.position,
                                                measurement.covariance, measurement.t)
            near.append((metres, chi2, track_id, measurement.t))
        near.sort()
        if not near:
            return None
        metres, chi2, track_id, t = near[0]
        travelled = self.filter.speed_mps * max(0.0, t - self.filter.t)
        if metres > self.config.handover_max_m + 0.5 * travelled or \
                chi2 > self.config.gate_chi2:
            return None
        if len(near) > 1 and near[1][0] < 1.5 * max(metres, 1.0):
            return None
        return track_id

    def hand_over(self, track_id, now):
        """Follow the target under track_id from now on."""
        self.handed_over_from = self.command.target_id
        self.command = replace(self.command, target_id=str(track_id))
        if self.state == 'tracking':
            self.message = self._following_message()

    def geolocation_failed(self, reason, now):
        """A detection of the selected target that could not be placed."""
        if self.active:
            self.failure = reason

    def tick(self, now):
        """Timeouts: the target unseen for lost_timeout_s (or never
        placed) means hovering in place."""
        if self.state not in ('acquiring', 'tracking'):
            return []
        reference = self.last_seen if self.last_seen is not None else self.since
        if now - reference < self.config.lost_timeout_s:
            return []
        actions = ['hover'] if self.following else []
        target = self.command.target_id
        if self.failure:
            self._enter('error', now, f'cannot place {target}: {self.failure}; '
                        'hovering')
        else:
            self._enter('lost', now, f'{target} not seen for '
                        f'{now - reference:.1f} s; hovering')
        return actions

    def preempt(self, reason, now):
        """Something else now commands the aircraft (a map goal, a mission
        step): tracking ends; the follow behaviour is stopped."""
        if not self.active:
            return []
        actions = ['hover'] if self.following else []
        self._clear()
        self._enter('idle', now, f'pre-empted by {reason}')
        return actions

    def follow_failed(self, reason, now, retry_s=2.0):
        """The FollowReference behaviour refused or ended the follow;
        tried again retry_s later while the target is seen."""
        if not self.active:
            return []
        self.following = False
        self.retry_at = now + retry_s
        self._enter('error', now, f'cannot follow: {reason}; hovering')
        return []

    def target_state(self, now):
        """(position, velocity) of the target predicted to now, or None."""
        if self.filter is None:
            return None
        state, _ = self.filter.predicted(now)
        return state[:3], state[3:]

    def resolved_agl(self, drone, target_up, ground_fn):
        """The centered mode's height above ground: the command's, else
        the drone's height above the ground (or the target) when the
        command arrived, within the limits."""
        agl = self.command.altitude_agl_m
        if agl is None:
            ground = _ground(ground_fn, drone.position[0], drone.position[1],
                             target_up)
            agl = drone.position[2] - ground
            agl = min(max(agl, self.config.min_agl_m), self.config.max_agl_m)
            self.command = replace(self.command, altitude_agl_m=agl)
        return agl

    def _mode_setpoint(self, target, drone, ground_fn, mount_yaw_rad,
                       intrinsics, notes):
        if self.command.mode == 'centered':
            agl = self.resolved_agl(drone, target[2], ground_fn)
            yaw = self.yaw_hold if self.yaw_hold is not None else drone.yaw
            return centered_setpoint(target, agl, yaw, ground_fn, self.config)
        self.fov_min_depression = min_depression_for_fov(
            intrinsics, self.config.fov_margin_deg)
        depression = max(self.command.depression_deg, self.fov_min_depression)
        if depression > self.command.depression_deg:
            notes.append(f'depression held at {depression:.1f} deg '
                         '(camera field of view)')
        return slant_setpoint(
            target, drone.position, self.command.slant_range_m, depression,
            self.bearing, mount_yaw_rad, drone.yaw, ground_fn, self.config,
            intrinsics)

    def adapt_lead(self, now, drone, setpoint_now, velocity, turn_rate=0.0):
        """Learn the lead ArduPilot needs: while following a target moving
        steadily below the speed limit, the aircraft's distance behind (or
        ahead of) the set-point for the target now, along its velocity,
        divided by its speed, is lead time missing (or in excess).
        Integrated at lead_adapt_rate per second, within [-lead_s,
        max_lead_extra_s]; only while the target drives straight (a turn
        adds lag the arc prediction makes up), not while catching up (more
        than max_reference_lead_m off) or when the target outruns the
        limit."""
        last, self.adapted_at = self.adapted_at, now
        speed = math.hypot(velocity[0], velocity[1])
        if (last is None or not self.following or speed < 1.0 or
                speed > 0.9 * self.config.max_speed_mps or
                abs(turn_rate) > 0.05):
            return
        offset = np.subtract(drone.position[:2], setpoint_now.position[:2])
        if np.linalg.norm(offset) > self.config.max_reference_lead_m:
            return
        along = float(offset @ np.asarray(velocity[:2])) / speed
        dt = min(max(now - last, 0.0), 1.0)
        self.lead_extra_s = min(max(
            self.lead_extra_s - self.config.lead_adapt_rate * dt * along / speed,
            -self.config.lead_s), self.config.max_lead_extra_s)

    def setpoint(self, now, drone, ground_fn, mount_yaw_rad, intrinsics=None):
        """Where the aircraft should be: the mode's set-point for the target
        predicted lead_s (plus the learnt lead_extra_s) ahead, or None."""
        if self.state != 'tracking' or self.filter is None:
            self.adapted_at = None
            return None
        notes = []
        state_now = self.filter.predicted_turning(now)
        setpoint_now = self._mode_setpoint(state_now[:3], drone, ground_fn,
                                           mount_yaw_rad, intrinsics, [])
        self.adapt_lead(now, drone, setpoint_now, state_now[3:5],
                        self.filter.turn_rate)
        lead = self.config.lead_s + self.lead_extra_s
        state = self.filter.predicted_turning(now + lead)
        setpoint = self._mode_setpoint(state[:3], drone, ground_fn,
                                       mount_yaw_rad, intrinsics, notes)
        if self.command.mode == 'slant':
            self.bearing = setpoint.bearing
        if setpoint.clamped:
            notes.append(f'height {setpoint.clamped}')
        self.notes = tuple(notes)
        return setpoint

    def estimate(self, now):
        """The target's position: predicted to now while tracking, the
        last estimate while lost or failing; None without one."""
        if self.filter is None or not self.active:
            return None
        if self.state == 'tracking':
            return self.target_state(now)[0]
        return self.filter.position

    def status(self, now, drone_position=None, notes=()):
        """The /tracking/status object (tracking.md). slant_range_m and
        depression_deg are the current geometry from the drone to the
        target estimate."""
        message = self.message
        if self.state == 'tracking':
            extra = [note for note in tuple(self.notes) + tuple(notes) if note]
            if extra:
                message = '; '.join([message] + extra)
        status = {'state': self.state, 'mode': None, 'target_id': None,
                  'target': None, 'slant_range_m': None,
                  'depression_deg': None, 'message': message}
        if self.command is not None and self.active:
            status['mode'] = self.command.mode
            status['target_id'] = self.command.target_id
        position = self.estimate(now)
        if position is not None:
            status['target'] = {
                'east': round(float(position[0]), 2),
                'north': round(float(position[1]), 2),
                'up': round(float(position[2]), 2),
                'speed_mps': round(self.filter.speed_mps, 2)}
            if drone_position is not None:
                slant, depression = geometry(drone_position, position)
                status['slant_range_m'] = round(slant, 2)
                status['depression_deg'] = round(depression, 2)
        return status


def status_json(status):
    return json.dumps(status, separators=(', ', ': '))


def quaternion_up_component(quaternion):
    """Vertical component of the body z axis (cos roll cos pitch)."""
    return float(quaternion_matrix(*quaternion)[2, 2])
