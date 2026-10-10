#!/usr/bin/env python3
"""Check that an Aerodrone navigation simulation is healthy.

One validator for every Aerodrone stack: the Gazebo, Genesis and Isaac
compositions (``validation`` profile) and the lite SITL stack
(tools/gps_denied_sitl.yaml, run by tools/gps_denied_sitl_test.sh once the
vehicle is airborne). It passes (exit 0) as soon as all of these hold at the
same moment:

- raw MAVROS GPS, odometry and IMU (the ``sensor_measurements`` mirrors that
  DemNav, WildNav and the fusion node consume) are fresh and finite;
- the downward camera image and its camera_info (WildNav) and the depth image
  and its camera_info (DemNav) are fresh and well formed, each image agrees
  with its camera_info (frame, size), at least a quarter of the depth pixels
  are valid (DemNav's own threshold), and the rangefinder is fresh and sane
  when the stack has one;
- ``/navigation/origin`` (the latched EKF origin) has arrived and is a sane
  position near the deployment site;
- ``/navigation/odometry`` and ``/navigation/fix`` are fresh, finite, with a
  known, positive and bounded covariance, and the fix status is usable;
- the DemNav and WildNav ``valid`` streams are alive, their ``confidence``
  values lie in [0, 1], ``correction_source`` names a known source, and
  optionally DemNav/WildNav have produced a valid match and the fusion is
  applying corrections;
- the TF links the stack defines resolve, and dynamic ones are recent.

Otherwise it exits 1 after ``--timeout-s``, listing every missing, stale or
invalid item. Topic names default to the stack's environment (the contract in
systems/aerodrone/config/topics.env) and can be overridden on the command
line, so the same script serves every stack.

The checks are plain Python over message attributes; ROS is imported only by
run(), so tests/test_aerodrone_navsim_validator.py exercises them without it.
"""

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from functools import partial
from typing import Callable, Optional

import numpy as np

# sensor_msgs/NavSatFix and NavSatStatus constants, repeated so the checks
# run without ROS.
COVARIANCE_TYPE_UNKNOWN = 0
STATUS_FIX = 0
# std_msgs/String values of the fusion node's correction_source.
DEAD_RECKONING = 'dead_reckoning'
CORRECTION_SOURCES = ('demnav', 'wildnav', 'demnav+wildnav', DEAD_RECKONING)
NAV_SOURCES = ('demnav', 'wildnav')
# Depth encodings DemNav reads (cv_bridge 32FC1 conversion): numpy type.
DEPTH_ENCODINGS = {'32FC1': 'f4', '16UC1': 'u2', 'mono16': 'u2'}
# DemNav skips depth frames with fewer valid (finite) pixels than this.
MIN_VALID_DEPTH_FRACTION = 0.25


# ---------------------------------------------------------------------------
# Message checks: each returns None when the message is fine, else a reason.
# ---------------------------------------------------------------------------
def _finite(values):
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def _frame(msg):
    return str(getattr(msg.header, 'frame_id', '') or '')


def _zero_stamp(msg):
    stamp = msg.header.stamp
    return int(stamp.sec) == 0 and int(stamp.nanosec) == 0


def _frame_problem(msg, expected=''):
    frame = _frame(msg)
    if not frame:
        return 'empty header.frame_id'
    if expected and frame != expected:
        return f'header.frame_id {frame!r}, expected {expected!r}'
    return None


def _covariance_problem(covariance, size, positive=(), max_sigma_m=None,
                        symmetric=False):
    """Problem with a row-major size x size covariance, or None.

    positive: diagonal indices that must be > 0 (the others >= 0);
    max_sigma_m bounds the horizontal (first two) standard deviations."""
    values = list(covariance)
    if len(values) != size * size:
        return f'has {len(values)} elements, expected {size * size}'
    if not _finite(values):
        return 'is not finite'
    diagonal = [float(values[i * size + i]) for i in range(size)]
    if any(value < 0.0 for value in diagonal):
        return f'has a negative variance on the diagonal {diagonal}'
    for index in positive:
        if diagonal[index] <= 0.0:
            return f'has a zero variance at diagonal index {index}'
    if symmetric:
        for i in range(size):
            for j in range(i + 1, size):
                a, b = float(values[i * size + j]), float(values[j * size + i])
                if abs(a - b) > 1e-6 * max(1.0, abs(a), abs(b)):
                    return f'is not symmetric ({i},{j})'
    if max_sigma_m is not None:
        sigma = math.sqrt(max(diagonal[0], diagonal[1]))
        if sigma > max_sigma_m:
            return f'horizontal sigma {sigma:.0f} m exceeds {max_sigma_m:g} m'
    return None


def check_navsatfix(msg, require_covariance=False, require_fix=False,
                    max_sigma_m=None):
    lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
    if not _finite([lat, lon, alt]):
        return 'non-finite latitude/longitude/altitude'
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return f'latitude/longitude ({lat}, {lon}) out of range'
    if lat == 0.0 and lon == 0.0:
        return 'null position (0, 0)'
    known = int(msg.position_covariance_type) != COVARIANCE_TYPE_UNKNOWN
    if require_covariance and not known:
        return 'position covariance type UNKNOWN'
    if known:
        problem = _covariance_problem(
            msg.position_covariance, 3,
            positive=(0, 1) if require_covariance else (),
            max_sigma_m=max_sigma_m, symmetric=require_covariance)
        if problem:
            return f'position covariance {problem}'
    if require_fix and int(msg.status.status) < STATUS_FIX:
        return (f'status {int(msg.status.status)} (NO_FIX): not usable as a '
                'navigation prior')
    return _frame_problem(msg)


def check_odometry(msg, require_covariance=False, max_sigma_m=None,
                   expected_frame=''):
    pose = msg.pose.pose
    p, q = pose.position, pose.orientation
    if not _finite([p.x, p.y, p.z]):
        return 'non-finite position'
    if not _finite([q.x, q.y, q.z, q.w]):
        return 'non-finite orientation'
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if abs(norm - 1.0) > 0.05:
        return f'orientation quaternion norm {norm:.3f}, expected 1'
    twist = msg.twist.twist
    if not _finite([twist.linear.x, twist.linear.y, twist.linear.z,
                    twist.angular.x, twist.angular.y, twist.angular.z]):
        return 'non-finite twist'
    problem = _covariance_problem(
        msg.pose.covariance, 6,
        positive=(0, 1) if require_covariance else (),
        max_sigma_m=max_sigma_m, symmetric=require_covariance)
    if problem:
        return f'pose covariance {problem}'
    if not str(getattr(msg, 'child_frame_id', '') or ''):
        return 'empty child_frame_id'
    if require_covariance and _zero_stamp(msg):
        return 'zero header.stamp'
    return _frame_problem(msg, expected_frame)


def check_imu(msg):
    av, la = msg.angular_velocity, msg.linear_acceleration
    if not _finite([av.x, av.y, av.z]):
        return 'non-finite angular velocity'
    if not _finite([la.x, la.y, la.z]):
        return 'non-finite linear acceleration'
    if float(msg.orientation_covariance[0]) != -1.0:
        q = msg.orientation
        if not _finite([q.x, q.y, q.z, q.w]):
            return 'non-finite orientation'
    return _frame_problem(msg)


def _depth_valid_fraction(msg):
    """Fraction of usable depth pixels (finite floats, non-zero integers),
    or a reason string for an unreadable image."""
    dtype = DEPTH_ENCODINGS.get(str(msg.encoding))
    if dtype is None:
        return (f'unsupported depth encoding {msg.encoding!r} '
                f'(expected one of {sorted(DEPTH_ENCODINGS)})')
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    item = np.dtype(dtype).itemsize
    if step < width * item:
        return f'step {step} is smaller than width x {item} bytes'
    rows = np.frombuffer(msg.data, dtype=np.uint8, count=step * height)
    rows = rows.reshape(height, step)[:, :width * item]
    order = '>' if msg.is_bigendian else '<'
    depth = np.ascontiguousarray(rows).view(order + dtype)
    if dtype == 'f4':
        valid = np.isfinite(depth)
    else:
        valid = depth > 0
    return float(valid.mean())


def check_image(msg, depth=False, expected_frame='',
                min_valid_fraction=MIN_VALID_DEPTH_FRACTION):
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if width <= 0 or height <= 0:
        return f'empty image ({width}x{height})'
    if not str(msg.encoding):
        return 'empty encoding'
    if step < width:
        return f'step {step} is smaller than the width {width}'
    if len(msg.data) < step * height:
        return f'{len(msg.data)} data bytes, expected {step * height}'
    if _zero_stamp(msg):
        return 'zero header.stamp'
    problem = _frame_problem(msg, expected_frame)
    if problem:
        return problem
    if depth:
        fraction = _depth_valid_fraction(msg)
        if isinstance(fraction, str):
            return fraction
        if fraction < min_valid_fraction:
            return (f'only {100.0 * fraction:.0f}% valid depth pixels '
                    f'(DemNav needs {100.0 * min_valid_fraction:.0f}%)')
    return None


def check_camera_info(msg, expected_frame=''):
    width, height = int(msg.width), int(msg.height)
    if width <= 0 or height <= 0:
        return f'empty camera_info size ({width}x{height})'
    k = list(msg.k)
    if len(k) != 9 or not _finite(k):
        return 'non-finite intrinsics K'
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    if fx <= 0.0 or fy <= 0.0:
        return f'non-positive focal length (fx={fx}, fy={fy})'
    if not (0.0 <= cx <= width and 0.0 <= cy <= height):
        return f'principal point ({cx}, {cy}) outside the {width}x{height} image'
    if not _finite(list(msg.d)):
        return 'non-finite distortion coefficients'
    return _frame_problem(msg, expected_frame)


def check_camera_pair(image, info, exact_size=True):
    """An image and the camera_info its consumer pairs it with."""
    image_frame, info_frame = _frame(image), _frame(info)
    if image_frame != info_frame:
        return (f'image frame_id {image_frame!r} differs from camera_info '
                f'frame_id {info_frame!r}')
    size, info_size = (int(image.width), int(image.height)), (
        int(info.width), int(info.height))
    if exact_size:
        if size != info_size:
            return (f'image is {size[0]}x{size[1]} but camera_info is '
                    f'{info_size[0]}x{info_size[1]}')
    elif min(size) > 0 and min(info_size) > 0 and abs(
            size[0] / size[1] - info_size[0] / info_size[1]) > 0.01:
        return (f'image {size[0]}x{size[1]} and camera_info '
                f'{info_size[0]}x{info_size[1]} have different aspect ratios')
    return None


def check_range(msg):
    low, high, value = msg.min_range, msg.max_range, msg.range
    if not _finite([low, high]) or not 0.0 <= low < high:
        return f'invalid range limits [{low}, {high}]'
    if math.isnan(value):
        return 'NaN range'
    # REP 117: +inf/-inf mean nothing within / closer than the limits.
    if math.isfinite(value) and not low <= value <= high:
        return f'range {value} outside [{low}, {high}]'
    return _frame_problem(msg)


def _distance_km(lat0, lon0, lat1, lon1):
    """Great-circle distance (haversine), kilometres."""
    p0, p1 = math.radians(lat0), math.radians(lat1)
    dp, dl = p1 - p0, math.radians(lon1 - lon0)
    h = (math.sin(dp / 2) ** 2
         + math.cos(p0) * math.cos(p1) * math.sin(dl / 2) ** 2)
    return 2.0 * 6371.0088 * math.asin(min(1.0, math.sqrt(h)))


def check_origin(msg, expected=None, max_distance_km=50.0):
    position = msg.position
    lat, lon, alt = position.latitude, position.longitude, position.altitude
    if not _finite([lat, lon, alt]):
        return 'non-finite origin'
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return f'origin ({lat}, {lon}) out of range'
    if lat == 0.0 and lon == 0.0:
        return 'null origin (0, 0)'
    if not -1000.0 <= alt <= 10000.0:
        return f'origin altitude {alt:.1f} m is implausible'
    if expected is not None:
        distance = _distance_km(expected[0], expected[1], lat, lon)
        if distance > max_distance_km:
            return (f'origin ({lat:.6f}, {lon:.6f}) is {distance:.1f} km from '
                    f'the deployment site ({expected[0]:.6f}, '
                    f'{expected[1]:.6f}); limit {max_distance_km:g} km')
    return None


def check_confidence(msg):
    value = msg.data
    if not _finite([value]):
        return 'non-finite confidence'
    if not 0.0 <= float(value) <= 1.0:
        return f'confidence {float(value):.3f} outside [0, 1]'
    return None


def check_correction_source(msg, require_correction=False):
    source = str(msg.data)
    if source not in CORRECTION_SOURCES:
        return (f'unknown correction source {source!r} (expected one of '
                f'{", ".join(CORRECTION_SOURCES)})')
    if require_correction and source == DEAD_RECKONING:
        return ('dead_reckoning: no accepted DemNav/WildNav correction is '
                'active')
    return None


# ---------------------------------------------------------------------------
# Check bookkeeping
# ---------------------------------------------------------------------------
@dataclass(eq=False)
class TopicCheck:
    """One topic: its latest message must be valid and, for streams, recent.

    freshness_s None: arriving once is enough (latched topics, and outputs
    published only on events such as a DemNav terrain match). Optional
    checks only fail on an invalid message."""
    label: str
    topic: str
    kind: str
    validate: Optional[Callable] = None
    freshness_s: Optional[float] = 10.0
    required: bool = True
    latched: bool = False
    count: int = 0
    invalid_count: int = 0
    first_seen: Optional[float] = None
    last_seen: Optional[float] = None
    last_ok: Optional[float] = None
    last_msg: object = None
    error: Optional[str] = None

    def observe(self, msg, now):
        self.count += 1
        if self.first_seen is None:
            self.first_seen = now
        self.last_seen = now
        self.last_msg = msg
        try:
            reason = self.validate(msg) if self.validate else None
        except Exception as exc:  # a malformed message is a finding, not a crash
            reason = f'unreadable message ({type(exc).__name__}: {exc})'
        if reason:
            self.invalid_count += 1
            self.error = reason
        else:
            self.error = None
            self.last_ok = now
        return reason

    def problem(self, now):
        if self.count == 0:
            return 'missing: no message received' if self.required else None
        if self.error:
            return f'invalid: {self.error}'
        if self.freshness_s is not None:
            if now - self.last_ok > self.freshness_s:
                return (f'stale: no valid message for {now - self.last_ok:.1f} s '
                        f'(limit {self.freshness_s:g} s)')
            if self.count < 2:
                return 'only one message so far: not yet a stream'
        return None

    def summary(self, now):
        if self.count == 0:
            return 'no messages'
        text = f'{self.count} msgs'
        if self.freshness_s is not None and self.count > 1:
            span = max(self.last_seen - self.first_seen, 1e-3)
            text += f' ({(self.count - 1) / span:.1f} Hz)'
        if self.invalid_count:
            text += f', {self.invalid_count} invalid'
        return text


@dataclass
class CameraPair:
    label: str
    image: TopicCheck
    info: TopicCheck
    exact_size: bool = True

    def problem(self):
        if self.image.last_msg is None or self.info.last_msg is None:
            return None  # reported as missing by the topic checks
        try:
            return check_camera_pair(
                self.image.last_msg, self.info.last_msg, self.exact_size)
        except Exception as exc:
            return f'unreadable ({type(exc).__name__}: {exc})'


class NavsimValidator:
    """All checks of one stack, independent of ROS."""

    def __init__(self, checks, pairs=(), require_valid=()):
        self.checks = list(checks)
        self.pairs = list(pairs)
        # Bool checks that must report data=true at least once.
        self.require_valid = list(require_valid)
        self.valid_seen = set()

    def observe(self, check, msg, now):
        reason = check.observe(msg, now)
        if check in self.require_valid and not reason and bool(msg.data):
            self.valid_seen.add(check.topic)

    def problems(self, now, publishers=None, extra=()):
        """Every failing item, one line each. publishers(topic) -> count
        lets a missing topic say whether anything publishes it at all."""
        lines = []
        for check in self.checks:
            problem = check.problem(now)
            if problem is None:
                continue
            if check.count == 0 and publishers is not None:
                try:
                    count = publishers(check.topic)
                except Exception:
                    count = None
                if count == 0:
                    problem += ' (no publisher on this topic)'
                elif count:
                    problem += f' ({count} publisher(s), none delivered)'
            lines.append(f'{check.label} [{check.topic}]: {problem}')
        for pair in self.pairs:
            problem = pair.problem()
            if problem:
                lines.append(f'{pair.label}: {problem}')
        for check in self.require_valid:
            if check.topic not in self.valid_seen:
                lines.append(f'{check.label} [{check.topic}]: never reported '
                             'a valid match (data: true)')
        lines.extend(extra)
        return lines

    def report(self, now):
        width = max(len(check.label) for check in self.checks)
        return [f'{check.label:<{width}}  {check.topic}  {check.summary(now)}'
                for check in self.checks]


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
def derived_topic(navigation_odom_topic, leaf):
    """The fusion node's sibling outputs: /navigation/odometry ->
    /navigation/<leaf> (navigation_fusion._derived_topic)."""
    prefix = (navigation_odom_topic.rsplit('/', 1)[0]
              if '/' in navigation_odom_topic else '')
    return f'{prefix}/{leaf}'


def parse_tf_pairs(values):
    pairs = []
    for value in values:
        for token in str(value).replace(',', ' ').split():
            parent, sep, child = token.partition(':')
            if not sep or not parent or not child:
                raise ValueError(f'TF pair {token!r} is not parent:child')
            pairs.append((parent, child))
    return pairs


def _split(value):
    return [item for item in str(value or '').replace(',', ' ').split() if item]


def _env(name, default=''):
    return os.environ.get(name, default)


def _env_float(name):
    value = os.environ.get(name, '')
    try:
        return float(value) if value else None
    except ValueError:
        return None


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Check that an Aerodrone navigation simulation is healthy. '
                    'Topic defaults come from the environment '
                    '(systems/aerodrone/config/topics.env).')
    add = parser.add_argument
    add('--timeout-s', type=float, default=180.0,
        help='give up and fail after this long')
    add('--freshness-s', type=float, default=10.0,
        help='maximum age of the latest valid message of a stream')
    add('--min-duration-s', type=float, default=10.0,
        help='observe at least this long before passing')
    add('--wildnav-rate-hz', type=float,
        default=_env_float('WILDNAV_RATE_HZ') or 0.2,
        help='WildNav match rate; its streams may be 4 periods old')
    add('--gps-topic', default=_env('GPS_TOPIC', '/aerodrone/sensor_measurements/gps'))
    add('--odom-topic', default=_env(
        'RAW_ODOM_TOPIC', _env('ODOM_TOPIC', '/aerodrone/sensor_measurements/odom')))
    add('--imu-topic', default=_env('IMU_TOPIC', '/aerodrone/sensor_measurements/imu'))
    add('--image-topic', default=_env(
        'WILDNAV_IMAGE_TOPIC', '/aerodrone/sensor_measurements/downward_rgb/image_raw'),
        help='downward camera image WildNav matches')
    add('--image-info-topic', default=_env(
        'WILDNAV_CAMERA_INFO_TOPIC', '/aerodrone/sensor_measurements/downward_rgb/camera_info'))
    add('--depth-topic', default=_env(
        'DEPTH_TOPIC', '/aerodrone/sensor_measurements/downward_rgbd/depth'),
        help='depth image DemNav matches')
    add('--depth-info-topic', default=_env(
        'CAMERA_INFO_TOPIC', '/aerodrone/sensor_measurements/downward_rgbd/depth/camera_info'))
    add('--camera-frame', default=_env('NAVSIM_CAMERA_FRAME'),
        help='expected frame_id of the camera images and camera_info (empty: any)')
    add('--rangefinder-topic', default=_env('RANGEFINDER_TOPIC'),
        help='downward rangefinder; empty when the stack has none')
    add('--origin-topic', default='/navigation/origin')
    add('--navigation-odom-topic', default=_env('NAVIGATION_ODOM_TOPIC', '/navigation/odometry'))
    add('--navigation-frame', default='odom',
        help='frame_id of the fused odometry (the as2 state estimator odom frame)')
    add('--demnav-prefix', default='/demnav')
    add('--wildnav-prefix', default='/wildnav')
    add('--expected-lat', type=float, default=_env_float('INITIAL_LAT'))
    add('--expected-lon', type=float, default=_env_float('INITIAL_LON'))
    add('--max-origin-distance-km', type=float, default=50.0)
    add('--max-sigma-m', type=float, default=1000.0,
        help='largest sane horizontal sigma of the fused odometry/fix')
    add('--required-tf', nargs='*', default=_split(_env('NAVSIM_REQUIRED_TF')),
        metavar='PARENT:CHILD', help='TF links that must resolve')
    add('--tf-freshness-s', type=float, default=None,
        help='maximum age of a dynamic TF link (default --freshness-s, 0 off)')
    add('--require-valid', nargs='*', choices=NAV_SOURCES,
        default=[s for s in _split(_env('NAVSIM_REQUIRE_VALID')) if s in NAV_SOURCES],
        help='sources that must report at least one valid match')
    add('--require-correction', action='store_true',
        default=_truthy(_env('NAVSIM_REQUIRE_CORRECTION', 'false')),
        help='the fusion must be applying DemNav/WildNav corrections')
    args = parser.parse_args(argv)
    try:
        args.required_tf = parse_tf_pairs(args.required_tf)
    except ValueError as exc:
        parser.error(str(exc))
    if args.tf_freshness_s is None:
        args.tf_freshness_s = args.freshness_s
    return args


def build_validator(args):
    fresh = args.freshness_s
    wildnav_fresh = max(fresh, 4.0 / max(args.wildnav_rate_hz, 1e-3))
    checks = []

    def add(label, topic, kind, validate=None, freshness=fresh, **kwargs):
        check = TopicCheck(label, topic, kind, validate, freshness, **kwargs)
        checks.append(check)
        return check

    add('MAVROS GPS', args.gps_topic, 'navsatfix', check_navsatfix)
    add('MAVROS odometry', args.odom_topic, 'odometry', check_odometry)
    add('MAVROS IMU', args.imu_topic, 'imu', check_imu)
    frame = args.camera_frame
    image = add('WildNav image', args.image_topic, 'image',
                partial(check_image, expected_frame=frame))
    image_info = add('WildNav camera_info', args.image_info_topic, 'camera_info',
                     partial(check_camera_info, expected_frame=frame))
    depth = add('DemNav depth', args.depth_topic, 'image',
                partial(check_image, depth=True, expected_frame=frame))
    depth_info = add('DemNav camera_info', args.depth_info_topic, 'camera_info',
                     partial(check_camera_info, expected_frame=frame))
    if args.rangefinder_topic:
        add('rangefinder', args.rangefinder_topic, 'range', check_range)
    expected = None
    if args.expected_lat is not None and args.expected_lon is not None:
        expected = (args.expected_lat, args.expected_lon)
    add('EKF origin', args.origin_topic, 'geopoint',
        partial(check_origin, expected=expected,
                max_distance_km=args.max_origin_distance_km),
        freshness=None, latched=True)
    navigation = args.navigation_odom_topic
    add('fused odometry', navigation, 'odometry',
        partial(check_odometry, require_covariance=True,
                max_sigma_m=args.max_sigma_m,
                expected_frame=args.navigation_frame))
    add('fused fix', derived_topic(navigation, 'fix'), 'navsatfix',
        partial(check_navsatfix, require_covariance=True, require_fix=True,
                max_sigma_m=args.max_sigma_m))
    add('correction source', derived_topic(navigation, 'correction_source'),
        'string', partial(check_correction_source,
                          require_correction=args.require_correction))
    add('correction confidence',
        derived_topic(navigation, 'correction_confidence'), 'float32',
        check_confidence)
    require_valid = []
    for name, label, prefix, freshness in (
            ('demnav', 'DemNav', args.demnav_prefix, fresh),
            ('wildnav', 'WildNav', args.wildnav_prefix, wildnav_fresh)):
        valid = add(f'{label} valid', f'{prefix}/valid', 'bool', None, freshness)
        # DemNav publishes confidence only for a terrain correlation.
        add(f'{label} confidence', f'{prefix}/confidence', 'float32',
            check_confidence, freshness=None)
        add(f'{label} fix', f'{prefix}/fix', 'navsatfix',
            partial(check_navsatfix, require_covariance=True),
            freshness=None, required=False)
        if name in args.require_valid:
            require_valid.append(valid)
    pairs = [CameraPair('WildNav image vs camera_info', image, image_info, True),
             # DemNav scales the intrinsics to the depth image size.
             CameraPair('DemNav depth vs camera_info', depth, depth_info, False)]
    return NavsimValidator(checks, pairs, require_valid)


# ---------------------------------------------------------------------------
# ROS
# ---------------------------------------------------------------------------
def run(args):
    import rclpy
    from geographic_msgs.msg import GeoPointStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix, Range
    from std_msgs.msg import Bool, Float32, String
    from tf2_ros import Buffer, TransformException, TransformListener

    types = {'navsatfix': NavSatFix, 'odometry': Odometry, 'imu': Imu,
             'image': Image, 'camera_info': CameraInfo, 'range': Range,
             'geopoint': GeoPointStamped, 'bool': Bool, 'float32': Float32,
             'string': String}
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    validator = build_validator(args)

    rclpy.init()
    node = Node('aerodrone_navsim_validator')
    for check in validator.checks:
        # Best-effort subscriptions also match reliable publishers.
        node.create_subscription(
            types[check.kind], check.topic,
            lambda msg, check=check: validator.observe(check, msg, time.monotonic()),
            latched if check.latched else qos_profile_sensor_data)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)  # noqa: F841 (keeps subscriptions)

    def tf_problems():
        problems = []
        now_ns = node.get_clock().now().nanoseconds
        for parent, child in args.required_tf:
            try:
                transform = tf_buffer.lookup_transform(parent, child, Time())
            except TransformException as exc:
                reason = ' '.join(str(exc).split())[:160]
                problems.append(f'TF {parent} -> {child}: {reason}')
                continue
            stamp = transform.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if args.tf_freshness_s > 0 and stamp_ns > 0:
                age = (now_ns - stamp_ns) / 1e9
                if abs(age) > args.tf_freshness_s:
                    problems.append(
                        f'TF {parent} -> {child}: stale, latest transform is '
                        f'{age:.1f} s old (limit {args.tf_freshness_s:g} s)')
        return problems

    topics = [check.topic for check in validator.checks]
    print(f'Aerodrone navsim validation: {len(topics)} topics, '
          f'{len(args.required_tf)} TF links, timeout {args.timeout_s:g} s',
          flush=True)
    start = time.monotonic()
    deadline = start + args.timeout_s
    last_eval = last_progress = start
    problems = ['not evaluated']
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_eval < 0.5:
                continue
            last_eval = now
            problems = validator.problems(
                now, node.count_publishers, tf_problems())
            if not problems and now - start >= args.min_duration_s:
                print(f'Aerodrone navsim validation PASSED after '
                      f'{now - start:.1f} s', flush=True)
                for line in validator.report(now):
                    print(f'  {line}', flush=True)
                for parent, child in args.required_tf:
                    print(f'  TF {parent} -> {child}  ok', flush=True)
                return 0
            if now - last_progress >= 30.0:
                last_progress = now
                if problems:
                    print(f'waiting ({now - start:.0f} s): {len(problems)} open: '
                          + '; '.join(problems[:3])
                          + (' ...' if len(problems) > 3 else ''), flush=True)
                else:
                    print(f'all checks passing ({now - start:.0f} s); observing '
                          f'for {args.min_duration_s:g} s', flush=True)
        now = time.monotonic()
        problems = validator.problems(now, node.count_publishers, tf_problems())
    except KeyboardInterrupt:
        problems = ['interrupted'] + validator.problems(
            time.monotonic(), node.count_publishers, tf_problems())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    elapsed = time.monotonic() - start
    passing = [check.label for check in validator.checks
               if check.problem(time.monotonic()) is None and check.count]
    print(f'Aerodrone navsim validation FAILED after {elapsed:.1f} s; '
          f'{len(problems)} problem(s):', file=sys.stderr, flush=True)
    for problem in problems:
        print(f'  - {problem}', file=sys.stderr, flush=True)
    if passing:
        print('  passing: ' + ', '.join(passing), file=sys.stderr, flush=True)
    return 1


def main(argv=None):
    return run(parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
