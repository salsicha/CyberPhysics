#!/usr/bin/env python3
"""WildNav matching benchmark: latency, CPU and error of settings profiles.

Runs the WildNav node's own matcher (WildNavNode._run_match, ROS messages
and parameters as in flight, publishers captured) on stills made as the
Gazebo stack's synthetic downward camera makes them
(applications/gazebo/scripts/aerodrone_sensor_sim.py: a 128 x 96 px crop of
the cached tile at the vehicle, turned to the heading, at 640 x 480), at
random positions about the site, from a prior off the truth by its own
sigma. It reports per profile the time from still to fix, the CPU time the
match took (all threads), where it went, and the fix error against the
truth. tools/wildnav_profile.md has the results and how to run it on the
Pi; tests/test_wildnav_benchmark.py holds the frame geometry to the
simulator's.

Run it in the WildNav image (rclpy and wildnav_pkg) on a copy of a seeded
tile cache, with no network:

  docker run --rm --network none \\
    -v hil_sim_wildnav_gazebo_cache:/data/wildnav_cache:ro \\
    -v "$PWD/tools:/tools:ro" salsicha/cyberphysics:wildnav-latest \\
    python3 /tools/wildnav_benchmark.py --frames 20 \\
      --profile site --profile site_lite

A profile is a name from PROFILES or name=param:value,param:value (with
base:<PROFILES name> to start from one); each runs in a fresh node. The cache is copied to a scratch directory first, so
tile features a profile needs at another max_features are computed there
(as a first flight over unseeded tiles would) and kept out of the timing.
"""
import argparse
import json
import math
import os
import random
import shutil
import statistics
import sys
import tempfile
import time

import cv2
import numpy as np

# The simulated site (applications/aerostack2/config/mount_tamalpais.env).
SITE = (37.9234, -122.5967)

# The companion computer's lighter matching (compositions/blueos_nav_core.yaml).
LITE = {'match_stop_confidence': 0.5, 'opencv_threads': 2}

# Settings profiles: WildNav parameters over the node's defaults.
PROFILES = {
    # The simulated site's (mount_tamalpais.env), every candidate tile
    # matched: the HIL Pi before the lighter profile.
    'site': {
        'zoom': 17, 'search_radius_m': 1100.0, 'max_search_radius_m': 3300.0,
        'max_tiles': 169,
    },
    # The same with the lighter profile: the HIL Pi now.
    'site_lite': {
        'zoom': 17, 'search_radius_m': 1100.0, 'max_search_radius_m': 3300.0,
        'max_tiles': 169, **LITE,
    },
    # The aircraft's own tile search (blueos_nav_core.yaml without a site
    # env; the image seeds it about --site 37.7694,-122.4862, --area-m 250).
    'aircraft': {
        'zoom': 18, 'search_radius_m': 250.0, 'max_search_radius_m': 750.0,
        'max_tiles': 49,
    },
    'aircraft_lite': {
        'zoom': 18, 'search_radius_m': 250.0, 'max_search_radius_m': 750.0,
        'max_tiles': 49, **LITE,
    },
}

GAZEBO_CROP = (128, 96)
GAZEBO_SIZE = (640, 480)
GAZEBO_FOV_DEG = 69.0


def tile_position(lat, lon, zoom):
    from wildnav_pkg.satellite_cache import latlon_to_tile
    x, y = latlon_to_tile(lat, lon, zoom)
    tile_x, tile_y = int(x), int(y)
    return tile_x, tile_y, (x - tile_x) * 256.0, (y - tile_y) * 256.0


def gazebo_frame(tile, pixel_x, pixel_y, yaw_rad, size=GAZEBO_SIZE,
                 crop=GAZEBO_CROP):
    """The Gazebo stack's synthetic downward still of a vehicle at
    (pixel_x, pixel_y) of its 256 px tile, heading yaw_rad (ENU), as
    aerodrone_sensor_sim.py's _synthetic_satellite_image and
    _body_fixed_view make it: the vehicle at the principal point."""
    width, height = size
    crop_w, _ = crop
    side = int(math.ceil(math.hypot(*crop)))
    centre_x = int(np.clip(pixel_x, side * 0.5, 256 - side * 0.5))
    centre_y = int(np.clip(pixel_y, side * 0.5, 256 - side * 0.5))
    x0, y0 = centre_x - side // 2, centre_y - side // 2
    patch = tile[y0:y0 + side, x0:x0 + side]
    scale = width / crop_w
    side_px = int(round(side * scale))
    image = cv2.resize(patch, (side_px, side_px),
                       interpolation=cv2.INTER_CUBIC)
    centre = ((pixel_x - x0 + 0.5) * scale - 0.5,
              (pixel_y - y0 + 0.5) * scale - 0.5)
    matrix = cv2.getRotationMatrix2D(centre, 90.0 - math.degrees(yaw_rad), 1.0)
    matrix[0, 2] += width * 0.5 - centre[0]
    matrix[1, 2] += height * 0.5 - centre[1]
    return cv2.warpAffine(image, matrix, (width, height),
                          flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_REPLICATE)


def offset_latlon(lat, lon, east_m, north_m):
    from synthetic_world import METERS_PER_DEG_LAT
    return (lat + north_m / METERS_PER_DEG_LAT,
            lon + east_m / (METERS_PER_DEG_LAT * math.cos(math.radians(lat))))


def make_frames(cache_dir, zoom, count, area_m, prior_sigma_m, seed,
                site=SITE):
    """count (truth, prior, yaw, still) cases inside the cached tiles."""
    rng = random.Random(seed)
    frames = []
    attempts = 0
    while len(frames) < count:
        attempts += 1
        if attempts > count * 50:
            raise RuntimeError(
                f'no cached zoom-{zoom} tiles about {site} in {cache_dir}')
        radius = area_m * math.sqrt(rng.random())
        bearing = rng.uniform(0.0, 2.0 * math.pi)
        lat, lon = offset_latlon(*site, radius * math.cos(bearing),
                                 radius * math.sin(bearing))
        tile_x, tile_y, pixel_x, pixel_y = tile_position(lat, lon, zoom)
        tile = cv2.imread(os.path.join(cache_dir, f'{zoom}_{tile_x}_{tile_y}.jpg'),
                          cv2.IMREAD_COLOR)
        if tile is None:
            continue
        yaw = rng.uniform(-math.pi, math.pi)
        prior = offset_latlon(lat, lon, rng.gauss(0.0, prior_sigma_m),
                              rng.gauss(0.0, prior_sigma_m))
        frames.append({
            'truth': (lat, lon), 'prior': prior, 'yaw': yaw,
            'image': gazebo_frame(tile, pixel_x, pixel_y, yaw)})
    return frames


def parse_profile(text):
    """A PROFILES name, or name=param:value,... over the profile 'base'
    names (base:site,match_stop_confidence:0.5) or over the node's
    defaults."""
    if '=' not in text:
        return text, dict(PROFILES[text])
    name, body = text.split('=', 1)
    params = dict(PROFILES.get(name, {}))
    for item in filter(None, body.split(',')):
        key, value = item.split(':', 1)
        if key == 'base':
            params = {**PROFILES[value], **params}
            continue
        for kind in (int, float):
            try:
                value = kind(value)
                break
            except ValueError:
                pass
        if value in ('true', 'false'):
            value = value == 'true'
        params[key] = value
    return name, params


def quantiles(values):
    if not values:
        return None
    ordered = sorted(values)

    def at(q):
        return ordered[min(len(ordered) - 1, int(round(q * (len(ordered) - 1))))]
    return {'median': round(statistics.median(ordered), 3),
            'p95': round(at(0.95), 3), 'max': round(ordered[-1], 3)}


class Timers:
    """Wall time spent in the matcher's stages, wrapped around the node's
    cache and matcher (all calls are on the worker thread)."""

    def __init__(self, node):
        self.totals = {}
        cache = node.cache
        cache.describe = self._wrap('describe', cache.describe)
        cache.get_features = self._wrap('tile_features', cache.get_features)
        make_matcher = cache.matcher
        timers = self

        class Matcher:
            def __init__(self):
                self.inner = make_matcher()

            def knnMatch(self, *args, **kwargs):
                started = time.perf_counter()
                try:
                    return self.inner.knnMatch(*args, **kwargs)
                finally:
                    timers.add('knn_match', time.perf_counter() - started)
        cache.matcher = Matcher

    def add(self, name, seconds):
        self.totals[name] = self.totals.get(name, 0.0) + seconds

    def _wrap(self, name, function):
        def wrapped(*args, **kwargs):
            started = time.perf_counter()
            try:
                return function(*args, **kwargs)
            finally:
                self.add(name, time.perf_counter() - started)
        return wrapped


def run_profile(name, params, frames, cache_dir, site=SITE):
    import rclpy
    from geometry_msgs.msg import Quaternion
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, NavSatFix, NavSatStatus
    from std_msgs.msg import Header
    from wildnav_pkg.wildnav_node import WildNavNode, horizontal_distance_m

    params = dict(params)
    params.setdefault('cache_dir', cache_dir)
    params.setdefault('max_tile_downloads_per_cycle', 0)
    params.setdefault('initial_lat', site[0])
    params.setdefault('initial_lon', site[1])
    # The simulated imagery's covariance (aerostack2_sim.yaml).
    params.setdefault('attitude_sigma_deg', 0.5)
    params.setdefault('match_sigma_floor_m', 1.0)
    args = ['--ros-args']
    for key, value in params.items():
        if isinstance(value, bool):
            value = 'true' if value else 'false'
        args += ['-p', f'wildnav_node:{key}:={value}']
    # opencv_threads sets OpenCV's process-wide pool: restored after.
    threads = cv2.getNumThreads()
    rclpy.init(args=args)
    try:
        node = WildNavNode()
        node.timer.cancel()
        captured = []

        class Sink:
            def __init__(self, keep=False):
                self.keep = keep

            def publish(self, message):
                if self.keep:
                    captured.append(message)
        node.fix_pub = Sink(keep=True)
        for attribute in ('odom_pub', 'confidence_pub', 'match_count_pub',
                          'valid_pub'):
            setattr(node, attribute, Sink())

        width, height = GAZEBO_SIZE
        focal = width / (2.0 * math.tan(math.radians(GAZEBO_FOV_DEG) / 2.0))
        info = CameraInfo(width=width, height=height,
                          distortion_model='plumb_bob')
        info.k = [focal, 0.0, width * 0.5, 0.0, focal, height * 0.5,
                  0.0, 0.0, 1.0]
        info.d = [0.0] * 5

        # Warm-up: tile features at this profile's max_features, outside
        # the timing (a seeded cache already has them).
        zoom = int(node.cache.zoom)
        for frame in frames:
            for tile_x, tile_y in node.cache.candidate_tiles(
                    frame['prior'][0], frame['prior'][1],
                    float(node.get_parameter('max_search_radius_m').value),
                    int(node.get_parameter('max_tiles').value)):
                if node.cache.is_cached(tile_x, tile_y):
                    node.cache.get_features(tile_x, tile_y,
                                            allow_download=False)

        timers = Timers(node)
        results = []
        stamp_s = 1_800_000_000
        for index, frame in enumerate(frames):
            stamp_s += 5
            header = Header(frame_id='oak1_image_highres_optical_frame')
            header.stamp.sec = stamp_s
            yaw = frame['yaw']
            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.pose.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
            for offset_ns in (-50_000_000, 0, 50_000_000):
                sample = Odometry()
                sample.pose = odom.pose
                total = stamp_s * 1_000_000_000 + offset_ns
                sample.header.stamp.sec = total // 1_000_000_000
                sample.header.stamp.nanosec = total % 1_000_000_000
                node._on_odom(sample)
            prior = NavSatFix()
            prior.status.status = NavSatStatus.STATUS_FIX
            prior.latitude, prior.longitude = frame['prior']
            prior.altitude = 781.0
            sigma = float(frame['prior_sigma_m'])
            prior.position_covariance = [sigma ** 2, 0.0, 0.0,
                                         0.0, sigma ** 2, 0.0,
                                         0.0, 0.0, 4.0]
            prior.position_covariance_type = (
                NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN)
            before = dict(timers.totals)
            captured.clear()
            cpu0, wall0 = time.process_time(), time.perf_counter()
            node._run_match(frame['image'], header, odom, prior, info)
            wall, cpu = time.perf_counter() - wall0, time.process_time() - cpu0
            result = {'frame': index, 'latency_s': wall, 'cpu_s': cpu,
                      'stages': {key: value - before.get(key, 0.0)
                                 for key, value in timers.totals.items()}}
            if captured:
                fix = captured[-1]
                result['error_m'] = horizontal_distance_m(
                    fix.latitude, fix.longitude, *frame['truth'])
                result['sigma_m'] = math.sqrt(fix.position_covariance[0])
            results.append(result)
        opencv_threads = cv2.getNumThreads()
        node.destroy_node()
    finally:
        rclpy.shutdown()
        cv2.setNumThreads(threads)

    errors = [item['error_m'] for item in results if 'error_m' in item]
    stages = {}
    for item in results:
        for key, value in item['stages'].items():
            stages.setdefault(key, []).append(value)
    return {
        'profile': name, 'params': params, 'zoom': zoom,
        'opencv_threads': opencv_threads, 'frames': len(results),
        'fixes': len(errors),
        'latency_s': quantiles([item['latency_s'] for item in results]),
        'cpu_s': quantiles([item['cpu_s'] for item in results]),
        'stages_median_s': {key: round(statistics.median(values), 3)
                            for key, values in sorted(stages.items())},
        'error_m': quantiles(errors),
        'beyond_3_sigma': sum(
            1 for item in results
            if 'error_m' in item and item['error_m'] > 3 * item['sigma_m']),
        'per_frame': [{key: (round(value, 3) if isinstance(value, float)
                             else value)
                       for key, value in item.items() if key != 'stages'}
                      for item in results],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--cache-dir', default='/data/wildnav_cache')
    parser.add_argument('--profile', action='append', default=[],
                        help='a PROFILES name or name=param:value,...')
    parser.add_argument('--frames', type=int, default=20)
    parser.add_argument('--area-m', type=float, default=900.0,
                        help='truth positions within this of the site')
    parser.add_argument('--prior-sigma-m', type=float, default=10.0,
                        help="the prior's error (standard deviation)")
    parser.add_argument('--stated-sigma-m', type=float,
                        help="the sigma the prior states (default: its "
                             "error's; less is an overconfident prior)")
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--site', default=f'{SITE[0]},{SITE[1]}',
                        help='lat,lon the stills are spread about')
    parser.add_argument('--output', help='write the full results as JSON')
    args = parser.parse_args(argv)
    site = tuple(float(value) for value in args.site.split(','))

    scratch = tempfile.mkdtemp(prefix='wildnav_benchmark_')
    try:
        cache_dir = os.path.join(scratch, 'cache')
        shutil.copytree(args.cache_dir, cache_dir)
        profiles = [parse_profile(text) for text in (args.profile or ['site'])]
        summaries = []
        frames_by_zoom = {}
        for name, params in profiles:
            zoom = int(params.get('zoom', 18))
            if zoom not in frames_by_zoom:
                frames_by_zoom[zoom] = make_frames(
                    cache_dir, zoom, args.frames, args.area_m,
                    args.prior_sigma_m, args.seed, site)
                for frame in frames_by_zoom[zoom]:
                    frame['prior_sigma_m'] = (
                        args.prior_sigma_m if args.stated_sigma_m is None
                        else args.stated_sigma_m)
            summary = run_profile(name, params, frames_by_zoom[zoom],
                                  cache_dir, site)
            summaries.append(summary)
            brief = {key: summary[key] for key in (
                'profile', 'opencv_threads', 'frames', 'fixes', 'latency_s',
                'cpu_s',
                'stages_median_s', 'error_m', 'beyond_3_sigma')}
            print(json.dumps(brief), flush=True)
        if args.output:
            with open(args.output, 'w') as handle:
                json.dump({'cpu_count': os.cpu_count(), 'site': site,
                           'prior_sigma_m': args.prior_sigma_m,
                           'stated_sigma_m': args.stated_sigma_m,
                           'area_m': args.area_m, 'seed': args.seed,
                           'profiles': summaries}, handle, indent=1)
    finally:
        shutil.rmtree(scratch, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
