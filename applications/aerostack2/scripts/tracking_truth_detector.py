#!/usr/bin/env python3
"""Ground-truth detector for the Aerodrone tracking tests: stands in for the
YOLO detector of systems/aerodrone/tracking.md in the simulators, so the
tracker's control can be tested apart from detection errors.

For every simulated camera frame (its ``camera_info``, which the simulators
publish with each image) it projects the simulator's ground-truth targets
(``/simulation/targets``, ENU about the navigation origin) through the
camera in the pose of /navigation/odometry at the frame's stamp, with the
camera mount the tracker uses, and publishes:

- ``<prefix>/detections``: one Detection2D per target whose centre is in
  the image, ``id`` the target's name, the box of its projected footprint
  (class-sized, oriented by its yaw) clipped to the image, stamped with the
  frame;
- ``<prefix>/camera_info``: the frame's intrinsics, scaled to at most
  ``max_width`` pixels wide (the detector's image size).

No image is published. Because the pose is the navigation estimate, the
boxes are exact in the navigation frame: what remains between the
tracker's target estimate and the truth is its own geolocation and
filtering, and between the aircraft and the truth its control plus the
navigation error.

``pixel_noise_px`` and ``dropout`` degrade the boxes on purpose; names
published on ``hide_topic`` (comma-separated, empty for none), and targets
the simulator marks ``"hidden": true`` (hidden under the terrain through
/simulation/targets/hide), are not detected, to test a lost target.

``synthetic_targets`` (until the simulators publish /simulation/targets):
targets move along ``synthetic_spec`` routes on the DEM's ground in
simulation time and are published, in the /simulation/targets format, on
``synthetic_targets_topic``. Spec entries, ';'-separated:
``name,class,shape,east,north,a,b,speed_mps`` with shape ``circle``
(radius a about east/north), ``stadium`` (straights of length a along
east, end radius b), ``line`` (to and fro to east+a, north+b) or ``static``.
"""
import json
import math
import os
import random
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tracking_core as tc  # noqa: E402

# Footprint (length, width) in metres seen from above, by class.
CLASS_SIZES = {'car': (4.5, 1.9), 'truck': (8.0, 2.5), 'bus': (12.0, 2.6),
               'person': (0.6, 0.6), 'bicycle': (1.8, 0.6),
               'motorcycle': (2.0, 0.8), 'boat': (6.0, 2.2)}
DEFAULT_SIZE = (2.0, 2.0)
# A car lapping a 60 m x 30 m stadium (15 m ends: 1.1 m/s^2 in the turns)
# and a person walking to and fro through its middle, 40 m east of home.
DEFAULT_SPEC = ('car_1,car,stadium,40,0,60,15,4.0;'
                'person_1,person,line,30,-5,20,10,1.4')
PARAMETERS = {
    'topic_prefix': '/tracking',
    'targets_topic': '/simulation/targets',
    'camera_info_topic': '/aerodrone/sensor_measurements/downward_rgb/camera_info',
    'odometry_topic': '/navigation/odometry',
    'origin_topic': '/navigation/origin',
    'clock_topic': '/clock',
    'hide_topic': '/tracking/truth/hide',
    'camera_mount_yaw_deg': 0.0,
    'camera_stabilized': False,
    'max_width': 640,
    'fallback_rate_hz': 5.0,
    'fallback_fov_deg': 69.0,
    'pixel_noise_px': 0.0,
    'dropout': 0.0,
    'seed': 1,
    'synthetic_targets': False,
    'synthetic_targets_topic': '/tracking/truth/targets',
    'synthetic_spec': DEFAULT_SPEC,
    'dem_cache_dir': '/data/demnav_cache',
    'geoid_file': '/usr/share/GeographicLib/geoids/egm96-5.pgm',
}


def parse_spec(text):
    """[(name, class, shape, east, north, a, b, speed)] of a route spec."""
    routes = []
    for entry in str(text).split(';'):
        fields = [item.strip() for item in entry.split(',')]
        if len(fields) != 8 or not fields[0]:
            continue
        name, cls, shape = fields[:3]
        if shape not in ('circle', 'stadium', 'line', 'static'):
            raise ValueError(f'unknown route shape {shape!r}')
        routes.append((name, cls, shape) + tuple(float(v) for v in fields[3:]))
    return routes


def route_position(route, t):
    """(east, north, heading) of a route at t seconds."""
    _, _, shape, east, north, a, b, speed = route
    distance = speed * t
    if shape == 'circle' and a > 0.0:
        angle = distance / a
        return (east + a * math.cos(angle), north + a * math.sin(angle),
                angle + math.pi / 2.0)
    if shape == 'stadium' and a + b > 0.0:
        lap = 2.0 * a + 2.0 * math.pi * b
        s = distance % lap
        if s < a:  # bottom straight, eastward
            return east - a / 2.0 + s, north - b, 0.0
        s -= a
        if s < math.pi * b:  # east end
            angle = -math.pi / 2.0 + s / b
            return (east + a / 2.0 + b * math.cos(angle),
                    north + b * math.sin(angle), angle + math.pi / 2.0)
        s -= math.pi * b
        if s < a:  # top straight, westward
            return east + a / 2.0 - s, north + b, math.pi
        s -= a
        angle = math.pi / 2.0 + s / b
        return (east - a / 2.0 + b * math.cos(angle),
                north + b * math.sin(angle), angle + math.pi / 2.0)
    if shape == 'line':
        length = math.hypot(a, b)
        if length > 0.0:
            phase = (distance / length) % 2.0
            fraction = phase if phase <= 1.0 else 2.0 - phase
            heading = math.atan2(b, a) + (0.0 if phase <= 1.0 else math.pi)
            return east + fraction * a, north + fraction * b, heading
    return east, north, 0.0


def footprint_box(target, camera_position, rotation, intrinsics):
    """(cx, cy, size_x, size_y) of a target's footprint box clipped to the
    image, or None when its centre is outside it or behind the camera."""
    centre = tc.project_point((target['east'], target['north'], target['up']),
                              camera_position, rotation, intrinsics)
    if centre is None or not (0.0 <= centre[0] < intrinsics.width and
                              0.0 <= centre[1] < intrinsics.height):
        return None
    length, width = CLASS_SIZES.get(target.get('class'), DEFAULT_SIZE)
    yaw = float(target.get('yaw', 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    us, vs = [], []
    for dl, dw in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        e = target['east'] + 0.5 * (dl * length * c - dw * width * s)
        n = target['north'] + 0.5 * (dl * length * s + dw * width * c)
        corner = tc.project_point((e, n, target['up']), camera_position,
                                  rotation, intrinsics)
        if corner is None:
            return None
        us.append(corner[0])
        vs.append(corner[1])
    u0, u1 = max(0.0, min(us)), min(float(intrinsics.width), max(us))
    v0, v1 = max(0.0, min(vs)), min(float(intrinsics.height), max(vs))
    return 0.5 * (u0 + u1), 0.5 * (v0 + v1), u1 - u0, v1 - v0


def scaled_intrinsics(intrinsics, max_width):
    if intrinsics.width <= max_width:
        return intrinsics, 1.0
    scale = max_width / float(intrinsics.width)
    return tc.Intrinsics(intrinsics.fx * scale, intrinsics.fy * scale,
                         (intrinsics.cx + 0.5) * scale - 0.5,
                         (intrinsics.cy + 0.5) * scale - 0.5,
                         int(round(intrinsics.width * scale)),
                         int(round(intrinsics.height * scale))), scale


def interpolate_targets(history, t):
    """Targets at time t from [(stamp, {name: target})] (linear between
    the bracketing messages, the nearest within 0.5 s otherwise)."""
    if not history:
        return {}
    stamps = [item[0] for item in history]
    if t <= stamps[0]:
        return history[0][1] if stamps[0] - t <= 0.5 else {}
    if t >= stamps[-1]:
        return history[-1][1] if t - stamps[-1] <= 0.5 else {}
    index = next(i for i, stamp in enumerate(stamps) if stamp >= t)
    (t0, a), (t1, b) = history[index - 1], history[index]
    fraction = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
    result = {}
    for name, target in b.items():
        if name not in a:
            continue
        item = dict(target)
        for key in ('east', 'north', 'up'):
            item[key] = a[name][key] + fraction * (target[key] - a[name][key])
        result[name] = item
    return result


class TruthDetector:
    def __init__(self, node):
        from geographic_msgs.msg import GeoPointStamped
        from nav_msgs.msg import Odometry
        from rcl_interfaces.msg import ParameterDescriptor
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo
        from std_msgs.msg import String
        from vision_msgs.msg import (
            Detection2D,
            Detection2DArray,
            ObjectHypothesisWithPose,
        )

        self.node = node
        self.CameraInfo = CameraInfo
        self.Detection2D = Detection2D
        self.Detection2DArray = Detection2DArray
        self.Hypothesis = ObjectHypothesisWithPose
        self.String = String
        dynamic = ParameterDescriptor(dynamic_typing=True)
        for name, default in PARAMETERS.items():
            node.declare_parameter(name, default, dynamic)
        param = lambda name: node.get_parameter(name).value  # noqa: E731
        prefix = '/' + str(param('topic_prefix')).strip('/')
        self.mount_yaw = math.radians(float(param('camera_mount_yaw_deg')))
        self.stabilized = str(param('camera_stabilized')).lower() in ('true', '1', 'yes')
        self.max_width = int(param('max_width'))
        self.pixel_noise = float(param('pixel_noise_px'))
        self.dropout = float(param('dropout'))
        self.random = random.Random(int(param('seed')))
        self.synthetic = str(param('synthetic_targets')).lower() in ('true', '1', 'yes')
        self.routes = parse_spec(param('synthetic_spec')) if self.synthetic else []
        self.dem_cache_dir = str(param('dem_cache_dir'))
        self.geoid_file = str(param('geoid_file'))
        self.fallback_fov = float(param('fallback_fov_deg'))
        self.lock = threading.Lock()
        self.poses = tc.PoseHistory()
        self.history = []          # [(sim stamp s, {name: target})]
        self.hidden = set()
        self.origin = None
        self.dem = None
        self.dem_started = False
        self.dem_deadline = None
        self.sim_start = None
        self.clock = []            # [(sim s, wall s)]
        self.last_frame = 0.0
        self.frames = 0

        sensor = qos_profile_sensor_data
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.detections_pub = node.create_publisher(
            Detection2DArray, f'{prefix}/detections', 10)
        self.info_pub = node.create_publisher(CameraInfo, f'{prefix}/camera_info', 10)
        node.create_subscription(CameraInfo, str(param('camera_info_topic')),
                                 self._on_camera_info, sensor)
        node.create_subscription(Odometry, str(param('odometry_topic')),
                                 self._on_odometry, sensor)
        node.create_subscription(GeoPointStamped, str(param('origin_topic')),
                                 self._on_origin, latched)
        node.create_subscription(Clock, str(param('clock_topic')), self._on_clock, sensor)
        node.create_subscription(String, str(param('hide_topic')), self._on_hide, 10)
        if self.synthetic:
            self.targets_pub = node.create_publisher(
                String, str(param('synthetic_targets_topic')), 10)
            node.create_timer(0.1, self._publish_synthetic)
        else:
            node.create_subscription(String, str(param('targets_topic')),
                                     self._on_targets, 10)
        node.create_timer(1.0 / float(param('fallback_rate_hz')), self._fallback_frame)
        node.get_logger().info(
            f'truth detector: {"synthetic targets" if self.synthetic else param("targets_topic")}'
            f' -> {prefix}/detections, camera mount {math.degrees(self.mount_yaw):g} deg')

    # --- time -------------------------------------------------------------
    def _on_clock(self, msg):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        with self.lock:
            self.poses.add_clock(int(sim * 1e9), time.time_ns())
            if not self.clock or sim - self.clock[-1][0] >= 0.05:
                self.clock.append((sim, time.time()))
                del self.clock[:-400]

    def sim_now(self):
        """Simulation seconds now (wall seconds without a /clock)."""
        if self.clock and time.time() - self.clock[-1][1] < 2.0:
            sim, wall = self.clock[-1]
            return sim + (time.time() - wall)
        return time.time()

    def sim_time_of(self, stamp_s):
        """A frame stamp on the targets' simulation clock."""
        if not self.clock or abs(stamp_s - self.clock[-1][0]) < 10.0:
            return stamp_s
        # A wall-clock frame stamp in a simulator: map it through /clock.
        best = min(self.clock, key=lambda item: abs(item[1] - stamp_s))
        return best[0] + (stamp_s - best[1])

    # --- inputs -------------------------------------------------------------
    def _on_odometry(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self.lock:
            self.poses.add(stamp, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _on_origin(self, msg):
        self.origin = (msg.position.latitude, msg.position.longitude,
                       msg.position.altitude)
        if self.synthetic and not self.dem_started:
            self.dem_started = True
            self.dem_deadline = time.time() + 30.0
            threading.Thread(target=self._load_dem, daemon=True).start()

    def _load_dem(self):
        entry = tc.best_dem(self.dem_cache_dir, *self.origin[:2])
        if entry is None:
            self.node.get_logger().warn('no cached DEM: synthetic targets on flat ground')
            return
        geoid = None
        try:
            from demnav_pkg.geoid import GeoidGrid
            geoid = GeoidGrid(self.geoid_file)
        except (ImportError, OSError, ValueError):
            pass
        self.dem = tc.DemTerrain(tc.load_heightmap(entry), self.origin, geoid)
        self.node.get_logger().info(f'synthetic targets on DEM {entry[0]}')

    def _on_hide(self, msg):
        self.hidden = {name.strip() for name in msg.data.split(',') if name.strip()}
        self.node.get_logger().info(f'hidden targets: {sorted(self.hidden) or "none"}')

    def _on_targets(self, msg):
        try:
            data = json.loads(msg.data)
            stamp = float(data['stamp'])
            targets = {str(item['name']): {
                'name': str(item['name']), 'class': str(item.get('class', '')),
                'east': float(item['east']), 'north': float(item['north']),
                'up': float(item['up']), 'yaw': float(item.get('yaw', 0.0)),
                'hidden': bool(item.get('hidden', False))}
                for item in data['targets']}
        except (ValueError, KeyError, TypeError) as error:
            self.node.get_logger().warn(f'bad targets message: {error}',
                                        throttle_duration_sec=10.0)
            return
        with self.lock:
            if self.history and stamp < self.history[-1][0]:
                self.history.clear()
            self.history.append((stamp, targets))
            while self.history and stamp - self.history[0][0] > 5.0:
                self.history.pop(0)

    def synthetic_targets(self, sim_s):
        if self.sim_start is None:
            self.sim_start = sim_s
        t = sim_s - self.sim_start
        targets = {}
        for route in self.routes:
            east, north, yaw = route_position(route, t)
            up = self.dem.height(east, north) if self.dem is not None else None
            targets[route[0]] = {'name': route[0], 'class': route[1],
                                 'east': east, 'north': north,
                                 'up': 0.0 if up is None else up, 'yaw': yaw}
        return targets

    def synthetic_ready(self):
        """The routes start once their ground is known: the DEM, or flat
        ground 30 s after the origin arrived without one."""
        return self.origin is not None and (
            self.dem is not None or self.sim_start is not None or
            time.time() > self.dem_deadline)

    def _publish_synthetic(self):
        if not self.synthetic_ready():
            return
        sim = self.sim_now()
        targets = self.synthetic_targets(sim)
        self.targets_pub.publish(self.String(data=json.dumps({
            'stamp': round(sim, 4),
            'targets': [{key: (round(value, 3) if isinstance(value, float) else value)
                         for key, value in item.items()}
                        for item in targets.values()]})))

    def targets_at(self, sim_s):
        if self.synthetic:
            if not self.synthetic_ready():
                return {}
            return self.synthetic_targets(sim_s)
        with self.lock:
            return interpolate_targets(list(self.history), sim_s)

    # --- frames -------------------------------------------------------------
    def _on_camera_info(self, msg):
        intrinsics = tc.intrinsics_from_camera_info(msg)
        if intrinsics is None:
            return
        self.last_frame = time.time()
        self._frame(msg.header, intrinsics)

    def _fallback_frame(self):
        """Frames without a simulated camera: the latest odometry stamp and
        a centred 640x480 pinhole of fallback_fov_deg."""
        if time.time() - self.last_frame < 2.0:
            return
        pose = self.poses.latest()
        if pose is None:
            return
        from std_msgs.msg import Header
        header = Header()
        header.stamp.sec, header.stamp.nanosec = divmod(pose.stamp_ns, 1_000_000_000)
        header.frame_id = 'camera_optical_frame'
        focal = 320.0 / math.tan(math.radians(self.fallback_fov) / 2.0)
        self._frame(header, tc.Intrinsics(focal, focal, 319.5, 239.5, 640, 480))

    def _frame(self, header, intrinsics):
        stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        with self.lock:
            pose = self.poses.at(stamp_ns)
        intrinsics, _ = scaled_intrinsics(intrinsics, self.max_width)
        info = self.CameraInfo()
        info.header = header
        info.width, info.height = intrinsics.width, intrinsics.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [intrinsics.fx, 0.0, intrinsics.cx, 0.0, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)
        message = self.Detection2DArray()
        message.header = header
        if pose is None:
            self.detections_pub.publish(message)
            return
        rotation = tc.camera_world_rotation(pose.quaternion, self.mount_yaw,
                                            self.stabilized)
        targets = self.targets_at(self.sim_time_of(stamp_ns * 1e-9))
        for name, target in sorted(targets.items()):
            if name in self.hidden or target.get('hidden') or (
                    self.dropout > 0.0 and self.random.random() < self.dropout):
                continue
            box = footprint_box(target, pose.position, rotation, intrinsics)
            if box is None:
                continue
            cx, cy, size_x, size_y = box
            if self.pixel_noise > 0.0:
                cx += self.random.gauss(0.0, self.pixel_noise)
                cy += self.random.gauss(0.0, self.pixel_noise)
            detection = self.Detection2D()
            detection.header = header
            detection.id = name
            detection.bbox.center.position.x = float(cx)
            detection.bbox.center.position.y = float(cy)
            detection.bbox.size_x = float(max(size_x, 1.0))
            detection.bbox.size_y = float(max(size_y, 1.0))
            hypothesis = self.Hypothesis()
            hypothesis.hypothesis.class_id = target.get('class', '')
            hypothesis.hypothesis.score = 0.99
            detection.results.append(hypothesis)
            message.detections.append(detection)
        self.detections_pub.publish(message)
        self.frames += 1
        if self.frames % 100 == 1:
            self.node.get_logger().info(
                f'frame {self.frames}: {len(message.detections)} of {len(targets)} '
                'targets in view')


def main():
    import rclpy
    rclpy.init()
    node = rclpy.create_node('tracking_truth_detector')
    TruthDetector(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

