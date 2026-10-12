#!/usr/bin/env python3
"""Stand in for the aerodrone detector and tracker, to run the web UI alone.

Publishes the topics of systems/aerodrone/tracking.md that the web UI
(applications/aerostack2/src/as2_platform_blueos/web) reads, from a made-up
nadir view of a few ground objects:

- ``<prefix>/detections`` (vision_msgs/Detection2DArray, ``--detection-rate``)
  with stable track IDs and class names;
- ``<prefix>/image/compressed`` (JPEG, ``--image-rate``), each frame with the
  stamp of one detection message;
- ``<prefix>/camera_info`` (1 Hz);
- ``<prefix>/status`` (JSON, reliable + transient-local, 2 Hz), from a small
  state machine driven by ``<prefix>/command``: ``track`` goes acquiring,
  then tracking, and the simulated drone follows the target in the mode's
  geometry; ``stop`` goes idle; an unknown target or an invalid command
  goes to ``error``.

Every command received, and every map goal on /aerostack/map_goal, is
logged on stdout and, with ``--command-log``, appended to that file as a
JSON line {"rx": wall time, "topic": ..., "data": raw string, "command":
parsed command or null}. ``--navigation`` also publishes /navigation/fix,
/navigation/odometry and /navigation/correction_source about
``--origin-lat``/``--origin-lon``, for the map, and MAVROS' raw GPS fix
(/mavros/global_position/raw/fix), as a healthy GPS without vision
corrections looks (the page's navigation badge: GPS); never use it next to
a real navigation stack. ``--static`` freezes the objects and the drone, for
repeatable browser checks (tools/tracking_ui_test.sh).

``--launcher`` adds the foam dart launcher of tracking.md ("Foam dart
launcher"; MockLauncher): ``launcher_arm``, ``launcher_disarm``, ``reload``
and ``fire`` on ``<prefix>/command``, the status's launcher fields with the
real tracker's gates in its order (launcher_core.GATES: armed, tracking
centered, the target id, not a person, no person in view, hovering, the
height window, on target, at rest, darts, interval) and a fire cycle that
takes the ST3215's time (prime, chamber, trigger), aborted by a gate
failing before the release. With it the drone flies to the tracker's goal
even with ``--static`` (the objects stay still), and ``<prefix>/mock_person``
(std_msgs/String) ``in`` puts a person (#31) 4 m from the car #3, ``out``
takes it away, as a person walking into the camera's view would.

MockWorld and MockTracker are plain Python; ROS, NumPy and OpenCV are
imported only by run(), so tests/test_aerodrone_tracking_ui.py checks them
without ROS.
"""

import argparse
import json
import math
import signal
import sys
import time

# The tracker's limits (tracking.md, "Limits").
MIN_AGL_M, MAX_AGL_M = 15.0, 120.0
MIN_DEPRESSION_DEG, MAX_DEPRESSION_DEG = 60.0, 90.0
HFOV_DEG = 69.0  # the OAK-1's horizontal field of view
METRES_PER_DEGREE = 111320.0

# id, class, east, north (m), length, width (m), path: (kind, a, b, speed)
OBJECTS = (
    ('3', 'car', -12.0, 6.0, 4.5, 1.9, ('circle', 20.0, 0.0, 2.5)),
    ('7', 'person', 6.0, -4.0, 0.9, 0.9, ('line', 6.0, 0.0, 0.7)),
    ('12', 'truck', 14.0, 9.0, 8.0, 2.6, ('line', 0.0, 18.0, 1.5)),
    ('21', 'person', -4.0, -9.0, 0.9, 0.9, ('line', 0.0, 4.0, 0.5)),
)
# The person --launcher's <prefix>/mock_person puts next to the car #3.
EXTRA_PERSON = ('31', 'person', -8.0, 6.0, 0.9, 0.9, ('line', 0.0, 0.0, 0.0))
# The launcher (tracking.md, "Foam dart launcher"; launcher_core.py): the
# fire control's limits, and the fire cycle's steps at the ST3215's speed
# (prime and chamber the 38 mm rack at 41 mm/s, the trigger to 0.52 rad at
# 3.07 rad/s).
LAUNCHER_CAPACITY = 6
FIRE_MIN_AGL_M, FIRE_MAX_AGL_M = 8.0, 20.0
FIRE_MAX_OFFSET_M = 1.0
FIRE_MAX_SPEED_MPS = 0.5
FIRE_PERSON_CLEAR_S = 3.0
FIRE_MIN_INTERVAL_S = 2.0
CYCLE_S = (('priming', 0.92), ('chambering', 0.92), ('firing', 0.37))
PERSON_CLASSES = ('person', 'pedestrian', 'people')
GATES = ('launcher', 'armed', 'ready', 'tracking', 'target', 'not_person', 'no_person',
         'target_seen', 'hover', 'height', 'on_target', 'launcher_rest', 'darts',
         'interval')


def validate_command(command):
    """(ok, message) for a /tracking/command object, per the contract."""
    if not isinstance(command, dict):
        return False, 'command is not a JSON object'
    action = command.get('action')
    if action == 'stop':
        return True, ''
    if action != 'track':
        return False, f'unknown action {action!r}'
    if not isinstance(command.get('target_id'), str) or not command['target_id']:
        return False, 'target_id must be a non-empty string'
    mode = command.get('mode', 'centered')
    if mode == 'centered':
        altitude = command.get('altitude_agl_m')
        if altitude is not None and (not _number(altitude) or
                                     not MIN_AGL_M <= altitude <= MAX_AGL_M):
            return False, f'altitude_agl_m must be {MIN_AGL_M:g}-{MAX_AGL_M:g} m'
        return True, ''
    if mode == 'slant':
        depression = command.get('depression_deg')
        slant = command.get('slant_range_m')
        if not _number(depression) or not MIN_DEPRESSION_DEG <= depression <= MAX_DEPRESSION_DEG:
            return False, (f'depression_deg must be '
                           f'{MIN_DEPRESSION_DEG:g}-{MAX_DEPRESSION_DEG:g} deg')
        if not _number(slant) or slant <= 0.0:
            return False, 'slant_range_m must be a positive number'
        return True, ''
    return False, f'unknown mode {mode!r}'


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and \
        math.isfinite(value)


class MockWorld:
    """Ground objects (ENU metres) and a drone above them looking straight
    down, image top to the north."""

    def __init__(self, width=640, height=480, static=False, start_agl_m=40.0, fly=False):
        self.width = width
        self.height = height
        self.static = static
        self.fly = fly  # static objects, but the drone flies (--launcher)
        self.t = 0.0
        self.drone = [0.0, 0.0, start_agl_m]  # east, north, height above ground
        self.speed_mps = 0.0
        self.extra = []  # objects added at run time (EXTRA_PERSON)

    def focal_px(self):
        return self.width / 2.0 / math.tan(math.radians(HFOV_DEG) / 2.0)

    def objects(self):
        """[{id, class, east, north, length, width, speed_mps}] now."""
        result = []
        for track_id, cls, east, north, length, width, path in OBJECTS + tuple(self.extra):
            kind, a, b, speed = path
            t = 0.0 if self.static else self.t
            if kind == 'circle':
                angle = speed * t / a
                e, n = east + a * math.cos(angle) - a, north + a * math.sin(angle)
            else:
                span = math.hypot(a, b)
                phase = math.sin(speed * t / span) if span else 0.0
                e, n = east + a * phase, north + b * phase
            result.append({'id': track_id, 'class': cls, 'east': e, 'north': n,
                           'length': length, 'width': width,
                           'speed_mps': 0.0 if self.static else speed})
        return result

    def to_pixel(self, east, north):
        """Image pixel of a ground point (pinhole, nadir, image top north)."""
        scale = self.focal_px() / self.drone[2]
        return (self.width / 2.0 + (east - self.drone[0]) * scale,
                self.height / 2.0 - (north - self.drone[1]) * scale)

    def boxes(self):
        """The visible objects' axis-aligned boxes in image pixels."""
        scale = self.focal_px() / self.drone[2]
        result = []
        for item in self.objects():
            u, v = self.to_pixel(item['east'], item['north'])
            size_x, size_y = item['length'] * scale, item['width'] * scale
            if (u + size_x / 2 < 0 or u - size_x / 2 > self.width or
                    v + size_y / 2 < 0 or v - size_y / 2 > self.height):
                continue
            result.append(dict(item, cx=u, cy=v, size_x=size_x, size_y=size_y,
                               score=0.9 if item['class'] != 'person' else 0.75))
        return result

    def step(self, dt, goal=None, max_speed_mps=6.0):
        """Advance dt seconds; the drone moves toward goal (east, north, agl)."""
        if self.static and not self.fly:
            return
        if not self.static:
            self.t += dt
        before = list(self.drone)
        if goal is not None:
            for axis in range(3):
                delta = goal[axis] - self.drone[axis]
                limit = max_speed_mps * dt
                self.drone[axis] += max(-limit, min(limit, delta))
        self.speed_mps = (math.hypot(self.drone[0] - before[0], self.drone[1] - before[1]) / dt
                          if dt > 0 else 0.0)


class MockTracker:
    """The tracker's command handling and status, against a MockWorld."""

    ACQUIRE_S = 1.0

    def __init__(self, world):
        self.world = world
        self.state = 'idle'
        self.command = None
        self.since = 0.0
        self.message = 'waiting for a command'

    def handle(self, text, now):
        try:
            command = json.loads(text)
        except ValueError:
            command = None
        ok, error = validate_command(command)
        if not ok:
            self.state, self.message = 'error', f'rejected command: {error}'
            return command
        if command['action'] == 'stop':
            self.state, self.command, self.message = 'idle', None, 'stopped: hovering'
            return command
        if self._target(command['target_id']) is None:
            self.state, self.command = 'error', None
            self.message = f'target {command["target_id"]} is not in view'
            return command
        self.command = dict(command, mode=command.get('mode', 'centered'))
        self.state, self.since, self.message = 'acquiring', now, 'acquiring the target'
        return command

    def _target(self, target_id):
        return next((item for item in self.world.objects() if item['id'] == target_id), None)

    def goal(self):
        """Where the drone flies for the command (east, north, agl), or None."""
        if self.state not in ('acquiring', 'tracking') or not self.command:
            return None
        target = self._target(self.command['target_id'])
        if target is None:
            return None
        agl = self.world.drone[2]
        if self.command['mode'] == 'centered':
            agl = self.command.get('altitude_agl_m', min(max(agl, MIN_AGL_M), MAX_AGL_M))
            return target['east'], target['north'], agl
        slant = self.command['slant_range_m']
        angle = math.radians(self.command['depression_deg'])
        standoff = slant * math.cos(angle)
        bearing = math.atan2(target['north'] - self.world.drone[1],
                             target['east'] - self.world.drone[0])
        return (target['east'] - standoff * math.cos(bearing),
                target['north'] - standoff * math.sin(bearing), slant * math.sin(angle))

    def update(self, now):
        if self.state == 'acquiring' and now - self.since >= self.ACQUIRE_S:
            self.state, self.message = 'tracking', 'following the target'

    def status(self):
        status = {'state': self.state, 'mode': None, 'target_id': None, 'target': None,
                  'slant_range_m': None, 'depression_deg': None, 'message': self.message}
        if self.command:
            target = self._target(self.command['target_id'])
            status['mode'] = self.command['mode']
            status['target_id'] = self.command['target_id']
            if target:
                drone = self.world.drone
                horizontal = math.hypot(target['east'] - drone[0], target['north'] - drone[1])
                status['target'] = {'east': target['east'], 'north': target['north'],
                                    'up': 0.0, 'speed_mps': target['speed_mps']}
                status['slant_range_m'] = math.hypot(horizontal, drone[2])
                status['depression_deg'] = math.degrees(math.atan2(drone[2], horizontal))
        return status


class MockLauncher:
    """The launcher's commands, gates and fire cycle against a MockWorld
    and MockTracker (module doc): what the page sees of the real fire
    control (launcher_core.FireControl), in its status fields."""

    def __init__(self, world, tracker, fitted=True):
        self.world, self.tracker, self.fitted = world, tracker, fitted
        self.armed = False
        self.darts = 0
        self.shots = 0
        self.cycle = None  # (target id, started at)
        self.last_fire = None
        self.last_end = -math.inf
        self.person_time = None
        self.person_label = ''
        self.fault = ''
        self.message = ''

    def persons(self, now):
        """The people in the camera's view now (and when one was last seen)."""
        seen = [box for box in self.world.boxes() if box['class'] in PERSON_CLASSES]
        if seen:
            self.person_time = now
            self.person_label = ', '.join(f'#{box["id"]} {box["class"]}' for box in seen)
        return seen

    def target(self):
        command = self.tracker.command
        return None if command is None else self.tracker._target(command['target_id'])

    def gates(self, now, target_id, launcher=True):
        """[(gate, reason)] that do not hold, in launcher_core.GATES order."""
        failed = []
        tracker, world = self.tracker, self.world
        if not self.fitted:
            failed.append(('launcher', 'no launcher'))
        if not self.armed:
            failed.append(('armed', 'launcher disarmed'))
            failed.append(('ready', 'launcher not ready (no servo bridge, or it reports '
                                    'not ready)'))
        command = tracker.command or {}
        if tracker.state != 'tracking':
            failed.append(('tracking', f'not tracking (tracker {tracker.state})'))
        elif command.get('mode') != 'centered':
            failed.append(('tracking', f'not in centered mode ({command.get("mode")})'))
        tracked = command.get('target_id')
        wanted = '' if target_id is None else str(target_id)
        if not wanted:
            failed.append(('target', "fire needs the tracked target's target_id"))
        elif wanted != tracked:
            failed.append(('target', f'{wanted} is not the tracked target ({tracked or "none"})'))
        target = self.target()
        if target is None:
            failed.append(('not_person', "the target's class is not known"))
        elif target['class'] in PERSON_CLASSES:
            failed.append(('not_person', f'never at people: the target is a {target["class"]}'))
        if self.persons(now):
            failed.append(('no_person', f'a person is in view ({self.person_label})'))
        elif self.person_time is not None and now - self.person_time <= FIRE_PERSON_CLEAR_S:
            failed.append(('no_person', f'a person was in view {now - self.person_time:.1f} s '
                                        f'ago ({self.person_label})'))
        if target is None or not any(box['id'] == target['id'] for box in world.boxes()):
            failed.append(('target_seen', 'the target is not in the latest frames'))
        if world.speed_mps > FIRE_MAX_SPEED_MPS:
            failed.append(('hover', f'not hovering: {world.speed_mps:.1f} m/s '
                                    f'(at most {FIRE_MAX_SPEED_MPS:g})'))
        agl = world.drone[2]
        if not FIRE_MIN_AGL_M <= agl <= FIRE_MAX_AGL_M:
            failed.append(('height', f'{agl:.1f} m above ground, outside '
                                     f'{FIRE_MIN_AGL_M:g}-{FIRE_MAX_AGL_M:g} m'))
        offset = self.offset()
        if offset is None:
            failed.append(('on_target', "the target's or the bore's ground position is not known"))
        elif offset > FIRE_MAX_OFFSET_M:
            failed.append(('on_target', f'the target is {offset:.1f} m from the bore\'s ground '
                                        f'point (at most {FIRE_MAX_OFFSET_M:g} m)'))
        if launcher:
            if self.fault:
                failed.append(('launcher_rest', f'launcher fault: {self.fault}'))
            elif self.cycle is not None:
                failed.append(('launcher_rest', 'a fire cycle is running'))
            if self.darts <= 0:
                failed.append(('darts', 'no darts'))
            wait = FIRE_MIN_INTERVAL_S - (now - self.last_end)
            if wait > 0.0:
                failed.append(('interval', f'{wait:.1f} s to the next shot'))
        return failed

    def offset(self):
        target = self.target()
        if target is None:
            return None
        return math.hypot(target['east'] - self.world.drone[0],
                          target['north'] - self.world.drone[1])

    def handle(self, command, now):
        action = command.get('action')
        if action == 'launcher_arm':
            self.armed, self.fault = True, ''
            self.message = 'launcher armed'
        elif action == 'launcher_disarm':
            self.abort(now, 'launcher disarmed')
            self.armed = False
            self.message = 'launcher disarmed'
        elif action == 'reload':
            darts = command.get('darts')
            if isinstance(darts, bool) or not isinstance(darts, int) or \
                    not 0 <= darts <= LAUNCHER_CAPACITY:
                self.message = f'reload needs darts: a whole number from 0 to {LAUNCHER_CAPACITY}'
            elif self.armed:
                self.message = 'reload refused: disarm the launcher first'
            else:
                self.darts, self.message = darts, f'reloaded: {darts} darts'
        elif action == 'fire':
            target_id = command.get('target_id')
            failed = self.gates(now, target_id)
            if failed:
                self.last_fire = {'target_id': str(target_id or ''), 'result': 'refused',
                                  'reason': failed[0][1], 'stamp': round(now, 3)}
                self.message = f'fire refused: {failed[0][1]}'
            else:
                self.cycle = (str(target_id), now)
                self.last_fire = {'target_id': str(target_id), 'result': 'running',
                                  'reason': '', 'stamp': round(now, 3)}
                self.message = f'firing at {target_id}'

    def abort(self, now, reason):
        if self.cycle is None:
            return
        target_id, _ = self.cycle
        self.cycle, self.fault, self.last_end = None, f'aborted ({reason})', now
        self.last_fire = {'target_id': target_id, 'result': 'fault',
                          'reason': self.fault, 'stamp': round(now, 3)}
        self.message = f'launcher fault: {self.fault}'

    def phase(self, now):
        if self.cycle is None:
            return None
        elapsed = now - self.cycle[1]
        for name, seconds in CYCLE_S:
            if elapsed < seconds:
                return name
            elapsed -= seconds
        return 'done'

    def update(self, now):
        """Advance the cycle: a gate that must hold until the release
        aborts it; at the end, the shot."""
        if self.cycle is None:
            return
        failed = self.gates(now, self.cycle[0], launcher=False)
        if failed:
            self.abort(now, failed[0][1])
        elif self.phase(now) == 'done':
            target_id, _ = self.cycle
            self.cycle, self.last_end = None, now
            self.darts, self.shots = max(0, self.darts - 1), self.shots + 1
            self.last_fire = {'target_id': target_id, 'result': 'fired', 'reason': '',
                              'stamp': round(now, 3)}
            self.message = f'fired at {target_id}; {self.darts} darts left'

    def status(self, now):
        """The launcher's fields of /tracking/status."""
        if not self.fitted:
            state = 'none'
        elif self.fault:
            state = 'fault'
        elif self.cycle is not None:
            state = {'done': 'firing'}.get(self.phase(now), self.phase(now))
        else:
            state = 'ready' if self.darts > 0 else 'empty'
        target = self.target()
        failed = self.gates(now, self.tracker.command and self.tracker.command['target_id'])
        names = {name for name, _ in failed}
        return {'launcher': state, 'launcher_armed': self.armed,
                'launcher_ready': self.fitted and self.armed,
                'darts': self.darts, 'shots': self.shots,
                'target_class': target['class'] if target else None,
                'fire_block': failed[0][1] if failed else '',
                'fire_checks': {name: name not in names for name in GATES},
                'last_fire': self.last_fire, 'agl_m': round(self.world.drone[2], 2),
                'bore_offset_m': None if self.offset() is None else round(self.offset(), 2),
                'launcher_message': self.message}


LAUNCHER_ACTIONS = ('fire', 'launcher_arm', 'launcher_disarm', 'reload')


def render(world, boxes, stamp_text, np, cv2):
    """A JPEG-ready BGR frame: textured ground, the objects, the stamp."""
    scale = world.focal_px() / world.drone[2]
    ys, xs = np.mgrid[0:world.height, 0:world.width].astype(np.float32)
    east = world.drone[0] + (xs - world.width / 2) / scale
    north = world.drone[1] - (ys - world.height / 2) / scale
    ground = (70 + 25 * np.sin(east * 0.35) * np.cos(north * 0.27) +
              20 * np.sin(east * 0.05 + north * 0.08))
    image = np.stack([ground * 0.55, ground * 1.05, ground * 0.75], axis=-1)
    # A 10 m grid of tracks, at least a pixel wide.
    half = max(0.1, 0.6 / scale)
    grid = (np.abs((east % 10.0) - 5.0) < half) | (np.abs((north % 10.0) - 5.0) < half)
    image[grid] = (95, 110, 110)
    image = np.clip(image, 0, 255).astype(np.uint8)
    colours = {'car': (40, 40, 200), 'truck': (30, 140, 230), 'person': (200, 120, 40)}
    for box in boxes:
        x0, y0 = int(box['cx'] - box['size_x'] / 2), int(box['cy'] - box['size_y'] / 2)
        x1, y1 = int(box['cx'] + box['size_x'] / 2), int(box['cy'] + box['size_y'] / 2)
        cv2.rectangle(image, (x0, y0), (x1, y1), colours.get(box['class'], (200, 200, 200)), -1)
    cv2.putText(image, f'mock {stamp_text}', (8, world.height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1, cv2.LINE_AA)
    return image


def run(args):
    import cv2
    import numpy as np
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, CompressedImage, NavSatFix
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

    rclpy.init()
    node = Node('mock_tracking_publisher')
    world = MockWorld(args.width, args.height, static=args.static, fly=args.launcher)
    tracker = MockTracker(world)
    launcher = MockLauncher(world, tracker, fitted=args.launcher)
    prefix = args.prefix.rstrip('/')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    detections_pub = node.create_publisher(Detection2DArray, f'{prefix}/detections', 5)
    image_pub = node.create_publisher(CompressedImage, f'{prefix}/image/compressed', 2)
    info_pub = node.create_publisher(CameraInfo, f'{prefix}/camera_info', 2)
    status_pub = node.create_publisher(String, f'{prefix}/status', latched)
    command_log = open(args.command_log, 'a', encoding='utf-8') if args.command_log else None  # noqa: SIM115

    def log(topic, data, command=None):
        record = {'rx': time.time(), 'topic': topic, 'data': data, 'command': command}
        print(f'received: {json.dumps(record)}', flush=True)
        if command_log:
            command_log.write(json.dumps(record) + '\n')
            command_log.flush()

    def on_command(message):
        now = time.monotonic()
        try:
            command = json.loads(message.data)
        except ValueError:
            command = None
        if isinstance(command, dict) and command.get('action') in LAUNCHER_ACTIONS:
            launcher.handle(command, now)
            log(f'{prefix}/command', message.data, command)
        else:
            if isinstance(command, dict) and command.get('action') in ('track', 'stop'):
                launcher.abort(now, f'{command["action"]} command')
            log(f'{prefix}/command', message.data, tracker.handle(message.data, now))
        publish_status()

    def on_mock_person(message):
        world.extra = [EXTRA_PERSON] if message.data.strip() == 'in' else []
        log(f'{prefix}/mock_person', message.data)

    node.create_subscription(String, f'{prefix}/command', on_command, 10)
    node.create_subscription(String, '/aerostack/map_goal',
                             lambda message: log('/aerostack/map_goal', message.data), 10)
    if args.launcher:
        node.create_subscription(String, f'{prefix}/mock_person', on_mock_person, 10)

    def publish_status():
        status = tracker.status()
        status.update(launcher.status(time.monotonic()))
        status_pub.publish(String(data=json.dumps(status)))

    frame = {'count': 0, 'last': time.monotonic()}
    image_every = max(1, round(args.detection_rate / args.image_rate))

    def on_detections():
        now = time.monotonic()
        tracker.update(now)
        world.step(now - frame['last'], tracker.goal())
        launcher.update(now)
        frame['last'] = now
        stamp = node.get_clock().now().to_msg()
        boxes = world.boxes()
        message = Detection2DArray()
        message.header.stamp = stamp
        message.header.frame_id = args.frame_id
        for box in boxes:
            detection = Detection2D()
            detection.header = message.header
            detection.id = box['id']
            detection.bbox.center.position.x = float(box['cx'])
            detection.bbox.center.position.y = float(box['cy'])
            detection.bbox.size_x = float(box['size_x'])
            detection.bbox.size_y = float(box['size_y'])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = box['class']
            hypothesis.hypothesis.score = float(box['score'])
            detection.results.append(hypothesis)
            message.detections.append(detection)
        detections_pub.publish(message)
        if frame['count'] % image_every == 0:
            image = render(world, boxes, f'{stamp.sec}.{stamp.nanosec:09d}', np, cv2)
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                compressed = CompressedImage()
                compressed.header = message.header
                compressed.format = 'jpeg'
                compressed.data = jpeg.tobytes()
                image_pub.publish(compressed)
        frame['count'] += 1

    def on_camera_info():
        info = CameraInfo()
        info.header.stamp = node.get_clock().now().to_msg()
        info.header.frame_id = args.frame_id
        info.width, info.height = args.width, args.height
        focal = world.focal_px()
        info.k = [focal, 0.0, args.width / 2.0, 0.0, focal, args.height / 2.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, args.width / 2.0, 0.0, 0.0, focal, args.height / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info_pub.publish(info)

    node.create_timer(1.0 / args.detection_rate, on_detections)
    node.create_timer(1.0, on_camera_info)
    node.create_timer(0.5, publish_status)

    if args.navigation:
        fix_pub = node.create_publisher(NavSatFix, '/navigation/fix', 5)
        odom_pub = node.create_publisher(Odometry, '/navigation/odometry', 5)
        source_pub = node.create_publisher(String, '/navigation/correction_source', 5)
        gps_pub = node.create_publisher(NavSatFix, '/mavros/global_position/raw/fix', 5)

        def on_navigation():
            east, north, agl = world.drone
            stamp = node.get_clock().now().to_msg()
            fix = NavSatFix()
            fix.header.stamp = stamp
            fix.header.frame_id = 'base_link'
            fix.status.status = 0
            fix.latitude = args.origin_lat + north / METRES_PER_DEGREE
            fix.longitude = args.origin_lon + east / (
                METRES_PER_DEGREE * math.cos(math.radians(args.origin_lat)))
            fix.altitude = agl
            fix_pub.publish(fix)
            gps_pub.publish(fix)
            # The fusion's value whenever no DemNav/WildNav correction is
            # active, GPS or not.
            source_pub.publish(String(data='dead_reckoning'))
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = east
            odom.pose.pose.position.y = north
            odom.pose.pose.position.z = agl
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)

        node.create_timer(0.2, on_navigation)

    publish_status()
    print(f'mock tracking publisher on {prefix}: {args.width}x{args.height}, '
          f'{args.detection_rate:g} Hz detections, {args.image_rate:g} Hz images'
          f'{", static" if args.static else ""}', flush=True)
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if command_log:
            command_log.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--prefix', default='/tracking')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--detection-rate', type=float, default=10.0)
    parser.add_argument('--image-rate', type=float, default=3.0)
    parser.add_argument('--frame-id', default='camera_optical_frame')
    parser.add_argument('--static', action='store_true',
                        help='freeze the objects and the drone')
    parser.add_argument('--navigation', action='store_true',
                        help='also publish /navigation/fix, odometry and correction_source')
    parser.add_argument('--origin-lat', type=float, default=37.9234)
    parser.add_argument('--origin-lon', type=float, default=-122.5967)
    parser.add_argument('--command-log', help='append every command here as JSON lines')
    parser.add_argument('--launcher', action='store_true',
                        help='a foam dart launcher; the drone flies even with --static')
    args = parser.parse_args(argv)
    if args.detection_rate <= 0 or args.image_rate <= 0:
        parser.error('rates must be > 0')
    return args


if __name__ == '__main__':
    sys.exit(run(parse_args()))
