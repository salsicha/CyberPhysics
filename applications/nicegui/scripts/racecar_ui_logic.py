"""Decision logic of the RACECAR Neo / RACECAR Turret / PanTiltROS turret
web UI (main.py).

Plain Python (and NumPy for the map image), no ROS and no NiceGUI, so it
is unit tested on its own (tests/test_racecar_turret_ui.py). Conventions,
from systems/racecar_turret/interfaces.md and REP-103:

- on-screen joysticks report x to the right and y up, in [-1, 1];
- image coordinates are pixels from the top-left corner, u and v the same
  divided by the image size;
- map coordinates are metres in the map frame; the occupancy grid's first
  cell is at info.origin and its rows grow with y.
"""
import html
import json
import math
from dataclasses import dataclass

import numpy as np


def yaw_from_quaternion(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def finite(*values) -> bool:
    return all(isinstance(value, (int, float)) and math.isfinite(value) for value in values)


# --- Joysticks --------------------------------------------------------------

@dataclass(frozen=True)
class DriveLimits:
    """Full-stick manual driving; cmd_vel_to_ackermann clamps again."""
    max_speed: float = 0.3      # m/s
    max_steering: float = 0.42  # rad, front wheels
    wheelbase: float = 0.33     # m
    deadzone: float = 0.1       # fraction of full stick, per axis


def stick(x: float, y: float, deadzone: float) -> tuple[float, float]:
    """Clamp a joystick reading to the unit disc and apply a per-axis dead
    zone, so pushing straight ahead does not also steer a little."""
    if not finite(x, y):
        return 0.0, 0.0
    norm = math.hypot(x, y)
    if norm > 1.0:
        x, y = x / norm, y / norm

    def axis(value):
        magnitude = (abs(value) - deadzone) / (1.0 - deadzone)
        return math.copysign(magnitude, value) if magnitude > 0.0 else 0.0
    return axis(x), axis(y)


def drive_twist(x: float, y: float, limits: DriveLimits) -> tuple[float, float]:
    """Joystick to (linear.x, angular.z) for /cmd_vel_teleop.

    y is the speed; x sets the front-wheel angle (right is negative,
    REP-103) as a fraction of the lock, at any speed and in reverse:
    cmd_vel_to_ackermann recovers it as atan(angular * wheelbase / linear).
    Standing still the car cannot turn, so the yaw rate is zero there.
    """
    sx, sy = stick(x, y, limits.deadzone)
    speed = sy * limits.max_speed
    steering = -sx * limits.max_steering
    return speed, speed * math.tan(steering) / limits.wheelbase


def turret_rates(x: float, y: float, max_rate: float, deadzone: float = 0.1) -> tuple[float, float]:
    """Joystick to (pan_rate, tilt_rate) in rad/s: pan positive turns the
    camera right, tilt positive turns it up (interfaces.md)."""
    sx, sy = stick(x, y, deadzone)
    return sx * max_rate, sy * max_rate


def manual_command(pan_rate: float, tilt_rate: float) -> dict:
    return {'action': 'manual', 'pan_rate': round(pan_rate, 4), 'tilt_rate': round(tilt_rate, 4)}


class Hold:
    """One on-screen joystick held in one browser.

    The browser reports the stick at 10 Hz while it is held and once more
    with held=False on release (joystick.js). tick() gives the vector to
    send at each 10 Hz period: the stick while reports keep arriving
    (deadman_s), then a single (0, 0) when the stick is released or the
    reports stop (the tab went to the background or lost its connection),
    then nothing. Reports carry the browser's send time; one that arrives
    more than deadman_s later than the quickest so far was held up on the
    way (a reconnecting socket flushes its buffer) and does not count.
    """

    def __init__(self, deadman_s: float = 0.35):
        self.deadman_s = deadman_s
        self.vector = (0.0, 0.0)
        self.last_report = None  # time of the last held report; None when released
        self.zero_pending = False
        self.min_delay = None  # smallest receive time minus send time seen

    def report(self, x: float, y: float, held: bool, now: float, sent=None) -> bool:
        """Record a report; True when it starts a hold."""
        if not held:
            if self.last_report is not None:
                self.zero_pending = True
            self.last_report = None
            self.vector = (0.0, 0.0)
            return False
        if finite(sent):
            delay = now - sent
            if self.min_delay is None or delay < self.min_delay:
                self.min_delay = delay
            if delay - self.min_delay > self.deadman_s:
                return False
        started = self.last_report is None or now - self.last_report > self.deadman_s
        self.vector = (float(x), float(y)) if finite(x, y) else (0.0, 0.0)
        self.last_report = now
        self.zero_pending = False
        return started

    def held(self, now: float) -> bool:
        return self.last_report is not None and now - self.last_report <= self.deadman_s

    def tick(self, now: float):
        if self.last_report is not None:
            if now - self.last_report <= self.deadman_s:
                return self.vector
            self.last_report = None
            self.zero_pending = True
        if self.zero_pending:
            self.zero_pending = False
            return (0.0, 0.0)
        return None


# --- Map --------------------------------------------------------------------

@dataclass(frozen=True)
class MapView:
    """Where the rendered occupancy-grid image lies in the map frame.

    The image is the grid flipped so that +y is up, possibly reduced by an
    integer factor (resolution is metres per image pixel). Its bottom-left
    corner is the grid origin.
    """
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int

    def to_map(self, px: float, py: float) -> tuple[float, float]:
        gx = px * self.resolution
        gy = (self.height - py) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return self.origin_x + c * gx - s * gy, self.origin_y + s * gx + c * gy

    def to_pixel(self, x: float, y: float) -> tuple[float, float]:
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        gx, gy = c * dx + s * dy, -s * dx + c * dy
        return gx / self.resolution, self.height - gy / self.resolution


def occupancy_image(data, width: int, height: int, resolution: float,
                    origin: tuple[float, float, float], max_size: int = 1024):
    """Render an OccupancyGrid as a grey image (unknown 205, free white,
    occupied black) and return (image, MapView).

    Grids larger than max_size are reduced by keeping the darkest cell of
    each block, so one-cell walls survive.
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    image = np.full(grid.shape, 205, dtype=np.uint8)
    known = grid >= 0
    image[known] = (255 - np.clip(grid[known], 0, 100) * 255 // 100).astype(np.uint8)
    step = max(1, math.ceil(max(width, height) / max_size))
    if step > 1:
        # Pad at the far rows and columns (+x, +y) so the origin corner stays put.
        rows, cols = math.ceil(height / step) * step, math.ceil(width / step) * step
        padded = np.full((rows, cols), 205, dtype=np.uint8)
        padded[:height, :width] = image
        image = padded.reshape(rows // step, step, cols // step, step).min(axis=(1, 3))
    image = np.ascontiguousarray(image[::-1])
    view = MapView(resolution * step, origin[0], origin[1], origin[2], image.shape[1], image.shape[0])
    return image, view


def tap_goal(view: MapView, px: float, py: float, robot=None, default_yaw: float = 0.0):
    """Goal (x, y, yaw) for a tap on the map image: heading away from the
    robot, the direction an Ackermann car arrives in, or default_yaw
    without a pose or for a tap on the robot."""
    x, y = view.to_map(px, py)
    yaw = default_yaw
    if robot is not None:
        rx, ry = robot[0], robot[1]
        if math.hypot(x - rx, y - ry) > 0.05:
            yaw = math.atan2(y - ry, x - rx)
    return x, y, yaw


def _arrow(x, y, yaw, size, colour):
    # A triangle pointing along yaw; image y grows downwards.
    points = []
    for along, across in ((1.0, 0.0), (-0.6, 0.55), (-0.3, 0.0), (-0.6, -0.55)):
        px = x + size * (along * math.cos(yaw) - across * math.sin(yaw))
        py = y - size * (along * math.sin(yaw) + across * math.cos(yaw))
        points.append(f'{px:.1f},{py:.1f}')
    return (f'<polygon points="{" ".join(points)}" fill="{colour}" stroke="#0f172a" '
            f'stroke-width="1.5" vector-effect="non-scaling-stroke"/>')


def map_overlay_svg(view: MapView, robot=None, path=(), goal=None) -> str:
    """SVG drawn over the map image (its pixels): the Nav2 path, the goal
    and the robot, each (x, y[, yaw]) in the map frame. Image axes are the
    grid's, so a heading on the image is the yaw less the grid's yaw."""
    size = max(0.45 / view.resolution, max(view.width, view.height) / 45.0)
    parts = []
    points = [view.to_pixel(x, y) for x, y in path if finite(x, y)]
    if len(points) >= 2:
        coordinates = ' '.join(f'{px:.1f},{py:.1f}' for px, py in points)
        parts.append(f'<polyline points="{coordinates}" fill="none" stroke="#22d3ee" '
                     f'stroke-width="3" stroke-linejoin="round" vector-effect="non-scaling-stroke"/>')
    if goal is not None and finite(*goal):
        gx, gy = view.to_pixel(goal[0], goal[1])
        parts.append(f'<circle cx="{gx:.1f}" cy="{gy:.1f}" r="{size * 0.45:.1f}" fill="none" '
                     f'stroke="#f59e0b" stroke-width="3" vector-effect="non-scaling-stroke"/>')
        parts.append(_arrow(gx, gy, goal[2] - view.origin_yaw, size * 0.6, '#f59e0b'))
    if robot is not None and finite(*robot):
        rx, ry = view.to_pixel(robot[0], robot[1])
        parts.append(_arrow(rx, ry, robot[2] - view.origin_yaw, size, '#38bdf8'))
    return ''.join(parts)


# --- Turret camera ----------------------------------------------------------

@dataclass(frozen=True)
class Detection:
    """One detection, its box centre and size as fractions of the image."""
    id: str
    cls: str
    confidence: float
    u: float
    v: float
    w: float
    h: float

    def contains(self, u: float, v: float) -> bool:
        return abs(u - self.u) <= self.w / 2.0 and abs(v - self.v) <= self.h / 2.0


@dataclass(frozen=True)
class Segments:
    stamp: float | None
    width: int
    height: int
    detections: tuple


def _stamp(value):
    if isinstance(value, dict):
        value = float(value.get('sec', 0)) + float(value.get('nanosec', 0)) * 1e-9
    return float(value) if finite(value) else None


def _image_size(meta):
    for keys in (('width', 'height'), ('image_width', 'image_height')):
        width, height = meta.get(keys[0]), meta.get(keys[1])
        if finite(width, height) and width > 0 and height > 0:
            return int(width), int(height)
    size = meta.get('image_size') or meta.get('size')
    if isinstance(size, dict):
        size = (size.get('width'), size.get('height'))
    if isinstance(size, (list, tuple)) and len(size) == 2 and finite(*size) and min(size) > 0:
        return int(size[0]), int(size[1])
    return None


def parse_segments(text: str, fallback_size=None):
    """Parse /turret/yolo/segments into Segments, or None.

    The tracker publishes a list of detections, each with an id, class,
    confidence and bbox_xywh (the box centre and size in pixels) and the
    image stamp, image_width and image_height. An object with the stamp and
    size and a "detections" list is accepted too, and so is the older
    tracker's list without ids or sizes, with fallback_size (width, height).
    """
    try:
        payload = json.loads(text)
    except (TypeError, ValueError):
        return None
    if isinstance(payload, list):
        items = payload
        meta = next((item for item in items if isinstance(item, dict)), {})
    elif isinstance(payload, dict):
        meta = payload
        items = meta.get('detections', meta.get('segments'))
    else:
        return None
    if not isinstance(items, list):
        return None
    size = _image_size(meta) or fallback_size
    if size is None:
        return None
    width, height = int(size[0]), int(size[1])
    detections = []
    for item in items:
        if not isinstance(item, dict):
            continue
        box = item.get('bbox_xywh')
        if not (isinstance(box, (list, tuple)) and len(box) == 4 and finite(*box)):
            continue
        cx, cy, bw, bh = (float(value) for value in box)
        if bw <= 0.0 or bh <= 0.0:
            continue
        identifier = item.get('id', item.get('track_id'))
        confidence = item.get('confidence', 0.0)
        detections.append(Detection(
            id='' if identifier is None else str(identifier),
            cls=str(item.get('class', '')),
            confidence=float(confidence) if finite(confidence) else 0.0,
            u=cx / width, v=cy / height, w=bw / width, h=bh / height))
    return Segments(_stamp(meta.get('stamp')), width, height, tuple(detections))


def image_point(image_x: float, image_y: float, width: float, height: float):
    """(u, v) of a click on an image of that size, or None outside it."""
    if not (finite(image_x, image_y, width, height) and width > 0 and height > 0):
        return None
    u, v = image_x / width, image_y / height
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return None
    return u, v


def pick_detection(segments, u: float, v: float):
    """The detection whose box contains (u, v), the nearest box centre if
    several do (the tracker's rule for a point lock), or None."""
    if segments is None:
        return None
    hits = [detection for detection in segments.detections if detection.contains(u, v)]
    if not hits:
        return None
    return min(hits, key=lambda d: math.hypot((d.u - u) * segments.width, (d.v - v) * segments.height))


def lock_command(segments, u: float, v: float) -> dict:
    """/turret/command for a tap at (u, v): lock the tapped box by its id,
    or, on no box or a box without an id, the point (the tracker picks the
    box there or reports that none is)."""
    hit = pick_detection(segments, u, v)
    if hit is not None and hit.id:
        return {'action': 'lock', 'target_id': hit.id}
    return {'action': 'lock', 'point': [round(u, 4), round(v, 4)]}


def detections_svg(segments, width: int, height: int, target_id: str = '', locked: bool = False,
                   tagged_ids=()) -> str:
    """SVG boxes over a width x height rendering of the turret image; the
    tracked target is drawn thicker, green once locked, amber before, and
    tagged detections violet and dashed."""
    if segments is None:
        return ''
    stroke = max(2.0, width / 240.0)
    font = max(12.0, width / 30.0)
    tagged = set(tagged_ids)
    parts = []
    for detection in segments.detections:
        is_target = bool(target_id) and detection.id == target_id
        is_tagged = bool(detection.id) and detection.id in tagged
        if is_target:
            colour = '#22c55e' if locked else '#f59e0b'
        else:
            colour = '#a78bfa' if is_tagged else '#22d3ee'
        x = (detection.u - detection.w / 2.0) * width
        y = (detection.v - detection.h / 2.0) * height
        label = html.escape(f'{detection.cls} {detection.confidence:.2f}'
                            + (f' #{detection.id}' if detection.id else '')
                            + (' tagged' if is_tagged else ''))
        dash = f' stroke-dasharray="{stroke * 3:.1f} {stroke * 2:.1f}"' if is_tagged else ''
        parts.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{detection.w * width:.1f}" '
            f'height="{detection.h * height:.1f}" fill="{colour}" fill-opacity="0.08" '
            f'stroke="{colour}" stroke-width="{stroke * (2 if is_target else 1):.1f}"{dash}/>'
            f'<text x="{x + 3:.1f}" y="{max(font, y - 4):.1f}" fill="{colour}" font-size="{font:.0f}" '
            f'font-family="sans-serif" font-weight="bold" stroke="#0f172a" stroke-width="3" '
            f'paint-order="stroke">{label}</text>')
    return ''.join(parts)


# --- Status -----------------------------------------------------------------

CONTROL_SOURCES = ('nav2', 'teleop', 'none')
TURRET_MODES = ('idle', 'auto', 'manual', 'estop')
LAUNCHER_STATES = ('none', 'ready', 'priming', 'chambering', 'firing', 'empty', 'fault')


def _json_object(text):
    try:
        payload = json.loads(text)
    except (TypeError, ValueError):
        return None
    return payload if isinstance(payload, dict) else None


def _number(value):
    return float(value) if finite(value) and not isinstance(value, bool) else None


def parse_control_status(text: str):
    """/racecarneo/control_status, normalised; None if it is not JSON."""
    payload = _json_object(text)
    if payload is None:
        return None
    source = payload.get('source')
    return {'source': source if source in CONTROL_SOURCES else 'none',
            'stopped': payload.get('stopped') is True,
            'speed_mps': _number(payload.get('speed_mps')),
            'steering_rad': _number(payload.get('steering_rad'))}


def _last_tag(value):
    if not isinstance(value, dict):
        return None
    return {'target_id': str(value.get('target_id') or ''),
            'result': str(value.get('result') or ''),
            'reason': str(value.get('reason') or ''),
            'stamp': _number(value.get('stamp'))}


def parse_turret_status(text: str):
    """/turret/status, normalised; None if it is not JSON. A tracker without
    the launcher's fields reads as launcher "none", never ready to fire."""
    payload = _json_object(text)
    if payload is None:
        return None
    mode = payload.get('mode')
    detections = payload.get('detections')
    launcher = payload.get('launcher')
    tagged = payload.get('tagged_ids')
    tags, darts = payload.get('tags'), payload.get('darts')
    fire_block = payload.get('fire_block')
    return {'mode': mode if mode in TURRET_MODES else 'unknown',
            'target_id': str(payload.get('target_id') or ''),
            'target_class': str(payload.get('target_class') or ''),
            'locked': payload.get('locked') is True,
            'aim_error_deg': _number(payload.get('aim_error_deg')),
            'base_compensation': payload.get('base_compensation') is True,
            'detections': int(detections) if finite(detections) else 0,
            'message': str(payload.get('message') or ''),
            # Foam dart launcher (interfaces.md, "Foam dart launcher")
            'launcher': launcher if launcher in LAUNCHER_STATES else 'none',
            'darts': int(darts) if finite(darts) and not isinstance(darts, bool) else 0,
            'tags': int(tags) if finite(tags) and not isinstance(tags, bool) else 0,
            'tagged_ids': tuple(str(i) for i in tagged) if isinstance(tagged, list) else (),
            'last_tag': _last_tag(payload.get('last_tag')),
            # None: the tracker does not say (an older one), so never ready
            'fire_block': fire_block if isinstance(fire_block, str) else None,
            'auto_next': payload.get('auto_next') is True,
            'stamp': _number(payload.get('stamp'))}


def newer_status(previous_stamp, stamp, restart_s=2.0) -> bool:
    """Whether a status stamped `stamp` replaces the one stamped
    `previous_stamp`. The page reads /turret/status twice (latched and
    volatile), so each message arrives twice and a copy can come after a
    newer one: those are dropped. A stamp well before the last one is a
    restarted tracker and is taken; so is a status without a stamp."""
    if stamp is None or previous_stamp is None:
        return True
    return not previous_stamp - restart_s < stamp <= previous_stamp


def fire_button(status, armed, actuator_ready):
    """(enabled, why not) for the Fire button. Enabled only while the
    contract's conditions hold according to a fresh /turret/status (the
    tracker's fire_block is empty: locked on the target within the aim
    error at its aim point, not too close, the launcher ready with darts,
    past the minimum interval) and the page's own view agrees (armed,
    actuator ready, locked, launcher ready). The tracker checks the gates
    again when the command arrives."""
    if status is None:
        return False, 'no status from the tracker'
    if status['launcher'] == 'none':
        return False, 'no launcher'
    if armed is not True:
        return False, 'turret disarmed'
    if actuator_ready is not True:
        return False, 'actuator not ready'
    if status['mode'] != 'auto' or not status['target_id']:
        return False, 'no locked target'
    if status['fire_block'] is None:
        return False, 'the tracker does not report fire_block'
    if status['fire_block']:
        return False, status['fire_block']
    if not status['locked']:
        return False, 'target not locked'
    if status['darts'] <= 0:
        return False, 'no darts'
    if status['launcher'] != 'ready':
        return False, f'launcher {status["launcher"]}'
    return True, ''


def fire_command(status) -> dict:
    """The fire command ("tag") for the target the status reports locked."""
    return {'action': 'tag', 'target_id': status['target_id']}


def reload_command(darts):
    """{"action": "reload", "darts": N} for a whole N from 0, else None."""
    try:
        count = float(darts)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(count) or count < 0 or count != int(count) or count > 1000:
        return None
    return {'action': 'reload', 'darts': int(count)}


def auto_next_command(enabled: bool) -> dict:
    return {'action': 'auto_next', 'enabled': bool(enabled)}


def launcher_summary(status) -> str:
    """The launcher line: its state, the darts left, the tag count and the last shot."""
    if status is None:
        return 'Launcher: no status from the tracker'
    if status['launcher'] == 'none':
        return 'Launcher: none fitted'
    darts, count = status['darts'], status['tags']
    text = (f'Launcher: {status["launcher"]}, {darts} dart{"" if darts == 1 else "s"}, '
            f'{count} tag{"" if count == 1 else "s"}')
    last = status['last_tag']
    if last is not None and last['result']:
        text += f'; last: {last["result"]}'
        if last['target_id']:
            text += f' #{last["target_id"]}'
        if last['reason'] and last['result'] != 'fired':
            text += f' ({last["reason"]})'
    return text


def control_summary(status) -> str:
    if status is None:
        return 'Drive: no status from cmd_vel_to_ackermann'
    source = {'nav2': 'Nav2', 'teleop': 'manual', 'none': 'idle'}[status['source']]
    text = f'Drive: {source}'
    if status['speed_mps'] is not None:
        text += f', {status["speed_mps"]:+.2f} m/s'
    if status['steering_rad'] is not None:
        steering = math.degrees(status['steering_rad'])
        side = 'left' if steering > 0.5 else 'right' if steering < -0.5 else 'straight'
        text += f', steer {abs(steering):.0f} deg {side}' if side != 'straight' else ', straight'
    if status['stopped']:
        text += ', STOPPED'
    return text


def turret_summary(status) -> str:
    if status is None:
        return 'Turret: no status from the tracker'
    text = f'Turret: {status["mode"]}'
    if status['target_id'] or status['target_class']:
        text += f', target {status["target_class"] or "?"}'
        if status['target_id']:
            text += f' #{status["target_id"]}'
        text += ' (locked)' if status['locked'] else ' (searching)'
    if status['aim_error_deg'] is not None:
        text += f', aim error {status["aim_error_deg"]:.1f} deg'
    text += f', base compensation {"on" if status["base_compensation"] else "off"}'
    text += f', {status["detections"]} detections'
    if status['message']:
        text += f'. {status["message"]}'
    return text


def fresh(now: float, stamp, max_age_s: float = 1.0) -> bool:
    return stamp is not None and now - stamp <= max_age_s
