"""RACECAR Neo, RACECAR Turret and PanTiltROS turret operator page, one
NiceGUI page on port 8080.

Car (unless UI_CAR=0): the SLAM map with the robot and the Nav2 path (tap it
to send a NavigateToPose goal, or type one), cancel, a drive joystick
(/cmd_vel_teleop at 10 Hz while held; cmd_vel_to_ackermann gives it
priority over Nav2), Stop/Resume (/racecarneo/stop_request), the drive
status, the front camera and pipeline freshness. Turret (UI_TURRET=1, the
RACECAR Turret; with UI_CAR=0 the standalone turret's turret-only layout):
the turret camera with the tracker's boxes (tap one to lock it), Release,
Center, an aim joystick, Arm/Disarm (/turret/emergency_stop), the tracker
status, and the foam dart launcher: Fire (enabled only while /turret/status
says every gate holds; one {"action": "tag", "target_id": <locked id>} per
tap), Reload with the dart count, Next, Clear tags, an auto-next toggle, and
the launcher's state, darts and tags. Stop All stops the car, cancels Nav2
and engages the turret emergency stop. Topics and commands:
systems/racecar_turret/interfaces.md.

rclpy spins in a daemon thread; page handlers publish from NiceGUI's event
loop. Images are encoded at a modest rate in the ROS thread and served
at /frames/<name>; their rate, width and JPEG quality are set by
UI_CAMERA_* and UI_TURRET_CAMERA_* (racecar_ui_logic.image_stream), and the
front camera is only sent while its panel is open. The decisions
(joysticks to commands, taps to goals and locks, status parsing) live in
racecar_ui_logic.py.
"""
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from action_msgs.srv import CancelGoal
from cv_bridge import CvBridge
from fastapi import Response
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from nicegui import app, ui, ui_run

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import racecar_ui_logic as logic  # noqa: E402
from racecar_ui_logic import quaternion_from_yaw, yaw_from_quaternion  # noqa: E402,F401


def env_flag(name: str, default: bool = False) -> bool:
    value = os.environ.get(name, '').strip().lower()
    return default if not value else value in ('1', 'true', 'yes', 'on')


# UI_TURRET=1: the RACECAR Turret page (compositions/racecar_turret_*.yaml);
# with UI_CAR=0 too, the turret-only page of the standalone PanTiltROS
# turret (compositions/turret_hardware.yaml). The car part needs no flag.
TURRET = env_flag('UI_TURRET')
CAR = env_flag('UI_CAR', True) or not TURRET
TITLE = ('RACECAR Turret' if CAR else 'PanTiltROS Turret') if TURRET else 'RACECAR Neo'
# The camera the tracker uses (the standalone turret's may be a RealSense's)
TURRET_CAMERA_TOPIC = os.environ.get('UI_TURRET_CAMERA_TOPIC', '/turret/camera/image_raw')
# The depth freshness light. The car's nvblox reads the RealSense's aligned
# depth, so the hardware stacks watch its camera_info (a few hundred bytes)
# rather than the 614 kB images; the simulators keep Depth Anything's image.
DEPTH_TOPIC = os.environ.get('UI_DEPTH_TOPIC', '/depth_anything/depth/image')
# Manual driving at full stick, below cmd_vel_to_ackermann's own limits.
DRIVE_LIMITS = logic.DriveLimits(
    max_speed=float(os.environ.get('UI_MAX_SPEED', '0.3')),
    max_steering=float(os.environ.get('UI_MAX_STEERING', '0.42')),
    wheelbase=float(os.environ.get('UI_WHEELBASE', '0.33')))
# Manual aim at full stick (rad/s); the tracker clamps to the servo limits.
TURRET_MAX_RATE = float(os.environ.get('UI_TURRET_MAX_RATE', '1.0'))
# JPEG streams: <prefix>_HZ (the front camera's 0 turns it off),
# <prefix>_MAX_WIDTH, <prefix>_JPEG_QUALITY. Over Wi-Fi HaLow the hardware
# compositions send less (systems/racecar_turret/README.md, "Long-range link").
CAMERA_STREAM = logic.image_stream(os.environ, 'UI_CAMERA', logic.CAMERA_STREAM, allow_off=True)
TURRET_STREAM = logic.image_stream(os.environ, 'UI_TURRET_CAMERA', logic.TURRET_STREAM)

# Nav2 goals are expressed in the map frame, so the pose shown next to the
# waypoint inputs is map -> base_link too. /odom drifts from map as SLAM
# corrects it, so showing it would offset goals typed from the display.
MAP_FRAME = 'map'
BASE_FRAME = 'base_link'
# Cancelling through these covers goals sent by other clients (the mission runner).
NAV_ACTIONS = ('navigate_to_pose', 'navigate_through_poses')
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
STALE_S = 1.0
# SLAM Toolbox republishes /map every few seconds (map_update_interval).
MAP_STALE_S = 15.0


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    pose_time: float = 0.0
    odom_time: float = 0.0
    camera_time: float = 0.0
    depth_time: float = 0.0
    map_time: float = 0.0
    control_time: float = 0.0
    plan_time: float = 0.0
    nav_status: str = 'Waiting for Nav2'
    goal: Optional[tuple] = None
    path: tuple = ()
    map_view: Optional[logic.MapView] = None
    control: Optional[dict] = None
    # Turret
    turret_camera_time: float = 0.0
    segments_time: float = 0.0
    turret_status_time: float = 0.0
    turret_image_size: Optional[tuple] = None
    segments: Optional[logic.Segments] = None
    turret_status: Optional[dict] = None
    turret_estop: Optional[bool] = None
    actuator_ready: Optional[bool] = None


class Frames:
    """The latest encoded image per name, served at /frames/<name>."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.images: dict[str, tuple[bytes, str, int, tuple[int, int]]] = {}

    def put(self, name: str, data: bytes, media_type: str, size: tuple[int, int]) -> None:
        with self.lock:
            version = self.images[name][2] + 1 if name in self.images else 1
            self.images[name] = (data, media_type, version, size)

    def get(self, name: str):
        with self.lock:
            return self.images.get(name)

    def version(self, name: str) -> int:
        image = self.get(name)
        return image[2] if image else 0

    def size(self, name: str):
        image = self.get(name)
        return image[3] if image else None


def placeholder_svg(text: str, width: int = 640, height: int = 480) -> bytes:
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">'
            f'<rect width="100%" height="100%" fill="#1e293b"/>'
            f'<text x="50%" y="50%" fill="#94a3b8" font-family="sans-serif" font-size="22" '
            f'text-anchor="middle">{text}</text></svg>').encode()


class RacecarNeoGuiNode(Node):
    car = True  # the car part (the tests build the node without __init__)
    cancel_clients: list = []

    def __init__(self, state: VehicleState, frames: Frames, turret: bool = False,
                 car: bool = True) -> None:
        super().__init__('racecarneo_nicegui')
        self.state = state
        self.frames = frames
        self.turret = turret
        self.car = car
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_turret_frame = None
        self.latest_map = None
        self.cancel_clients = []
        if car:
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.cancel_clients = [self.create_client(CancelGoal, f'{action}/_action/cancel_goal')
                                   for action in NAV_ACTIONS]
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.teleop_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
            self.stop_pub = self.create_publisher(Bool, '/racecarneo/stop_request', LATCHED_QOS)
            self.create_timer(0.1, self.update_map_pose)
            if CAMERA_STREAM.rate_hz > 0:
                self.create_timer(CAMERA_STREAM.period_s, self.encode_latest_frame)
            self.create_timer(0.5, self.render_map)
            self.create_subscription(Odometry, '/odom', self.handle_odom, 10)
            self.create_subscription(Image, '/camera/color/image_raw', self.handle_camera,
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo if DEPTH_TOPIC.endswith('camera_info') else Image,
                                     DEPTH_TOPIC, self.handle_depth, qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, '/map', self.handle_map, LATCHED_QOS)
            self.create_subscription(NavPath, '/plan', self.handle_plan, 10)
            self.create_subscription(String, '/racecarneo/control_status', self.handle_control_status, 10)
        if turret:
            self.turret_command_pub = self.create_publisher(String, '/turret/command', 10)
            self.turret_estop_pub = self.create_publisher(Bool, '/turret/emergency_stop', LATCHED_QOS)
            self.create_timer(TURRET_STREAM.period_s, self.encode_turret_frame)
            self.create_subscription(Image, TURRET_CAMERA_TOPIC, self.handle_turret_camera,
                                     qos_profile_sensor_data)
            self.create_subscription(String, '/turret/yolo/segments', self.handle_segments, 10)
            # Latched readers get the last value from transient-local publishers;
            # the volatile ones also match default-QoS publishers.
            for qos in (LATCHED_QOS, 10):
                self.create_subscription(String, '/turret/status', self.handle_turret_status, qos)
                self.create_subscription(Bool, '/turret/emergency_stop', self.handle_turret_estop, qos)
            self.create_subscription(Bool, '/turret/actuator_ready', self.handle_actuator_ready, LATCHED_QOS)

    # --- Car state ---------------------------------------------------------

    def update_map_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, Time())
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.state.x = translation.x
        self.state.y = translation.y
        self.state.yaw = yaw_from_quaternion(rotation.z, rotation.w)
        self.state.pose_time = time.monotonic()

    def handle_odom(self, msg: Odometry) -> None:
        # Pipeline health only; the displayed pose comes from map -> base_link.
        self.state.odom_time = time.monotonic()

    def handle_camera(self, msg: Image) -> None:
        self.state.camera_time = time.monotonic()
        # Store the raw message; conversion and encoding happen at UI rate.
        with self.frame_lock:
            self.latest_frame = msg

    def handle_depth(self, msg) -> None:
        self.state.depth_time = time.monotonic()

    def handle_map(self, msg: OccupancyGrid) -> None:
        self.state.map_time = time.monotonic()
        with self.frame_lock:
            self.latest_map = msg

    def handle_plan(self, msg: NavPath) -> None:
        if msg.header.frame_id not in ('', MAP_FRAME):
            return
        poses = msg.poses
        step = max(1, len(poses) // 300)
        self.state.path = tuple((pose.pose.position.x, pose.pose.position.y) for pose in poses[::step])
        self.state.plan_time = time.monotonic()

    def handle_control_status(self, msg: String) -> None:
        status = logic.parse_control_status(msg.data)
        if status is not None:
            self.state.control = status
            self.state.control_time = time.monotonic()

    def encode_latest_frame(self) -> None:
        with self.frame_lock:
            msg = self.latest_frame
            self.latest_frame = None
        if msg is not None:
            self.encode('camera', msg, CAMERA_STREAM)

    def encode(self, name: str, msg: Image, stream: logic.ImageStream) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = frame.shape[:2]
            size = logic.scaled_size(width, height, stream.max_width)
            if size != (width, height):
                frame = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), stream.jpeg_quality])
            if ok:
                self.frames.put(name, encoded.tobytes(), 'image/jpeg', (frame.shape[1], frame.shape[0]))
        except Exception as exc:
            self.get_logger().warn(f'Could not render {name} frame: {exc}', throttle_duration_sec=5.0)

    def render_map(self) -> None:
        with self.frame_lock:
            msg = self.latest_map
            self.latest_map = None
        if msg is None:
            return
        info = msg.info
        origin = info.origin
        try:
            image, view = logic.occupancy_image(
                msg.data, info.width, info.height, info.resolution,
                (origin.position.x, origin.position.y,
                 yaw_from_quaternion(origin.orientation.z, origin.orientation.w)))
            ok, encoded = cv2.imencode('.png', image)
        except Exception as exc:
            self.get_logger().warn(f'Could not render the map: {exc}', throttle_duration_sec=5.0)
            return
        if ok:
            self.frames.put('map', encoded.tobytes(), 'image/png', (view.width, view.height))
            self.state.map_view = view

    # --- Car commands ------------------------------------------------------

    def send_waypoint(self, x: float, y: float, yaw: float) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.state.nav_status = 'Nav2 action server unavailable'
            return

        z, w = quaternion_from_yaw(yaw)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        self.state.nav_status = f'Navigating to x={x:.2f}, y={y:.2f}'
        self.state.goal = (float(x), float(y), float(yaw))

        future = self.nav_client.send_goal_async(goal, feedback_callback=self.handle_feedback)
        future.add_done_callback(self.handle_goal_response)

    def handle_feedback(self, feedback_msg) -> None:
        distance = feedback_msg.feedback.distance_remaining
        self.state.nav_status = f'Navigating, {distance:.2f} m remaining'

    def handle_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state.nav_status = 'Waypoint rejected'
            self.state.goal = None
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_result)

    def handle_result(self, future) -> None:
        status = future.result().status
        self.state.goal = None
        if status == 4:
            self.state.nav_status = 'Waypoint reached'
        elif status == 5:
            self.state.nav_status = 'Navigation canceled'
        else:
            self.state.nav_status = f'Navigation finished with status {status}'

    def cancel_navigation(self, reason: str = 'Cancel') -> bool:
        """Cancel every Nav2 navigation goal, whoever sent it (a zero goal
        ID and stamp in CancelGoal means all goals)."""
        ready = [client for client in self.cancel_clients if client.service_is_ready()]
        for client in ready:
            client.call_async(CancelGoal.Request())
        if ready:
            self.state.nav_status = f'{reason}: Nav2 goals canceled'
        return bool(ready)

    def publish_teleop(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.teleop_pub.publish(msg)

    def request_stop(self, stop: bool) -> None:
        if self.car:
            self.stop_pub.publish(Bool(data=bool(stop)))

    # --- Turret ------------------------------------------------------------

    def handle_turret_camera(self, msg: Image) -> None:
        self.state.turret_camera_time = time.monotonic()
        self.state.turret_image_size = (msg.width, msg.height)
        with self.frame_lock:
            self.latest_turret_frame = msg

    def encode_turret_frame(self) -> None:
        with self.frame_lock:
            msg = self.latest_turret_frame
            self.latest_turret_frame = None
        if msg is not None:
            self.encode('turret', msg, TURRET_STREAM)

    def handle_segments(self, msg: String) -> None:
        segments = logic.parse_segments(msg.data, fallback_size=self.state.turret_image_size)
        if segments is not None:
            self.state.segments = segments
            self.state.segments_time = time.monotonic()

    def handle_turret_status(self, msg: String) -> None:
        status = logic.parse_turret_status(msg.data)
        previous = self.state.turret_status
        if status is not None and logic.newer_status(
                previous['stamp'] if previous is not None else None, status['stamp']):
            self.state.turret_status = status
            self.state.turret_status_time = time.monotonic()

    def handle_turret_estop(self, msg: Bool) -> None:
        self.state.turret_estop = bool(msg.data)

    def handle_actuator_ready(self, msg: Bool) -> None:
        self.state.actuator_ready = bool(msg.data)

    def send_turret_command(self, command: dict) -> None:
        if self.turret:
            self.turret_command_pub.publish(String(data=json.dumps(command)))

    def set_turret_estop(self, engaged: bool) -> None:
        if self.turret:
            self.turret_estop_pub.publish(Bool(data=bool(engaged)))

    def stop_all(self) -> None:
        if self.car:
            self.request_stop(True)
            self.cancel_navigation('Stop all')
        self.set_turret_estop(True)


state = VehicleState()
frames = Frames()
node_holder: dict[str, Optional[RacecarNeoGuiNode]] = {'node': None}
JOYSTICK_JS = (Path(__file__).parent / 'joystick.js').read_text()
PLACEHOLDERS = {
    'camera': placeholder_svg('Waiting for /camera/color/image_raw' if CAMERA_STREAM.rate_hz > 0
                              else 'Front camera off (UI_CAMERA_HZ=0)'),
    'turret': placeholder_svg(f'Waiting for {TURRET_CAMERA_TOPIC}'),
    'map': placeholder_svg('Waiting for /map'),
}
CSS = """
.joy-pad { position: relative; width: 136px; height: 136px; border-radius: 50%; flex: none;
  background: radial-gradient(circle, #334155 0%, #1e293b 70%); border: 2px solid #475569;
  touch-action: none; user-select: none; -webkit-user-select: none; -webkit-touch-callout: none;
  cursor: grab; }
.joy-pad.joy-active { border-color: #38bdf8; cursor: grabbing; }
.joy-knob { position: absolute; left: 50%; top: 50%; width: 52px; height: 52px; margin: -26px 0 0 -26px;
  border-radius: 50%; background: #94a3b8; box-shadow: 0 2px 6px #0008; pointer-events: none; }
.joy-pad.joy-active .joy-knob { background: #38bdf8; }
.joy-label { position: absolute; bottom: -1.4rem; width: 100%; text-align: center; font-size: 0.75rem;
  color: #94a3b8; pointer-events: none; }
@media (min-width: 768px) { .joy-pad { width: 160px; height: 160px; } }
"""


@app.get('/frames/{name}')
def serve_frame(name: str):
    image = frames.get(name)
    if image is None:
        if name not in PLACEHOLDERS:
            return Response(status_code=404)
        return Response(content=PLACEHOLDERS[name], media_type='image/svg+xml',
                        headers={'Cache-Control': 'no-store'})
    return Response(content=image[0], media_type=image[1], headers={'Cache-Control': 'no-store'})


def ros() -> Optional[RacecarNeoGuiNode]:
    node = node_holder['node']
    if node is None:
        ui.notify('ROS is not up yet', type='warning')
    return node


def number(value, default: float = 0.0) -> float:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def freshness_html(now: float) -> str:
    items = []
    if CAR:
        items += [('pose', state.pose_time), ('odom', state.odom_time), ('map', state.map_time),
                  ('camera', state.camera_time), ('depth', state.depth_time),
                  ('drive', state.control_time)]
    if TURRET:
        items += [('turret cam', state.turret_camera_time), ('detections', state.segments_time),
                  ('tracker', state.turret_status_time)]
    chips = []
    for name, stamp in items:
        ok = logic.fresh(now, stamp, MAP_STALE_S if name == 'map' else STALE_S) if stamp else False
        colour = '#22c55e' if ok else '#ef4444'
        chips.append(f'<span style="display:inline-flex;align-items:center;gap:4px;margin-right:10px;'
                     f'white-space:nowrap"><span style="width:9px;height:9px;border-radius:50%;'
                     f'background:{colour}"></span>{name}</span>')
    return ''.join(chips)


@ui.page('/')
def index() -> None:
    ui.dark_mode(True)
    ui.add_css(CSS)
    ui.add_head_html(f'<script>{JOYSTICK_JS}</script>')
    ui.page_title(TITLE)
    ui.query('body').classes('bg-slate-950 text-slate-100')
    ui.query('.nicegui-content').classes('p-0 gap-0')
    holds = {'drive': logic.Hold(), 'aim': logic.Hold()}
    shown = {'map': 0, 'camera': 0, 'turret': 0, 'map_overlay': None, 'turret_overlay': None, 'armed': None,
             'fire': None, 'syncing': False, 'auto_next_sent': -math.inf}

    def stop_all() -> None:
        node = ros()
        if node is not None:
            node.stop_all()
            parts = (['car stopped', 'Nav2 canceled'] if CAR else []) + (['turret disarmed'] if TURRET else [])
            ui.notify('Stop all: ' + ', '.join(parts), type='negative')

    with ui.row().classes('sticky top-0 z-50 w-full items-center gap-2 px-3 py-2 '
                          'bg-slate-900/95 border-b border-slate-700 flex-nowrap'):
        ui.label(TITLE).classes('text-lg font-semibold truncate')
        ui.space()
        ui.button('Stop all', icon='front_hand', on_click=stop_all) \
            .props('color=red unelevated data-testid=stop-all').classes('text-base font-bold px-4')
    fresh_html = ui.html('', sanitize=False).classes('w-full px-3 py-1 text-xs text-slate-300 '
                                                       'flex flex-wrap gap-y-1')

    # Car and turret side by side on a wide screen; the turret-only page one
    # column, not wider than a tablet.
    layout = 'lg:grid-cols-2' if CAR and TURRET else 'max-w-3xl mx-auto'
    with ui.element('div').classes(f'w-full grid grid-cols-1 {layout} gap-3 p-3 pb-56'):
        # --- Car -----------------------------------------------------------
        if CAR:
            with ui.card().props('flat bordered').classes('w-full min-w-0 bg-slate-900 gap-2'):
                with ui.row().classes('w-full items-center gap-2'):
                    ui.label('Car').classes('text-lg font-medium')
                    stopped_badge = ui.badge('STOPPED', color='red').classes('text-sm')
                control_label = ui.label('').classes('text-sm text-cyan-200')
                nav_status = ui.label('Waiting for Nav2').classes('text-sm')
                pose_label = ui.label('Pose: --').classes('text-sm text-slate-300')
                map_image = ui.interactive_image('/frames/map?v=0', events=['click'], sanitize=False) \
                    .classes('w-full rounded bg-slate-800').props('data-testid=map')
                ui.label('Tap the map to send a Nav2 goal there').classes('text-xs text-slate-400')
                with ui.row().classes('w-full gap-2'):
                    ui.button('Cancel goal', icon='cancel', on_click=lambda: cancel_goal()) \
                        .props('outline color=cyan data-testid=cancel-goal')
                    ui.button('Stop', icon='stop', on_click=lambda: set_stop(True)) \
                        .props('color=orange data-testid=car-stop')
                    ui.button('Resume', icon='play_arrow', on_click=lambda: set_stop(False)) \
                        .props('color=green data-testid=car-resume')
                with ui.expansion('Waypoint', icon='place').classes('w-full'):
                    with ui.row().classes('w-full gap-2 items-end'):
                        x_input = ui.number('x', value=2.0, format='%.2f').classes('w-24')
                        y_input = ui.number('y', value=0.0, format='%.2f').classes('w-24')
                        yaw_input = ui.number('yaw rad', value=0.0, format='%.2f').classes('w-24')
                        ui.button('Send waypoint', on_click=lambda: send_form_goal()) \
                            .props('color=cyan data-testid=send-waypoint')
                # Sent only while open: over a slow link the turret camera gets the bandwidth.
                with ui.expansion('Front camera', icon='videocam').classes('w-full') as camera_panel:
                    camera = ui.interactive_image('/frames/camera?v=0', events=[]).classes('w-full rounded')

        # --- Turret --------------------------------------------------------
        if TURRET:
            with ui.card().props('flat bordered').classes('w-full min-w-0 bg-slate-900 gap-2'):
                with ui.row().classes('w-full items-center gap-2'):
                    ui.label('Turret').classes('text-lg font-medium')
                    armed_badge = ui.badge('', color='grey').classes('text-sm')
                turret_label = ui.label('').classes('text-sm text-cyan-200').props('data-testid=turret-status')
                turret_image = ui.interactive_image('/frames/turret?v=0', events=['click'], sanitize=False) \
                    .classes('w-full rounded bg-slate-800').props('data-testid=turret-image')
                ui.label('Tap a box to track it').classes('text-xs text-slate-400')
                with ui.row().classes('w-full gap-2'):
                    ui.button('Release', icon='lock_open',
                              on_click=lambda: turret_command({'action': 'release'})) \
                        .props('outline color=cyan data-testid=turret-release')
                    ui.button('Center', icon='center_focus_strong',
                              on_click=lambda: turret_command({'action': 'center'})) \
                        .props('outline color=cyan data-testid=turret-center')
                    ui.button('Arm', icon='power', on_click=lambda: set_turret_estop(False)) \
                        .props('color=green data-testid=turret-arm')
                    ui.button('Disarm', icon='power_off', on_click=lambda: set_turret_estop(True)) \
                        .props('color=orange data-testid=turret-disarm')
                # The dart launcher: one shot per tap of Fire, at the locked target only
                ui.separator().classes('bg-slate-700')
                with ui.row().classes('w-full items-center gap-2'):
                    fire_button = ui.button('Fire', icon='ads_click', on_click=lambda: fire()) \
                        .props('color=red unelevated data-testid=turret-fire') \
                        .classes('text-base font-bold px-5')
                    fire_button.disable()
                    ui.button('Next', icon='skip_next', on_click=lambda: turret_command({'action': 'next'})) \
                        .props('outline color=cyan data-testid=turret-next')
                    ui.button('Clear tags', icon='clear_all',
                              on_click=lambda: turret_command({'action': 'clear_tags'})) \
                        .props('outline color=cyan data-testid=turret-clear-tags')
                    auto_next = ui.switch('Auto next', on_change=lambda e: on_auto_next(e.value)) \
                        .props('color=cyan data-testid=turret-auto-next')
                with ui.row().classes('w-full items-end gap-2'):
                    darts_input = ui.number('darts', value=6, min=0, max=1000, step=1, format='%d') \
                        .classes('w-24').props('data-testid=turret-darts')
                    ui.button('Reload', icon='refresh', on_click=lambda: reload()) \
                        .props('outline color=amber data-testid=turret-reload')
                launcher_label = ui.label('').classes('text-sm text-violet-200') \
                    .props('data-testid=turret-launcher')
                fire_reason = ui.label('').classes('text-xs text-slate-400') \
                    .props('data-testid=turret-fire-reason')

    # Joysticks: docked at the bottom so both thumbs reach them on a phone.
    with ui.row().classes('fixed bottom-0 left-0 right-0 z-40 justify-around items-center flex-nowrap '
                          'gap-2 px-3 pt-3 pb-7 bg-slate-900/95 border-t border-slate-700'):
        if CAR:
            ui.html('<div class="joy-pad" data-joystick="drive"><div class="joy-knob"></div>'
                    '<div class="joy-label">drive</div></div>', sanitize=False).props('data-testid=joy-drive')
        dock_label = ui.label('').classes('text-xs text-slate-300 text-center whitespace-pre-line min-w-0')
        if TURRET:
            ui.html('<div class="joy-pad" data-joystick="aim"><div class="joy-knob"></div>'
                    '<div class="joy-label">aim</div></div>', sanitize=False).props('data-testid=joy-aim')

    images = {}
    if CAR:
        images.update(map=map_image, camera=camera)
    if TURRET:
        images['turret'] = turret_image

    def cancel_goal() -> None:
        node = ros()
        if node is not None and not node.cancel_navigation('Cancel'):
            ui.notify('Nav2 cancel service unavailable', type='warning')

    def set_stop(stop: bool) -> None:
        node = ros()
        if node is not None:
            node.request_stop(stop)
            ui.notify('Car stopped' if stop else 'Car released', type='warning' if stop else 'positive')

    def send_form_goal() -> None:
        node = ros()
        if node is not None:
            node.send_waypoint(number(x_input.value), number(y_input.value), number(yaw_input.value))

    def on_map_click(e) -> None:
        view = state.map_view
        node = ros()
        if view is None or node is None:
            ui.notify('No map yet', type='warning')
            return
        robot = (state.x, state.y, state.yaw) if logic.fresh(time.monotonic(), state.pose_time, STALE_S) else None
        x, y, yaw = logic.tap_goal(view, e.image_x, e.image_y, robot, number(yaw_input.value))
        x_input.value, y_input.value, yaw_input.value = round(x, 2), round(y, 2), round(yaw, 2)
        node.send_waypoint(x, y, yaw)

    if CAR:
        map_image.on_mouse(on_map_click)

    def turret_command(command: dict) -> None:
        node = ros()
        if node is not None:
            node.send_turret_command(command)

    def set_turret_estop(engaged: bool) -> None:
        node = ros()
        if node is not None:
            node.set_turret_estop(engaged)
            ui.notify('Turret disarmed' if engaged else 'Turret armed', type='warning' if engaged else 'positive')

    def turret_status(now: float):
        return state.turret_status if logic.fresh(now, state.turret_status_time, 2.0 * STALE_S) else None

    def fire_state(now: float):
        return logic.fire_button(turret_status(now), state.turret_estop is False, state.actuator_ready)

    def fire() -> None:
        # Checked again at the tap: the button may not have caught up yet.
        now = time.monotonic()
        enabled, why = fire_state(now)
        if not enabled:
            ui.notify(f'Not firing: {why}', type='warning')
            return
        status = turret_status(now)
        turret_command(logic.fire_command(status))
        ui.notify(f'Fire at #{status["target_id"]}', type='positive')

    def reload() -> None:
        command = logic.reload_command(darts_input.value)
        if command is None:
            ui.notify('Reload: the dart count must be a whole number from 0', type='warning')
            return
        turret_command(command)
        ui.notify(f'Reload: {command["darts"]} darts')

    def on_auto_next(value) -> None:
        if shown['syncing']:
            return  # set from the tracker's status, not by the operator
        shown['auto_next_sent'] = time.monotonic()
        turret_command(logic.auto_next_command(bool(value)))

    def on_turret_click(e) -> None:
        size = frames.size('turret')
        point = logic.image_point(e.image_x, e.image_y, *size) if size else None
        if point is None:
            ui.notify('No turret image yet', type='warning')
            return
        segments = state.segments if logic.fresh(time.monotonic(), state.segments_time, STALE_S) else None
        command = logic.lock_command(segments, *point)
        turret_command(command)
        ui.notify(f'Lock target {command["target_id"]}' if 'target_id' in command
                  else 'Lock the object at that point')

    if TURRET:
        turret_image.on_mouse(on_turret_click)

    def on_joystick(e) -> None:
        args = e.args[0] if isinstance(e.args, list) and e.args else e.args
        if not isinstance(args, dict) or args.get('name') not in holds:
            return
        name = args['name']
        started = holds[name].report(number(args.get('x')), number(args.get('y')), bool(args.get('held')),
                                     time.monotonic(), sent=args.get('t'))
        node = node_holder['node']
        if started and name == 'drive' and node is not None:
            # Manual driving takes over: stop Nav2 from resuming afterwards.
            node.cancel_navigation('Manual driving')

    ui.on('joystick', on_joystick)

    def joystick_tick() -> None:
        node = node_holder['node']
        if node is None:
            return
        now = time.monotonic()
        vector = holds['drive'].tick(now)
        if vector is not None and CAR:
            node.publish_teleop(*logic.drive_twist(*vector, DRIVE_LIMITS))
        vector = holds['aim'].tick(now)
        if vector is not None and TURRET:
            node.send_turret_command(logic.manual_command(*logic.turret_rates(*vector, TURRET_MAX_RATE)))

    ui.timer(0.1, joystick_tick)

    def refresh_ui() -> None:
        now = time.monotonic()
        fresh_html.content = freshness_html(now)
        dock = []
        for name, image in images.items():
            if name == 'camera' and not camera_panel.value:
                continue
            version = frames.version(name)
            if version != shown[name]:
                shown[name] = version
                image.set_source(f'/frames/{name}?v={version}')
        if CAR:
            nav_status.text = state.nav_status
            if logic.fresh(now, state.pose_time, STALE_S):
                pose_label.text = f'Pose ({MAP_FRAME}): x={state.x:.2f}, y={state.y:.2f}, yaw={state.yaw:.2f}'
            else:
                pose_label.text = f'Pose ({MAP_FRAME}): waiting for {MAP_FRAME} -> {BASE_FRAME}'
            control = state.control if logic.fresh(now, state.control_time, STALE_S) else None
            control_label.text = logic.control_summary(control)
            stopped_badge.set_visibility(bool(control and control['stopped']))
            dock.append(f'{"MANUAL" if holds["drive"].held(now) else "drive"}: '
                        f'{control["source"] if control else "?"}')
            view = state.map_view
            if view is not None:
                robot = (state.x, state.y, state.yaw) if logic.fresh(now, state.pose_time, STALE_S) else None
                # Nav2 republishes the plan while it navigates (also goals sent by others).
                path = state.path if state.goal or logic.fresh(now, state.plan_time, 3.0) else ()
                overlay = logic.map_overlay_svg(view, robot, path, state.goal)
                if overlay != shown['map_overlay']:
                    shown['map_overlay'] = overlay
                    map_image.content = overlay
        if TURRET:
            status = turret_status(now)
            turret_label.text = logic.turret_summary(status)
            if state.turret_estop is None:
                badge = ('DISARMED (no e-stop state)', 'grey')
            elif state.turret_estop:
                badge = ('DISARMED', 'orange')
            else:
                badge = ('ARMED, actuator ' + ('ready' if state.actuator_ready else 'not ready'), 'green')
            if badge != shown['armed']:
                shown['armed'] = badge
                armed_badge.set_text(badge[0])
                armed_badge.props(f'color={badge[1]}')
            size = frames.size('turret')
            segments = state.segments if logic.fresh(now, state.segments_time, STALE_S) else None
            overlay = ''
            if size is not None:
                overlay = logic.detections_svg(segments, size[0], size[1],
                                               status['target_id'] if status else '',
                                               bool(status and status['locked']),
                                               status['tagged_ids'] if status else ())
            if overlay != shown['turret_overlay']:
                shown['turret_overlay'] = overlay
                turret_image.content = overlay
            # The dart launcher
            enabled, why = fire_state(now)
            label = f'Fire #{status["target_id"]}' if enabled else 'Fire'
            if (enabled, label) != shown['fire']:
                shown['fire'] = (enabled, label)
                fire_button.set_text(label)
                fire_button.set_enabled(enabled)
            launcher_label.text = logic.launcher_summary(status)
            fire_reason.text = 'Ready to fire' if enabled else f'Fire: {why}'
            if (status is not None and status['auto_next'] != auto_next.value
                    and now - shown['auto_next_sent'] > 2.0):
                shown['syncing'] = True
                try:
                    auto_next.value = status['auto_next']
                finally:
                    shown['syncing'] = False
            dock.append(f'{"MANUAL" if holds["aim"].held(now) else "aim"}: {status["mode"] if status else "?"}')
        dock_label.text = '\n'.join(dock)

    ui.timer(0.2, refresh_ui)


def main() -> None:
    pass


def ros_main() -> None:
    rclpy.init()
    node = RacecarNeoGuiNode(state, frames, turret=TURRET, car=CAR)
    node_holder['node'] = node
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


app.on_startup(lambda: threading.Thread(target=ros_main, daemon=True).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'
ui.run(host='0.0.0.0', port=int(os.environ.get('UI_PORT', '8080')), title=TITLE,
       uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='R')
