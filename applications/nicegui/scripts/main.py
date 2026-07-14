import base64
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from nicegui import app, ui, ui_run


def yaw_from_quaternion(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    odom_time: float = 0.0
    camera_time: float = 0.0
    depth_time: float = 0.0
    camera_image: str = ''
    nav_status: str = 'Waiting for Nav2'


class RacecarNeoGuiNode(Node):
    def __init__(self, state: VehicleState) -> None:
        super().__init__('racecarneo_nicegui')
        self.state = state
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Odometry, '/odom', self.handle_odom, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.handle_camera, 5)
        self.create_subscription(Image, '/depth_anything/depth/image', self.handle_depth, 5)

    def handle_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.state.x = pose.position.x
        self.state.y = pose.position.y
        self.state.yaw = yaw_from_quaternion(pose.orientation.z, pose.orientation.w)
        self.state.odom_time = time.monotonic()

    def handle_camera(self, msg: Image) -> None:
        self.state.camera_time = time.monotonic()
        # Store the raw message; conversion and encoding happen at UI rate.
        with self.frame_lock:
            self.latest_frame = msg

    def encode_latest_frame(self) -> None:
        with self.frame_lock:
            msg = self.latest_frame
            self.latest_frame = None
        if msg is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (480, 270), interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                data = base64.b64encode(encoded).decode('ascii')
                self.state.camera_image = f'data:image/jpeg;base64,{data}'
        except Exception as exc:
            self.get_logger().warn(f'Could not render camera frame: {exc}')

    def handle_depth(self, msg: Image) -> None:
        self.state.depth_time = time.monotonic()

    def send_waypoint(self, x: float, y: float, yaw: float) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.state.nav_status = 'Nav2 action server unavailable'
            return

        z, w = quaternion_from_yaw(yaw)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        self.state.nav_status = f'Navigating to x={x:.2f}, y={y:.2f}'

        future = self.nav_client.send_goal_async(goal, feedback_callback=self.handle_feedback)
        future.add_done_callback(self.handle_goal_response)

    def handle_feedback(self, feedback_msg) -> None:
        distance = feedback_msg.feedback.distance_remaining
        self.state.nav_status = f'Navigating, {distance:.2f} m remaining'

    def handle_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state.nav_status = 'Waypoint rejected'
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_result)

    def handle_result(self, future) -> None:
        status = future.result().status
        if status == 4:
            self.state.nav_status = 'Waypoint reached'
        else:
            self.state.nav_status = f'Navigation finished with status {status}'


state = VehicleState()
node_holder: dict[str, Optional[RacecarNeoGuiNode]] = {'node': None}


@ui.page('/')
def index() -> None:
    ui.query('body').classes('bg-slate-950 text-slate-100')
    with ui.row().classes('w-full min-h-screen gap-0'):
        with ui.column().classes('w-[360px] p-4 gap-4 bg-slate-900'):
            ui.label('RACECAR Neo').classes('text-2xl font-semibold')
            nav_status = ui.label('Waiting for Nav2').classes('text-sm text-cyan-200')
            pose_label = ui.label('Pose: --').classes('text-sm')
            pipeline_label = ui.label('Pipeline: --').classes('text-sm text-slate-300')

            ui.separator().classes('bg-slate-700')
            ui.label('Waypoint').classes('text-lg font-medium')
            x_input = ui.number('x', value=2.0, format='%.2f').classes('w-full')
            y_input = ui.number('y', value=0.0, format='%.2f').classes('w-full')
            yaw_input = ui.number('yaw rad', value=0.0, format='%.2f').classes('w-full')
            ui.button(
                'Send waypoint',
                on_click=lambda: node_holder['node'] and node_holder['node'].send_waypoint(
                    float(x_input.value or 0.0),
                    float(y_input.value or 0.0),
                    float(yaw_input.value or 0.0),
                ),
            ).props('color=cyan').classes('w-full')

        with ui.column().classes('flex-1 p-4 gap-4'):
            camera = ui.image('').classes('w-full max-w-[960px] aspect-video bg-slate-900 object-contain')
            with ui.scene(960, 420).classes('w-full max-w-[960px] bg-slate-900') as scene:
                scene.axes_helper(size=1.0)
                with scene.group() as robot_3d:
                    body = [[-0.28, -0.16], [0.22, -0.16], [0.32, 0.0], [0.22, 0.16], [-0.28, 0.16]]
                    scene.extrusion(body, 0.08).material('#38bdf8')

    def refresh_ui() -> None:
        now = time.monotonic()
        node = node_holder['node']
        if node is not None:
            node.encode_latest_frame()
        nav_status.text = state.nav_status
        pose_label.text = f'Pose: x={state.x:.2f}, y={state.y:.2f}, yaw={state.yaw:.2f}'
        odom_ok = now - state.odom_time < 1.0
        camera_ok = now - state.camera_time < 1.0
        depth_ok = now - state.depth_time < 1.0
        pipeline_label.text = f'Pipeline: odom={odom_ok} camera={camera_ok} depth={depth_ok}'
        if state.camera_image:
            camera.source = state.camera_image
        robot_3d.move(state.x, state.y, 0.0)
        robot_3d.rotate(0.0, 0.0, state.yaw)

    ui.timer(0.2, refresh_ui)


def main() -> None:
    pass


def ros_main() -> None:
    rclpy.init()
    node = RacecarNeoGuiNode(state)
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
ui.run(host='0.0.0.0', port=8080, uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='R')
