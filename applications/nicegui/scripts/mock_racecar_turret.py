#!/usr/bin/env python3
"""Fake RACECAR Turret ROS graph for the web UI (main.py), no robot needed.

Publishes what the page shows: a /map of a walled room (transient-local),
TF map -> odom -> base_link of a car that drives where it is told, /plan,
/odom, front-camera and depth heartbeats, /racecarneo/control_status
(unless --no-control-status, when the real cmd_vel_to_ackermann runs
alongside), the turret camera (/turret/camera/image_raw and camera_info,
640x480 rgb8 at 15 Hz, with people drawn as coloured figures),
/turret/yolo/segments for them with stable ids, /turret/status
(transient-local) and /turret/actuator_ready.

It acts on what the page sends (systems/racecar_turret/interfaces.md):
NavigateToPose goals (an action server that drives straight to the goal,
with feedback and cancel), /cmd_vel_teleop (priority, 0.5 s timeout),
/racecarneo/stop_request, /turret/command (lock by id or point, release,
manual, center) on a simulated pan/tilt turret whose view the people move
through, and /turret/emergency_stop (engaged until a false arrives, like
the tracker). --command-log FILE appends every command received as a JSON
line, for scripted checks; --static keeps the people still.

    python3 mock_racecar_turret.py [--command-log FILE] [--no-control-status] [--static]
"""
import argparse
import json
import math
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
WIDTH, HEIGHT, HFOV = 640, 480, math.radians(70.0)
FX = WIDTH / 2.0 / math.tan(HFOV / 2.0)
PAN_LIMIT, TILT_MIN, TILT_MAX, MAX_RATE = 1.5708, -0.65, 0.75, 1.5
WHEELBASE, MAX_SPEED = 0.33, 0.35
# People around the car: azimuth (rad, positive right of the car's heading),
# elevation of the box centre (rad), distance (m), drift (rad/s), colour.
PEOPLE = [
    {'id': '3', 'class': 'person', 'az': -0.25, 'el': 0.0, 'dist': 4.0, 'drift': 0.05, 'rgb': (230, 80, 60)},
    {'id': '7', 'class': 'person', 'az': 0.15, 'el': -0.02, 'dist': 6.0, 'drift': -0.03, 'rgb': (70, 160, 240)},
    {'id': '12', 'class': 'dog', 'az': 0.45, 'el': -0.12, 'dist': 3.0, 'drift': 0.04, 'rgb': (240, 200, 60)},
]


class MockRacecarTurret(Node):
    def __init__(self, args):
        super().__init__('mock_racecar_turret')
        self.args = args
        self.lock = threading.Lock()
        self.log = open(args.command_log, 'a', buffering=1) if args.command_log else None
        # Car
        self.x, self.y, self.yaw, self.speed, self.steering = 0.0, 0.0, 0.0, 0.0, 0.0
        self.teleop = None          # (time, linear, angular)
        self.stopped = False
        self.goal = None            # (x, y, yaw) of the active NavigateToPose goal
        # Turret
        self.pan = self.tilt = 0.0
        self.estop = True
        self.mode = 'estop'
        self.target_id = ''
        self.manual = None          # (time, pan_rate, tilt_rate)
        self.centering = False
        self.message = 'waiting for the emergency stop to be released'
        self.people = [dict(person) for person in PEOPLE]
        self.detections = []

        group = ReentrantCallbackGroup()
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.plan_pub = self.create_publisher(NavPath, '/plan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/color/image_raw', 5)
        self.depth_pub = self.create_publisher(Image, '/depth_anything/depth/image', 5)
        self.control_pub = self.create_publisher(String, '/racecarneo/control_status', 10)
        self.turret_image_pub = self.create_publisher(Image, '/turret/camera/image_raw', 5)
        self.turret_info_pub = self.create_publisher(CameraInfo, '/turret/camera/camera_info', 5)
        self.segments_pub = self.create_publisher(String, '/turret/yolo/segments', 10)
        self.status_pub = self.create_publisher(String, '/turret/status', LATCHED)
        self.joint_pub = self.create_publisher(JointState, '/turret/joint_states', 10)
        self.ready_pub = self.create_publisher(Bool, '/turret/actuator_ready', LATCHED)

        self.create_subscription(Twist, '/cmd_vel_teleop', self.on_teleop, 10)
        for qos in (LATCHED, 10):
            self.create_subscription(Bool, '/racecarneo/stop_request', self.on_stop_request, qos)
            self.create_subscription(Bool, '/turret/emergency_stop', self.on_estop, qos)
        self.create_subscription(String, '/turret/command', self.on_turret_command, 10)
        self.nav_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', execute_callback=self.execute_goal,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=self.on_cancel, callback_group=group)

        self.publish_static()
        self.publish_map()
        self.ready_pub.publish(Bool(data=True))
        self.create_timer(0.05, self.step_car)
        self.create_timer(1.0 / 15.0, self.step_turret)
        self.create_timer(0.2, self.publish_slow)
        self.create_timer(0.5, self.publish_control_status)
        self.create_timer(1.0, self.publish_plan)
        self.create_timer(5.0, self.publish_map)       # as SLAM Toolbox's map updates
        print('mock racecar turret publisher ready', flush=True)

    def record(self, topic, data):
        if self.log is not None:
            self.log.write(json.dumps({'t': time.time(), 'topic': topic, 'data': data}) + '\n')

    # --- Car ---------------------------------------------------------------

    def publish_static(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)

    def publish_map(self):
        resolution, width, height = 0.05, 320, 200
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:4, :] = grid[-4:, :] = grid[:, :4] = grid[:, -4:] = 100
        grid[60:80, 120:140] = 100                      # a box obstacle
        grid[130:134, 200:290] = 100                    # a low wall
        grid[150:, :40] = -1                            # an unexplored corner
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = resolution
        msg.info.width, msg.info.height = width, height
        msg.info.origin.position.x, msg.info.origin.position.y = -3.0, -5.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self.map_pub.publish(msg)

    def on_teleop(self, msg):
        self.record('/cmd_vel_teleop', {'linear_x': msg.linear.x, 'angular_z': msg.angular.z})
        with self.lock:
            self.teleop = (time.monotonic(), msg.linear.x, msg.angular.z)

    def on_stop_request(self, msg):
        self.record('/racecarneo/stop_request', bool(msg.data))
        self.stopped = bool(msg.data)

    def on_cancel(self, _goal_handle):
        self.record('navigate_to_pose/cancel', True)
        return CancelResponse.ACCEPT

    def execute_goal(self, goal_handle):
        pose = goal_handle.request.pose.pose
        goal = (pose.position.x, pose.position.y, 2.0 * math.atan2(pose.orientation.z, pose.orientation.w))
        self.record('navigate_to_pose/goal', {'x': goal[0], 'y': goal[1], 'yaw': goal[2],
                                              'frame_id': goal_handle.request.pose.header.frame_id})
        with self.lock:
            self.goal = goal
        result = NavigateToPose.Result()
        while rclpy.ok():
            with self.lock:
                if self.goal is not goal:       # replaced by a newer goal
                    goal_handle.abort()
                    return result
                distance = math.hypot(goal[0] - self.x, goal[1] - self.y)
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.goal = None
                goal_handle.canceled()
                return result
            if distance < 0.15:
                with self.lock:
                    self.goal = None
                goal_handle.succeed()
                return result
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = float(distance)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)
        return result

    def source(self, now):
        if self.teleop is not None and now - self.teleop[0] <= 0.5:
            return 'teleop'
        return 'nav2' if self.goal is not None else 'none'

    def step_car(self):
        dt, now = 0.05, time.monotonic()
        with self.lock:
            source = self.source(now)
            if source == 'teleop':
                speed, yaw_rate = self.teleop[1], self.teleop[2]
            elif source == 'nav2':
                heading = math.atan2(self.goal[1] - self.y, self.goal[0] - self.x)
                error = math.atan2(math.sin(heading - self.yaw), math.cos(heading - self.yaw))
                speed = 0.3 if abs(error) < 1.2 else 0.1
                yaw_rate = max(-1.0, min(1.0, 1.5 * error))
            else:
                speed, yaw_rate = 0.0, 0.0
            if self.stopped:
                speed = 0.0
            speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
            self.speed = speed
            if abs(speed) > 1e-3:
                self.steering = max(-0.42, min(0.42, math.atan(yaw_rate * WHEELBASE / speed)))
                yaw_rate = speed * math.tan(self.steering) / WHEELBASE
            else:
                yaw_rate = 0.0
            self.yaw += yaw_rate * dt
            self.x += speed * math.cos(self.yaw) * dt
            self.y += speed * math.sin(self.yaw) * dt
            x, y, yaw = self.x, self.y, self.yaw
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = x, y
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x = self.speed
        self.odom_pub.publish(odom)

    def publish_plan(self):
        with self.lock:
            goal, x, y = self.goal, self.x, self.y
        if goal is None:
            return
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for i in range(21):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x + (goal[0] - x) * i / 20
            pose.pose.position.y = y + (goal[1] - y) * i / 20
            path.poses.append(pose)
        self.plan_pub.publish(path)

    def publish_control_status(self):
        if self.args.no_control_status:
            return
        with self.lock:
            status = {'source': self.source(time.monotonic()), 'stopped': self.stopped,
                      'speed_mps': self.speed, 'steering_rad': self.steering}
        self.control_pub.publish(String(data=json.dumps(status)))

    def publish_slow(self):
        stamp = self.get_clock().now().to_msg()
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        frame[:120] = (120, 160, 200)
        frame[120:] = (90, 90, 80)
        cv2.putText(frame, f'front camera {time.strftime("%H:%M:%S")}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        self.camera_pub.publish(self.image(frame, stamp, 'racecarneo/camera_color_optical_frame'))
        depth = Image()
        depth.header.stamp, depth.header.frame_id = stamp, 'racecarneo/camera_color_optical_frame'
        depth.height, depth.width, depth.encoding, depth.step = 24, 32, '32FC1', 32 * 4
        depth.data = np.full((24, 32), 3.0, dtype=np.float32).tobytes()
        self.depth_pub.publish(depth)
        self.publish_turret_status()

    @staticmethod
    def image(frame, stamp, frame_id):
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding, msg.step = 'rgb8', frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg

    # --- Turret ------------------------------------------------------------

    def on_estop(self, msg):
        self.record('/turret/emergency_stop', bool(msg.data))
        with self.lock:
            self.estop = bool(msg.data)
            if self.estop:
                self.mode, self.manual, self.centering = 'estop', None, False
                self.message = 'emergency stop engaged'
            elif self.mode == 'estop':
                self.mode = 'auto' if self.target_id else 'idle'
                self.message = ''

    def on_turret_command(self, msg):
        try:
            command = json.loads(msg.data)
        except ValueError:
            command = None
        self.record('/turret/command', command)
        if not isinstance(command, dict):
            return
        action = command.get('action')
        with self.lock:
            if action == 'manual':
                self.manual = (time.monotonic(), float(command.get('pan_rate', 0.0)),
                               float(command.get('tilt_rate', 0.0)))
                self.target_id, self.centering = '', False
                self.set_mode('manual')
            elif action == 'lock':
                self.manual, self.centering = None, False
                if 'target_id' in command:
                    self.target_id = str(command['target_id'])
                    self.message = ''
                else:
                    u, v = command.get('point', (0.5, 0.5))
                    hit = self.detection_at(u * WIDTH, v * HEIGHT)
                    self.target_id = hit['id'] if hit else ''
                    self.message = '' if hit else 'no detection at that point'
                self.set_mode('auto' if self.target_id else 'idle')
            elif action in ('release', 'center'):
                self.target_id, self.manual = '', None
                self.centering = action == 'center'
                self.message = ''
                self.set_mode('idle')

    def set_mode(self, mode):
        self.mode = 'estop' if self.estop else mode

    def detection_at(self, x, y):
        hits = [d for d in self.detections
                if abs(x - d['bbox_xywh'][0]) <= d['bbox_xywh'][2] / 2
                and abs(y - d['bbox_xywh'][1]) <= d['bbox_xywh'][3] / 2]
        return min(hits, key=lambda d: math.hypot(x - d['bbox_xywh'][0], y - d['bbox_xywh'][1]), default=None)

    def step_turret(self):
        dt, now = 1.0 / 15.0, time.monotonic()
        with self.lock:
            if not self.args.static:
                for person in self.people:
                    person['az'] += person['drift'] * dt
                    if abs(person['az']) > 1.2:
                        person['drift'] = -person['drift']
            target = next((d for d in self.detections if d['id'] == self.target_id), None)
            if not self.estop:
                if self.manual is not None and now - self.manual[0] <= 0.5:
                    self.pan += max(-MAX_RATE, min(MAX_RATE, self.manual[1])) * dt
                    self.tilt += max(-MAX_RATE, min(MAX_RATE, self.manual[2])) * dt
                elif self.mode == 'auto' and target is not None:
                    # Centre the box, as the tracker does.
                    u, v = target['bbox_xywh'][:2]
                    self.pan += max(-MAX_RATE * dt, min(MAX_RATE * dt, 0.4 * math.atan((u - WIDTH / 2) / FX)))
                    self.tilt += max(-MAX_RATE * dt, min(MAX_RATE * dt, 0.4 * math.atan((HEIGHT / 2 - v) / FX)))
                elif self.centering:
                    self.pan -= max(-MAX_RATE * dt, min(MAX_RATE * dt, 0.5 * self.pan))
                    self.tilt -= max(-MAX_RATE * dt, min(MAX_RATE * dt, 0.5 * self.tilt))
                self.pan = max(-PAN_LIMIT, min(PAN_LIMIT, self.pan))
                self.tilt = max(TILT_MIN, min(TILT_MAX, self.tilt))
            pan, tilt, people = self.pan, self.tilt, [dict(p) for p in self.people]
        stamp = self.get_clock().now().to_msg()
        frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        horizon = int(HEIGHT / 2 + FX * math.tan(tilt))
        frame[:max(0, min(HEIGHT, horizon))] = (150, 190, 230)
        frame[max(0, min(HEIGHT, horizon)):] = (110, 120, 90)
        for column in range(-6, 7):     # ground markers so panning is visible
            u = WIDTH / 2 + FX * math.tan(column * 0.25 - pan)
            if 0 <= u < WIDTH and abs(column * 0.25 - pan) < 1.4:
                cv2.line(frame, (int(u), max(0, horizon)), (int(u), HEIGHT), (90, 100, 70), 2)
        detections = []
        for person in sorted(people, key=lambda p: -p['dist']):
            relative = person['az'] - pan
            if abs(relative) > 1.3:
                continue
            u = WIDTH / 2 + FX * math.tan(relative)
            v = HEIGHT / 2 - FX * math.tan(person['el'] - tilt)
            h = FX * (1.7 if person['class'] == 'person' else 0.6) / person['dist']
            w = h * (0.4 if person['class'] == 'person' else 1.4)
            if u + w / 2 < 0 or u - w / 2 > WIDTH or v + h / 2 < 0 or v - h / 2 > HEIGHT:
                continue
            x1, y1, x2, y2 = int(u - w / 2), int(v - h / 2), int(u + w / 2), int(v + h / 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), person['rgb'], -1)
            cv2.circle(frame, (int(u), int(y1 - w * 0.35)), max(2, int(w * 0.35)), person['rgb'], -1)
            # The box as a detector would report it, clipped to the image.
            bx1, by1 = max(0, x1), max(0, int(y1 - w * 0.7))
            bx2, by2 = min(WIDTH, x2), min(HEIGHT, y2)
            detections.append({'id': person['id'], 'class': person['class'], 'confidence': 0.87,
                               'bbox_xywh': [(bx1 + bx2) / 2, (by1 + by2) / 2, bx2 - bx1, by2 - by1]})
        cv2.circle(frame, (WIDTH // 2, HEIGHT // 2), 6, (255, 255, 255), 1)
        with self.lock:
            self.detections = detections
        self.turret_image_pub.publish(self.image(frame, stamp, 'turret/camera_optical_frame'))
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, 'turret/camera_optical_frame'
        info.width, info.height, info.distortion_model = WIDTH, HEIGHT, 'plumb_bob'
        info.k = [FX, 0.0, WIDTH / 2, 0.0, FX, HEIGHT / 2, 0.0, 0.0, 1.0]
        info.p = [FX, 0.0, WIDTH / 2, 0.0, 0.0, FX, HEIGHT / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = [0.0] * 5
        self.turret_info_pub.publish(info)
        # The tracker's form: each detection carries the image stamp and size.
        seconds = stamp.sec + stamp.nanosec * 1e-9
        self.segments_pub.publish(String(data=json.dumps([
            {**d, 'selected': d['id'] == self.target_id, 'stamp': seconds,
             'image_width': WIDTH, 'image_height': HEIGHT} for d in detections])))
        joints = JointState()
        joints.header.stamp = stamp
        joints.name, joints.position = ['pan_joint', 'tilt_joint'], [pan, tilt]
        self.joint_pub.publish(joints)

    def publish_turret_status(self):
        with self.lock:
            target = next((d for d in self.detections if d['id'] == self.target_id), None)
            error = None
            if target is not None:
                du = target['bbox_xywh'][0] - WIDTH / 2
                dv = target['bbox_xywh'][1] - HEIGHT / 2
                error = math.degrees(math.atan(math.hypot(du, dv) / FX))
            target_class = next((p['class'] for p in self.people if p['id'] == self.target_id), '')
            status = {'mode': self.mode, 'target_id': self.target_id, 'target_class': target_class,
                      'locked': error is not None and error < 2.0,
                      'aim_error_deg': None if error is None else round(error, 2),
                      'base_compensation': True, 'detections': len(self.detections),
                      'message': self.message if self.message or target is not None or not self.target_id
                      else 'target not in view'}
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--command-log', help='append every command received to this JSON-lines file')
    parser.add_argument('--no-control-status', action='store_true',
                        help='leave /racecarneo/control_status to the real cmd_vel_to_ackermann')
    parser.add_argument('--static', action='store_true', help='keep the people still')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = MockRacecarTurret(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
