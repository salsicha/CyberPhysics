#!/usr/bin/env python3
"""RACECAR Turret Isaac evaluation run (systems/racecar_turret/interfaces.md,
"Isaac evaluation"; tools/racecar_turret_isaac_test.sh runs it).

The `evaluation` service of compositions/racecar_turret_isaac.yaml, on
simulation time:

1. waits for the simulator's /simulation/truth, the tracker's
   /turret/status and segments, and Nav2's lifecycle manager;
2. selects the scenario's target as an operator does in the UI: the
   /turret/yolo/segments box that contains the target's centroid, projected
   into the turret camera from the truth pose and /turret/camera/camera_info,
   locked with {"action": "lock", "target_id": id} on /turret/command (a
   point lock when the segments carry no ids), again until /turret/status
   reports that target;
3. drives the mission's course with NavigateThroughPoses: each goal holds
   the course's next poses inside the map SLAM has built so far, replaced
   by a longer one as the map grows (drive_course); meanwhile it acts as the
   operator watching the turret image: when the turret has been more than
   10 degrees off the target for --reselect-after-s while the target is in
   sight and within the turret's reach (the tracker lost it or follows
   another person), it steers the turret toward it with `manual` commands
   (the UI's joystick, 10 Hz) until the target is in the image, selects it
   there again the same way and records a `reselected` event;
4. records /simulation/truth, /turret/status, /turret/joint_states, the
   turret commands and detections, /odom, the SLAM pose (map -> base_link),
   Nav2's feedback and the result as JSON lines (`t`: simulation seconds)
   for tools/racecar_turret_scorer.py.

Exit status: 0 recorded a course (whatever its result), 2 the stack never
became ready, 3 the target could not be selected.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

TRUTH_TOPIC = '/simulation/truth'
STATUS_TOPIC = '/turret/status'
COMMAND_TOPIC = '/turret/command'
SEGMENTS_TOPIC = '/turret/yolo/segments'
GOAL_STATUS = {0: 'unknown', 1: 'accepted', 2: 'executing', 3: 'canceling',
               4: 'succeeded', 5: 'canceled', 6: 'aborted'}


# --- pure helpers (tests/test_racecar_turret_isaac.py) ----------------------

def _sub(a, b):
    return [x - y for x, y in zip(a, b)]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def project_truth(truth, point, intrinsics):
    """Pixel (u, v) of a point (odom frame) in the turret camera of a truth
    message (position, forward, up of turret/camera_link), or None when it
    is behind the camera. intrinsics: (fx, fy, cx, cy)."""
    camera = truth['turret_camera']
    forward, up = camera['forward'], camera['up']
    right = _cross(forward, up)
    delta = _sub(point, camera['position'])
    depth = _dot(delta, forward)
    if depth <= 1e-6:
        return None
    fx, fy, cx, cy = intrinsics
    return cx + fx * _dot(delta, right) / depth, cy - fy * _dot(delta, up) / depth


def segment_detections(payload):
    """(stamp or None, detections) of a /turret/yolo/segments message: a
    list of detections, or an object with them under 'detections' (or
    'segments') and the image 'stamp'."""
    if isinstance(payload, list):
        detections = [d for d in payload if isinstance(d, dict)]
        stamps = [d.get('stamp') for d in detections if isinstance(d.get('stamp'), (int, float))]
        return (float(stamps[0]) if stamps else None), detections
    if isinstance(payload, dict):
        detections = payload.get('detections', payload.get('segments', []))
        stamp = payload.get('stamp')
        if isinstance(stamp, dict):
            stamp = float(stamp.get('sec', 0)) + float(stamp.get('nanosec', 0)) * 1e-9
        return (float(stamp) if isinstance(stamp, (int, float)) else None,
                [d for d in detections if isinstance(d, dict)])
    return None, []


def box_contains(detection, u, v):
    """Whether pixel (u, v) is inside a detection's bbox_xywh (centre, size)."""
    box = detection.get('bbox_xywh')
    if not box or len(box) != 4:
        return False
    x, y, w, h = (float(value) for value in box)
    return abs(u - x) <= w / 2.0 and abs(v - y) <= h / 2.0


def select_detection(detections, u, v, target_class='person'):
    """The detection of target_class whose box contains (u, v), the one
    with the nearest centre when several do; None when none does (the UI
    then says so, contract "Turret commands")."""
    candidates = [d for d in detections
                  if d.get('class') == target_class and box_contains(d, u, v)]
    if not candidates:
        return None
    return min(candidates, key=lambda d: math.hypot(float(d['bbox_xywh'][0]) - u,
                                                    float(d['bbox_xywh'][1]) - v))


def lock_command(detection, u, v, width, height):
    """/turret/command JSON locking a detection: by its id when it has one,
    otherwise by the normalised point (contract "Turret commands")."""
    if detection is not None and str(detection.get('id', '')) != '':
        return {'action': 'lock', 'target_id': str(detection['id'])}
    return {'action': 'lock', 'point': [round(u / width, 4), round(v / height, 4)]}


def target_of(truth, name):
    for target in truth.get('targets', []):
        if target.get('name') == name:
            return target
    return None


def mission_poses(mission_file, mission_id):
    """(frame_id, [(x, y, yaw)]) of a run_nav2_mission.py mission."""
    data = json.loads(Path(mission_file).read_text(encoding='utf-8'))
    missions = {mission['id']: mission for mission in data.get('missions', [])}
    if mission_id not in missions:
        raise SystemExit(f'Mission {mission_id!r} not found in {mission_file}')
    mission = missions[mission_id]
    frame = mission.get('frame_id', data.get('default_frame_id', 'map'))
    poses = [tuple(float(v) for v in waypoint['pose'])
             for _ in range(max(1, int(mission.get('laps', 1))))
             for waypoint in mission.get('waypoints', [])]
    return frame, poses


# --- ROS node ---------------------------------------------------------------

def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def make_node(args, recorder):
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from nav2_msgs.action import NavigateThroughPoses
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import CameraInfo, JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener

    latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

    class Evaluator(Node):
        def __init__(self):
            super().__init__('racecar_turret_evaluator',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.truth = None
            self.status = None
            self.segments = (None, [])
            self.camera_info = None
            self.lock_info = {}
            self.recording = False
            self.odom_count = 0
            self.create_subscription(String, TRUTH_TOPIC, self._truth, 50)
            self.create_subscription(String, STATUS_TOPIC, self._status, latched)
            self.create_subscription(String, SEGMENTS_TOPIC, self._segments, 10)
            self.create_subscription(JointState, '/turret/joint_states', self._joints, 50)
            self.create_subscription(JointState, '/turret/joint_commands', self._joint_commands, 50)
            self.create_subscription(CameraInfo, '/turret/camera/camera_info', self._camera_info, 5)
            self.create_subscription(Odometry, '/odom', self._odom, 50)
            self.map_bounds = None
            self.create_subscription(OccupancyGrid, '/map', self._map, latched)
            self.command_pub = self.create_publisher(String, COMMAND_TOPIC, 10)
            self.nav2_active = self.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
            self.navigate = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(0.5, self._slam_pose)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def record(self, topic, data):
            if self.recording:
                recorder.write(json.dumps({'t': round(self.now(), 4), 'topic': topic, 'data': data},
                                          separators=(',', ':')) + '\n')

        def event(self, name, **fields):
            recorder.write(json.dumps({'t': round(self.now(), 4), 'wall': round(time.time(), 3),
                                       'event': name, **fields}, separators=(',', ':')) + '\n')
            recorder.flush()
            self.get_logger().info(f'{name}: {fields}')

        def _truth(self, msg):
            self.truth = json.loads(msg.data)
            self.record(TRUTH_TOPIC, self.truth)

        def _status(self, msg):
            try:
                self.status = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            self.record(STATUS_TOPIC, self.status)

        def _segments(self, msg):
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            self.segments = segment_detections(payload)
            self.record(SEGMENTS_TOPIC, {'stamp': self.segments[0], 'detections': [
                {key: d.get(key) for key in ('id', 'class', 'confidence', 'bbox_xywh', 'selected')}
                for d in self.segments[1]]})

        def _joints(self, msg):
            self.record('/turret/joint_states', {
                'stamp': stamp_seconds(msg.header.stamp), 'name': list(msg.name),
                'position': [round(float(v), 5) for v in msg.position]})

        def _joint_commands(self, msg):
            self.record('/turret/joint_commands', {
                'stamp': stamp_seconds(msg.header.stamp), 'name': list(msg.name),
                'position': [round(float(v), 5) for v in msg.position]})

        def _camera_info(self, msg):
            self.camera_info = msg

        def _map(self, msg):
            info = msg.info
            self.map_bounds = (info.origin.position.x, info.origin.position.y,
                               info.width * info.resolution, info.height * info.resolution)
            self.record('map', {'bounds': [round(v, 3) for v in self.map_bounds]})

        def _odom(self, msg):
            self.odom_count += 1
            if self.odom_count % 10 == 0:  # 6 Hz of the simulator's 60
                pose = msg.pose.pose
                self.record('/odom', {
                    'stamp': stamp_seconds(msg.header.stamp), 'x': round(pose.position.x, 4),
                    'y': round(pose.position.y, 4),
                    'yaw': round(2.0 * math.atan2(pose.orientation.z, pose.orientation.w), 5)})

        def _slam_pose(self):
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            except TransformException:
                return
            q = transform.transform.rotation
            self.record('slam_pose', {
                'stamp': stamp_seconds(transform.header.stamp),
                'x': round(transform.transform.translation.x, 4),
                'y': round(transform.transform.translation.y, 4),
                'yaw': round(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)), 5)})

        def intrinsics(self):
            k = self.camera_info.k
            return float(k[0]), float(k[4]), float(k[2]), float(k[5])

        def send_command(self, command):
            msg = String()
            msg.data = json.dumps(command)
            self.command_pub.publish(msg)

        def pose_stamped(self, frame, pose):
            from geometry_msgs.msg import PoseStamped

            x, y, yaw = pose
            msg = PoseStamped()
            msg.header.frame_id = frame
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y = x, y
            msg.pose.orientation.z = math.sin(0.5 * yaw)
            msg.pose.orientation.w = math.cos(0.5 * yaw)
            return msg

    return Evaluator()


def spin_for(node, seconds):
    import rclpy

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))


def wait_until(node, condition, timeout_s, period_s=0.2):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if condition():
            return True
        spin_for(node, period_s)
    return condition()


def nav2_is_active(node):
    from std_srvs.srv import Trigger
    import rclpy

    if not node.nav2_active.service_is_ready():
        return False
    future = node.nav2_active.call_async(Trigger.Request())
    end = time.monotonic() + 2.0
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return bool(future.done() and future.result() is not None and future.result().success)


def lock_on_target(node, args, margin_px=0):
    """Send the lock an operator would: the target's box in the turret
    image (its centroid projected from the truth, at least margin_px inside
    the image). Returns the command, or None when the target is not in the
    image or not detected."""
    truth, (_, detections) = node.truth, node.segments
    target = target_of(truth, args.target) if truth else None
    # A lock that matches no detection leaves the tracker as it was.
    if target is None or node.camera_info is None or not (node.status or {}).get('detections'):
        return None
    pixel = project_truth(truth, target['centroid'], node.intrinsics())
    width, height = node.camera_info.width, node.camera_info.height
    if pixel is None or not (margin_px <= pixel[0] < width - margin_px
                             and margin_px <= pixel[1] < height - margin_px):
        return None
    detection = select_detection(detections, *pixel)
    if detection is None:
        return None
    command = lock_command(detection, *pixel, width, height)
    node.send_command(command)
    node.lock_info = {'pixel': [round(v, 1) for v in pixel], 'detection': detection,
                      'command': command}
    return command


def select_target(node, args):
    """Lock the scenario's target as the UI does; True once /turret/status
    reports it."""
    deadline = time.monotonic() + args.selection_timeout_s
    attempt = 0
    while time.monotonic() < deadline:
        spin_for(node, 0.5)
        command = lock_on_target(node, args)
        if command is None:
            continue
        attempt += 1
        node.event('lock_sent', attempt=attempt, target=args.target, **node.lock_info)

        def locked():
            status = node.status or {}
            expected = command.get('target_id')
            return (status.get('mode') == 'auto'
                    and (expected is None or str(status.get('target_id')) == expected))

        if wait_until(node, locked, args.lock_confirm_s):
            node.event('selected', target=args.target, status=node.status)
            return True
    return False


PAN_LIMIT = 1.5708
TILT_LIMITS = (-0.65, 0.75)


def sight_angles(truth, point):
    """(right, up) angles (rad) of a point (odom frame) from the turret
    camera's optical axis: positive right and up, as pan and tilt turn."""
    camera = truth['turret_camera']
    forward, up = camera['forward'], camera['up']
    right = _cross(forward, up)
    delta = _sub(point, camera['position'])
    ahead, across, above = _dot(delta, forward), _dot(delta, right), _dot(delta, up)
    return math.atan2(across, ahead), math.atan2(above, math.hypot(ahead, across))


def reachable(truth, target):
    """Whether pan and tilt can point the turret camera at the target."""
    dx, dy, dz = _sub(target['centroid'], truth['turret_camera']['position'])
    bearing = math.atan2(dy, dx) - float(truth['vehicle']['yaw'])
    pan = -math.atan2(math.sin(bearing), math.cos(bearing))
    return abs(pan) <= PAN_LIMIT and TILT_LIMITS[0] <= math.atan2(dz, math.hypot(dx, dy)) <= TILT_LIMITS[1]


def lock_trouble(truth, status, target_name, on_target_deg=5.0, off_target_deg=10.0):
    """Why an operator watching the turret image would take over, or None:
    the target is in sight and within the turret's reach, but the turret is
    more than off_target_deg off it, and the tracker is not tracking,
    reports it lost, or aims at another person instead."""
    target = target_of(truth, target_name) if truth else None
    if target is None or not target.get('visible') or not reachable(truth, target):
        return None

    def error(person):
        right, up = sight_angles(truth, person['centroid'])
        return math.degrees(math.acos(max(-1.0, min(1.0, math.cos(right) * math.cos(up)))))

    if error(target) <= off_target_deg:
        return None
    status = status or {}
    if status.get('mode') != 'auto':
        return 'not tracking'
    others = [error(person) for person in truth.get('targets', [])
              if person.get('name') != target_name and person.get('visible')]
    if others and min(others) < on_target_deg:
        return 'on another person'
    if not status.get('locked'):
        return 'lost'
    return None


def steer_rates(truth, point, gain=2.0, limit=1.0):
    """The UI joystick's pan and tilt rates (rad/s) that turn the turret
    camera toward a point."""
    right, up = sight_angles(truth, point)
    return (max(-limit, min(limit, gain * right)), max(-limit, min(limit, gain * up)))


def known_prefix(poses, grid, margin=0.3):
    """The leading poses inside the SLAM map's bounds (Nav2's planner
    rejects a goal outside the global costmap, which is the map's size)."""
    if grid is None:
        return []
    x0, y0, size_x, size_y = grid
    count = 0
    for x, y, _ in poses:
        if not (x0 + margin <= x <= x0 + size_x - margin and y0 + margin <= y <= y0 + size_y - margin):
            break
        count += 1
    return poses[:count]


def drive_course(node, frame, poses, args):
    """Drive the course with NavigateThroughPoses. The map SLAM builds
    covers only what the lidar has seen, so each goal holds the course's
    next poses inside it; while the car drives and the map grows, the goal
    is replaced with the longer list (Nav2 pre-empts the running goal), and
    after a goal succeeds the next one is sent. Records goal_sent (the
    course's start), each goal and the course's result."""
    from nav2_msgs.action import NavigateThroughPoses

    index = 0  # first pose not reached yet
    start = None
    active = None  # [sequence, first pose index, count, handle future, result future]
    sequence = 0
    last_sent = -math.inf
    progress = {}
    recorded = {}

    def feedback_of(number):
        def callback(feedback):
            fields = feedback.feedback
            progress[number] = int(getattr(fields, 'number_of_poses_remaining', -1))
            if node.now() - recorded.get(number, -math.inf) < 0.5:
                return  # recorded at 2 Hz
            recorded[number] = node.now()
            data = {'goal': number,
                    'distance_remaining': round(float(fields.distance_remaining), 3),
                    'recoveries': int(fields.number_of_recoveries),
                    'poses_remaining': int(getattr(fields, 'number_of_poses_remaining', -1))}
            node.record('nav2_feedback', data)
        return callback

    trouble_since = None
    takeover = None  # (reason, started) while the operator steers the turret
    while True:
        spin_for(node, 0.1)
        now = node.now()
        # The operator (see the module's docstring): after reselect_after_s
        # off the target, steer the turret toward it with the joystick
        # until it is in the image, then select it there again.
        trouble = lock_trouble(node.truth, node.status, args.target)
        if takeover is not None:
            target = target_of(node.truth, args.target)
            if lock_on_target(node, args, margin_px=40) is not None:
                node.event('reselected', reason=takeover[0], steered_s=round(now - takeover[1], 2),
                           **node.lock_info)
                takeover, trouble_since = None, None
            elif now - takeover[1] > 10.0 or target is None or not target.get('visible'):
                takeover, trouble_since = None, None
            else:
                pan_rate, tilt_rate = steer_rates(node.truth, target['centroid'])
                node.send_command({'action': 'manual', 'pan_rate': round(pan_rate, 3),
                                   'tilt_rate': round(tilt_rate, 3)})
        elif trouble is None or start is None:
            trouble_since = None
        elif trouble_since is None:
            trouble_since = now
        elif args.reselect_after_s > 0 and now - trouble_since >= args.reselect_after_s:
            takeover = (trouble, now)
        if start is not None and now - start > args.course_timeout_s:
            if active is not None and active[3].done() and active[3].result() is not None:
                active[3].result().cancel_goal_async()
            node.event('result', status=-1, status_name='timeout', duration_s=round(now - start, 3),
                       reached=index)
            return 0
        if active is not None:
            number, first, count, handle_future, result_future = active
            if progress.get(number, -1) >= 0:
                index = max(index, first + count - progress[number])
            if handle_future.done() and result_future is None:
                handle = handle_future.result()
                if handle is None or not handle.accepted:
                    node.event('result', status=0, status_name='rejected', reached=index)
                    return 0
                active[4] = result_future = handle.get_result_async()
            if result_future is not None and result_future.done():
                outcome = result_future.result()
                status = int(getattr(outcome, 'status', 0))
                if status == 4 and first + count >= len(poses):
                    node.event('result', status=4, status_name='succeeded',
                               duration_s=round(now - start, 3), goals=sequence)
                    spin_for(node, 1.0)
                    return 0
                if status != 4:
                    node.event('result', status=status, status_name=GOAL_STATUS.get(status, str(status)),
                               duration_s=round(now - start, 3), reached=index,
                               error_code=int(getattr(outcome.result, 'error_code', 0)))
                    return 0
                index = first + count
                active = None
        known = known_prefix(poses[index:], node.map_bounds)
        sent_remaining = 0 if active is None else active[1] + active[2] - index
        if known and (active is None or (len(known) > sent_remaining and now - last_sent > 2.0)):
            sequence += 1
            goal = NavigateThroughPoses.Goal()
            goal.poses = [node.pose_stamped(frame, pose) for pose in known]
            if start is None:
                start = now
                node.event('goal_sent', frame=frame, poses=[list(p) for p in poses])
            node.event('goal', goal=sequence, first=index, count=len(known))
            active = [sequence, index, len(known),
                      node.navigate.send_goal_async(goal, feedback_callback=feedback_of(sequence)),
                      None]
            last_sent = now


def run(args, recorder):
    import rclpy

    rclpy.init()
    node = make_node(args, recorder)
    try:
        frame, poses = mission_poses(args.mission_file, args.mission_id)
        node.event('start', mission=args.mission_id, target=args.target, poses=len(poses),
                   label=args.label)
        ready = wait_until(node, lambda: node.truth is not None and node.status is not None
                           and node.camera_info is not None, args.setup_timeout_s, 1.0)
        ready = ready and wait_until(node, lambda: nav2_is_active(node), args.setup_timeout_s, 2.0)
        ready = ready and node.navigate.wait_for_server(timeout_sec=60.0)
        if not ready:
            node.event('error', reason='stack not ready', truth=node.truth is not None,
                       status=node.status is not None, camera_info=node.camera_info is not None)
            return 2
        node.event('ready', status=node.status, intrinsics=node.intrinsics(),
                   image=[node.camera_info.width, node.camera_info.height])
        node.recording = True
        if not select_target(node, args):
            node.event('error', reason='target not selected', status=node.status)
            return 3
        return drive_course(node, frame, poses, args)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', required=True, help='recording (JSON lines)')
    parser.add_argument('--mission-file',
                        default='/workspace/systems/racecar_turret/missions/isaac_course.json')
    parser.add_argument('--mission-id', default='arena_lap')
    parser.add_argument('--target', default='person_1', help='scenario name of the target')
    parser.add_argument('--label', default='', help='run label (e.g. compensated)')
    parser.add_argument('--setup-timeout-s', type=float, default=900.0, help='wall clock')
    parser.add_argument('--selection-timeout-s', type=float, default=180.0, help='wall clock')
    parser.add_argument('--lock-confirm-s', type=float, default=5.0, help='wall clock')
    parser.add_argument('--course-timeout-s', type=float, default=900.0, help='simulation time')
    parser.add_argument('--reselect-after-s', type=float, default=3.0,
                        help='simulation time off the target in view before the operator '
                             'selects it again (0: never)')
    args = parser.parse_args(argv)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as recorder:
        return run(args, recorder)


if __name__ == '__main__':
    sys.exit(main())
