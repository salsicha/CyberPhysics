#!/usr/bin/env python3
"""Aerodrone object tracker: follows the ground object picked in the web UI
(systems/aerodrone/tracking.md).

Reads the detector's ``<prefix>/detections`` and ``<prefix>/camera_info``,
geolocates the selected track's box centre on the terrain in the camera
pose at the image stamp (/navigation/odometry, the camera mount, the DemNav
DEM; tracking_core.py), filters it, and flies the aircraft through
Aerostack2's FollowReference behaviour (as2_behaviors_motion 1.1.3):

- the behaviour sends its position reference (``earth``, the navigation
  frame) to the motion controller, which hands it to the BlueOS platform's
  POSITION/LOCAL_ENU mode, i.e. ArduPilot GUIDED position targets. If this
  node dies the behaviour keeps the last reference: the aircraft holds a
  point, it does not keep flying a velocity;
- the reference is updated through the behaviour's ``_behavior/modify``
  service at ``control_rate_hz``, moving at most ``max_speed_mps``;
- stopping the behaviour (cancel) makes it hover in place: ``stop`` and a
  target lost for ``lost_timeout_s``.

A selected track id that the detector replaces by a new one for the same
object is followed under the new id (tracking_core, "Track handover"); the
status then carries the new ``target_id``. The id is followed only while
its class is of the kind first seen (a car's id that the detector moves
onto a person is no longer the target).

Pre-emption: a track command stops a running GoToBehavior or
FollowPathBehavior (their ``_behavior/stop`` service; the map-goal tree's
GoTo then fails and ``restart: always`` starts the tree again for the next
goal). A map goal (``map_goal_topic``) or any of those behaviours, or
TakeoffBehavior or LandBehavior, starting (IDLE -> RUNNING on its
``_behavior/behavior_status``) ends tracking (state idle). A track command
is refused while TakeoffBehavior or LandBehavior runs, or while the
platform is not flying.

Publishes ``<prefix>/status`` (JSON, reliable + transient-local, 2 Hz and
on every change) and ``<prefix>/target`` (PointStamped, navigation frame).
"""
import math
import os
import sys
import threading
import time
import uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tracking_core as tc  # noqa: E402

PARAMETERS = {
    'namespace': 'aerodrone',
    'topic_prefix': '/tracking',
    'odometry_topic': '/navigation/odometry',
    'origin_topic': '/navigation/origin',
    'clock_topic': '/clock',
    'rangefinder_topic': '',
    'rangefinder_timeout_s': 0.5,
    'rangefinder_bias_hold_s': 30.0,
    'map_goal_topic': '/aerostack/map_goal',
    'camera_mount_yaw_deg': 0.0,
    'camera_stabilized': False,
    'dem_cache_dir': '/data/demnav_cache',
    'geoid_file': '/usr/share/GeographicLib/geoids/egm96-5.pgm',
    'control_rate_hz': 10.0,
    'status_rate_hz': 2.0,
    'max_attitude_offset_s': 0.25,
    'follow_behavior': 'FollowReferenceBehavior',
    'preempt_behaviors': 'GoToBehavior,FollowPathBehavior',
    'busy_behaviors': 'TakeoffBehavior,LandBehavior',
    'follow_retry_s': 2.0,
}
CONFIG_FIELDS = {name: value for name, value in
                 vars(tc.TrackerConfig()).items()}


def flag(value):
    """A boolean parameter that launch or compose may pass as a string."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def detection_class(detection):
    """A Detection2D's class name (its first hypothesis), or ''."""
    return str(detection.results[0].hypothesis.class_id) if detection.results else ''


def box_side(detection):
    """The longer side of a Detection2D's box, in pixels."""
    return max(float(detection.bbox.size_x), float(detection.bbox.size_y))


class Terrain:
    """Ground height in the navigation frame: the DEM (offset by the
    rangefinder where it reads), else flat ground at the rangefinder's
    height below the aircraft, else flat at the origin (home) height."""

    def __init__(self):
        self.dem = None
        self.bias = 0.0
        self.bias_time = None
        self.range_ground = None   # (up, time)

    def source(self, now, hold_s):
        if self.dem is not None:
            return 'dem'
        if self.range_ground is not None and now - self.range_ground[1] <= hold_s:
            return 'rangefinder'
        return 'flat'

    def height_fn(self, now, hold_s):
        bias = self.bias if (self.bias_time is not None and
                             now - self.bias_time <= hold_s) else 0.0
        dem = self.dem
        range_ground = self.range_ground
        flat = 0.0
        if range_ground is not None and now - range_ground[1] <= hold_s:
            flat = range_ground[0]

        def height(east, north):
            if dem is not None:
                value = dem.height(east, north)
                if value is not None:
                    return value + bias
            return flat
        return height


class Tracker:
    """The ROS side of TrackerCore."""

    def __init__(self, node):
        import rclpy
        from as2_msgs.action import FollowReference
        from as2_msgs.msg import BehaviorStatus, PlatformInfo
        from geographic_msgs.msg import GeoPointStamped
        from geometry_msgs.msg import PointStamped
        from nav_msgs.msg import Odometry
        from rcl_interfaces.msg import ParameterDescriptor
        from rclpy.action import ActionClient
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Range
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from vision_msgs.msg import Detection2DArray

        self.rclpy = rclpy
        self.node = node
        self.FollowReference = FollowReference
        self.BehaviorStatus = BehaviorStatus
        self.PointStamped = PointStamped
        self.String = String
        self.Trigger = Trigger
        # Compose passes 3 for 3.0 and "true" for true: accept either.
        dynamic = ParameterDescriptor(dynamic_typing=True)
        for name, default in {**PARAMETERS, **CONFIG_FIELDS}.items():
            node.declare_parameter(name, default, dynamic)
        param = lambda name: node.get_parameter(name).value  # noqa: E731
        values = {}
        for name, default in CONFIG_FIELDS.items():
            value = param(name)
            values[name] = (flag(value) if isinstance(default, bool) else
                            type(default)(value))
        self.config = tc.TrackerConfig(**values)
        self.core = tc.TrackerCore(self.config)
        self.ns = '/' + str(param('namespace')).strip('/')
        prefix = '/' + str(param('topic_prefix')).strip('/')
        self.mount_yaw = math.radians(float(param('camera_mount_yaw_deg')))
        self.stabilized = flag(param('camera_stabilized'))
        self.rangefinder_timeout_s = float(param('rangefinder_timeout_s'))
        self.bias_hold_s = float(param('rangefinder_bias_hold_s'))
        self.dem_cache_dir = str(param('dem_cache_dir'))
        self.follow_retry_s = float(param('follow_retry_s'))
        self.poses = tc.PoseHistory(max_offset_s=float(param('max_attitude_offset_s')))
        self.terrain = Terrain()
        self.lock = threading.RLock()
        self.intrinsics = None
        self.origin = None
        self.odom_frame = 'odom'
        self.latest_range = None
        self.platform = None
        self.behaviors = {}
        self.follow_handle = None
        self.follow_pending = False
        self.cancel_requested = False
        self.modify_future = None
        self.modify_sent = 0.0
        self.shaper = tc.ReferenceShaper(self.config)
        self.last_control = None
        self.last_status = None
        self.dem_entry = None
        self.dem_loading = False
        self.dem_checked = 0.0
        self.geoid = self._load_geoid(str(param('geoid_file')))

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = node.create_publisher(String, f'{prefix}/status', latched)
        self.target_pub = node.create_publisher(PointStamped, f'{prefix}/target', 10)
        node.create_subscription(String, f'{prefix}/command', self._on_command, 10)
        node.create_subscription(Detection2DArray, f'{prefix}/detections',
                                 self._on_detections, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'{prefix}/camera_info',
                                 self._on_camera_info, qos_profile_sensor_data)
        node.create_subscription(Odometry, str(param('odometry_topic')),
                                 self._on_odometry, qos_profile_sensor_data)
        node.create_subscription(GeoPointStamped, str(param('origin_topic')),
                                 self._on_origin, latched)
        if str(param('clock_topic')):
            node.create_subscription(Clock, str(param('clock_topic')),
                                     self._on_clock, qos_profile_sensor_data)
        if str(param('rangefinder_topic')):
            node.create_subscription(Range, str(param('rangefinder_topic')),
                                     self._on_range, qos_profile_sensor_data)
        if str(param('map_goal_topic')):
            node.create_subscription(String, str(param('map_goal_topic')),
                                     self._on_map_goal, qos_profile_sensor_data)
        node.create_subscription(PlatformInfo, f'{self.ns}/platform/info',
                                 self._on_platform, 10)
        self.preempt_names = [n for n in str(param('preempt_behaviors')).split(',') if n]
        self.busy_names = [n for n in str(param('busy_behaviors')).split(',') if n]
        self.stop_clients = {}
        for name in self.preempt_names + self.busy_names:
            node.create_subscription(
                BehaviorStatus, f'{self.ns}/{name}/_behavior/behavior_status',
                lambda msg, name=name: self._on_behavior(name, msg), 10)
        for name in self.preempt_names:
            self.stop_clients[name] = node.create_client(
                Trigger, f'{self.ns}/{name}/_behavior/stop')
        follow = str(param('follow_behavior'))
        self.follow_client = ActionClient(node, FollowReference, f'{self.ns}/{follow}')
        self.modify_client = node.create_client(
            FollowReference.Impl.SendGoalService, f'{self.ns}/{follow}/_behavior/modify')
        self.follow_stop_client = node.create_client(
            Trigger, f'{self.ns}/{follow}/_behavior/stop')
        node.create_timer(1.0 / float(param('control_rate_hz')), self._control)
        node.create_timer(1.0 / float(param('status_rate_hz')), self._publish_status)
        node.get_logger().info(
            f'tracker ready: {prefix}/command, behaviours in {self.ns}, '
            f'camera mount {math.degrees(self.mount_yaw):g} deg'
            f'{" (stabilised)" if self.stabilized else ""}, DEMs in {self.dem_cache_dir}')
        self._publish_status()

    # --- inputs ---------------------------------------------------------
    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _load_geoid(self, path):
        try:
            from demnav_pkg.geoid import GeoidGrid
            return GeoidGrid(path)
        except (ImportError, OSError, ValueError) as error:
            self.node.get_logger().warn(
                f'no geoid model ({error}): DEM heights are taken as ellipsoid '
                'heights (off by the local geoid height, about 30 m in California)')
            return None

    def _on_camera_info(self, msg):
        intrinsics = tc.intrinsics_from_camera_info(msg)
        if intrinsics is not None:
            self.intrinsics = intrinsics

    def _on_odometry(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self.lock:
            self.poses.add(stamp_ns(msg.header.stamp), (p.x, p.y, p.z),
                           (q.x, q.y, q.z, q.w))
            if msg.header.frame_id:
                self.odom_frame = msg.header.frame_id

    def _on_clock(self, msg):
        with self.lock:
            self.poses.add_clock(stamp_ns(msg.clock), time.time_ns())

    def _on_origin(self, msg):
        origin = (msg.position.latitude, msg.position.longitude,
                  msg.position.altitude)
        if all(math.isfinite(value) for value in origin) and origin != self.origin:
            self.node.get_logger().info(
                f'navigation origin {origin[0]:.7f}, {origin[1]:.7f}, {origin[2]:.2f} m')
            with self.lock:
                self.origin = origin
                self.dem_entry = None
                self.terrain.dem = None

    def _on_range(self, msg):
        """A rangefinder reading: the ground below the aircraft, and the
        DEM's offset there (low-pass)."""
        with self.lock:
            now = self.now()
            self.latest_range = (msg, now)
            pose = self.poses.latest()
            agl = None if pose is None else self._rangefinder_agl(now, pose)
            if agl is None:
                return
            ground = pose.position[2] - agl
            self.terrain.range_ground = (ground, now)
            dem = self.terrain.dem
            height = None if dem is None else dem.height(pose.position[0],
                                                         pose.position[1])
            if height is None:
                return
            terrain = self.terrain
            if terrain.bias_time is None or now - terrain.bias_time > self.bias_hold_s:
                terrain.bias = ground - height
            else:
                terrain.bias += 0.2 * (ground - height - terrain.bias)
            terrain.bias_time = now

    def _on_platform(self, msg):
        self.platform = msg

    def _on_behavior(self, name, msg):
        previous = self.behaviors.get(name)
        self.behaviors[name] = msg.status
        started = (previous is not None and previous != msg.status and
                   msg.status == self.BehaviorStatus.RUNNING)
        if started:
            self._preempt(name)

    def _on_map_goal(self, msg):
        self._preempt(f'a map goal ({msg.data})')

    def _preempt(self, reason):
        with self.lock:
            if not self.core.active:
                return
            self.node.get_logger().info(f'tracking pre-empted by {reason}')
            self._execute(self.core.preempt(reason, self.now()), hover_reason='pre-empted')
        self._publish_status()

    def _on_command(self, msg):
        with self.lock:
            now = self.now()
            drone = self._drone(now)
            before = (self.core.state, self.core.message)
            actions = self.core.handle_command(msg.data, now, drone)
            self.node.get_logger().info(
                f'command {msg.data!r}: {before[0]} -> {self.core.state} '
                f'({self.core.message})')
            self._execute(actions)
        self._publish_status()

    def _on_detections(self, msg):
        with self.lock:
            core = self.core
            if not core.active or core.command is None:
                return
            target_id = core.command.target_id
            detection = next((item for item in msg.detections
                              if item.id == target_id), None)
            if detection is not None and core.target_class and not tc.same_kind(
                    detection_class(detection), core.target_class):
                # The id now marks another kind of object (the detector's
                # tracker moved it, e.g. from a car onto a person beside
                # it): not the target.
                detection = None
            now = self.now()
            if detection is None and not core.handover_open(now):
                return
            before = core.state
            reason = ''
            pose = self.poses.at(stamp_ns(msg.header.stamp))
            if self.intrinsics is None:
                reason = 'no camera_info'
            elif pose is None:
                reason = 'no odometry at the image time'
            if detection is None:
                # The selected id is gone from this frame: a new id for the
                # same object (tracking_core, "Track handover")?
                if reason:
                    return
                detection, measurement = self._handover(msg, pose, now)
                if detection is None:
                    return
            else:
                geo = None if reason else self._geolocate(detection, pose, now)
                if not reason and geo is None:
                    reason = 'the ray does not meet the ground'
                if reason:
                    core.geolocation_failed(reason, now)
                    return
                measurement = tc.Measurement(geo.position, geo.covariance,
                                             pose.stamp_ns * 1e-9)
            # The kind is the one selected: kept across class changes within
            # it (car, van) and handovers.
            core.target_class = core.target_class or detection_class(detection) or None
            core.target_side_px = box_side(detection)
            self._execute(core.measurement(measurement, now))
            changed = core.state != before
            handed_over = core.command is not None and core.command.target_id != target_id
        if changed:
            self.node.get_logger().info(f'{before} -> {core.state}: {core.message}')
        if changed or handed_over:
            self._publish_status()

    def _geolocate(self, detection, pose, now):
        self._update_dem(pose.position, now)
        center = detection.bbox.center.position
        return tc.geolocate(
            float(center.x), float(center.y),
            (float(detection.bbox.size_x), float(detection.bbox.size_y)),
            pose, self.intrinsics, self.terrain.height_fn(now, self.bias_hold_s),
            self.mount_yaw, self.stabilized, self.config)

    def _handover(self, msg, pose, now):
        """(detection, measurement) of the track the selected target is
        handed over to in this frame, or (None, None)."""
        core = self.core
        candidates, detections = {}, {}
        for item in msg.detections:
            cls = detection_class(item)
            if not item.id or not tc.same_kind(cls, core.target_class):
                continue
            geo = self._geolocate(item, pose, now)
            if geo is not None:
                candidates[item.id] = (tc.Measurement(geo.position, geo.covariance,
                                                      pose.stamp_ns * 1e-9), cls, box_side(item))
                detections[item.id] = item
        track_id = core.handover(candidates, now)
        if track_id is None:
            return None, None
        previous = core.command.target_id
        core.hand_over(track_id, now)
        _, cls, side = candidates[track_id]
        self.node.get_logger().info(
            f'track {previous} gone: the target is now track {track_id} '
            f'({cls}, {side:.0f} px)')
        return detections[track_id], candidates[track_id][0]

    # --- the aircraft -----------------------------------------------------
    def _update_dem(self, position, now):
        """Load the cached DEM that holds the aircraft (in the background)."""
        if self.origin is None or self.dem_loading or now - self.dem_checked < 10.0:
            return
        self.dem_checked = now
        lat, lon = tc.local_to_latlon(position[0], position[1], *self.origin[:2])
        dem = self.terrain.dem
        if dem is not None and dem.contains(position[0], position[1], 1000.0):
            return
        entry = tc.best_dem(self.dem_cache_dir, lat, lon)
        if entry is None or entry == self.dem_entry:
            return
        self.dem_loading = True
        origin = self.origin

        def load():
            try:
                heightmap = tc.load_heightmap(entry)
                terrain = tc.DemTerrain(heightmap, origin, self.geoid)
                with self.lock:
                    if self.origin == origin:
                        self.terrain.dem = terrain
                        self.dem_entry = entry
                self.node.get_logger().info(f'terrain: DEM {entry[0]}')
            except Exception as error:  # noqa: BLE001 - keep tracking on flat ground
                self.node.get_logger().error(f'cannot load DEM {entry[0]}: {error}')
            finally:
                self.dem_loading = False

        threading.Thread(target=load, daemon=True).start()

    def _rangefinder_agl(self, now, pose):
        if self.latest_range is None:
            return None
        msg, received = self.latest_range
        if now - received > self.rangefinder_timeout_s:
            return None
        reading = float(msg.range)
        if not (math.isfinite(reading) and msg.min_range <= reading <= msg.max_range):
            return None
        return reading * tc.quaternion_up_component(pose.quaternion)

    def _drone(self, now):
        """The aircraft now (Drone), or None without odometry."""
        pose = self.poses.latest()
        if pose is None:
            return None
        self._update_dem(pose.position, now)
        agl_range = self._rangefinder_agl(now, pose)
        ground_fn = self.terrain.height_fn(now, self.bias_hold_s)
        agl = agl_range
        if agl is None:
            agl = pose.position[2] - ground_fn(pose.position[0], pose.position[1])
        flying = False
        busy = ''
        if self.platform is not None:
            flying = self.platform.status.state == 3  # PlatformStatus.FLYING
        for name in self.busy_names:
            if self.behaviors.get(name) == self.BehaviorStatus.RUNNING:
                busy = name
        return tc.Drone(pose.position, tc.yaw_of(pose.quaternion), agl, flying, busy)

    # --- actions ----------------------------------------------------------
    def _execute(self, actions, hover_reason='stopped'):
        for action in actions:
            if action == 'preempt':
                self._stop_preemptible()
            elif action == 'follow':
                self._start_follow()
            elif action == 'hover':
                self._stop_follow(hover_reason)

    def _stop_preemptible(self):
        for name, client in self.stop_clients.items():
            if self.behaviors.get(name) != self.BehaviorStatus.RUNNING:
                continue
            if not client.service_is_ready():
                self.node.get_logger().warn(f'{name} runs but its stop service is missing')
                continue
            self.node.get_logger().info(f'stopping {name} for tracking')
            client.call_async(self.Trigger.Request())

    def _goal(self, position, yaw):
        from as2_msgs.msg import YawMode
        goal = self.FollowReference.Goal()
        goal.target_pose.header.frame_id = 'earth'
        goal.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.target_pose.point.x = float(position[0])
        goal.target_pose.point.y = float(position[1])
        goal.target_pose.point.z = float(position[2])
        goal.yaw.mode = YawMode.FIXED_YAW
        goal.yaw.angle = float(yaw)
        # The behaviour copies a modified goal's speeds as given: never 0.
        goal.max_speed_x = goal.max_speed_y = float(self.config.max_speed_mps)
        goal.max_speed_z = float(self.config.max_vertical_speed_mps)
        return goal

    def _reference(self, now, drone, reset=False):
        setpoint = self.core.setpoint(
            now, drone, self.terrain.height_fn(now, self.bias_hold_s),
            self.mount_yaw, self.intrinsics)
        if setpoint is None:
            return None
        if reset or self.shaper.position is None:
            self.shaper.reset(drone.position, drone.yaw)
            self.last_control = now
        dt = 0.0 if self.last_control is None else now - self.last_control
        self.last_control = now
        return self.shaper.step(setpoint.position, setpoint.yaw, drone.position, dt)

    def _start_follow(self):
        if self.follow_handle is not None or self.follow_pending:
            return
        now = self.now()
        drone = self._drone(now)
        if drone is None:
            self.core.follow_failed('no odometry', now, self.follow_retry_s)
            return
        if not self.follow_client.server_is_ready():
            self.core.follow_failed('the FollowReference behaviour is not available',
                                    now, self.follow_retry_s)
            return
        reference = self._reference(now, drone, reset=True)
        if reference is None:
            return
        self.follow_pending = True
        self.cancel_requested = False
        future = self.follow_client.send_goal_async(self._goal(*reference))
        future.add_done_callback(self._on_follow_response)

    def _on_follow_response(self, future):
        with self.lock:
            self.follow_pending = False
            handle = future.result()
            now = self.now()
            if handle is None or not handle.accepted:
                self.core.follow_failed(
                    'FollowReference refused the goal (flying in GUIDED?)', now,
                    self.follow_retry_s)
                self.node.get_logger().warn(self.core.message)
                return
            if self.cancel_requested or self.core.state != 'tracking':
                handle.cancel_goal_async()
                return
            self.follow_handle = handle
            self.core.following = True
            handle.get_result_async().add_done_callback(
                lambda result, handle=handle: self._on_follow_result(handle, result))
            self.node.get_logger().info('following through FollowReference')
        self._publish_status()

    def _on_follow_result(self, handle, future):
        with self.lock:
            if handle is not self.follow_handle:
                return
            self.follow_handle = None
            self.modify_future = None
            if self.cancel_requested:
                return
            status = future.result().status if future.result() is not None else -1
            self.core.follow_failed(f'FollowReference ended (status {status})',
                                    self.now(), self.follow_retry_s)
            self.node.get_logger().warn(self.core.message)
        self._publish_status()

    def _stop_follow(self, reason):
        self.core.following = False
        self.cancel_requested = True
        self.modify_future = None
        if self.follow_handle is not None:
            self.node.get_logger().info(f'{reason}: stopping FollowReference (hover)')
            self.follow_handle.cancel_goal_async()
            self.follow_handle = None
        elif self.follow_stop_client.service_is_ready():
            self.follow_stop_client.call_async(self.Trigger.Request())

    def _send_modify(self, position, yaw, now):
        if self.modify_future is not None and not self.modify_future.done():
            if now - self.modify_sent > 2.0:
                self.node.get_logger().warn(
                    'FollowReference modify unanswered for 2 s', throttle_duration_sec=10.0)
            return
        if not self.modify_client.service_is_ready():
            return
        request = self.FollowReference.Impl.SendGoalService.Request()
        request.goal_id.uuid = list(uuid.uuid4().bytes)
        request.goal = self._goal(position, yaw)
        self.modify_future = self.modify_client.call_async(request)
        self.modify_sent = now

    # --- timers -----------------------------------------------------------
    def _control(self):
        with self.lock:
            now = self.now()
            before = self.core.state
            self._execute(self.core.tick(now), hover_reason='target lost')
            if before != self.core.state:
                self.node.get_logger().info(f'{before} -> {self.core.state}: '
                                            f'{self.core.message}')
                self._publish_status()
            drone = self._drone(now)
            if drone is None:
                return
            if self.core.state == 'tracking' and self.follow_handle is not None:
                reference = self._reference(now, drone)
                if reference is not None:
                    self._send_modify(*reference, now)
                    self.node.get_logger().info(
                        f'lead {self.config.lead_s:.1f} + {self.core.lead_extra_s:.2f} s',
                        throttle_duration_sec=10.0)
            position = self.core.estimate(now)
            if position is not None and self.core.state in ('tracking', 'lost'):
                msg = self.PointStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = self.odom_frame
                msg.point.x, msg.point.y, msg.point.z = (float(v) for v in position)
                self.target_pub.publish(msg)

    def _notes(self, now):
        notes = []
        source = self.terrain.source(now, self.bias_hold_s)
        if source == 'rangefinder':
            notes.append('no DEM: flat ground at the rangefinder height')
        elif source == 'flat':
            notes.append('no DEM or rangefinder: flat ground at the home height')
        if self.platform is not None and not self.platform.offboard:
            notes.append('not in GUIDED (offboard): not moving')
        return notes

    def _publish_status(self):
        with self.lock:
            now = self.now()
            pose = self.poses.latest()
            status = self.core.status(now, pose.position if pose else None,
                                      self._notes(now))
        self.status_pub.publish(self.String(data=tc.status_json(status)))


def main():
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    node = rclpy.create_node('aerodrone_tracker')
    Tracker(node)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
