#!/usr/bin/env python3
"""The OAK-1 and the rangefinder of the Aerodrone, played by the simulator
host in hardware-in-the-loop runs (tools/aerodrone_hil_test.md).

In the HIL setup the simulator and ArduPilot SITL run on the workstation and
the companion stack runs on the Pi, which on the aircraft reads its OAK-1 and
its rangefinder over USB. This node turns the simulated downward cameras into
the aircraft's camera topics, at the OAK-1's resolutions and rates
(applications/oak1/README.md), stamped in wall time as the OAK-1 stamps its
frames (the device clock converted to ROS time): the Pi then runs on wall time
like the aircraft, and no /clock has to cross the network.

  <root>/relative_depth   the simulator's metric depth (DemNav runs with
                          depth_is_metric) resized to MegaDepth's 256x192, at
                          MegaDepth's rate: 4.6 Hz, or 2.9 Hz while something
                          subscribes to the tracking detections (the detector
                          shares the OAK-1's Myriad X)
  <root>/camera_info      its intrinsics
  <root>/image_raw        the depth camera's colour image, same size and rate
  <root>/image_highres    the colour camera's frames at the stills' 0.5 Hz (the
                          simulator's 640x480, not the OAK-1's 1280x720)
  <root>/image_highres/camera_info
  <tracking_input>/image_raw, /camera_info
                          every colour frame, wall-stamped, for the detector
                          that stands in for the OAK-1's on-camera YOLO
                          (applications/oak1/scripts/sim_tracking_detector.py)
  rangefinder topic       the depth image's centre (the camera and the TFmini-S
                          both look straight down from the body), 0.1-12 m,
                          +inf beyond, -inf below, at the depth camera's rate

Rates count simulation time. Frames older than the /clock history, or before
the first /clock, are dropped.
"""

import bisect
import collections
import math
import threading
import time

import numpy as np

# MegaDepth's input on the OAK-1 (applications/oak1/README.md) and its rates
# there: alone, and with the tracking detector running beside it.
DEPTH_WIDTH = 256
DEPTH_HEIGHT = 192
DEPTH_RATE_HZ = 4.6
DEPTH_RATE_TRACKING_HZ = 2.9
HIGHRES_RATE_HZ = 0.5
# Benewake TFmini-S (applications/rangefinder/scripts/benewake.py).
RANGE_MIN_M = 0.1
RANGE_MAX_M = 12.0
RANGE_FOV_RAD = math.radians(2.0)

ENCODING_BYTES = {'32FC1': 4, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4,
                  'mono8': 1, '16UC1': 2, 'mono16': 2}
ENCODING_DTYPE = {'32FC1': np.float32, '16UC1': np.uint16, 'mono16': np.uint16}


class SimToWall:
    """Simulation seconds -> wall seconds: the wall time at which /clock
    reached a simulation instant, interpolated between /clock samples (as
    DemNav's attitude history maps simulator stamps)."""

    def __init__(self, keep_s=60.0, min_step_s=0.005):
        self.keep_s = keep_s
        self.min_step_s = min_step_s
        self.sim = collections.deque()
        self.wall = collections.deque()

    def add(self, sim_s, wall_s):
        if self.sim and sim_s < self.sim[-1] - 1.0:
            self.sim.clear()  # the simulation restarted
            self.wall.clear()
        if self.sim and sim_s - self.sim[-1] < self.min_step_s:
            return
        self.sim.append(sim_s)
        self.wall.append(wall_s)
        while self.sim and self.sim[-1] - self.sim[0] > self.keep_s:
            self.sim.popleft()
            self.wall.popleft()

    def rate(self, window_s=2.0):
        """Simulation seconds per wall second over the last window."""
        if len(self.sim) < 2:
            return None
        i = max(0, bisect.bisect_left(self.sim, self.sim[-1] - window_s) - 1)
        dw = self.wall[-1] - self.wall[i]
        return (self.sim[-1] - self.sim[i]) / dw if dw > 0 else None

    def to_wall(self, sim_s):
        """Wall seconds of a simulation instant, or None outside the history
        (a frame ahead of /clock is extrapolated at the recent rate)."""
        if len(self.sim) < 2 or sim_s < self.sim[0]:
            return None
        if sim_s >= self.sim[-1]:
            rate = self.rate() or 1.0
            return self.wall[-1] + (sim_s - self.sim[-1]) / rate
        i = bisect.bisect_right(self.sim, sim_s)
        s0, s1 = self.sim[i - 1], self.sim[i]
        w0, w1 = self.wall[i - 1], self.wall[i]
        return w0 + (sim_s - s0) * (w1 - w0) / (s1 - s0)


class RateLimiter:
    """Passes frames at rate_hz on average from a faster stream (credit per
    elapsed second of the frames' own clock, like a camera running its
    network at its own pace rather than every n-th frame)."""

    def __init__(self, rate_hz):
        self.rate_hz = rate_hz
        self.credit = 0.0
        self.last = None

    def accept(self, t_s, rate_hz=None):
        rate = self.rate_hz if rate_hz is None else rate_hz
        if self.last is None or t_s < self.last:
            self.last = t_s
            self.credit = 0.0
            return True
        # At most one frame owed after a gap: no burst.
        self.credit = min(self.credit + rate * (t_s - self.last), 2.0)
        self.last = t_s
        if self.credit >= 1.0 - 1e-6:
            self.credit -= 1.0
            return True
        return False


def resize_nearest(array, width, height):
    """Nearest-neighbour resize (keeps metric depth values and invalid
    pixels as they are)."""
    rows = ((np.arange(height) + 0.5) * array.shape[0] / height).astype(int)
    cols = ((np.arange(width) + 0.5) * array.shape[1] / width).astype(int)
    return array[rows[:, None], cols]


def scale_intrinsics(k, p, width, height, new_width, new_height):
    """K (9) and P (12) of a camera resampled from width x height to
    new_width x new_height (pixel centres map onto pixel centres)."""
    sx, sy = new_width / float(width), new_height / float(height)
    k = list(k)
    p = list(p)
    k[0], k[2] = k[0] * sx, (k[2] + 0.5) * sx - 0.5
    k[4], k[5] = k[4] * sy, (k[5] + 0.5) * sy - 0.5
    if len(p) == 12:
        p[0], p[2], p[3] = p[0] * sx, (p[2] + 0.5) * sx - 0.5, p[3] * sx
        p[5], p[6], p[7] = p[5] * sy, (p[6] + 0.5) * sy - 0.5, p[7] * sy
    return k, p


def centre_range(depth, half_window=2, min_m=RANGE_MIN_M, max_m=RANGE_MAX_M):
    """The rangefinder's reading from a metric depth image: the median of
    the finite, positive centre pixels; +inf with no return or beyond max_m,
    -inf below min_m (applications/rangefinder's convention)."""
    rows, cols = depth.shape[:2]
    r0, c0 = rows // 2, cols // 2
    window = np.asarray(depth[max(0, r0 - half_window):r0 + half_window + 1,
                              max(0, c0 - half_window):c0 + half_window + 1], dtype=float)
    values = window[np.isfinite(window) & (window > 0.0)]
    if values.size == 0:
        return math.inf
    value = float(np.median(values))
    if value > max_m:
        return math.inf
    if value < min_m:
        return -math.inf
    return value


def image_array(msg):
    """(H, W[, C]) numpy view of a sensor_msgs/Image."""
    encoding = msg.encoding
    channels = ENCODING_BYTES.get(encoding)
    if channels is None:
        raise ValueError(f'unsupported encoding {encoding}')
    dtype = ENCODING_DTYPE.get(encoding, np.uint8)
    itemsize = np.dtype(dtype).itemsize
    per_pixel = channels // itemsize if dtype is not np.uint8 else channels
    data = np.frombuffer(bytes(msg.data), dtype=dtype)
    row = msg.step // itemsize
    array = data.reshape(msg.height, row)[:, :msg.width * per_pixel]
    if per_pixel > 1:
        array = array.reshape(msg.height, msg.width, per_pixel)
    if msg.is_bigendian and itemsize > 1:
        array = array.byteswap()
    return array


def main():  # pragma: no cover - ROS runtime
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Range

    class Oak1Standin(Node):
        def __init__(self):
            super().__init__('hil_oak1_standin')
            param = self.declare_parameter
            param('depth_topic', '/aerodrone/sensor_measurements/downward_rgbd/depth')
            param('depth_info_topic',
                  '/aerodrone/sensor_measurements/downward_rgbd/depth/camera_info')
            param('aligned_image_topic', '')
            param('image_topic', '/aerodrone/sensor_measurements/downward_rgb/image_raw')
            param('image_info_topic', '/aerodrone/sensor_measurements/downward_rgb/camera_info')
            param('clock_topic', '/clock')
            param('output_root', '/oak1')
            param('tracking_input_prefix', '/hil/oak1/tracking_input')
            param('tracking_demand_topic', '/tracking/detections')
            param('rangefinder_topic', '/aerodrone/rangefinder')
            param('depth_width', DEPTH_WIDTH)
            param('depth_height', DEPTH_HEIGHT)
            param('depth_rate_hz', DEPTH_RATE_HZ)
            param('depth_rate_tracking_hz', DEPTH_RATE_TRACKING_HZ)
            param('highres_rate_hz', HIGHRES_RATE_HZ)
            def value(name):
                return self.get_parameter(name).value
            root = str(value('output_root')).rstrip('/')
            tracking = str(value('tracking_input_prefix')).rstrip('/')
            self.width = int(value('depth_width'))
            self.height = int(value('depth_height'))
            self.depth_rate = float(value('depth_rate_hz'))
            self.depth_rate_tracking = float(value('depth_rate_tracking_hz'))
            self.demand_topic = str(value('tracking_demand_topic'))
            self.clock = SimToWall()
            self.lock = threading.Lock()
            self.depth_info = None
            self.image_info = None
            self.aligned = None
            self.tracking_demand = False
            self.depth_limiter = RateLimiter(self.depth_rate)
            self.highres_limiter = RateLimiter(float(value('highres_rate_hz')))
            self.counts = collections.Counter()
            self.dropped = collections.Counter()
            reliable = 10  # the OAK-1 node's publishers (megadepth_ros.py)
            self.depth_pub = self.create_publisher(Image, f'{root}/relative_depth', reliable)
            self.depth_info_pub = self.create_publisher(CameraInfo, f'{root}/camera_info', reliable)
            self.aligned_pub = self.create_publisher(Image, f'{root}/image_raw', reliable)
            self.highres_pub = self.create_publisher(Image, f'{root}/image_highres', reliable)
            self.highres_info_pub = self.create_publisher(
                CameraInfo, f'{root}/image_highres/camera_info', reliable)
            self.tracking_pub = self.create_publisher(
                Image, f'{tracking}/image_raw', qos_profile_sensor_data)
            self.tracking_info_pub = self.create_publisher(
                CameraInfo, f'{tracking}/camera_info', qos_profile_sensor_data)
            self.range_pub = self.create_publisher(
                Range, str(value('rangefinder_topic')), qos_profile_sensor_data)
            sensor = qos_profile_sensor_data
            self.create_subscription(Clock, str(value('clock_topic')), self._on_clock, sensor)
            self.create_subscription(CameraInfo, str(value('depth_info_topic')),
                                     lambda m: setattr(self, 'depth_info', m), sensor)
            self.create_subscription(CameraInfo, str(value('image_info_topic')),
                                     lambda m: setattr(self, 'image_info', m), sensor)
            self.create_subscription(Image, str(value('depth_topic')), self._on_depth, sensor)
            self.create_subscription(Image, str(value('image_topic')), self._on_image, sensor)
            if value('aligned_image_topic'):
                self.create_subscription(Image, str(value('aligned_image_topic')),
                                         lambda m: setattr(self, 'aligned', m), sensor)
            self.CameraInfo, self.Image, self.Range = CameraInfo, Image, Range
            self.create_timer(1.0, self._check_demand)
            self.create_timer(30.0, self._report)
            self.started = time.time()
            self.get_logger().info(
                f'OAK-1 stand-in: {root}/* ({self.width}x{self.height} depth at '
                f'{self.depth_rate} Hz, {self.depth_rate_tracking} Hz while '
                f'{self.demand_topic} is subscribed), {tracking}/*, wall-time stamps')

        def _on_clock(self, msg):
            with self.lock:
                self.clock.add(msg.clock.sec + msg.clock.nanosec * 1e-9, time.time())

        def _check_demand(self):
            demand = self.count_subscribers(self.demand_topic) > 0
            if demand != self.tracking_demand:
                self.get_logger().info(
                    f'Tracking detector {"subscribed" if demand else "unsubscribed"}: depth at '
                    f'{self.depth_rate_tracking if demand else self.depth_rate} Hz')
            self.tracking_demand = demand

        def _wall_stamp(self, stamp):
            """(sim seconds, wall builtin_interfaces/Time) of a frame, or None."""
            sim = stamp.sec + stamp.nanosec * 1e-9
            with self.lock:
                wall = self.clock.to_wall(sim)
            if wall is None:
                return None
            out = type(stamp)()
            out.sec, nanosec = divmod(round(wall * 1e9), 1_000_000_000)
            out.nanosec = nanosec
            return sim, out

        def _info(self, info, stamp, width=None, height=None):
            out = self.CameraInfo()
            out.header.frame_id = info.header.frame_id
            out.header.stamp = stamp
            out.distortion_model = info.distortion_model
            out.d = list(info.d)
            out.r = list(info.r)
            out.binning_x, out.binning_y = info.binning_x, info.binning_y
            if width is None:
                out.width, out.height = info.width, info.height
                out.k, out.p = list(info.k), list(info.p)
            else:
                k, p = scale_intrinsics(info.k, info.p, info.width, info.height, width, height)
                out.width, out.height, out.k, out.p = width, height, k, p
            return out

        def _image(self, msg, stamp, array=None):
            out = self.Image()
            out.header.frame_id = msg.header.frame_id
            out.header.stamp = stamp
            out.encoding = msg.encoding
            out.is_bigendian = 0
            if array is None:
                out.height, out.width, out.step, out.data = (
                    msg.height, msg.width, msg.step, msg.data)
            else:
                array = np.ascontiguousarray(array)
                out.height, out.width = array.shape[:2]
                out.step = int(array.strides[0])
                out.data = array.tobytes()
            return out

        def _on_depth(self, msg):
            mapped = self._wall_stamp(msg.header.stamp)
            if mapped is None:
                self.dropped['depth'] += 1
                return
            sim, stamp = mapped
            try:
                depth = image_array(msg)
            except ValueError as error:
                self.get_logger().error(str(error), throttle_duration_sec=30.0)
                return
            value = centre_range(depth)
            reading = self.Range()
            reading.header.stamp = stamp
            reading.header.frame_id = 'rangefinder_link'
            reading.radiation_type = self.Range.INFRARED
            reading.field_of_view = RANGE_FOV_RAD
            reading.min_range, reading.max_range = RANGE_MIN_M, RANGE_MAX_M
            reading.range = float(value)
            self.range_pub.publish(reading)
            self.counts['range'] += 1
            rate = self.depth_rate_tracking if self.tracking_demand else self.depth_rate
            if not self.depth_limiter.accept(sim, rate):
                return
            self.depth_pub.publish(
                self._image(msg, stamp, resize_nearest(depth, self.width, self.height)))
            self.counts['depth'] += 1
            if self.depth_info is not None:
                self.depth_info_pub.publish(
                    self._info(self.depth_info, stamp, self.width, self.height))
            aligned = self.aligned
            if (aligned is not None and aligned.header.stamp == msg.header.stamp and
                    self.aligned_pub.get_subscription_count() > 0):
                self.aligned_pub.publish(self._image(
                    aligned, stamp, resize_nearest(image_array(aligned), self.width, self.height)))
                self.counts['aligned'] += 1

        def _on_image(self, msg):
            mapped = self._wall_stamp(msg.header.stamp)
            if mapped is None or self.image_info is None:
                self.dropped['image'] += 1
                return
            sim, stamp = mapped
            info = self._info(self.image_info, stamp)
            self.tracking_pub.publish(self._image(msg, stamp))
            self.tracking_info_pub.publish(info)
            self.counts['tracking_input'] += 1
            if self.highres_limiter.accept(sim):
                self.highres_pub.publish(self._image(msg, stamp))
                self.highres_info_pub.publish(info)
                self.counts['highres'] += 1

        def _report(self):
            elapsed = max(1.0, time.time() - self.started)
            with self.lock:
                rate = self.clock.rate(window_s=10.0)
            rates = ', '.join(f'{name} {count / elapsed:.2f} Hz'
                              for name, count in sorted(self.counts.items()))
            dropped = dict(self.dropped) or 'none'
            self.get_logger().info(
                f'published {rates} (wall time); real-time factor '
                f'{rate if rate is None else round(rate, 3)}; dropped before /clock: {dropped}')

    rclpy.init()
    node = Oak1Standin()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
