#!/usr/bin/env python3
"""The nadir OAK-1's ROS node: MegaDepth relative depth for DemNav, WildNav
stills and, with tracking_enabled, the object-tracking detector
(systems/aerodrone/tracking.md), all from one DepthAI pipeline on the
camera's Myriad X (oak1_pipeline.py); only one process can hold the
device.

Tracking publishes <tracking_topic_prefix>/detections (the on-device
ObjectTracker's tracklets with their IDs), /image/compressed (the frame
they come from, JPEG-encoded on the camera) and /camera_info, stamped with
the frame's capture time.

MegaDepth and the detector share the Myriad X's two neural compute
engines: MegaDepth alone runs about 4.6 Hz, and every detector inference
slows it (applications/oak1/README.md, "Object tracking"). With tracking
on, MegaDepth's input is capped (megadepth_rate_hz) so the detector keeps
tracking_rate_hz.
"""

import math
from pathlib import Path
import sys
import time

import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import oak1_pipeline  # noqa: E402
import tracking_messages as tm  # noqa: E402

# OAK-1 (IMX378): 69 deg HFOV across the full 4056 px wide sensor. The
# THE_1080_P stream is the central 3840x2160 binned by 2, and the preview
# and still outputs keep their own aspect ratio by centre-cropping it.
SENSOR_HFOV_DEG = 69.0
SENSOR_WIDTH_PX = 4056

# MegaDepth input rate cap with tracking on (auto): with the detector at
# 5 Hz the measured split is MegaDepth ~2.9 Hz (applications/oak1/README.md).
TRACKING_MEGADEPTH_RATE_HZ = 3.0

TRACKLET_PUBLISHED = ('NEW', 'TRACKED')


def approximate_intrinsics(width, height):
    """Uncalibrated pinhole K for a width x height preview/still of the
    1080P stream. The 256x192 MegaDepth preview is a 4:3 crop, ~52 deg
    wide, not the sensor's 69 deg."""
    focal_1080 = (0.5 * SENSOR_WIDTH_PX /
                  math.tan(math.radians(SENSOR_HFOV_DEG) / 2.0) *
                  1920.0 / 3840.0)
    focal = focal_1080 * max(width / 1920.0, height / 1080.0)
    return [focal, 0.0, width / 2.0,
            0.0, focal, height / 2.0,
            0.0, 0.0, 1.0]


def megadepth_rate(requested, tracking_enabled):
    """MegaDepth input cap: requested if >= 0, else (auto) none without
    tracking and TRACKING_MEGADEPTH_RATE_HZ with it."""
    if requested >= 0.0:
        return requested
    return TRACKING_MEGADEPTH_RATE_HZ if tracking_enabled else 0.0


def tracking_rates(active, tracking_rate_hz, megadepth_cap_hz):
    """(detector rate, MegaDepth cap) to send to the device: the
    configured pair while tracking is wanted, else the detector paused
    (None) and MegaDepth uncapped (0)."""
    if active:
        return tracking_rate_hz, megadepth_cap_hz
    return None, 0.0


def tracklet_boxes(tracklets, classes, width, height, publish_lost=False):
    """TrackedBoxes of a DepthAI Tracklets message: NEW and TRACKED
    tracklets (and LOST ones with publish_lost), roi in pixels of the
    width x height tracking image."""
    statuses = TRACKLET_PUBLISHED + (('LOST',) if publish_lost else ())
    boxes = []
    for tracklet in tracklets:
        if tracklet.status.name not in statuses:
            continue
        label = int(tracklet.label)
        name = classes[label] if 0 <= label < len(classes) else str(label)
        roi = tracklet.roi
        top_left, bottom_right = roi.topLeft(), roi.bottomRight()
        boxes.append(tm.box_from_normalized(
            tracklet.id, name, tracklet.srcImgDetection.confidence,
            top_left.x, top_left.y, bottom_right.x, bottom_right.y,
            width, height))
    return boxes


class MegaDepthNode(Node):
    """Run MegaDepth (and the tracking detector) on the OAK-1 Myriad X."""

    def __init__(self):
        super().__init__('oak1_megadepth')

        self.declare_parameter(
            'blob_path',
            '/models/megadepth_192x256_openvino_2021.4_6shave.blob')
        self.declare_parameter('device_ip', 'auto')
        self.declare_parameter('image_topic', '/oak1/image_raw')
        self.declare_parameter('depth_topic', '/oak1/relative_depth')
        self.declare_parameter('camera_info_topic', '/oak1/camera_info')
        self.declare_parameter('highres_image_topic', '/oak1/image_highres')
        self.declare_parameter(
            'highres_camera_info_topic',
            '/oak1/image_highres/camera_info')
        self.declare_parameter('frame_id', 'oak1_optical_frame')
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('highres_width', 1280)
        self.declare_parameter('highres_height', 720)
        self.declare_parameter('highres_capture_hz', 0.5)
        # MegaDepth inference threads (one per neural compute engine) and
        # input queue: one queued frame keeps the camera from stalling on
        # MegaDepth (DepthAI's default queue held every stream at its rate).
        self.declare_parameter('megadepth_threads', 2)
        self.declare_parameter('megadepth_queue_size', 1)
        # Frames per second into MegaDepth; -1: none, or 3 with tracking.
        self.declare_parameter('megadepth_rate_hz', -1.0)

        self.declare_parameter('tracking_enabled', False)
        # The detector's json (models/export_yolo.py): input size, classes
        # and its blob, beside it unless tracking_blob_path says otherwise.
        self.declare_parameter(
            'tracking_model', '/models/visdrone_yolov8n_640x352.json')
        self.declare_parameter('tracking_blob_path', '')
        self.declare_parameter('tracking_topic_prefix', '/tracking')
        self.declare_parameter('tracking_rate_hz', 5.0)
        # VisDrone YOLOv8n: 0.2 trades precision for recall of people
        # seen from 30-60 m (applications/oak1/README.md).
        self.declare_parameter('tracking_confidence', 0.2)
        self.declare_parameter('tracking_iou', 0.5)
        self.declare_parameter('tracking_classes',
                               ','.join(tm.DEFAULT_CLASSES))
        self.declare_parameter('tracking_tracker_type',
                               'SHORT_TERM_IMAGELESS')
        # A tracklet unseen this long loses its ID.
        self.declare_parameter('tracking_max_lost_s', 2.0)
        self.declare_parameter('tracking_publish_lost', False)
        self.declare_parameter('tracking_jpeg_quality', 80)
        # Run the detector only while something subscribes to the tracking
        # topics (the tracker, the web page); otherwise pause it and give
        # MegaDepth its whole rate back.
        self.declare_parameter('tracking_on_demand', True)

        self.width, self.height = oak1_pipeline.MEGADEPTH_SIZE
        self.highres_width = int(self.get_parameter('highres_width').value)
        self.highres_height = int(self.get_parameter('highres_height').value)
        self.highres_period = 1.0 / max(
            0.02, float(self.get_parameter('highres_capture_hz').value))
        self.last_highres_request = 0.0
        self.frame_id = self.get_parameter('frame_id').value
        blob_path = Path(self.get_parameter('blob_path').value)
        if not blob_path.is_file():
            raise FileNotFoundError(f'MegaDepth blob not found: {blob_path}')
        if blob_path.stat().st_size < 1_000_000:
            raise RuntimeError(
                f'MegaDepth model is a Git LFS pointer, not a blob: {blob_path}. '
                'Run git lfs pull before building.')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            Image, self.get_parameter('image_topic').value, 10)
        self.depth_pub = self.create_publisher(
            Image, self.get_parameter('depth_topic').value, 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter('camera_info_topic').value, 10)
        self.highres_image_pub = self.create_publisher(
            Image, self.get_parameter('highres_image_topic').value, 10)
        self.highres_camera_info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter('highres_camera_info_topic').value, 10)

        self.tracking = None
        self.tracking_on_demand = bool(
            self.get_parameter('tracking_on_demand').value)
        if bool(self.get_parameter('tracking_enabled').value):
            self.tracking = self._tracking_config()
        self.fps = fps = float(self.get_parameter('fps').value)
        md_rate = self.megadepth_cap = megadepth_rate(
            float(self.get_parameter('megadepth_rate_hz').value),
            self.tracking is not None)
        if self.tracking is not None and self.tracking.start_paused:
            md_rate = 0.0  # until the detector runs
        pipeline, _ = oak1_pipeline.build_pipeline(
            blob_path, fps=fps,
            highres_size=(self.highres_width, self.highres_height),
            megadepth_threads=int(
                self.get_parameter('megadepth_threads').value),
            megadepth_queue_size=int(
                self.get_parameter('megadepth_queue_size').value),
            megadepth_rate_hz=md_rate,
            tracking=self.tracking)

        device_ip = self.get_parameter('device_ip').value
        if device_ip and device_ip != 'auto':
            self.device = dai.Device(pipeline, dai.DeviceInfo(device_ip))
        else:
            self.device = dai.Device(pipeline)

        self.camera_queue = self.device.getOutputQueue(
            oak1_pipeline.STREAM_CAMERA, maxSize=8, blocking=False)
        self.depth_queue = self.device.getOutputQueue(
            oak1_pipeline.STREAM_DEPTH, maxSize=8, blocking=False)
        self.highres_queue = self.device.getOutputQueue(
            oak1_pipeline.STREAM_HIGHRES, maxSize=2, blocking=False)
        self.control_queue = self.device.getInputQueue(
            oak1_pipeline.STREAM_CONTROL)
        self.camera_packets = {}
        self.depth_packets = {}
        self.camera_info = self._read_camera_info(self.width, self.height)
        self.highres_camera_info = self._read_camera_info(
            self.highres_width, self.highres_height)
        if self.tracking is not None:
            self._start_tracking()
        self.timer = self.create_timer(0.005, self._poll)

        self.get_logger().info(
            f'OAK-1 MegaDepth ready: {blob_path.name} at '
            f'{self.width}x{self.height}'
            + (f', input capped at {md_rate:g} Hz' if md_rate else '')
            + (', tracking paused until subscribed'
               if self.tracking is not None and self.tracking.start_paused
               else ''))

    def _tracking_config(self):
        model = Path(self.get_parameter('tracking_model').value)
        meta = tm.load_model_meta(model)
        blob = (self.get_parameter('tracking_blob_path').value or
                str(model.parent / meta['blob']))
        self.tracking_classes = list(meta['classes'])
        rate = float(self.get_parameter('tracking_rate_hz').value)
        lost_s = float(self.get_parameter('tracking_max_lost_s').value)
        return oak1_pipeline.TrackingConfig(
            blob_path=blob,
            num_classes=len(self.tracking_classes),
            width=int(meta['input_width']),
            height=int(meta['input_height']),
            confidence_threshold=float(
                self.get_parameter('tracking_confidence').value),
            iou_threshold=float(self.get_parameter('tracking_iou').value),
            tracker_type=self.get_parameter('tracking_tracker_type').value,
            labels_to_track=tm.labels_to_track(
                self.tracking_classes,
                self.get_parameter('tracking_classes').value),
            tracklet_max_lifespan=max(1, int(math.ceil(lost_s * rate))),
            jpeg_quality=int(
                self.get_parameter('tracking_jpeg_quality').value),
            rate_hz=rate, start_paused=self.tracking_on_demand)

    def _start_tracking(self):
        prefix = self.get_parameter('tracking_topic_prefix').value.rstrip('/')
        from sensor_msgs.msg import CompressedImage
        from vision_msgs.msg import Detection2DArray

        self.detections_pub = self.create_publisher(
            Detection2DArray, f'{prefix}/detections', 10)
        self.tracking_image_pub = self.create_publisher(
            CompressedImage, f'{prefix}/image/compressed', 10)
        self.tracking_info_pub = self.create_publisher(
            CameraInfo, f'{prefix}/camera_info', 10)
        self.tracking_publish_lost = bool(
            self.get_parameter('tracking_publish_lost').value)
        self.tracklets_queue = self.device.getOutputQueue(
            oak1_pipeline.STREAM_TRACKLETS, maxSize=8, blocking=False)
        self.jpeg_queue = self.device.getOutputQueue(
            oak1_pipeline.STREAM_JPEG, maxSize=8, blocking=False)
        self.tracklet_packets = {}
        self.jpeg_packets = {}
        self.tracking_info = self._read_camera_info(
            self.tracking.width, self.tracking.height)
        self.tracking_rate_queue = self.device.getInputQueue(
            oak1_pipeline.STREAM_TRACKING_RATE)
        self.megadepth_rate_queue = self.device.getInputQueue(
            oak1_pipeline.STREAM_MEGADEPTH_RATE)
        self.tracking_active = not self.tracking.start_paused
        if self.tracking_on_demand:
            self.create_timer(1.0, self._update_tracking_demand)
        self.get_logger().info(
            f'OAK-1 tracking: {Path(self.tracking.blob_path).name} at '
            f'{self.tracking.width}x{self.tracking.height}, '
            f'{self.tracking.rate_hz:g} Hz, on {prefix}/detections')

    def _tracking_subscribers(self):
        return sum(publisher.get_subscription_count() for publisher in (
            self.detections_pub, self.tracking_image_pub,
            self.tracking_info_pub))

    def _update_tracking_demand(self):
        subscribers = self._tracking_subscribers()
        active = subscribers > 0
        if active == self.tracking_active:
            return
        self.tracking_active = active
        tracking_rate, megadepth_cap = tracking_rates(
            active, self.tracking.rate_hz, self.megadepth_cap)
        self.tracking_rate_queue.send(oak1_pipeline.period_buffer(
            oak1_pipeline.limiter_period(tracking_rate, self.fps)))
        self.megadepth_rate_queue.send(oak1_pipeline.period_buffer(
            oak1_pipeline.limiter_period(megadepth_cap, self.fps)))
        if active:
            self.get_logger().info(
                f'Tracking on ({subscribers} subscribers): detector at '
                f'{tracking_rate:g} Hz, MegaDepth capped at '
                f'{megadepth_cap:g} Hz')
        else:
            self.get_logger().info(
                'Tracking paused (no subscribers): MegaDepth at full rate')

    def _read_camera_info(self, width, height):
        info = CameraInfo()
        info.width = width
        info.height = height
        try:
            calibration = self.device.readCalibration()
            socket = getattr(dai.CameraBoardSocket, 'CAM_A', None)
            if socket is None:
                socket = dai.CameraBoardSocket.RGB
            # keepAspectRatio (default) crops like the preview does.
            intrinsic = calibration.getCameraIntrinsics(
                socket, width, height)
            distortion = calibration.getDistortionCoefficients(socket)
            info.k = [
                intrinsic[0][0], intrinsic[0][1], intrinsic[0][2],
                intrinsic[1][0], intrinsic[1][1], intrinsic[1][2],
                intrinsic[2][0], intrinsic[2][1], intrinsic[2][2],
            ]
            info.d = list(distortion)
            info.distortion_model = 'plumb_bob'
        except Exception as exc:
            info.k = approximate_intrinsics(width, height)
            self.get_logger().warning(
                f'Using approximate OAK-1 intrinsics: {exc}')
        return info

    @staticmethod
    def _store_packet(queue, packets):
        while True:
            packet = queue.tryGet()
            if packet is None:
                break
            packets[packet.getSequenceNum()] = packet
        while len(packets) > 8:
            packets.pop(min(packets))

    @staticmethod
    def _pop_latest_common(first, second):
        """The newest sequence number in both, popped from both with every
        older packet; None if they share none."""
        matching = set(first).intersection(second)
        if not matching:
            return None
        sequence = max(matching)
        pair = (first.pop(sequence), second.pop(sequence))
        for packets in (first, second):
            for old in [s for s in packets if s < sequence]:
                packets.pop(old)
        return pair

    def _poll(self):
        now = time.monotonic()
        if now - self.last_highres_request >= self.highres_period:
            control = dai.CameraControl()
            control.setCaptureStill(True)
            self.control_queue.send(control)
            self.last_highres_request = now

        highres_packet = self.highres_queue.tryGet()
        if highres_packet is not None:
            highres_frame = highres_packet.getCvFrame()
            highres_stamp = self.get_clock().now().to_msg()
            highres_msg = self.bridge.cv2_to_imgmsg(
                highres_frame, encoding='bgr8')
            highres_msg.header.stamp = highres_stamp
            highres_msg.header.frame_id = self.frame_id
            self.highres_camera_info.header = highres_msg.header
            self.highres_image_pub.publish(highres_msg)
            self.highres_camera_info_pub.publish(self.highres_camera_info)

        if self.tracking is not None:
            self._poll_tracking()

        self._store_packet(self.camera_queue, self.camera_packets)
        self._store_packet(self.depth_queue, self.depth_packets)
        pair = self._pop_latest_common(self.camera_packets,
                                       self.depth_packets)
        if pair is None:
            return
        camera_packet, depth_packet = pair

        frame = camera_packet.getCvFrame()
        relative_depth = np.asarray(
            depth_packet.getFirstLayerFp16(),
            dtype=np.float32).reshape(self.height, self.width)
        # Keep invalid pixels NaN so downstream isfinite masks (demnav) work.
        relative_depth[~np.isfinite(relative_depth)] = np.nan

        stamp = self.get_clock().now().to_msg()
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id

        depth_msg = self.bridge.cv2_to_imgmsg(
            relative_depth, encoding='32FC1')
        depth_msg.header = image_msg.header

        self.camera_info.header = image_msg.header
        self.image_pub.publish(image_msg)
        self.depth_pub.publish(depth_msg)
        self.camera_info_pub.publish(self.camera_info)

    def _poll_tracking(self):
        self._store_packet(self.tracklets_queue, self.tracklet_packets)
        self._store_packet(self.jpeg_queue, self.jpeg_packets)
        pair = self._pop_latest_common(self.tracklet_packets,
                                       self.jpeg_packets)
        if pair is None:
            return
        tracklets, jpeg = pair
        stamp_ns = tm.capture_time_ns(
            self.get_clock().now().nanoseconds,
            dai.Clock.now().total_seconds(),
            jpeg.getTimestamp().total_seconds())
        info = self.tracking_info
        tm.set_stamp(info.header.stamp, stamp_ns)
        info.header.frame_id = self.frame_id
        boxes = tracklet_boxes(
            tracklets.tracklets, self.tracking_classes, info.width,
            info.height, self.tracking_publish_lost)
        self.detections_pub.publish(tm.detection_array(info.header, boxes))
        self.tracking_image_pub.publish(
            tm.compressed_jpeg(info.header, jpeg.getData()))
        self.tracking_info_pub.publish(info)

    def destroy_node(self):
        self.device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MegaDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
