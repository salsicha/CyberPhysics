#!/usr/bin/env python3
"""Object-tracking detector of the Gazebo, Genesis and Isaac simulations
(systems/aerodrone/tracking.md), where there is no OAK-1.

Runs the same YOLO network as the aircraft (the ONNX its OAK-1 blob is
compiled from) with ONNX Runtime on the simulated downward camera
(tracking_yolo.py), gives the detections stable IDs with a host-side
tracker (host_tracker.py) in place of DepthAI's ObjectTracker, and
publishes the contract topics: <prefix>/detections, the frame they refer
to as <prefix>/image/compressed (JPEG, at most max_image_width wide) and
<prefix>/camera_info, all with the camera image's stamp.

The newest image is processed when the detector is free; older ones are
dropped (max_rate_hz caps the rate below the camera's).
"""
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from vision_msgs.msg import Detection2DArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from host_tracker import HostTracker  # noqa: E402
import tracking_messages as tm  # noqa: E402
from tracking_yolo import YoloDetector  # noqa: E402


def output_size(width, height, max_width):
    """Published image size: the camera's, scaled down to max_width."""
    if max_width <= 0 or width <= max_width:
        return width, height
    return max_width, int(round(height * max_width / float(width)))


def tracked_boxes(tracks, class_names, sx=1.0, sy=1.0):
    """TrackedBoxes from HostTracker output, scaled into the published
    image."""
    boxes = []
    for track_id, box, score, cls in tracks:
        name = class_names[cls] if isinstance(cls, (int, np.integer)) else cls
        boxes.append(tm.TrackedBox(
            int(track_id), tm.canonical_class(name), float(score),
            float(box[0]) * sx, float(box[1]) * sy,
            float(box[2]) * sx, float(box[3]) * sy))
    return boxes


class SimTrackingDetector(Node):
    def __init__(self):
        super().__init__('sim_tracking_detector')
        self.declare_parameter(
            'image_topic', '/aerodrone/sensor_measurements/downward_rgb/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/aerodrone/sensor_measurements/downward_rgb/camera_info')
        self.declare_parameter('topic_prefix', '/tracking')
        # The detector's json (models/export_yolo.py), its ONNX beside it.
        self.declare_parameter('model', '/models/visdrone_yolov8n_640x352.json')
        # Detections that start or confirm a track (VisDrone YOLOv8n: 0.2
        # trades precision for recall of people from 30-60 m, README).
        self.declare_parameter('confidence', 0.2)
        self.declare_parameter('track_low_confidence', 0.1)
        self.declare_parameter('iou', 0.5)
        self.declare_parameter('classes', ','.join(tm.DEFAULT_CLASSES))
        self.declare_parameter('max_lost_s', 2.0)
        self.declare_parameter('min_hits', 2)
        self.declare_parameter('max_rate_hz', 0.0)
        self.declare_parameter('max_image_width', 640)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('threads', 4)

        meta = Path(self.get_parameter('model').value)
        model = meta.parent / tm.load_model_meta(meta)['onnx']
        self.confidence = float(self.get_parameter('confidence').value)
        low = min(self.confidence,
                  float(self.get_parameter('track_low_confidence').value))
        # The network keeps the weak detections the tracker's second
        # association pass uses; only tracks are published.
        self.detector = YoloDetector(
            model, meta, conf_threshold=low,
            iou_threshold=float(self.get_parameter('iou').value),
            threads=int(self.get_parameter('threads').value))
        self.classes = self.detector.classes
        wanted = set(tm.labels_to_track(
            self.classes, self.get_parameter('classes').value))
        self.wanted = np.array([i in wanted for i in range(len(self.classes))])
        self.tracker = HostTracker(
            high_score=self.confidence, low_score=low,
            min_hits=int(self.get_parameter('min_hits').value),
            max_lost_s=float(self.get_parameter('max_lost_s').value))
        self.min_period = 0.0
        rate = float(self.get_parameter('max_rate_hz').value)
        if rate > 0.0:
            self.min_period = 1.0 / rate
        self.max_width = int(self.get_parameter('max_image_width').value)
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY,
                            int(self.get_parameter('jpeg_quality').value)]

        prefix = self.get_parameter('topic_prefix').value.rstrip('/')
        self.detections_pub = self.create_publisher(
            Detection2DArray, f'{prefix}/detections', 10)
        self.image_pub = self.create_publisher(
            CompressedImage, f'{prefix}/image/compressed', 10)
        self.info_pub = self.create_publisher(
            CameraInfo, f'{prefix}/camera_info', 10)
        self.bridge = CvBridge()
        self.camera_info = None
        self.pending = None
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.last_processed = 0.0
        self.processed = 0
        self.processing_s = 0.0
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image,
            qos_profile_sensor_data)
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f'Tracking detector: {Path(model).name} '
            f'({self.detector.input_size[0]}x{self.detector.input_size[1]}) '
            f'on {self.get_parameter("image_topic").value} -> {prefix}/detections')

    def _on_info(self, msg):
        self.camera_info = msg

    def _on_image(self, msg):
        with self.lock:
            self.pending = msg
        self.wake.set()

    def _run(self):
        while rclpy.ok():
            if not self.wake.wait(timeout=0.5):
                continue
            self.wake.clear()
            wait = self.min_period - (time.monotonic() - self.last_processed)
            if wait > 0.0:
                time.sleep(wait)
            with self.lock:
                msg, self.pending = self.pending, None
            if msg is None:
                continue
            self.last_processed = time.monotonic()
            try:
                self._process(msg)
            except Exception as exc:  # keep the detector alive
                self.get_logger().error(f'Detection failed: {exc}',
                                        throttle_duration_sec=5.0)

    def _process(self, msg):
        started = time.monotonic()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        boxes, scores, classes = self.detector.detect(frame)
        keep = self.wanted[classes] if len(classes) else np.zeros(0, bool)
        tracks = self.tracker.update(
            boxes[keep], scores[keep], [int(c) for c in classes[keep]],
            tm.stamp_ns(msg.header.stamp) * 1e-9)
        out_w, out_h = output_size(width, height, self.max_width)
        sx, sy = out_w / float(width), out_h / float(height)
        image = frame
        if (out_w, out_h) != (width, height):
            image = cv2.resize(frame, (out_w, out_h),
                               interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode('.jpg', image, self.jpeg_params)
        if not ok:
            raise RuntimeError('JPEG encoding failed')

        header = msg.header
        self.detections_pub.publish(tm.detection_array(
            header, tracked_boxes(tracks, self.classes, sx, sy)))
        self.image_pub.publish(tm.compressed_jpeg(header, jpeg.tobytes()))
        info = self.camera_info
        if info is not None and info.width and info.height:
            # The camera's intrinsics are for its own size; scale them to
            # the frame's (and the published) size.
            if (info.width, info.height) != (width, height):
                info = tm.scaled_camera_info(info, width, height)
            self.info_pub.publish(tm.scaled_camera_info(
                info, out_w, out_h, header))
        self.processed += 1
        self.processing_s += time.monotonic() - started

    def _report(self):
        if self.processed:
            self.get_logger().info(
                f'{self.processed / 10.0:.1f} frames/s, '
                f'{1000.0 * self.processing_s / self.processed:.0f} ms each, '
                f'{len(self.tracker.tracks)} tracks')
        else:
            self.get_logger().warning(
                'No camera images yet', throttle_duration_sec=30.0)
        self.processed = 0
        self.processing_s = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = SimTrackingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
