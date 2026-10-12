"""Host-side YOLO for the simulator tracking detector.

Runs the ONNX the OAK-1 blob is compiled from (models/export_yolo.py) with
ONNX Runtime and decodes its anchor-free "yolov6r2" outputs as DepthAI's
YoloDetectionNetwork does on the camera: per stride s and grid cell (i, j)
the box is ((j + 0.5 - l) s, (i + 0.5 - t) s, (j + 0.5 + r) s,
(i + 0.5 + b) s) in network-input pixels, the score is the best class
probability, then non-maximum suppression across classes (on the camera
a car also scored as a van leaves one box, measured against the blob).

A frame is scaled to the network's width and, where it is then taller
than the network, covered by overlapping windows (two for a 640x480
simulator frame and a 640x352 network), so the simulator sees as many
pixels per metre as the aircraft; the windows' detections are merged by
the same suppression.

numpy only, apart from the optional ONNX Runtime session and cv2 resize.
"""
import json
import math
from pathlib import Path

import numpy as np

LETTERBOX_FILL = 114  # Ultralytics' padding grey


def decode(outputs, strides, conf_threshold):
    """(boxes [N, 4] x1 y1 x2 y2, scores [N], classes [N]) in network-input
    pixels from the per-stride [1, 5 + classes, H, W] outputs."""
    boxes, scores, classes = [], [], []
    for output, stride in zip(outputs, strides):
        out = np.asarray(output, dtype=np.float32)[0]
        cls = out[5:]
        best = cls.argmax(axis=0)
        score = np.take_along_axis(cls, best[None], axis=0)[0]
        rows, cols = np.nonzero(score >= conf_threshold)
        if rows.size == 0:
            continue
        ltrb = out[:4, rows, cols]
        cx = cols.astype(np.float32) + 0.5
        cy = rows.astype(np.float32) + 0.5
        boxes.append(np.stack([cx - ltrb[0], cy - ltrb[1],
                               cx + ltrb[2], cy + ltrb[3]], axis=1) * stride)
        scores.append(score[rows, cols])
        classes.append(best[rows, cols])
    if not boxes:
        return (np.zeros((0, 4), np.float32), np.zeros(0, np.float32),
                np.zeros(0, np.int64))
    return (np.concatenate(boxes).astype(np.float32),
            np.concatenate(scores).astype(np.float32),
            np.concatenate(classes).astype(np.int64))


def iou_matrix(a, b):
    """IoU of every box in a [N, 4] with every box in b [M, 4] (x1y1x2y2)."""
    a = np.asarray(a, dtype=np.float64).reshape(-1, 4)
    b = np.asarray(b, dtype=np.float64).reshape(-1, 4)
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    union = area_a[:, None] + area_b[None, :] - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-12), 0.0)


def nms(boxes, scores, classes, iou_threshold, max_detections=300,
        agnostic=True):
    """Indices kept by greedy suppression, best score first; across classes
    as DepthAI does, or per class with agnostic=False."""
    order = np.argsort(-np.asarray(scores), kind='stable')
    keep = []
    while order.size and len(keep) < max_detections:
        best = order[0]
        keep.append(best)
        rest = order[1:]
        if rest.size == 0:
            break
        overlap = iou_matrix(boxes[best][None], boxes[rest])[0]
        suppressed = overlap > iou_threshold
        if not agnostic:
            suppressed &= classes[rest] == classes[best]
        order = rest[~suppressed]
    return np.asarray(keep, dtype=np.int64)


def windows(length, window):
    """Start offsets of the fewest windows of `window` covering `length`,
    evenly spaced, first at 0 and last flush with the end."""
    if length <= window:
        return [0]
    count = math.ceil(length / window)
    while (length - window) / (count - 1) > window - 64 and count < 64:
        count += 1  # keep at least 64 px of overlap
    step = (length - window) / (count - 1)
    return [int(round(i * step)) for i in range(count)]


def plan(image_size, input_size, scale=None):
    """How a frame (w, h) is fed to a network (w, h): (scale, scaled size,
    x window offsets, y window offsets), offsets in scaled pixels. By
    default the frame is scaled to the network's width; windows shorter
    than the network are padded at the right and bottom."""
    width, height = image_size
    net_w, net_h = input_size
    if scale is None:
        scale = net_w / float(width)
    scaled = (int(round(width * scale)), int(round(height * scale)))
    return (scale, scaled, windows(scaled[0], net_w),
            windows(scaled[1], net_h))


class YoloDetector:
    """The tracking ONNX on ONNX Runtime with its metadata json."""

    def __init__(self, onnx_path, meta_path=None, providers=None,
                 conf_threshold=0.3, iou_threshold=0.5, threads=0):
        import onnxruntime as ort

        onnx_path = Path(onnx_path)
        meta_path = Path(meta_path) if meta_path else onnx_path.with_suffix('.json')
        self.meta = json.loads(meta_path.read_text())
        self.classes = list(self.meta['classes'])
        self.strides = list(self.meta['strides'])
        self.input_size = (int(self.meta['input_width']),
                           int(self.meta['input_height']))
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        options = ort.SessionOptions()
        if threads:
            options.intra_op_num_threads = int(threads)
        # No busy-waiting between frames: with it the threads spin on after
        # each 5 Hz frame, 3.2 cores of CPU for 47 ms a frame; without, 0.9
        # cores for 59 ms (4 threads, Gazebo's frames).
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        self.session = ort.InferenceSession(
            str(onnx_path), sess_options=options,
            providers=providers or ['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = list(self.meta['outputs'])

    def infer(self, rgb):
        """Raw outputs for one network-sized RGB uint8 image."""
        tensor = np.ascontiguousarray(
            rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        return self.session.run(self.output_names, {self.input_name: tensor})

    def detect(self, bgr, scale=None):
        """Detections in a BGR frame of any size: (boxes x1y1x2y2 in frame
        pixels, scores, class indices). scale: network pixels per frame
        pixel (default: the frame's width fills the network's)."""
        import cv2

        height, width = bgr.shape[:2]
        net_w, net_h = self.input_size
        scale, (scaled_w, scaled_h), xs, ys = plan(
            (width, height), self.input_size, scale)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if (scaled_w, scaled_h) != (width, height):
            rgb = cv2.resize(rgb, (scaled_w, scaled_h),
                             interpolation=cv2.INTER_AREA if scale < 1.0
                             else cv2.INTER_LINEAR)
        all_boxes, all_scores, all_classes = [], [], []
        for y0 in ys:
            for x0 in xs:
                window = rgb[y0:y0 + net_h, x0:x0 + net_w]
                if window.shape[:2] != (net_h, net_w):
                    padded = np.full((net_h, net_w, 3), LETTERBOX_FILL,
                                     np.uint8)
                    padded[:window.shape[0], :window.shape[1]] = window
                    window = padded
                boxes, scores, classes = decode(
                    self.infer(window), self.strides, self.conf_threshold)
                if len(boxes) and (len(xs) > 1 or len(ys) > 1):
                    # A box cut by an inner window edge is whole in a
                    # neighbouring window.
                    cut = np.zeros(len(boxes), bool)
                    if x0 > 0:
                        cut |= boxes[:, 0] <= 1.0
                    if y0 > 0:
                        cut |= boxes[:, 1] <= 1.0
                    if x0 + net_w < scaled_w:
                        cut |= boxes[:, 2] >= net_w - 1.0
                    if y0 + net_h < scaled_h:
                        cut |= boxes[:, 3] >= net_h - 1.0
                    boxes, scores, classes = (boxes[~cut], scores[~cut],
                                              classes[~cut])
                all_boxes.append(boxes + np.array([x0, y0, x0, y0],
                                                  np.float32))
                all_scores.append(scores)
                all_classes.append(classes)
        boxes = np.concatenate(all_boxes)
        scores = np.concatenate(all_scores)
        classes = np.concatenate(all_classes)
        keep = nms(boxes, scores, classes, self.iou_threshold)
        boxes = boxes[keep] / scale
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, width)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, height)
        return boxes, scores[keep], classes[keep]
