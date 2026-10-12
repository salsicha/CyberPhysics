"""The detector side of the aerodrone tracking contract
(systems/aerodrone/tracking.md): <prefix>/detections, <prefix>/image/compressed
and <prefix>/camera_info, shared by the OAK-1 node (megadepth_ros.py) and
the simulator detector (sim_tracking_detector.py).

Message classes are imported where messages are built, so the plain helpers
(class names, boxes, stamps, labels) need no ROS.
"""
from dataclasses import dataclass
import json
from pathlib import Path

# Class names published in results[0].hypothesis.class_id: COCO's names,
# with VisDrone's (and older COCO label files') spellings mapped onto them.
CLASS_ALIASES = {
    'pedestrian': 'person',
    'people': 'person',
    'motor': 'motorcycle',
    'motorbike': 'motorcycle',
    'awning-tricycle': 'tricycle',
}

# What the aircraft follows: people and road vehicles.
DEFAULT_CLASSES = ('person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck',
                   'van', 'tricycle')


def canonical_class(name):
    name = str(name).strip().lower()
    return CLASS_ALIASES.get(name, name)


def parse_class_list(value):
    """'person, car' or ['person', 'car'] -> canonical names; empty -> []
    (every class)."""
    if isinstance(value, str):
        value = value.split(',')
    return [canonical_class(v) for v in value if str(v).strip()]


def labels_to_track(model_classes, wanted):
    """Indices of the model's classes whose canonical name is wanted (all
    when wanted is empty)."""
    wanted = set(parse_class_list(wanted))
    return [i for i, name in enumerate(model_classes)
            if not wanted or canonical_class(name) in wanted]


def load_model_meta(path):
    """The json export_yolo.py writes beside each model."""
    meta = json.loads(Path(path).read_text())
    for key in ('classes', 'input_width', 'input_height'):
        if key not in meta:
            raise ValueError(f'{path}: no "{key}"')
    return meta


@dataclass
class TrackedBox:
    """One tracked object in pixels of the published image."""
    track_id: int
    class_name: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center(self):
        return (0.5 * (self.x1 + self.x2), 0.5 * (self.y1 + self.y2))

    @property
    def size(self):
        return (self.x2 - self.x1, self.y2 - self.y1)


def box_from_normalized(track_id, class_name, score, xmin, ymin, xmax, ymax,
                        width, height):
    """A TrackedBox from DepthAI's [0, 1] coordinates, clipped to the
    image."""
    def clip(value):
        return min(1.0, max(0.0, float(value)))
    return TrackedBox(int(track_id), canonical_class(class_name),
                      clip(score), clip(xmin) * width, clip(ymin) * height,
                      clip(xmax) * width, clip(ymax) * height)


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def set_stamp(stamp, nanoseconds):
    nanoseconds = int(nanoseconds)
    stamp.sec = nanoseconds // 1_000_000_000
    stamp.nanosec = nanoseconds % 1_000_000_000
    return stamp


def capture_time_ns(ros_now_ns, device_now_s, frame_s):
    """ROS time of a frame captured at device-clock frame_s, given ROS now
    and the device clock now (DepthAI's host-synchronised steady clock):
    the frame's age is the same on both clocks."""
    return int(ros_now_ns - round((device_now_s - frame_s) * 1e9))


def detection_array(header, boxes):
    """vision_msgs/Detection2DArray: one Detection2D per TrackedBox, id the
    track ID, bbox centre and size in pixels, results[0] the class and
    score."""
    from vision_msgs.msg import (Detection2D, Detection2DArray,
                                 ObjectHypothesisWithPose)

    msg = Detection2DArray()
    msg.header = header
    for box in boxes:
        detection = Detection2D()
        detection.header = header
        detection.id = str(box.track_id)
        cx, cy = box.center
        width, height = box.size
        detection.bbox.center.position.x = float(cx)
        detection.bbox.center.position.y = float(cy)
        detection.bbox.center.theta = 0.0
        detection.bbox.size_x = float(width)
        detection.bbox.size_y = float(height)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = box.class_name
        hypothesis.hypothesis.score = float(box.score)
        detection.results.append(hypothesis)
        msg.detections.append(detection)
    return msg


def compressed_jpeg(header, data):
    from sensor_msgs.msg import CompressedImage

    msg = CompressedImage()
    msg.header = header
    msg.format = 'jpeg'
    msg.data = bytes(data)
    return msg


def scaled_camera_info(info, width, height, header=None):
    """A copy of CameraInfo for the same view resized to width x height
    (pixel-centre convention, as DemNav scales intrinsics)."""
    from sensor_msgs.msg import CameraInfo

    out = CameraInfo()
    out.header = header if header is not None else info.header
    out.width, out.height = int(width), int(height)
    sx = width / float(info.width) if info.width else 1.0
    sy = height / float(info.height) if info.height else 1.0
    k = list(info.k)
    k[0], k[1] = k[0] * sx, k[1] * sx
    k[2] = (k[2] + 0.5) * sx - 0.5
    k[4] = k[4] * sy
    k[5] = (k[5] + 0.5) * sy - 0.5
    out.k = k
    p = list(info.p)
    if any(p):
        p[0], p[1] = p[0] * sx, p[1] * sx
        p[2] = (p[2] + 0.5) * sx - 0.5
        p[3] = p[3] * sx
        p[5] = p[5] * sy
        p[6] = (p[6] + 0.5) * sy - 0.5
        p[7] = p[7] * sy
    out.p = p
    out.d = list(info.d)
    out.distortion_model = info.distortion_model
    out.r = list(info.r)
    return out
