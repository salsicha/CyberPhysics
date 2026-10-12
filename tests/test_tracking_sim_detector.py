"""The simulators' tracking detector: the YOLO output decoding the OAK-1
does on the camera (tracking_yolo.py), the windows a 640x480 frame is cut
into, and the published messages (sim_tracking_detector.py)."""
import sys
from types import SimpleNamespace
from unittest import mock

import cv2
import numpy as np
import pytest

import tracking_fakes as fakes
from conftest import REPO, install_ros_stubs, load_script

sys.path.insert(0, str(REPO / 'applications/oak1/scripts'))
import tracking_yolo as ty  # noqa: E402

STRIDES = [8, 16, 32]


def outputs(width=640, height=352, classes=11, cells=()):
    """Raw per-stride outputs with chosen cells: (stride, row, col, ltrb,
    class, score)."""
    outs = [np.zeros((1, 5 + classes, height // s, width // s), np.float32)
            for s in STRIDES]
    for stride, row, col, ltrb, cls, score in cells:
        out = outs[STRIDES.index(stride)][0]
        out[:4, row, col] = ltrb
        out[4, row, col] = score
        out[5 + cls, row, col] = score
    return outs


def test_decode_matches_depthai_anchor_free_boxes():
    outs = outputs(cells=[(8, 10, 20, (2, 1, 3, 2), 3, 0.9),
                          (32, 2, 4, (1, 1, 1, 1), 0, 0.2)])
    boxes, scores, classes = ty.decode(outs, STRIDES, 0.3)
    np.testing.assert_allclose(boxes, [[148, 76, 188, 100]])
    assert scores.tolist() == pytest.approx([0.9]) and classes.tolist() == [3]
    boxes, _, classes = ty.decode(outs, STRIDES, 0.1)
    assert sorted(classes.tolist()) == [0, 3]
    assert [112, 48, 176, 112] in boxes.tolist()  # ((4.5, 2.5) -+ 1) * 32


def test_suppression_crosses_classes_like_the_camera():
    boxes = np.array([[0, 0, 10, 10], [1, 0, 11, 10], [50, 50, 60, 60]], float)
    scores = np.array([0.9, 0.8, 0.7])
    classes = np.array([3, 4, 3])  # car and van on the same object
    assert ty.nms(boxes, scores, classes, 0.5).tolist() == [0, 2]
    assert ty.nms(boxes, scores, classes, 0.5, agnostic=False).tolist() == [0, 1, 2]


def test_simulator_frames_are_cut_into_network_windows():
    # 640x480 into 640x352: two windows overlapping by 224 rows.
    assert ty.plan((640, 480), (640, 352)) == (1.0, (640, 480), [0], [0, 128])
    # 1280x720 is scaled to 640x360 first.
    assert ty.plan((1280, 720), (640, 352)) == (0.5, (640, 360), [0], [0, 8])
    assert ty.windows(352, 352) == [0]
    starts = ty.windows(2000, 640)
    assert starts[0] == 0 and starts[-1] == 2000 - 640
    assert all(b - a <= 640 - 64 for a, b in zip(starts, starts[1:]))


def detector_with(per_window):
    """A YoloDetector whose network returns per_window[y0] for the window
    starting at that row."""
    detector = object.__new__(ty.YoloDetector)
    detector.input_size = (640, 352)
    detector.strides = STRIDES
    detector.conf_threshold = 0.3
    detector.iou_threshold = 0.5
    detector.classes = ['person', 'car']
    calls = []

    def infer(rgb):
        y0 = [0, 128][len(calls) % 2]
        calls.append(rgb.shape)
        return per_window.get(y0, outputs(classes=2))
    detector.infer = infer
    return detector, calls


def test_window_detections_land_in_frame_pixels_once():
    # The same car seen by both windows (frame rows 200-216) and a person
    # only in the lower one (frame rows 400-408).
    top = outputs(classes=2, cells=[(8, 26, 40, (2, 1, 2, 1), 1, 0.8)])
    bottom = outputs(classes=2, cells=[(8, 10, 40, (2, 1, 2, 1), 1, 0.7),
                                       (8, 34, 10, (0.5, 0.5, 0.5, 0.5), 0, 0.6)])
    detector, calls = detector_with({0: top, 128: bottom})
    boxes, scores, classes = detector.detect(np.zeros((480, 640, 3), np.uint8))
    assert calls == [(352, 640, 3), (352, 640, 3)]
    order = np.argsort(classes)
    np.testing.assert_allclose(boxes[order], [[80, 400, 88, 408], [308, 204, 340, 220]])
    assert scores[order].tolist() == pytest.approx([0.6, 0.8])


def test_a_box_cut_by_an_inner_window_edge_is_dropped():
    # A box touching the top window's bottom edge (352) is whole in the
    # lower window, which reports it.
    cut = outputs(classes=2, cells=[(8, 43, 40, (2, 2, 2, 0.6), 1, 0.9)])
    whole = outputs(classes=2, cells=[(8, 28, 40, (2, 2, 2, 2), 1, 0.5)])
    detector, _ = detector_with({0: cut, 128: whole})
    boxes, scores, _ = detector.detect(np.zeros((480, 640, 3), np.uint8))
    assert scores.tolist() == pytest.approx([0.5])
    np.testing.assert_allclose(boxes, [[308, 340, 340, 372]])


@pytest.fixture
def node_module():
    with mock.patch.dict(sys.modules, fakes.message_modules()):
        install_ros_stubs()
        sys.modules['rclpy.node'].Node = object
        module = load_script(REPO / 'applications/oak1/scripts/sim_tracking_detector.py',
                             'sim_tracking_detector_review')
        yield module


def test_published_image_is_at_most_640_wide(node_module):
    assert node_module.output_size(640, 480, 640) == (640, 480)
    assert node_module.output_size(1280, 960, 640) == (640, 480)
    assert node_module.output_size(1280, 960, 0) == (1280, 960)


def test_one_frame_publishes_tracks_jpeg_and_intrinsics_with_its_stamp(node_module):
    from host_tracker import HostTracker

    published = {}
    node = object.__new__(node_module.SimTrackingDetector)
    frame = np.full((960, 1280, 3), 90, np.uint8)
    node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda msg, desired_encoding: frame)
    node.detector = SimpleNamespace(detect=lambda img: (
        np.array([[200.0, 100.0, 280.0, 140.0], [10, 10, 20, 20]]),
        np.array([0.9, 0.8]), np.array([3, 10])))
    node.classes = ['pedestrian', 'people', 'bicycle', 'car', 'van', 'truck',
                    'tricycle', 'awning-tricycle', 'bus', 'motor', 'others']
    node.wanted = np.array([True] * 10 + [False])  # 'others' is not tracked
    node.tracker = HostTracker(min_hits=1)
    node.max_width = 640
    node.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, 80]
    node.camera_info = fakes.CameraInfo(
        width=1280, height=960, k=[1108.6, 0, 639.5, 0, 1108.6, 479.5, 0, 0, 1],
        header=fakes.Header(frame_id='downward_camera_optical_frame'))
    for name in ('detections_pub', 'image_pub', 'info_pub'):
        setattr(node, name, SimpleNamespace(
            publish=lambda msg, name=name: published.__setitem__(name, msg)))
    node.processed, node.processing_s = 0, 0.0
    header = fakes.Header(stamp=fakes.Time(sec=321, nanosec=500_000_000),
                          frame_id='downward_camera_optical_frame')
    node._process(fakes.Image(header=header, width=1280, height=960))

    detections = published['detections_pub']
    assert detections.header is header
    (car,) = detections.detections
    assert car.id == '1'
    assert car.results[0].hypothesis.class_id == 'car'
    # Boxes in the published (half-size) image's pixels.
    assert (car.bbox.center.position.x, car.bbox.center.position.y) == (120.0, 60.0)
    assert (car.bbox.size_x, car.bbox.size_y) == (40.0, 20.0)
    jpeg = published['image_pub']
    assert jpeg.header is header and jpeg.format == 'jpeg'
    assert cv2.imdecode(np.frombuffer(jpeg.data, np.uint8), cv2.IMREAD_COLOR).shape == (480, 640, 3)
    info = published['info_pub']
    assert info.header is header and (info.width, info.height) == (640, 480)
    assert info.k[0] == pytest.approx(554.3) and info.k[2] == pytest.approx(319.5)
