"""The detector side of the tracking contract (systems/aerodrone/tracking.md):
<prefix>/detections, /image/compressed and /camera_info as the OAK-1 node
and the simulator detector build them."""
import sys
from types import SimpleNamespace
from unittest import mock

import numpy  # noqa: F401  (loaded before sys.modules is patched below)
import pytest

import tracking_fakes as fakes
from conftest import REPO, install_ros_stubs, load_script

sys.path.insert(0, str(REPO / 'applications/oak1/scripts'))
import tracking_messages as tm  # noqa: E402


@pytest.fixture
def ros_messages():
    with mock.patch.dict(sys.modules, fakes.message_modules()):
        yield


def header(sec=12, nanosec=345_000_000, frame='oak1_optical_frame'):
    return fakes.Header(stamp=fakes.Time(sec=sec, nanosec=nanosec),
                        frame_id=frame)


def test_detections_carry_pixel_boxes_track_ids_and_classes(ros_messages):
    boxes = [tm.TrackedBox(7, 'car', 0.81, 100.0, 50.0, 140.0, 70.0),
             tm.TrackedBox(12, 'person', 0.42, 300.5, 200.0, 306.5, 212.0)]
    msg = tm.detection_array(header(), boxes)
    assert [d.id for d in msg.detections] == ['7', '12']
    car, person = msg.detections
    assert (car.bbox.center.position.x, car.bbox.center.position.y) == (120.0, 60.0)
    assert (car.bbox.size_x, car.bbox.size_y) == (40.0, 20.0)
    assert (person.bbox.center.position.x, person.bbox.size_x) == (303.5, 6.0)
    assert car.bbox.center.theta == 0.0
    assert car.results[0].hypothesis.class_id == 'car'
    assert car.results[0].hypothesis.score == pytest.approx(0.81)
    assert len(car.results) == 1


def test_every_message_keeps_the_source_image_stamp_and_frame(ros_messages):
    source = header(sec=1700, nanosec=5, frame='downward_camera_optical_frame')
    detections = tm.detection_array(source, [tm.TrackedBox(1, 'car', 0.9, 0, 0, 1, 1)])
    image = tm.compressed_jpeg(source, b'\xff\xd8jpeg')
    assert detections.header is source
    assert detections.detections[0].header is source
    assert image.header is source
    assert (image.header.stamp.sec, image.header.stamp.nanosec) == (1700, 5)
    assert image.header.frame_id == 'downward_camera_optical_frame'
    assert image.format == 'jpeg' and image.data == b'\xff\xd8jpeg'


def test_empty_frames_still_publish_an_array(ros_messages):
    msg = tm.detection_array(header(), [])
    assert msg.detections == [] and msg.header.stamp.sec == 12


def test_normalized_boxes_are_clipped_into_the_image():
    box = tm.box_from_normalized(3, 'pedestrian', 1.2, -0.1, 0.25, 0.5, 1.4,
                                 640, 352)
    assert (box.x1, box.y1, box.x2, box.y2) == (0.0, 88.0, 320.0, 352.0)
    assert box.score == 1.0 and box.class_name == 'person'


def test_class_names_follow_coco_spelling():
    assert tm.canonical_class('Pedestrian') == 'person'
    assert tm.canonical_class('people') == 'person'
    assert tm.canonical_class('motor') == 'motorcycle'
    assert tm.canonical_class('car') == 'car'
    assert tm.parse_class_list(' person, car ,') == ['person', 'car']


def test_labels_to_track_selects_model_classes_by_canonical_name():
    visdrone = ['pedestrian', 'people', 'bicycle', 'car', 'van', 'truck',
                'tricycle', 'awning-tricycle', 'bus', 'motor', 'others']
    assert tm.labels_to_track(visdrone, 'person,car') == [0, 1, 3]
    assert tm.labels_to_track(visdrone, '') == list(range(11))
    assert 10 not in tm.labels_to_track(visdrone, tm.DEFAULT_CLASSES)


def test_capture_time_is_ros_now_minus_the_frame_age():
    # Frame captured 0.25 s ago on the device clock.
    assert tm.capture_time_ns(1_000_000_000_000, 50.75, 50.5) == 999_750_000_000
    stamp = tm.set_stamp(SimpleNamespace(sec=0, nanosec=0), 999_750_000_123)
    assert (stamp.sec, stamp.nanosec) == (999, 750_000_123)
    assert tm.stamp_ns(stamp) == 999_750_000_123


def test_camera_info_scales_with_the_published_image(ros_messages):
    info = fakes.CameraInfo(width=1280, height=960, distortion_model='plumb_bob',
                            d=[0.1, 0, 0, 0, 0],
                            k=[1108.6, 0, 639.5, 0, 1108.6, 479.5, 0, 0, 1],
                            p=[1108.6, 0, 639.5, 0, 0, 1108.6, 479.5, 0, 0, 0, 1, 0])
    out = tm.scaled_camera_info(info, 640, 480, header(frame='cam'))
    assert (out.width, out.height) == (640, 480)
    assert out.k[0] == pytest.approx(554.3) and out.k[4] == pytest.approx(554.3)
    # Pixel centres: the image centre stays the centre.
    assert (out.k[2], out.k[5]) == (pytest.approx(319.5), pytest.approx(239.5))
    assert (out.p[0], out.p[2], out.p[6]) == (
        pytest.approx(554.3), pytest.approx(319.5), pytest.approx(239.5))
    assert out.d == info.d and out.header.frame_id == 'cam'


def test_model_meta_needs_classes_and_input_size(tmp_path):
    good = tmp_path / 'model.json'
    good.write_text('{"classes": ["car"], "input_width": 640, "input_height": 352}')
    assert tm.load_model_meta(good)['classes'] == ['car']
    bad = tmp_path / 'bad.json'
    bad.write_text('{"classes": ["car"]}')
    with pytest.raises(ValueError):
        tm.load_model_meta(bad)


def test_oak1_tracklets_become_boxes_of_new_and_tracked_objects():
    with mock.patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['depthai'] = mock.MagicMock()
        node = load_script(REPO / 'applications/oak1/scripts/megadepth_ros.py',
                           'megadepth_tracking_review')

    def tracklet(track_id, label, status, x0, y0, x1, y1, conf):
        roi = SimpleNamespace(topLeft=lambda: SimpleNamespace(x=x0, y=y0),
                              bottomRight=lambda: SimpleNamespace(x=x1, y=y1))
        return SimpleNamespace(id=track_id, label=label,
                               status=SimpleNamespace(name=status), roi=roi,
                               srcImgDetection=SimpleNamespace(confidence=conf))

    classes = ['pedestrian', 'people', 'bicycle', 'car']
    tracklets = [tracklet(4, 3, 'TRACKED', 0.5, 0.5, 0.6, 0.55, 0.7),
                 tracklet(5, 0, 'NEW', 0.1, 0.1, 0.11, 0.12, 0.4),
                 tracklet(6, 3, 'LOST', 0.2, 0.2, 0.3, 0.3, 0.9),
                 tracklet(7, 3, 'REMOVED', 0.2, 0.2, 0.3, 0.3, 0.9)]
    boxes = node.tracklet_boxes(tracklets, classes, 640, 352)
    assert [(b.track_id, b.class_name) for b in boxes] == [(4, 'car'), (5, 'person')]
    car = boxes[0]
    assert (car.x1, car.y1, car.x2, car.y2) == pytest.approx((320, 176, 384, 193.6))
    assert car.score == pytest.approx(0.7)
    with_lost = node.tracklet_boxes(tracklets, classes, 640, 352, publish_lost=True)
    assert [b.track_id for b in with_lost] == [4, 5, 6]
    # MegaDepth's input cap: none without tracking, 3 Hz with it, or as set.
    assert node.megadepth_rate(-1.0, False) == 0.0
    assert node.megadepth_rate(-1.0, True) == node.TRACKING_MEGADEPTH_RATE_HZ
    assert node.megadepth_rate(2.0, True) == 2.0
    assert node.megadepth_rate(0.0, True) == 0.0
    # With nothing subscribed the detector pauses and MegaDepth runs free.
    assert node.tracking_rates(True, 5.0, 3.0) == (5.0, 3.0)
    assert node.tracking_rates(False, 5.0, 3.0) == (None, 0.0)
