"""The simulator detector's host-side tracker: stable IDs through short
occlusions and missed detections, never reused, with the class following
the detector."""
import sys

import numpy as np
import pytest

from conftest import REPO

sys.path.insert(0, str(REPO / 'applications/oak1/scripts'))
from host_tracker import HostTracker  # noqa: E402

DT = 0.2  # 5 Hz


def box(cx, cy, w=40.0, h=20.0):
    return [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]


def step(tracker, t, detections):
    """detections: list of (box, score, class) -> {track id: box}."""
    boxes = [d[0] for d in detections]
    scores = [d[1] for d in detections]
    classes = [d[2] for d in detections]
    return {tid: (b, s, c) for tid, b, s, c in tracker.update(boxes, scores, classes, t)}


def test_new_track_is_published_from_its_second_detection():
    tracker = HostTracker()
    assert step(tracker, 0.0, [(box(100, 100), 0.9, 'car')]) == {}
    out = step(tracker, DT, [(box(104, 100), 0.9, 'car')])
    assert list(out) == [1]
    assert out[1][2] == 'car'
    np.testing.assert_allclose(out[1][0], box(104, 100))


def test_a_single_false_detection_never_gets_an_id():
    tracker = HostTracker()
    step(tracker, 0.0, [(box(300, 300), 0.9, 'person')])
    assert step(tracker, DT, []) == {}
    assert step(tracker, 2 * DT, [(box(300, 300), 0.9, 'person')]) == {}
    assert step(tracker, 3 * DT, [(box(300, 300), 0.9, 'person')]).keys() == {2}


def test_moving_car_keeps_its_id_through_a_one_second_occlusion():
    tracker = HostTracker()
    ids = set()
    t = 0.0
    x = 50.0
    for i in range(30):
        x += 25.0 * DT * 5  # 25 px per frame, faster than its own width / 2
        hidden = 10 <= i < 15  # 5 frames = 1 s under a tree
        out = step(tracker, t, [] if hidden else [(box(x, 200), 0.85, 'car')])
        if hidden:
            assert out == {}
        ids |= set(out)
        t += DT
    assert ids == {1}


def test_weak_detections_carry_a_confirmed_track():
    tracker = HostTracker(high_score=0.4, low_score=0.1)
    step(tracker, 0.0, [(box(100, 100), 0.8, 'person')])
    step(tracker, DT, [(box(101, 100), 0.8, 'person')])
    out = step(tracker, 2 * DT, [(box(102, 100), 0.15, 'person')])
    assert list(out) == [1] and out[1][1] == pytest.approx(0.15)
    # ... but a weak detection alone never starts a track.
    assert step(tracker, 3 * DT, [(box(103, 100), 0.8, 'person'),
                                  (box(400, 300), 0.2, 'person')]).keys() == {1}
    assert step(tracker, 4 * DT, [(box(104, 100), 0.8, 'person'),
                                  (box(400, 300), 0.2, 'person')]).keys() == {1}


def test_two_people_crossing_keep_their_ids():
    tracker = HostTracker()
    seen = {}
    for i in range(20):
        t = i * DT
        a = (box(100 + 10 * i, 100, 8, 12), 0.7, 'person')
        b = (box(290 - 10 * i, 104, 8, 12), 0.7, 'person')
        out = step(tracker, t, [a, b])
        for tid, (bx, _, _) in out.items():
            seen.setdefault(tid, []).append((bx[0] + bx[2]) / 2)
    assert set(seen) == {1, 2}
    # each ID followed one person across: monotonic centre motion
    assert all(np.all(np.diff(xs) > 0) for xs in [seen[1]])
    assert all(np.all(np.diff(xs) < 0) for xs in [seen[2]])


def test_a_lost_track_expires_and_ids_are_not_reused():
    tracker = HostTracker(max_lost_s=1.0)
    step(tracker, 0.0, [(box(100, 100), 0.9, 'car')])
    step(tracker, DT, [(box(100, 100), 0.9, 'car')])
    step(tracker, 2.0, [])  # gone for 1.8 s
    step(tracker, 2.0 + DT, [(box(100, 100), 0.9, 'car')])
    out = step(tracker, 2.0 + 2 * DT, [(box(100, 100), 0.9, 'car')])
    assert list(out) == [2]


def test_small_fast_object_is_recovered_by_distance_after_missing_frames():
    # A 6 px person moving 8 px per frame: after two missed frames its
    # prediction no longer overlaps; the centre-distance pass reattaches it.
    tracker = HostTracker()
    t, x = 0.0, 100.0
    ids = set()
    for i in range(12):
        x += 8.0
        if i in (5, 6):
            step(tracker, t, [])
        else:
            ids |= set(step(tracker, t, [(box(x, 50, 6, 6), 0.6, 'person')]))
        t += DT
    assert ids == {1}


def test_class_follows_the_latest_confident_detection_without_new_id():
    tracker = HostTracker()
    step(tracker, 0.0, [(box(100, 100), 0.9, 'car')])
    step(tracker, DT, [(box(100, 100), 0.9, 'car')])
    out = step(tracker, 2 * DT, [(box(100, 100), 0.9, 'van')])
    assert list(out) == [1] and out[1][2] == 'van'
    out = step(tracker, 3 * DT, [(box(100, 100), 0.2, 'truck')])
    assert out[1][2] == 'van'  # weak detections do not relabel


def test_simulation_restart_resets_the_tracks():
    tracker = HostTracker()
    step(tracker, 100.0, [(box(100, 100), 0.9, 'car')])
    step(tracker, 100.2, [(box(100, 100), 0.9, 'car')])
    assert step(tracker, 1.0, [(box(100, 100), 0.9, 'car')]) == {}
    assert list(step(tracker, 1.2, [(box(100, 100), 0.9, 'car')])) == [2]


def test_a_second_offset_box_on_one_person_starts_no_track():
    # Seen from 30 m a person can come out as two boxes a few pixels apart
    # (body and shadow); the second must not take turns on the person.
    tracker = HostTracker()
    ids = set()
    for i in range(10):
        detections = [(box(300, 100, 12, 16), 0.5, 'person')]
        if i >= 3:
            detections.append((box(305, 104, 12, 16), 0.45, 'person'))
        ids |= set(step(tracker, i * DT, detections))
    assert ids == {1}


def test_people_standing_close_but_apart_keep_two_tracks():
    tracker = HostTracker()
    for i in range(4):
        out = step(tracker, i * DT, [(box(300, 100, 10, 14), 0.6, 'person'),
                                     (box(314, 100, 10, 14), 0.6, 'person')])
    assert set(out) == {1, 2}


def test_a_person_beside_a_car_is_not_a_duplicate_of_it():
    tracker = HostTracker()
    for i in range(4):
        out = step(tracker, i * DT, [(box(300, 100, 90, 40), 0.8, 'car'),
                                     (box(330, 110, 10, 14), 0.5, 'person')])
    assert set(out) == {1, 2}


def test_a_car_whose_box_jumps_does_not_take_over_a_person_track_beside_it():
    # Seen from a tilting aircraft the car's box jumps and misses its
    # prediction, landing on a person the detector missed in that frame:
    # the car's box must not take the person's id (it was nearer than the
    # car's own prediction), nor, later, a person's box the car's.
    tracker = HostTracker()
    person = (box(340, 140, 10, 14), 0.6, 'person')
    for i in range(4):
        out = step(tracker, i * DT, [(box(300, 100, 60, 30), 0.8, 'car'), person])
    assert set(out) == {1, 2} and out[2][2] == 'person'
    out = step(tracker, 4 * DT, [(box(335, 140, 60, 30), 0.8, 'car')])
    assert 2 not in out
    out = step(tracker, 5 * DT, [(box(330, 150, 10, 14), 0.6, 'person')])
    assert 1 not in out
    out = step(tracker, 6 * DT, [(box(335, 140, 60, 30), 0.8, 'car'),
                                 (box(330, 150, 10, 14), 0.6, 'person')])
    assert all(c == 'car' for tid, (_, _, c) in out.items() if tid == 1)
    assert all(c == 'person' for tid, (_, _, c) in out.items() if tid == 2)


def test_a_car_prediction_shrinking_onto_a_person_does_not_take_its_box():
    """A car leaving the image: its clipped box shrinks 5 px a frame down to
    40 px, then it is missed. Its prediction keeps shrinking to person size
    over a person standing there, whose box must not join the car's track
    (by IoU with the prediction, or by distance)."""
    tracker = HostTracker()
    for i in range(5):
        w = 60.0 - 5.0 * i
        out = step(tracker, i * DT, [(box(300, 100, w, w / 2), 0.8, 'car')])
    car_id = list(out)[0]
    for i in range(5, 9):
        step(tracker, i * DT, [])                      # the car is missed
    out = step(tracker, 9 * DT, [(box(300, 100, 10, 14), 0.6, 'person')])
    assert car_id not in out


def test_a_hidden_person_track_does_not_reach_a_person_standing_nearby():
    """Person A (walking, tracked) disappears; person B stands 40 px from
    where A's track is predicted a second later (3 box sizes, the reach of
    a track seen a moment ago): B's box starts its own track."""
    tracker = HostTracker()
    a = lambda i: (box(100 + 4 * i, 100, 12, 14), 0.6, 'person')  # noqa: E731
    for i in range(6):
        out = step(tracker, i * DT, [a(i)])
    a_id = list(out)[0]
    for i in range(6, 11):
        step(tracker, i * DT, [])                      # A hidden for 1 s
    b = (box(100 + 4 * 11 + 40, 100, 12, 14), 0.6, 'person')
    step(tracker, 11 * DT, [b])
    out = step(tracker, 12 * DT, [b])
    assert a_id not in out and len(out) == 1
