"""Host-side multi-object tracker for the simulator detector: stable track
IDs where the OAK-1 uses DepthAI's on-device ObjectTracker.

ByteTrack-style association on a constant-velocity Kalman filter per track
(state: box centre, width, height and their rates; time steps from the
image stamps, which need not be regular):

1. confident detections (score >= high_score) are matched to every live
   track, by IoU with its predicted box, when their size is comparable to
   the track's last box (longer sides within max_size_ratio: a car's
   prediction can shrink onto a person beside it);
2. the remaining confirmed tracks take the weaker detections (score >=
   low_score) by IoU, which carries a track through frames where the
   detector is unsure (partial occlusion);
3. still-unmatched confident detections are matched to the remaining
   tracks of a comparable size by
   centre distance, relative to box size, from the track's predicted box
   or its last one, whichever is nearer: a small or fast object may not
   overlap its prediction (a new track has no velocity yet, a hidden one
   may have turned), and a body-fixed camera on a manoeuvring aircraft
   moves everything in the image by tens of pixels a frame, back and forth
   (recover_distance 3 box sizes for a track seen within recover_window_s;
   in Gazebo's flights 1.5 changed the followed car's id every 4 s; 1.5 for
   one gone longer, which would otherwise reach a look-alike beside it), but
   a car's box must not take over a person's track next to it, nor a
   person's box the car's;
4. the remaining confident detections start tentative tracks, confirmed
   after min_hits consecutive frames; a confirmed track unmatched for
   max_lost_s is dropped;
5. like DepthAI's occlusion ratio, a detection covering more than
   occlusion_ratio of a comparable-sized track (or covered by it) starts
   no track, and of two such overlapping tracks the younger is dropped: a
   small person seen from above can come out as two offset boxes (body
   and shadow) that would otherwise take turns on one ID.

IDs are unique and never reused. Matching ignores the class (a model that
hesitates between car and van must not split a track); a track reports the
class of its latest confident detection. Greedy matching on the best pair
first: no scipy.
"""
import itertools

import numpy as np


def _iou(a, b):
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


def _overlap(a, b, max_area_ratio=4.0):
    """Intersection over the smaller area of every box in a with every box
    in b, 0 where one box is over max_area_ratio times the other (a person
    beside a car is not a duplicate of it)."""
    a = np.asarray(a, dtype=np.float64).reshape(-1, 4)
    b = np.asarray(b, dtype=np.float64).reshape(-1, 4)
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = ((a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1]))[:, None]
    area_b = ((b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1]))[None, :]
    small = np.maximum(np.minimum(area_a, area_b), 1e-9)
    ratio = inter / small
    comparable = np.maximum(area_a, area_b) <= max_area_ratio * small
    return np.where(comparable, ratio, 0.0)


def _longer_side(boxes):
    boxes = np.asarray(boxes, dtype=np.float64).reshape(-1, 4)
    return np.maximum(boxes[:, 2] - boxes[:, 0], boxes[:, 3] - boxes[:, 1])


def _comparable(track_boxes, det_boxes, max_ratio):
    """Whether each track's last box and each detection have longer sides
    within max_ratio of each other (a person's box is not a car's)."""
    side_t = _longer_side(track_boxes)[:, None]
    side_d = _longer_side(det_boxes)[None, :]
    return (np.maximum(side_t, side_d) <=
            max_ratio * np.maximum(np.minimum(side_t, side_d), 1.0))


def _greedy(score, threshold, higher_is_better=True):
    """Pairs (row, col) by best score first, each row and column once."""
    if score.size == 0:
        return []
    order = np.argsort(-score if higher_is_better else score, axis=None)
    rows, cols, pairs = set(), set(), []
    for flat in order:
        r, c = divmod(int(flat), score.shape[1])
        value = score[r, c]
        if (value < threshold) if higher_is_better else (value > threshold):
            break
        if r in rows or c in cols:
            continue
        rows.add(r)
        cols.add(c)
        pairs.append((r, c))
    return pairs


def box_to_state(box):
    x1, y1, x2, y2 = box
    return np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1])


def state_to_box(z):
    cx, cy, w, h = z[:4]
    w, h = max(w, 1e-3), max(h, 1e-3)
    return np.array([cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0])


class KalmanBox:
    """Constant-velocity Kalman filter on (cx, cy, w, h), noise scaled by
    the box size as in SORT/ByteTrack."""

    POSITION_STD = 1.0 / 20.0
    VELOCITY_STD = 1.0 / 10.0  # per second

    def __init__(self, box):
        z = box_to_state(box)
        self.x = np.concatenate([z, np.zeros(4)])
        size = max(z[2], z[3], 1.0)
        std = np.r_[[2 * self.POSITION_STD * size] * 4,
                    [10 * self.VELOCITY_STD * size] * 4]
        self.P = np.diag(std ** 2)

    def predict(self, dt):
        dt = max(0.0, float(dt))
        F = np.eye(8)
        F[:4, 4:] = np.eye(4) * dt
        size = max(self.x[2], self.x[3], 1.0)
        q_pos = (self.POSITION_STD * size) ** 2 * dt
        q_vel = (self.VELOCITY_STD * size) ** 2 * dt
        Q = np.diag([q_pos] * 4 + [q_vel] * 4)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        return state_to_box(self.x)

    def update(self, box):
        z = box_to_state(box)
        size = max(z[2], z[3], 1.0)
        R = np.diag([(self.POSITION_STD * size) ** 2] * 4)
        H = np.zeros((4, 8))
        H[:4, :4] = np.eye(4)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(8) - K @ H) @ self.P

    @property
    def box(self):
        return state_to_box(self.x)


class Track:
    TENTATIVE, CONFIRMED = 'tentative', 'confirmed'

    def __init__(self, track_id, box, score, cls, stamp):
        self.id = track_id
        self.kf = KalmanBox(box)
        self.box = np.asarray(box, dtype=np.float64)
        self.score = float(score)
        self.cls = cls
        self.hits = 1
        self.state = self.TENTATIVE
        self.last_seen = stamp
        self.updated = True

    def update(self, box, score, cls, stamp, confident):
        self.kf.update(box)
        self.box = np.asarray(box, dtype=np.float64)
        self.score = float(score)
        if confident:
            self.cls = cls
        self.hits += 1
        self.last_seen = stamp
        self.updated = True


class HostTracker:
    def __init__(self, high_score=0.4, low_score=0.1, iou_threshold=0.3,
                 low_iou_threshold=0.5, recover_distance=3.0, min_hits=2,
                 max_lost_s=2.0, occlusion_ratio=0.4, max_size_ratio=2.0,
                 recover_from_last=True, recover_window_s=0.5,
                 recover_distance_lost=1.5):
        self.high_score = float(high_score)
        self.low_score = float(low_score)
        self.iou_threshold = float(iou_threshold)
        self.low_iou_threshold = float(low_iou_threshold)
        self.recover_distance = float(recover_distance)
        self.recover_window_s = float(recover_window_s)
        self.recover_distance_lost = float(recover_distance_lost)
        self.max_size_ratio = float(max_size_ratio)
        self.recover_from_last = bool(recover_from_last)
        self.min_hits = int(min_hits)
        self.max_lost_s = float(max_lost_s)
        self.occlusion_ratio = float(occlusion_ratio)
        self.tracks = []
        self._ids = itertools.count(1)
        self.stamp = None

    def reset(self):
        self.tracks = []
        self.stamp = None

    def update(self, boxes, scores, classes, stamp):
        """Associate one frame's detections (pixels x1y1x2y2, scores, class
        labels) at time stamp (s). Returns the confirmed tracks updated by
        this frame, as (track_id, box, score, class)."""
        boxes = np.asarray(boxes, dtype=np.float64).reshape(-1, 4)
        scores = np.asarray(scores, dtype=np.float64).reshape(-1)
        classes = list(classes)
        if self.stamp is not None and stamp < self.stamp - 1.0:
            self.reset()  # time went back (simulation restarted)
        dt = 0.0 if self.stamp is None else max(0.0, stamp - self.stamp)
        self.stamp = stamp
        for track in self.tracks:
            track.updated = False
            track.kf.predict(dt)

        high = np.flatnonzero(scores >= self.high_score)
        low = np.flatnonzero((scores >= self.low_score) &
                             (scores < self.high_score))
        unmatched_high = list(high)
        free = list(range(len(self.tracks)))

        def associate(det_idx, track_idx, threshold):
            if not det_idx or not track_idx:
                return det_idx, track_idx
            predicted = np.array([self.tracks[t].kf.box for t in track_idx])
            iou = _iou(predicted, boxes[det_idx])
            # Sizes against each track's last seen box: a prediction can
            # shrink onto a small object beside it.
            last = np.array([self.tracks[t].box for t in track_idx])
            iou = np.where(_comparable(last, boxes[det_idx], self.max_size_ratio), iou, 0.0)
            matched_t, matched_d = set(), set()
            for r, c in _greedy(iou, threshold):
                t, d = track_idx[r], det_idx[c]
                self.tracks[t].update(boxes[d], scores[d], classes[d], stamp,
                                      scores[d] >= self.high_score)
                matched_t.add(t)
                matched_d.add(d)
            return ([d for d in det_idx if d not in matched_d],
                    [t for t in track_idx if t not in matched_t])

        # 1. confident detections, every live track
        unmatched_high, free = associate(unmatched_high, free,
                                         self.iou_threshold)
        # 2. weak detections, confirmed tracks only
        confirmed = [t for t in free
                     if self.tracks[t].state == Track.CONFIRMED]
        _, left = associate(list(low), confirmed, self.low_iou_threshold)
        free = [t for t in free if t not in confirmed or t in left]
        # 3. confident detections to the remaining tracks by distance
        if unmatched_high and free:
            predicted = np.array([self.tracks[t].kf.box for t in free])
            centres_t = (predicted[:, :2] + predicted[:, 2:]) / 2.0
            dets = boxes[unmatched_high]
            centres_d = (dets[:, :2] + dets[:, 2:]) / 2.0
            last = np.array([self.tracks[t].box for t in free])
            scale = np.maximum(_longer_side(last)[:, None], _longer_side(dets)[None, :])
            distance = np.linalg.norm(
                centres_t[:, None] - centres_d[None], axis=2) / np.maximum(scale, 1.0)
            if self.recover_from_last:
                centres_l = (last[:, :2] + last[:, 2:]) / 2.0
                distance = np.minimum(distance, np.linalg.norm(
                    centres_l[:, None] - centres_d[None], axis=2) / np.maximum(scale, 1.0))
            distance = np.where(_comparable(last, dets, self.max_size_ratio),
                                distance, np.inf)
            # The long reach only for a track seen a moment ago: one gone
            # longer would reach a look-alike beside the object.
            recent = np.array([stamp - self.tracks[t].last_seen <= self.recover_window_s
                               for t in free])
            distance = distance / np.where(recent, self.recover_distance,
                                           self.recover_distance_lost)[:, None]
            matched_d = set()
            for r, c in _greedy(distance, 1.0, higher_is_better=False):
                d = unmatched_high[c]
                self.tracks[free[r]].update(boxes[d], scores[d], classes[d],
                                            stamp, True)
                matched_d.add(d)
            unmatched_high = [d for d in unmatched_high if d not in matched_d]

        # 4. confirmation, deletion, births
        kept = []
        for track in self.tracks:
            if track.updated:
                if track.state == Track.TENTATIVE and track.hits >= self.min_hits:
                    track.state = Track.CONFIRMED
                kept.append(track)
            elif (track.state == Track.CONFIRMED and
                  stamp - track.last_seen <= self.max_lost_s):
                kept.append(track)
            # unmatched tentative tracks die at once
        # 5. duplicates: no birth on top of a seen track; of two seen tracks
        # on one object the older stays
        seen = [t for t in kept if t.updated]
        seen.sort(key=lambda t: t.id)
        drop = set()
        for i, older in enumerate(seen):
            if older.id in drop:
                continue
            for younger in seen[i + 1:]:
                if (younger.id not in drop and _overlap(
                        older.box, younger.box)[0, 0] > self.occlusion_ratio):
                    drop.add(younger.id)
        self.tracks = [t for t in kept if t.id not in drop]
        seen_boxes = [t.box for t in self.tracks if t.updated]
        for d in unmatched_high:
            if seen_boxes and _overlap(boxes[d], seen_boxes).max() > self.occlusion_ratio:
                continue
            self.tracks.append(Track(next(self._ids), boxes[d], scores[d],
                                     classes[d], stamp))
            seen_boxes.append(boxes[d])
        if self.min_hits <= 1:
            for track in self.tracks:
                if track.updated:
                    track.state = Track.CONFIRMED
        return [(t.id, t.box.copy(), t.score, t.cls) for t in self.tracks
                if t.updated and t.state == Track.CONFIRMED]
