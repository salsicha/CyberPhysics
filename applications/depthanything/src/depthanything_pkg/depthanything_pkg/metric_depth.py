"""Depth Anything V2 metric depth helpers shared by depth_node and
depth_accuracy: the model configurations and checkpoints, the range mask
the node publishes with, depth images in metres, per-frame scale alignment
to a metric reference and the error statistics of an estimate against a
reference depth.

numpy only at import time (tests/test_depthanything_metric_depth.py);
load_model imports torch and the upstream Depth-Anything-V2 code.
"""

from __future__ import annotations

import math
import sys

import numpy as np

UPSTREAM = '/opt/Depth-Anything-V2'
CHECKPOINT_DIR = f'{UPSTREAM}/checkpoints'

MODEL_CONFIGS = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]},
}
# The max_depth each metric checkpoint was trained with (upstream
# metric_depth/README.md): the sigmoid head's output range.
TRAINED_MAX_DEPTH = {'vkitti': 80.0, 'hypersim': 20.0}


def checkpoint_path(dataset, encoder, root=CHECKPOINT_DIR):
    return f'{root}/depth_anything_v2_metric_{dataset}_{encoder}.pth'


def load_model(encoder, dataset, max_depth, model_path='', device='cpu'):
    """A Depth Anything V2 metric model in eval mode on `device`, and the
    checkpoint it loaded."""
    import os

    import torch

    # metric_depth first, so DepthAnythingV2 is the metric implementation.
    if f'{UPSTREAM}/metric_depth' not in sys.path:
        sys.path.insert(0, f'{UPSTREAM}/metric_depth')
    from depth_anything_v2.dpt import DepthAnythingV2

    if encoder not in MODEL_CONFIGS:
        raise ValueError(f'Unsupported encoder {encoder}; expected one of {sorted(MODEL_CONFIGS)}')
    model_path = model_path or checkpoint_path(dataset, encoder)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f'Metric Depth Anything weights not found at {model_path}')
    model = DepthAnythingV2(**MODEL_CONFIGS[encoder], max_depth=float(max_depth))
    model.load_state_dict(torch.load(model_path, map_location='cpu'))
    return model.to(device).eval(), model_path


def mask_range(depth, min_depth, max_depth):
    """float32 depth with non-finite and out-of-range values as NaN (what
    the node publishes)."""
    depth = np.array(depth, dtype=np.float32)
    with np.errstate(invalid='ignore'):
        depth[(~np.isfinite(depth)) | (depth < min_depth) | (depth > max_depth)] = np.nan
    return depth


def depth_image_to_m(pixels, encoding):
    """A depth image (32FC1 metres, or 16UC1/mono16 millimetres as the
    RealSense driver publishes it) as float32 metres, no measurement
    (0, negative, NaN, inf) as NaN."""
    if encoding == '32FC1':
        depth = np.array(pixels, dtype=np.float32)
    elif encoding in ('16UC1', 'mono16'):
        depth = np.asarray(pixels, dtype=np.float32) / 1000.0
    else:
        raise ValueError(f'unsupported depth encoding {encoding!r}')
    with np.errstate(invalid='ignore'):
        depth[~(np.isfinite(depth) & (depth > 0.0))] = np.nan
    return depth


def resize_nearest(image, shape):
    """Nearest-neighbour resize of a 2D array to shape (rows, cols)."""
    image = np.asarray(image)
    if image.shape[:2] == tuple(shape):
        return image
    rows = np.minimum((np.arange(shape[0]) + 0.5) * image.shape[0] / shape[0],
                      image.shape[0] - 1).astype(int)
    cols = np.minimum((np.arange(shape[1]) + 0.5) * image.shape[1] / shape[1],
                      image.shape[1] - 1).astype(int)
    return image[rows][:, cols]


def median_scale(reference, estimate, min_m, max_m, stride=1, offset=0, min_samples=1):
    """Per-frame scale alignment: the median of reference / estimate over
    the pixels, sampled every `stride` rows and columns from `offset`, where
    the reference is valid within [min_m, max_m] and the estimate is valid
    and positive; None with fewer than min_samples such pixels. The
    reference is resized (nearest) to the estimate's shape."""
    estimate = np.asarray(estimate, dtype=np.float32)
    reference = resize_nearest(np.asarray(reference, dtype=np.float32), estimate.shape[:2])
    ref = reference[offset::stride, offset::stride]
    est = estimate[offset::stride, offset::stride]
    with np.errstate(invalid='ignore'):
        valid = (np.isfinite(ref) & np.isfinite(est) & (ref >= min_m) & (ref <= max_m)
                 & (est > 0.0))
    if int(valid.sum()) < max(1, int(min_samples)):
        return None
    return float(np.median(ref[valid] / est[valid]))


class ScaleAligner:
    """depth_node's optional per-frame scale alignment to a metric depth
    image registered to the colour camera (a RealSense's
    aligned_depth_to_color, a simulator's depth): each estimate is scaled
    by median_scale against the reference of the same stamp (within
    max_age_s). Without a usable reference the last scale holds for
    hold_s; after that scale_for returns None (publish nothing rather than
    an unscaled estimate)."""

    def __init__(self, min_m=0.3, max_m=4.0, stride=8, min_samples=100, max_age_s=0.05,
                 hold_s=1.0, keep=10):
        self.min_m, self.max_m = float(min_m), float(max_m)
        self.stride, self.min_samples = max(1, int(stride)), int(min_samples)
        self.max_age_s, self.hold_s, self.keep = float(max_age_s), float(hold_s), int(keep)
        self.references = []  # (stamp, depth in metres), oldest first
        self.scale = None
        self.scale_stamp = None

    def add_reference(self, stamp, depth_m):
        self.references.append((float(stamp), depth_m))
        del self.references[:-self.keep]

    def scale_for(self, stamp, estimate):
        """The scale for the estimate of the colour frame at `stamp`."""
        stamp = float(stamp)
        nearest = min(self.references, key=lambda r: abs(r[0] - stamp), default=None)
        if nearest is not None and abs(nearest[0] - stamp) <= self.max_age_s:
            scale = median_scale(nearest[1], estimate, self.min_m, self.max_m, self.stride,
                                 self.stride // 2, self.min_samples)
            if scale is not None:
                self.scale, self.scale_stamp = scale, stamp
                return scale
        if self.scale is not None and abs(stamp - self.scale_stamp) <= self.hold_s:
            return self.scale
        return None


class ErrorAccumulator:
    """Errors of depth estimates against a reference over the reference's
    pixels within a range band (e.g. 0.5-5 m), pooled over frames:
    coverage (estimate valid), absolute and relative error percentiles,
    the ratio estimate / reference (the scale error) pooled, per frame and
    per distance bin, and delta < 1.25 (max(e/r, r/e) under 1.25).
    Pixels are sampled every `stride` rows and columns."""

    def __init__(self, band=(0.5, 5.0), bins=(0.5, 1.0, 2.0, 3.5, 5.0), stride=2):
        self.band = (float(band[0]), float(band[1]))
        self.bins = tuple(float(b) for b in bins)
        self.stride = max(1, int(stride))
        self.abs, self.rel, self.ratio, self.ref = [], [], [], []
        self.frame_ratio = []
        self.band_pixels = 0
        self.frames = 0

    def add(self, estimate, reference):
        estimate = np.asarray(estimate, dtype=np.float32)
        reference = resize_nearest(np.asarray(reference, dtype=np.float32), estimate.shape[:2])
        est = estimate[::self.stride, ::self.stride]
        ref = reference[::self.stride, ::self.stride]
        with np.errstate(invalid='ignore'):
            in_band = np.isfinite(ref) & (ref >= self.band[0]) & (ref <= self.band[1])
            valid = in_band & np.isfinite(est) & (est > 0.0)
        self.frames += 1
        self.band_pixels += int(in_band.sum())
        if not valid.any():
            return
        e, r = est[valid].astype(np.float64), ref[valid].astype(np.float64)
        self.abs.append(np.abs(e - r))
        self.rel.append(np.abs(e - r) / r)
        self.ratio.append(e / r)
        self.ref.append(r)
        self.frame_ratio.append(float(np.median(e / r)))

    def summary(self):
        """JSON-ready statistics (None where nothing was valid)."""
        valid = sum(len(a) for a in self.abs)
        result = {
            'frames': self.frames,
            'band_m': list(self.band),
            'band_pixels': self.band_pixels,
            'coverage': round(valid / self.band_pixels, 4) if self.band_pixels else None,
        }
        if not valid:
            return result
        abs_err, rel_err = np.concatenate(self.abs), np.concatenate(self.rel)
        ratio, ref = np.concatenate(self.ratio), np.concatenate(self.ref)

        def pct(values, q, digits=3):
            return round(float(np.percentile(values, q)), digits)

        result.update({
            'abs_error_m': {'median': pct(abs_err, 50), 'p90': pct(abs_err, 90),
                            'p95': pct(abs_err, 95)},
            'rel_error': {'median': pct(rel_err, 50, 4), 'p90': pct(rel_err, 90, 4),
                          'p95': pct(rel_err, 95, 4)},
            'ratio': {'median': pct(ratio, 50, 3), 'p10': pct(ratio, 10, 3),
                      'p90': pct(ratio, 90, 3)},
            'frame_ratio': {'median': round(float(np.median(self.frame_ratio)), 3),
                            'min': round(float(np.min(self.frame_ratio)), 3),
                            'max': round(float(np.max(self.frame_ratio)), 3)},
            'delta_1_25': round(float(np.mean(np.maximum(ratio, 1.0 / ratio) < 1.25)), 4),
            'ratio_by_distance': {},
        })
        for low, high in zip(self.bins[:-1], self.bins[1:]):
            inside = (ref >= low) & (ref < high) if high < self.bins[-1] else (ref >= low) & (ref <= high)
            if inside.any():
                result['ratio_by_distance'][f'{low:g}-{high:g}'] = {
                    'median_ratio': pct(ratio[inside], 50, 3),
                    'median_abs_error_m': pct(abs_err[inside], 50),
                    'pixels': int(inside.sum()),
                }
        return result


def point_heights(depth, k, camera_height, row_step=1):
    """Height above a flat floor of each pixel's point for a level camera
    camera_height above it (optical frame: y down): camera_height - y.
    depth is z (distance to the image plane); row i is image row
    i * row_step; k the 3x3 intrinsics, row-major."""
    depth = np.asarray(depth, dtype=np.float64)
    rows = np.arange(depth.shape[0], dtype=np.float64)[:, None] * row_step
    fy, cy = float(k[4]), float(k[5])
    return camera_height - (rows - cy) * depth / fy


class SliceAccumulator:
    """What a depth estimate does to nvblox's 2D obstacle slice (points
    between band[0] and band[1] above the floor become obstacles) from a
    level camera camera_height above a flat floor, over the reference
    pixels within range_m: the fraction of floor pixels (reference under
    band[0]) whose estimated point lands inside the band (phantom
    obstacles), and the fraction of obstacle pixels (reference inside the
    band) whose estimate is inside the band and within tolerance_m of the
    reference (obstacles where they are)."""

    def __init__(self, k, camera_height, band=(0.02, 0.55), range_m=(0.5, 5.0),
                 tolerance_m=0.15, stride=2, row_step=1):
        self.k, self.camera_height = np.asarray(k, dtype=np.float64).reshape(-1), float(camera_height)
        self.band, self.range_m = tuple(band), tuple(range_m)
        self.tolerance_m, self.stride, self.row_step = float(tolerance_m), int(stride), int(row_step)
        self.floor = self.floor_phantom = self.obstacle = self.obstacle_hit = 0

    def add(self, estimate, reference):
        estimate = np.asarray(estimate, dtype=np.float32)
        reference = resize_nearest(np.asarray(reference, dtype=np.float32), estimate.shape[:2])
        est = estimate[::self.stride, ::self.stride]
        ref = reference[::self.stride, ::self.stride]
        step = self.stride * self.row_step
        with np.errstate(invalid='ignore'):
            h_ref = point_heights(ref, self.k, self.camera_height, step)
            h_est = point_heights(est, self.k, self.camera_height, step)
            in_range = np.isfinite(ref) & (ref >= self.range_m[0]) & (ref <= self.range_m[1])
            est_in_band = np.isfinite(est) & (h_est >= self.band[0]) & (h_est <= self.band[1])
            floor = in_range & (h_ref < self.band[0])
            obstacle = in_range & (h_ref >= self.band[0]) & (h_ref <= self.band[1])
            close = np.abs(est - ref) <= self.tolerance_m
        self.floor += int(floor.sum())
        self.floor_phantom += int((floor & est_in_band).sum())
        self.obstacle += int(obstacle.sum())
        self.obstacle_hit += int((obstacle & est_in_band & close).sum())

    def summary(self):
        return {
            'camera_height_m': self.camera_height, 'band_m': list(self.band),
            'tolerance_m': self.tolerance_m,
            'floor_pixels': self.floor,
            'floor_as_obstacle': round(self.floor_phantom / self.floor, 4) if self.floor else None,
            'obstacle_pixels': self.obstacle,
            'obstacle_in_place': round(self.obstacle_hit / self.obstacle, 4) if self.obstacle else None,
        }


def timing_summary(milliseconds):
    values = [float(v) for v in milliseconds if math.isfinite(float(v))]
    if not values:
        return None
    return {'median_ms': round(float(np.median(values)), 2),
            'p95_ms': round(float(np.percentile(values, 95)), 2),
            'frames': len(values)}
