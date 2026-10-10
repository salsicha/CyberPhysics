"""ROS-free geometry of DemNav's terrain correlation.

Kept out of the node so the camera model, template resampling and peak
selection can be exercised directly by tests.
"""
import math

import cv2
import numpy as np

from .camera_geometry import ground_projection

# Largest template side (DEM cells): a camera looking near the horizon
# would need an unbounded canvas.
MAX_TEMPLATE_CELLS = 4096


def derotation_angle_deg(yaw_rad: float, camera_mount_yaw_rad: float = 0.0
                         ) -> float:
    """cv2 rotation angle (degrees, counter-clockwise positive) that turns a
    body-fixed nadir frame north-up in level flight (ground_template's
    projection reduces to this rotation when roll and pitch are zero).

    Looking straight down, the image top points along the ground heading
    yaw + mount (ENU, counter-clockwise from east) and image right 90 deg
    clockwise of it, so the frame is an unmirrored map view rotated until
    that heading is up. North-up is heading 90 deg, so the content must
    turn counter-clockwise by heading - 90 deg."""
    return math.degrees(yaw_rad + camera_mount_yaw_rad) - 90.0


def height_below_camera(depth: np.ndarray, rotation: np.ndarray,
                        intrinsics: tuple[float, float, float, float]
                        ) -> np.ndarray:
    """Metric depth along the optical axis (z-depth) to height below the
    camera, with the world <- optical rotation at the frame time. A tilted
    camera's z-depth of flat ground ramps across the frame; this height
    does not. Level, it equals the depth."""
    fx, fy, cx, cy = intrinsics
    img_h, img_w = depth.shape
    cols = (np.arange(img_w, dtype=np.float64) - cx) / fx
    rows = (np.arange(img_h, dtype=np.float64) - cy) / fy
    # -z (down) component of each pixel's ray, scaled to unit z-depth.
    down = -(rotation[2, 0] * cols[None, :] + rotation[2, 1] * rows[:, None] +
             rotation[2, 2])
    return (depth * down).astype(np.float32)


def ground_template(depth: np.ndarray, valid: np.ndarray,
                    rotation: np.ndarray,
                    intrinsics: tuple[float, float, float, float],
                    height_m: float, cell_m: float):
    """Resample a depth frame onto the north-up DEM grid.

    Each depth pixel goes where its ray meets flat ground height_m below
    the camera (camera_geometry.ground_projection with the world <- optical
    rotation at the frame time), in DEM cells east and north of the point
    below the camera. A banked or pitched camera therefore keeps its true
    footprint, and the anchor is the point below the vehicle, not where the
    optical axis meets the ground. The canvas holds the whole footprint so
    no depth is cut off; the returned mask marks cells that came from valid
    pixels, which keeps the fill out of the correlation.

    Returns (template, mask, anchor) with anchor the (row, col) template
    coordinate of the point below the camera, or None when the footprint
    is unbounded (a corner at or above the horizon). Coordinates are pixel
    centres, as in cv2 and ROS CameraInfo."""
    img_h, img_w = depth.shape
    fx, fy = intrinsics[0], intrinsics[1]
    projection = ground_projection(rotation, intrinsics, height_m)
    corners = np.array([[-0.5, img_w - 0.5, -0.5, img_w - 0.5],
                        [-0.5, -0.5, img_h - 0.5, img_h - 0.5],
                        [1.0, 1.0, 1.0, 1.0]])
    mapped = projection @ corners
    if np.any(mapped[2] <= 1e-9):
        return None
    # Template columns grow east, rows south (north-up).
    cells = np.vstack((mapped[0] / mapped[2], -mapped[1] / mapped[2])) / cell_m
    lower = cells.min(axis=1)
    size = np.maximum(3, np.ceil(cells.max(axis=1) - lower)).astype(int)
    if size.max() > MAX_TEMPLATE_CELLS:
        return None
    offset = -0.5 - lower
    to_cells = np.array([[1.0 / cell_m, 0.0, offset[0]],
                         [0.0, -1.0 / cell_m, offset[1]],
                         [0.0, 0.0, 1.0]])
    matrix = to_cells @ projection

    fill_value = float(np.median(depth[valid])) if valid.any() else 0.0
    source = np.where(valid, depth, fill_value).astype(np.float32)
    # Anti-alias before sampling every few pixels (ground sampling below
    # the camera; it only coarsens away from it).
    sigma = 0.5 * cell_m * max(fx, fy) / height_m
    if sigma > 0.5:
        source = cv2.GaussianBlur(source, (0, 0), sigma)
    size = (int(size[0]), int(size[1]))
    template = cv2.warpPerspective(
        source, matrix, size, flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=fill_value)
    mask = cv2.warpPerspective(
        valid.astype(np.float32), matrix, size, flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0.0) > 0.99
    return template, mask, (float(offset[1]), float(offset[0]))


def _parabolic_offset(left: float, centre: float, right: float) -> float:
    curvature = left - 2.0 * centre + right
    if curvature >= 0.0 or not np.isfinite(curvature):
        return 0.0
    return float(np.clip(0.5 * (left - right) / curvature, -0.5, 0.5))


def correlate(dem: np.ndarray, template: np.ndarray, mask: np.ndarray,
              anchor: tuple[float, float], prior: tuple[float, float],
              radius_cells: float, exclusion_cells: float):
    """Find where the template's anchor best matches the DEM window.

    Only anchor positions within radius_cells of prior (both window
    (row, col) coordinates) are searched. Relative depth may encode depth
    or inverse depth, so the score is |NCC| and its sign gives the
    polarity. The runner-up is the best |NCC| at least exclusion_cells
    from the winner, in either polarity: a terrain look-alike anywhere in
    the search area makes the fix ambiguous however high the peak is.

    Returns None when no anchor position is searchable, else a dict with
    the peak 'score', 'polarity', 'second' score, top-left 'loc'
    (row, col) and the sub-cell anchor 'position' (row, col)."""
    scores = cv2.matchTemplate(dem, template, cv2.TM_CCOEFF_NORMED,
                               mask=mask.astype(np.float32))
    scores = np.nan_to_num(scores, nan=0.0, posinf=0.0, neginf=0.0)
    rows, cols = np.mgrid[0:scores.shape[0], 0:scores.shape[1]]
    searched = ((rows + anchor[0] - prior[0]) ** 2 +
                (cols + anchor[1] - prior[1]) ** 2 <= radius_cells ** 2)
    if not searched.any():
        return None
    magnitude = np.where(searched, np.abs(scores), -1.0)
    row, col = np.unravel_index(int(np.argmax(magnitude)), magnitude.shape)
    best = float(magnitude[row, col])
    polarity = 1.0 if scores[row, col] >= 0.0 else -1.0

    far = (rows - row) ** 2 + (cols - col) ** 2 > exclusion_cells ** 2
    rivals = magnitude[far & searched]
    second = max(0.0, float(rivals.max())) if rivals.size else 0.0

    signed = polarity * scores
    d_row = d_col = 0.0
    if 0 < row < scores.shape[0] - 1:
        d_row = _parabolic_offset(
            signed[row - 1, col], signed[row, col], signed[row + 1, col])
    if 0 < col < scores.shape[1] - 1:
        d_col = _parabolic_offset(
            signed[row, col - 1], signed[row, col], signed[row, col + 1])
    return {
        'score': best,
        'polarity': polarity,
        'second': second,
        'loc': (int(row), int(col)),
        'position': (row + d_row + anchor[0], col + d_col + anchor[1]),
    }
