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


def terrain_template(depth: np.ndarray, valid: np.ndarray,
                     rotation: np.ndarray,
                     intrinsics: tuple[float, float, float, float],
                     cell_m: float):
    """Rasterise metric depth onto the north-up DEM grid at each pixel's
    true ground position.

    A pixel's ray at its z-depth ends on the terrain: with the world <-
    optical rotation that point is known in 3D relative to the camera, so
    no flat ground is assumed. (Projected onto flat ground height below
    the camera, as ground_template does, terrain dh above that plane and r
    from the point below the camera lands about r dh / height further out:
    relief displacement, which on a slope shifts the whole template.) Each
    DEM cell holds the mean height below the camera of the points that
    fall in it; cells without points all around them (the footprint edge,
    terrain hidden behind a ridge) are masked, so every value is an
    average centred on its cell.

    Returns (template, mask, anchor) as ground_template does, or None when
    no pixel is valid or the footprint is too large."""
    fx, fy, cx, cy = intrinsics
    img_h, img_w = depth.shape
    x = ((np.arange(img_w, dtype=np.float64) - cx) / fx)[None, :]
    y = ((np.arange(img_h, dtype=np.float64) - cy) / fy)[:, None]
    z = depth[valid].astype(np.float64)
    if z.size == 0:
        return None
    # World offset of each pixel's point per unit z-depth: east, north, up.
    east = (rotation[0, 0] * x + rotation[0, 1] * y + rotation[0, 2])[valid]
    north = (rotation[1, 0] * x + rotation[1, 1] * y + rotation[1, 2])[valid]
    up = (rotation[2, 0] * x + rotation[2, 1] * y + rotation[2, 2])[valid]
    # Template columns grow east, rows south (north-up); cell centres are
    # integer coordinates.
    col = np.rint(east * z / cell_m).astype(np.int64)
    row = np.rint(-north * z / cell_m).astype(np.int64)
    col_min, row_min = int(col.min()), int(row.min())
    width = int(col.max()) - col_min + 1
    height = int(row.max()) - row_min + 1
    if max(width, height) > MAX_TEMPLATE_CELLS:
        return None
    index = (row - row_min) * width + (col - col_min)
    counts = np.bincount(index, minlength=width * height)
    sums = np.bincount(index, weights=-up * z, minlength=width * height)
    covered = (counts > 0).reshape(height, width)
    template = np.zeros((height, width), dtype=np.float32)
    template[covered] = (sums[counts > 0] / counts[counts > 0])
    mask = cv2.erode(covered.astype(np.uint8), np.ones((3, 3), np.uint8),
                     borderType=cv2.BORDER_CONSTANT, borderValue=0) > 0
    if mask.any():
        template[~mask] = float(np.median(template[mask]))
    return template, mask, (float(-row_min), float(-col_min))


def view_trend(rotation: np.ndarray, shape, quadratic_tilt_rad: float):
    """Trend images (template-shaped) of what a tilted view of flat ground
    adds to relative depth: a ramp along the optical axis' heading on the
    ground (exact for depth; inverse or log depth also curve along it, so
    the square is added beyond quadratic_tilt_rad). Empty when level."""
    axis_east, axis_south = rotation[0, 2], -rotation[1, 2]
    tilt_sin = math.hypot(axis_east, axis_south)
    if tilt_sin <= 1e-3:
        return ()
    rows, cols = np.indices(shape, dtype=np.float64)
    along = (cols * axis_east + rows * axis_south) / tilt_sin
    if tilt_sin <= math.sin(quadratic_tilt_rad):
        return (along,)
    along -= along.mean()
    return (along, along * along)


def match_sigma(score: float, curvature, cells: int, cell_m: float,
                scale: float, floor_cells: float, maximum_m: float) -> float:
    """1-sigma horizontal error (m, per axis) of a correlation peak.

    A template matching the map up to noise as large as its unexplained
    variance (1 - score^2) locates the shift with variance (1 - score^2) /
    (cells x peak curvature) cells^2 along the flattest direction (the
    Cramer-Rao bound of a shift; curvature is that of the correlation
    surface per cell^2). The residual is not white, so the bound is scaled;
    a floor of floor_cells DEM cells covers what the score does not show
    (map-to-terrain mismatch, resampling). A peak that is not a maximum in
    every direction, or on the window edge (curvature None), is located no
    better than maximum_m."""
    if curvature is None:
        return maximum_m
    flattest = float(np.linalg.eigvalsh(-np.asarray(curvature))[0])
    unexplained = max(0.0, 1.0 - score * score)
    if not flattest > 0.0:
        return maximum_m
    bound_m = cell_m * math.sqrt(unexplained / (max(cells, 1) * flattest))
    return min(math.hypot(floor_cells * cell_m, scale * bound_m), maximum_m)


def heading_sensitivity(dem: np.ndarray, template: np.ndarray,
                        mask: np.ndarray, anchor: tuple[float, float],
                        position: tuple[float, float], trend=(),
                        step_rad: float = math.radians(1.0)):
    """How far the match moves (cells per radian) when the template turns
    about its anchor: what a heading error does to this fix. A slope or
    ridge across the footprint turns a small heading error into metres, a
    compact feature into little. The template is turned by +-step_rad and
    matched again within 3 cells of position (the anchor's window
    coordinates); None when a turned template no longer matches."""
    rows, cols = template.shape
    top = round(position[0] - anchor[0]) - 4
    left = round(position[1] - anchor[1]) - 4
    if (top < 0 or left < 0 or top + rows + 8 > dem.shape[0] or
            left + cols + 8 > dem.shape[1]):
        return None
    window = dem[top:top + rows + 8, left:left + cols + 8]
    local = (position[0] - top, position[1] - left)
    moved = []
    for sign in (1.0, -1.0):
        turn = cv2.getRotationMatrix2D((anchor[1], anchor[0]),
                                       math.degrees(sign * step_rad), 1.0)
        turned = cv2.warpAffine(template, turn, (cols, rows),
                                flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_REPLICATE)
        turned_mask = cv2.warpAffine(
            mask.astype(np.float32), turn, (cols, rows),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
            borderValue=0.0) > 0.99
        if turned_mask.sum() < 9:
            return None
        match = correlate(window, turned, turned_mask, anchor, local, 3.0,
                          float('inf'), trend)
        if match is None:
            return None
        moved.append(np.asarray(match['position']))
    return float(np.linalg.norm(moved[0] - moved[1])) / (2.0 * step_rad)


def _parabolic_offset(left: float, centre: float, right: float) -> float:
    curvature = left - 2.0 * centre + right
    if curvature >= 0.0 or not np.isfinite(curvature):
        return 0.0
    return float(np.clip(0.5 * (left - right) / curvature, -0.5, 0.5))


def _detrended_scores(dem: np.ndarray, template: np.ndarray,
                      mask: np.ndarray, trend) -> np.ndarray:
    """Normalised cross-correlation of the template with every DEM window
    after removing from both, over the mask, their least-squares fit on a
    constant and the trend images: neither side's trend counts towards the
    score or its normalisation."""
    weight = mask.astype(np.float64)
    basis = []
    for image in [np.ones_like(weight)] + [np.asarray(t, np.float64)
                                            for t in trend]:
        vector = image * weight
        for unit in basis:
            vector = vector - (vector * unit).sum() * unit
        norm = math.sqrt(float((vector * vector).sum()))
        if norm > 1e-6 * math.sqrt(float(weight.sum()) + 1.0):
            basis.append(vector / norm)
    residual = template.astype(np.float64) * weight
    for unit in basis:
        residual = residual - (residual * unit).sum() * unit
    template_norm = math.sqrt(float((residual * residual).sum()))
    if template_norm <= 0.0:
        return np.zeros((dem.shape[0] - mask.shape[0] + 1,
                         dem.shape[1] - mask.shape[1] + 1), np.float32)
    numerator = cv2.matchTemplate(dem, residual.astype(np.float32),
                                  cv2.TM_CCORR).astype(np.float64)
    energy = cv2.matchTemplate(dem * dem, weight.astype(np.float32),
                               cv2.TM_CCORR).astype(np.float64)
    for unit in basis:
        energy -= cv2.matchTemplate(
            dem, unit.astype(np.float32), cv2.TM_CCORR).astype(np.float64) ** 2
    floor = 1e-6 * float(weight.sum())
    return (numerator / (template_norm * np.sqrt(np.maximum(energy, floor)))
            ).astype(np.float32)


def correlate(dem: np.ndarray, template: np.ndarray, mask: np.ndarray,
              anchor: tuple[float, float], prior: tuple[float, float],
              radius_cells: float, exclusion_cells: float, trend=()):
    """Find where the template's anchor best matches the DEM window.

    Only anchor positions within radius_cells of prior (both window
    (row, col) coordinates) are searched. Relative depth may encode depth
    or inverse depth, so the score is |NCC| and its sign gives the
    polarity. The runner-up is the best |NCC| at least exclusion_cells
    from the winner, in either polarity: a terrain look-alike anywhere in
    the search area makes the fix ambiguous however high the peak is.
    Trend images (template-shaped) are left out of both sides of the
    correlation (_detrended_scores).

    Returns None when no anchor position is searchable, else a dict with
    the peak 'score', 'polarity', 'second' score, top-left 'loc'
    (row, col), the sub-cell anchor 'position' (row, col) and the score's
    2 x 2 second derivative there per cell^2, 'curvature' (None on the
    edge of the score map)."""
    if len(trend):
        scores = _detrended_scores(dem, template, mask, trend)
    else:
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
    curvature = None
    if 0 < row < scores.shape[0] - 1 and 0 < col < scores.shape[1] - 1:
        s = signed[row - 1:row + 2, col - 1:col + 2].astype(np.float64)
        curvature = np.array([
            [s[0, 1] - 2.0 * s[1, 1] + s[2, 1],
             0.25 * (s[2, 2] - s[2, 0] - s[0, 2] + s[0, 0])],
            [0.0, s[1, 0] - 2.0 * s[1, 1] + s[1, 2]]])
        curvature[1, 0] = curvature[0, 1]
        # Sub-cell peak of the quadratic through the 3 x 3 scores, cross
        # term included: an elongated peak running diagonally (a ridge) is
        # not where row and column parabolas put it. Within a cell of the
        # sampled maximum only; otherwise the parabolas stand.
        if np.all(np.linalg.eigvalsh(curvature) < 0.0):
            step = -np.linalg.solve(curvature, [0.5 * (s[2, 1] - s[0, 1]),
                                                0.5 * (s[1, 2] - s[1, 0])])
            if np.all(np.abs(step) <= 1.0):
                d_row, d_col = float(step[0]), float(step[1])
    return {
        'score': best,
        'polarity': polarity,
        'second': second,
        'loc': (int(row), int(col)),
        'position': (row + d_row + anchor[0], col + d_col + anchor[1]),
        'curvature': curvature,
    }
