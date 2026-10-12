"""Shared SO-101 simulation constants and synthetic sensor helpers."""

import json
import math
import os
from pathlib import Path

import numpy as np


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# systems/so101/config/joint_limits.json is the single source of truth for the
# joint position limits; the URDF reads it too. They are TheRobotStudio's
# so101_new_calib.urdf limits (zero = LeRobot's calibrated mid-range) mapped
# into this URDF's straight-up zero with lerobot_joint_mapping.json, e.g. the
# elbow is -22.9..170.7 deg, not +/-100 deg. See that file and
# systems/so101/README.md. Containers find it under /workspace/systems.
JOINT_LIMITS_FILE_ENV = "SO101_JOINT_LIMITS_FILE"
JOINT_LIMITS_RELATIVE_PATH = Path("systems", "so101", "config", "joint_limits.json")
# systems/so101/scenarios/picking_table.json is the single source of truth for
# the scene: the robot's spawn pose, the pose and size of every asset and
# object (each one's "rests_on" names the surface it stands on; see
# scene_support_errors), and the fixed camera (pose, resolution, field of
# view). The camera constants below are read from it; Gazebo's world is
# generated from it (scenario_world.py), Isaac builds its scene from it, and
# the printed station (hardware/parts/so101) is derived from it.
SCENARIO_FILE_ENV = "SO101_SCENARIO_FILE"
SCENARIO_RELATIVE_PATH = Path("systems", "so101", "scenarios", "picking_table.json")


def _systems_file(relative_path, env_name, start, what):
    """$env_name, then ``relative_path`` under a directory above ``start``, then /workspace."""
    override = os.environ.get(env_name)
    if override:
        return Path(override)
    here = Path(start).resolve()
    candidates = [parent / relative_path for parent in here.parents]
    workspace = Path("/workspace") / relative_path
    if workspace not in candidates:
        candidates.append(workspace)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"SO-101 {what} not found: mount the repository's systems/ at /workspace/systems "
        f"or set {env_name} (searched {', '.join(str(path) for path in candidates)})"
    )


def joint_limits_file(start=__file__):
    """Locate joint_limits.json: $SO101_JOINT_LIMITS_FILE, then a systems/ dir above ``start``."""
    return _systems_file(JOINT_LIMITS_RELATIVE_PATH, JOINT_LIMITS_FILE_ENV, start, "joint limits")


def scenario_file(start=__file__):
    """Locate the default scenario: $SO101_SCENARIO_FILE, then a systems/ dir above ``start``."""
    return _systems_file(SCENARIO_RELATIVE_PATH, SCENARIO_FILE_ENV, start, "scenario")


def load_scenario_file(path=None):
    path = Path(path) if path is not None else scenario_file()
    return json.loads(path.read_text())


def load_joint_limits(path=None, names=JOINT_NAMES):
    """[lower, upper] per joint, in ``names`` order, from joint_limits.json."""
    path = Path(path) if path is not None else joint_limits_file()
    joints = json.loads(path.read_text())["joints"]
    limits = np.array(
        [[float(joints[name]["lower"]), float(joints[name]["upper"])] for name in names],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(limits)) or np.any(limits[:, 0] >= limits[:, 1]):
        raise ValueError(f"{path}: every joint needs finite limits with lower < upper")
    return limits


JOINT_LIMITS = load_joint_limits()

LOWER_LIMITS = JOINT_LIMITS[:, 0]
UPPER_LIMITS = JOINT_LIMITS[:, 1]
JOINT_VELOCITY_LIMITS = np.array([2.6, 2.6, 2.8, 3.4, 3.8, 0.12], dtype=np.float32)
JOINT_QUANTIZATION = np.array([0.0015, 0.0015, 0.0015, 0.0015, 0.0020, 0.0005], dtype=np.float32)
JOINT_BACKLASH = np.array([0.003, 0.003, 0.003, 0.0025, 0.0025, 0.0008], dtype=np.float32)
# Measured joints may read up to one encoder step past a limit (quantization
# of an at-limit position rounds outward); commands are still clamped exactly.
JOINT_MEASUREMENT_TOLERANCE = JOINT_QUANTIZATION.astype(np.float64)
# Physics engines enforce joint limits softly; a simulated joint further past
# its limit than this counts as a joint-limit violation in task metrics.
JOINT_LIMIT_VIOLATION_TOLERANCE = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.001], dtype=np.float64)

RGB_TOPIC = "/so101/camera/image_raw"
RGB_CAMERA_INFO_TOPIC = "/so101/camera/camera_info"
DEPTH_TOPIC = "/so101/camera/depth/image_rect_raw"
DEPTH_CAMERA_INFO_TOPIC = "/so101/camera/depth/camera_info"
CAMERA_IMU_TOPIC = "/so101/camera/imu"
CAMERA_LINK_FRAME_ID = "groot_camera_link"
CAMERA_FRAME_ID = "groot_camera_rgb_optical_frame"
DEPTH_FRAME_ID = "groot_camera_depth_optical_frame"
BASE_FRAME_ID = "base_link"
TABLE_FRAME_ID = "table_frame"
GRIPPER_FRAME_ID = "gripper_base_link"
TASK_STATUS_TOPIC = "/so101/task_status"
# Per-command JSON statistics from the policy bridge (latency, observation age,
# saturation, ground-truth use), consumed by the simulator's task metrics.
POLICY_STATS_TOPIC = "/so101/policy_stats"

# The scenario's "camera" block is the single source of truth for the
# simulated camera: Isaac and Gazebo render from it, CameraInfo is
# derived from its resolution and horizontal FOV, and the URDF/TF camera mount
# must equal its pose relative to base_link (enforced by
# validate_calibration_and_dataset.py and the tests). It models the RealSense
# D435i's 16:9 colour stream (69.4 x 42.5 deg) at 640x360, the mode the
# hardware compositions select. These are the default scenario's values.
DEFAULT_SCENARIO = load_scenario_file()
_DEFAULT_CAMERA = DEFAULT_SCENARIO["camera"]
CAMERA_WIDTH, CAMERA_HEIGHT = (int(value) for value in _DEFAULT_CAMERA["resolution"])
CAMERA_HFOV_DEG = float(_DEFAULT_CAMERA["horizontal_fov_deg"])  # simulators keep pixels square
# Pose of groot_camera_link in the scenario world frame, in ROS body axes
# (+X optical axis, +Y left, +Z up), i.e. Isaac's Camera camera_axes="world"
# and Gazebo's camera sensor frame.
DEFAULT_CAMERA_XYZ = tuple(float(value) for value in _DEFAULT_CAMERA["xyz"])
DEFAULT_CAMERA_RPY = tuple(float(value) for value in _DEFAULT_CAMERA["rpy"])
# systems/so101/urdf/so101.urdf.xacro world_to_base: base_link sits this far
# above the spawn pose of the URDF's root ``world`` link, which is the
# underside of the arm's base: base_link's box is BASE_LINK_SIZE.
SPAWN_TO_BASE_LINK = (0.0, 0.0, 0.0225)
BASE_LINK_SIZE = (0.12, 0.12, 0.045)


def camera_focal_length_px(width=CAMERA_WIDTH, horizontal_fov_deg=CAMERA_HFOV_DEG):
    """Pinhole focal length in pixels for a horizontal field of view."""
    return (float(width) / 2.0) / math.tan(math.radians(float(horizontal_fov_deg)) / 2.0)


CAMERA_FX = camera_focal_length_px()
CAMERA_FY = CAMERA_FX
CAMERA_CX = CAMERA_WIDTH / 2.0
CAMERA_CY = CAMERA_HEIGHT / 2.0

CAMERA_LINK_TO_OPTICAL_TRANSLATION = (0.0, 0.0, 0.0)
CAMERA_LINK_TO_OPTICAL_RPY = (-1.57079632679, 0.0, -1.57079632679)
CAMERA_TO_DEPTH_TRANSLATION = (0.0, 0.0, 0.0)
CAMERA_TO_DEPTH_RPY = (0.0, 0.0, 0.0)
WORLD_TO_TABLE_RPY = (0.0, 0.0, 0.0)

# Kinematic dimensions to the centre of the jaws.
SHOULDER_PIVOT_Z = 0.0225 + 0.065 + 0.045
ARM_LINK_LENGTHS = np.array([0.14, 0.13, 0.055 + 0.052 + 0.035], dtype=np.float64)


def quantize(values, resolution):
    values = np.asarray(values, dtype=np.float32)
    return np.round(values / resolution) * resolution


def forward_kinematics(joints, robot_spawn_xyz=(0.0, 0.0, 0.0)):
    """Return the world-space centre of the SO-101 gripper jaws."""
    joints = np.asarray(joints, dtype=np.float64).reshape(len(JOINT_NAMES))
    spawn = np.asarray(robot_spawn_xyz, dtype=np.float64)
    cumulative_pitch = np.cumsum(joints[1:4])
    radial = float(np.sum(ARM_LINK_LENGTHS * np.sin(cumulative_pitch)))
    vertical = float(np.sum(ARM_LINK_LENGTHS * np.cos(cumulative_pitch)))
    return np.array([
        spawn[0] + radial * math.cos(float(joints[0])),
        spawn[1] + radial * math.sin(float(joints[0])),
        spawn[2] + SHOULDER_PIVOT_Z + vertical,
    ], dtype=np.float64)


def safe_hardware_target(requested, current, max_joint_step):
    """Clamp one HIL command against limits and the measured hardware state."""
    requested = np.asarray(requested, dtype=np.float64).reshape(len(JOINT_NAMES))
    current = np.asarray(current, dtype=np.float64).reshape(len(JOINT_NAMES))
    if not np.all(np.isfinite(requested)) or not np.all(np.isfinite(current)):
        raise ValueError("SO-101 HIL state and command must be finite")
    if (np.any(current < LOWER_LIMITS - JOINT_MEASUREMENT_TOLERANCE)
            or np.any(current > UPPER_LIMITS + JOINT_MEASUREMENT_TOLERANCE)):
        raise ValueError("SO-101 HIL measured state is outside joint limits")
    step = np.asarray(max_joint_step, dtype=np.float64)
    if step.ndim == 0:
        step = np.full(len(JOINT_NAMES), float(step))
    if step.size != len(JOINT_NAMES) or not np.all(np.isfinite(step)) or np.any(step <= 0.0):
        raise ValueError("SO-101 HIL max joint step must be finite and positive")
    bounded = np.clip(requested, LOWER_LIMITS, UPPER_LIMITS)
    return np.clip(current + np.clip(bounded - current, -step, step), LOWER_LIMITS, UPPER_LIMITS)


def validated_joint_command(values):
    """Return a six-joint position command clamped to the joint limits.

    Raises ValueError for a wrong-sized or non-finite command: np.clip passes
    NaN through, so a simulator drive target must be rejected, not clamped.
    """
    command = np.asarray(values, dtype=np.float64).reshape(-1)
    if command.size != len(JOINT_NAMES):
        raise ValueError(f"expected {len(JOINT_NAMES)} SO-101 joint commands, got {command.size}")
    if not np.all(np.isfinite(command)):
        raise ValueError("SO-101 joint command contains a non-finite value")
    return np.clip(command, LOWER_LIMITS, UPPER_LIMITS)


def rpy_to_matrix(rpy):
    """Rotation matrix for URDF/ROS fixed-axis roll, pitch, yaw."""
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2, 0])))
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return roll, pitch, yaw


def scenario_camera(scenario=None):
    """The scenario camera block merged over the defaults above."""
    camera = {
        "xyz": list(DEFAULT_CAMERA_XYZ),
        "rpy": list(DEFAULT_CAMERA_RPY),
        "resolution": [CAMERA_WIDTH, CAMERA_HEIGHT],
        "horizontal_fov_deg": CAMERA_HFOV_DEG,
        # USD cameras default to a 1 m near plane, which would clip the
        # whole tabletop (0.5-0.9 m from the camera).
        "clipping_range_m": [0.01, 100.0],
    }
    camera.update((scenario or {}).get("camera", {}))
    return camera


def base_to_camera_link(scenario=None):
    """groot_camera_link pose relative to base_link for a scenario camera."""
    spawn = (scenario or {}).get("robot_spawn", {})
    spawn_rotation = rpy_to_matrix(spawn.get("rpy", [0.0, 0.0, 0.0]))
    base_xyz = np.asarray(spawn.get("xyz", [0.0, 0.0, 0.0]), dtype=np.float64) + (
        spawn_rotation @ np.asarray(SPAWN_TO_BASE_LINK, dtype=np.float64)
    )
    camera = scenario_camera(scenario)
    translation = spawn_rotation.T @ (np.asarray(camera["xyz"], dtype=np.float64) - base_xyz)
    rotation = spawn_rotation.T @ rpy_to_matrix(camera["rpy"])
    return tuple(float(value) for value in translation), matrix_to_rpy(rotation)


# Display colours of the scene's materials and object colours (linear RGB),
# shared by every simulator so they render the same scene.
SCENE_COLORS = {
    "black_anodized_aluminum": (0.08, 0.08, 0.09),
    "blue": (0.04, 0.18, 0.76),
    "blue_plastic": (0.06, 0.32, 0.58),
    "green": (0.08, 0.58, 0.22),
    "laminated_wood": (0.55, 0.49, 0.38),
    "matte_gray_cardboard": (0.42, 0.42, 0.38),
    "red": (0.85, 0.06, 0.04),
    "red_plastic": (0.66, 0.22, 0.14),
    "white": (0.92, 0.92, 0.88),
}
APRILGRID_COLOR = (0.94, 0.94, 0.90)


def scene_color(entry):
    if entry.get("type") == "aprilgrid":
        return APRILGRID_COLOR
    return SCENE_COLORS.get(entry.get("color") or entry.get("material"), (0.45, 0.45, 0.45))


def scene_entities(scenario):
    """{name: entry} for the scenario's static assets (by name) and objects (by id)."""
    entities = {entry["name"]: entry for entry in (scenario or {}).get("static_assets", [])}
    entities.update({entry["id"]: entry for entry in (scenario or {}).get("objects", [])})
    return entities


def entity_half_extents(entry):
    """World-axis half extents (x, y, z) of a box or z-axis cylinder after its rpy."""
    rotation = rpy_to_matrix(entry.get("rpy", [0.0, 0.0, 0.0]))
    if entry.get("shape") == "cylinder":
        axis = np.abs(rotation[:, 2])
        radius = float(entry["radius_m"])
        return axis * float(entry["height_m"]) / 2.0 + radius * np.sqrt(np.clip(1.0 - axis ** 2, 0.0, 1.0))
    return np.abs(rotation) @ (np.asarray(entry["size"][:3], dtype=np.float64) / 2.0)


def entity_bounds(entry):
    """(lower, upper) world corners of an entry's axis-aligned bounding box."""
    centre = np.asarray(entry["xyz"][:3], dtype=np.float64)
    half = entity_half_extents(entry)
    return centre - half, centre + half


def robot_base_bounds(scenario):
    """Bounding box of the URDF's base_link box at the scenario's spawn pose."""
    spawn = (scenario or {}).get("robot_spawn", {})
    entry = {
        "xyz": (np.asarray(spawn.get("xyz", [0.0, 0.0, 0.0]), dtype=np.float64)
                + rpy_to_matrix(spawn.get("rpy", [0.0, 0.0, 0.0])) @ np.asarray(SPAWN_TO_BASE_LINK)).tolist(),
        "rpy": spawn.get("rpy", [0.0, 0.0, 0.0]),
        "size": list(BASE_LINK_SIZE),
    }
    return entity_bounds(entry)


def table_entry(scenario):
    """The scenario's table (the static asset of type "table")."""
    for entry in (scenario or {}).get("static_assets", []):
        if entry.get("type") == "table":
            return entry
    raise ValueError("scenario has no static asset of type 'table'")


def table_top_xyz(scenario):
    """Centre of the table's top surface (the table_frame origin)."""
    table = table_entry(scenario)
    top = entity_bounds(table)[1][2]
    return (float(table["xyz"][0]), float(table["xyz"][1]), float(top))


def resting_xyz(scenario, entry, support=None):
    """Where ``entry`` (an asset or object of ``scenario``) is centred when it
    stands on top of ``support`` (default: its "rests_on"), at its x, y."""
    support = support or entry["rests_on"]
    support_top = entity_bounds(scene_entities(scenario)[support])[1][2]
    half = entity_half_extents(entry)
    xyz = [float(value) for value in entry["xyz"][:3]]
    xyz[2] = float(support_top + float(entry.get("standoff_m", 0.0)) + half[2])
    return xyz


def scene_support_errors(scenario, tolerance=1e-6):
    """Physical inconsistencies of a scenario, as messages (empty if none).

    Every static asset except the table, every object and the robot spawn
    must name the surface it stands on ("rests_on"); its underside must be
    on that surface's top (plus "standoff_m", e.g. a board standing in printed
    feet) and its centre over it. The robot's underside is its spawn z (the
    URDF's root link). No two bodies, the robot's base box included, may
    interpenetrate. This keeps one source of truth for the heights: an entity
    floating above, or sunk into, what it stands on is an error, whatever the
    simulator would do with it.
    """
    errors = []
    entities = scene_entities(scenario)
    bounds = {name: entity_bounds(entry) for name, entry in entities.items()}
    spawn = (scenario or {}).get("robot_spawn", {})
    bounds["robot_base"] = robot_base_bounds(scenario)
    standing = [("robot_spawn", spawn, float(bounds["robot_base"][0][2]), bounds["robot_base"])]
    for name, entry in entities.items():
        if entry.get("type") == "table":
            continue
        standing.append((name, entry, float(bounds[name][0][2]), bounds[name]))
    for name, entry, underside, (lower, upper) in standing:
        support = entry.get("rests_on")
        if support is None:
            errors.append(f"{name} does not say what it rests on (rests_on)")
            continue
        if support not in entities:
            errors.append(f"{name} rests on {support!r}, which is not in the scenario")
            continue
        support_lower, support_upper = bounds[support]
        expected = float(support_upper[2]) + float(entry.get("standoff_m", 0.0))
        if abs(underside - expected) > tolerance:
            errors.append(
                f"{name}'s underside is at {underside:.4f} m but {support}'s top"
                f"{' plus its standoff' if entry.get('standoff_m') else ''} is at {expected:.4f} m"
            )
        centre = (lower + upper) / 2.0
        if np.any(centre[:2] < support_lower[:2] - tolerance) or np.any(centre[:2] > support_upper[:2] + tolerance):
            errors.append(f"{name} is not over {support}")
    names = sorted(bounds)
    for index, first in enumerate(names):
        for second in names[index + 1:]:
            overlap = (np.minimum(bounds[first][1], bounds[second][1])
                       - np.maximum(bounds[first][0], bounds[second][0]))
            if np.all(overlap > tolerance):
                errors.append(f"{first} and {second} overlap by {1000.0 * float(np.min(overlap)):.1f} mm")
    return errors


BASE_TO_CAMERA_LINK_TRANSLATION, BASE_TO_CAMERA_LINK_RPY = base_to_camera_link(DEFAULT_SCENARIO)
WORLD_TO_TABLE_TRANSLATION = table_top_xyz(DEFAULT_SCENARIO)


def camera_info_values(width=CAMERA_WIDTH, height=CAMERA_HEIGHT, horizontal_fov_deg=CAMERA_HFOV_DEG):
    # Square pixels and a centred principal point, as Isaac's Camera renders
    # (set_horizontal_aperture keeps the vertical aperture in proportion).
    fx = fy = camera_focal_length_px(width, horizontal_fov_deg)
    cx = float(width) / 2.0
    cy = float(height) / 2.0
    return {
        "width": int(width),
        "height": int(height),
        "distortion_model": "plumb_bob",
        "d": [0.0, 0.0, 0.0, 0.0, 0.0],
        "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    }


def synthetic_rgbd(positions, width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
    """Generate deterministic RGB-D frames with tabletop, objects, and invalid depth."""
    positions = np.nan_to_num(
        np.asarray(positions, dtype=np.float32),
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )
    h = int(height)
    w = int(width)
    y_u8 = np.linspace(0, 255, h, dtype=np.uint8)[:, None]
    x_u8 = np.linspace(0, 255, w, dtype=np.uint8)[None, :]
    image = np.zeros((h, w, 3), dtype=np.uint8)
    x_phase = int((float(positions[0]) + 2.0) * 35.0) % 255
    y_phase = int((float(positions[1]) + 2.0) * 35.0) % 255
    image[:, :, 0] = (x_u8.astype(np.uint16) + x_phase) % 255
    image[:, :, 1] = (y_u8.astype(np.uint16) + y_phase) % 255
    image[:, :, 2] = 90

    x = np.linspace(-1.0, 1.0, w, dtype=np.float32)[None, :]
    y = np.linspace(-1.0, 1.0, h, dtype=np.float32)[:, None]
    depth = 0.74 + 0.045 * y + 0.015 * x

    target_cx = int(w * (0.5 + 0.22 * math.sin(float(positions[2]))))
    target_cy = int(h * (0.5 - 0.22 * math.sin(float(positions[3]))))
    half = max(8, int(min(w, h) * 0.025))
    y0 = max(0, target_cy - half)
    y1 = min(h, target_cy + half)
    x0 = max(0, target_cx - half)
    x1 = min(w, target_cx + half)
    image[y0:y1, x0:x1, :] = [255, 55, 35]
    depth[y0:y1, x0:x1] = 0.56

    cyl_cx = int(w * 0.61)
    cyl_cy = int(h * 0.44)
    rr = max(7, int(min(w, h) * 0.022))
    yy, xx = np.ogrid[:h, :w]
    cyl_mask = (xx - cyl_cx) ** 2 + (yy - cyl_cy) ** 2 <= rr ** 2
    image[cyl_mask] = [40, 80, 230]
    depth[cyl_mask] = 0.61

    invalid_phase = int(abs(float(positions[4])) * 100.0) % 97
    invalid_mask = ((xx + 2 * yy + invalid_phase) % 97) == 0
    depth[invalid_mask] = np.nan
    return image, depth.astype(np.float32)


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
