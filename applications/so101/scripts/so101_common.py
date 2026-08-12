"""Shared SO-101 simulation constants and synthetic sensor helpers."""

import math

import numpy as np


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

JOINT_LIMITS = np.array([
    [-1.9199, 1.9199],
    [-1.7453, 1.7453],
    [-1.7453, 1.7453],
    [-1.6581, 1.6581],
    [-2.7439, 2.7439],
    [0.0, 0.04],
])

LOWER_LIMITS = JOINT_LIMITS[:, 0]
UPPER_LIMITS = JOINT_LIMITS[:, 1]
JOINT_VELOCITY_LIMITS = np.array([2.6, 2.6, 2.8, 3.4, 3.8, 0.12], dtype=np.float32)
JOINT_QUANTIZATION = np.array([0.0015, 0.0015, 0.0015, 0.0015, 0.0020, 0.0005], dtype=np.float32)
JOINT_BACKLASH = np.array([0.003, 0.003, 0.003, 0.0025, 0.0025, 0.0008], dtype=np.float32)

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

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FX = 554.0
CAMERA_FY = 554.0
CAMERA_CX = CAMERA_WIDTH / 2.0
CAMERA_CY = CAMERA_HEIGHT / 2.0

# Matches systems/so101/urdf/so101.urdf.xacro base_to_groot_camera.
BASE_TO_CAMERA_LINK_TRANSLATION = (0.24, -0.28, 0.34)
BASE_TO_CAMERA_LINK_RPY = (0.0, 0.72, 2.36)
CAMERA_LINK_TO_OPTICAL_TRANSLATION = (0.0, 0.0, 0.0)
CAMERA_LINK_TO_OPTICAL_RPY = (-1.57079632679, 0.0, -1.57079632679)
CAMERA_TO_DEPTH_TRANSLATION = (0.0, 0.0, 0.0)
CAMERA_TO_DEPTH_RPY = (0.0, 0.0, 0.0)
WORLD_TO_TABLE_TRANSLATION = (0.42, 0.0, 0.425)
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
    step = np.asarray(max_joint_step, dtype=np.float64)
    if step.ndim == 0:
        step = np.full(len(JOINT_NAMES), float(step))
    if step.size != len(JOINT_NAMES) or not np.all(np.isfinite(step)) or np.any(step <= 0.0):
        raise ValueError("SO-101 HIL max joint step must be finite and positive")
    bounded = np.clip(requested, LOWER_LIMITS, UPPER_LIMITS)
    return np.clip(current + np.clip(bounded - current, -step, step), LOWER_LIMITS, UPPER_LIMITS)


def camera_info_values(width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
    sx = float(width) / CAMERA_WIDTH
    sy = float(height) / CAMERA_HEIGHT
    fx = CAMERA_FX * sx
    fy = CAMERA_FY * sy
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
