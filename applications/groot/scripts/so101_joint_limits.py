"""SO-101 joint position limits for the GR00T scripts.

systems/so101/config/joint_limits.json is the single source of truth for the
limits (the URDF reads it too); this module only locates and loads it. The
values are TheRobotStudio's so101_new_calib.urdf limits, whose zero is
LeRobot's calibrated mid-range, mapped into this repository's straight-up URDF
zero with systems/so101/config/lerobot_joint_mapping.json; see the comment in
joint_limits.json and systems/so101/README.md. The compose files mount
systems/ at /workspace/systems.
"""

import json
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
JOINT_LIMITS_FILE_ENV = "SO101_JOINT_LIMITS_FILE"
JOINT_LIMITS_RELATIVE_PATH = Path("systems", "so101", "config", "joint_limits.json")


def joint_limits_file(start=__file__) -> Path:
    """Locate joint_limits.json: $SO101_JOINT_LIMITS_FILE, then a systems/ dir above ``start``."""
    override = os.environ.get(JOINT_LIMITS_FILE_ENV)
    if override:
        return Path(override)
    here = Path(start).resolve()
    candidates = [parent / JOINT_LIMITS_RELATIVE_PATH for parent in here.parents]
    workspace = Path("/workspace") / JOINT_LIMITS_RELATIVE_PATH
    if workspace not in candidates:
        candidates.append(workspace)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"SO-101 joint limits not found: mount the repository's systems/ at /workspace/systems "
        f"or set {JOINT_LIMITS_FILE_ENV} (searched {', '.join(str(path) for path in candidates)})"
    )


def load_joint_limits(path=None, names=JOINT_NAMES) -> np.ndarray:
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
