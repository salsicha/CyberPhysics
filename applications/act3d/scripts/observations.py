"""Act3D's transport boundary, kept independent of torch for fast tests."""
import numpy as np
from contracts import config_digest


def checked_observation(obs, config):
    if str(obs["config_digest"]) != config_digest(config):
        raise ValueError("Simulator and checkpoint configurations differ")
    instruction = str(obs["instruction"])
    if instruction not in config["instructions"]:
        raise ValueError("Instruction is not part of this task's trained vocabulary")
    w, h = config["camera_resolution"]
    rgb, points, gripper = obs["rgb"], obs["points"], obs["gripper_pose"]
    if rgb.shape != (h, w, 3) or points.shape != rgb.shape or gripper.shape != (8,):
        raise ValueError("Observation shape mismatch")
    if (rgb.dtype != np.uint8 or points.dtype not in (np.float32, np.float64)
            or not np.isfinite(points).all() or not np.isfinite(gripper).all()):
        raise ValueError("Invalid RGB-D or gripper pose")
    if not np.isclose(np.linalg.norm(gripper[3:7]), 1., atol=.01) or not 0 <= gripper[7] <= 1:
        raise ValueError("Invalid gripper quaternion or opening")
    bounds = np.asarray(config["workspace_bounds"]).reshape(2, 3)
    inside = ((points >= bounds[0]) & (points <= bounds[1])).all(-1)
    if inside.sum() < 32:
        raise ValueError("Too few metric world-space points inside the workspace")
    return rgb, points, gripper.astype(np.float32), instruction
