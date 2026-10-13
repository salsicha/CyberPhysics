"""Pure observation/action checks shared by the simulator and policy tests."""
import hashlib
import json

import numpy as np


def config_digest(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()


def sample_scene(rng, config):
    bounds = config["spawn_bounds"]
    for _ in range(1000):
        xy = rng.uniform(bounds[:2], bounds[2:], (2, 2))
        if np.linalg.norm(xy[0] - xy[1]) > 0.15:
            return np.column_stack((xy, np.full(2, config["block_size"] / 2)))
    raise RuntimeError("Could not sample separated targets")


def checked_action(action, config):
    xyz = np.asarray(action["position"], dtype=float)
    euler = np.asarray(action["euler_degrees"], dtype=float)
    bounds = np.asarray(config["workspace_bounds"])
    if xyz.shape != (3,) or euler.shape != (3,) or not np.isfinite(np.r_[xyz, euler]).all():
        raise ValueError("Invalid target pose")
    if (xyz < bounds[:3]).any() or (xyz > bounds[3:]).any():
        raise ValueError("Target outside configured workspace")
    # This reaching demo must remain above the blocks and keep the gripper open.
    # Reject an unsuitable learned action; do not silently substitute an oracle.
    if xyz[2] < 0.12 or not action["gripper_open"] or action["ignore_collisions"]:
        raise ValueError("Action violates the reaching task's motion constraints")
    return xyz, euler
