"""Isaac adapter for the pinned, unmodified PerAct voxel-language encoder.

This demo trains a smaller configuration on Isaac reaching demonstrations.
It is not the published 18-task RLBench checkpoint.
"""
import importlib.util
import os
from pathlib import Path

import numpy as np
import torch
from voxel.voxel_grid import VoxelGrid
from contracts import config_digest  # re-exported for training/server

UPSTREAM_REF = "02fb87681c5a47be9dbf20141bedb836ee2d3ef9"

# Import the unchanged encoder file directly: the upstream package initializer
# eagerly imports its RLBench training runner, which requires CoppeliaSim.
_encoder_path = Path(os.environ.get("PERACT_ROOT", "/opt/peract")) / "agents/peract_bc/perceiver_lang_io.py"
_spec = importlib.util.spec_from_file_location("peract_encoder", _encoder_path)
_encoder = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_encoder)
PerceiverVoxelLangEncoder = _encoder.PerceiverVoxelLangEncoder


class Policy(torch.nn.Module):
    def __init__(self, config, device="cuda"):
        super().__init__()
        self.config = config
        self.device = torch.device(device)
        self.net = PerceiverVoxelLangEncoder(
            iterations=1, initial_dim=10, low_dim_size=4,
            num_rotation_classes=72, activation="lrelu",
            input_dropout=0.0, attn_dropout=0.0, decoder_dropout=0.0,
            **config["model"],
        ).to(self.device)
        self.bounds = torch.tensor(config["workspace_bounds"], device=self.device)[None]
        self.voxelizers = {}

    def voxelize(self, rgb, points):
        b, h, w, _ = rgb.shape
        key = (b, h, w)
        if key not in self.voxelizers:
            self.voxelizers[key] = VoxelGrid(
                self.config["workspace_bounds"], self.config["model"]["voxel_size"],
                self.device, b, 3, h * w,
            ).to(self.device)
        # Invalid depth is outside the workspace and discarded by the voxelizer.
        points = torch.nan_to_num(points, nan=-100.0, posinf=-100.0, neginf=-100.0)
        vox = self.voxelizers[key].coords_to_bounding_voxel_grid(
            points.reshape(b, -1, 3), coord_features=rgb.reshape(b, -1, 3).float() / 127.5 - 1,
            coord_bounds=self.bounds,
        )
        return vox.permute(0, 4, 1, 2, 3).detach()

    def forward(self, vox, proprio, sentence, tokens):
        return self.net(vox, proprio, sentence, tokens, None, self.bounds, None)

    @torch.inference_mode()
    def predict(self, rgb, points, proprio, language):
        self.eval()
        rgb = torch.as_tensor(rgb, device=self.device)[None]
        points = torch.as_tensor(points, device=self.device)[None]
        prop = torch.as_tensor(proprio, dtype=torch.float32, device=self.device)[None]
        sentence, tokens = [x.to(self.device)[None] for x in language]
        q, rotation_grip, collision = self(self.voxelize(rgb, points), prop, sentence, tokens)
        n = self.config["model"]["voxel_size"]
        index = int(q.flatten().argmax())
        xyz_index = np.asarray(np.unravel_index(index, (n, n, n)))
        bounds = np.asarray(self.config["workspace_bounds"])
        xyz = bounds[:3] + (xyz_index + 0.5) * (bounds[3:] - bounds[:3]) / n
        angles = rotation_grip[0, :216].reshape(3, 72).argmax(-1).cpu().numpy() * 5 - 180
        return {"position": xyz.tolist(), "euler_degrees": angles.tolist(),
                "gripper_open": bool(rotation_grip[0, -2:].argmax()),
                "ignore_collisions": bool(collision[0].argmax())}


def load_policy(path, device="cuda"):
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Missing trained checkpoint: {path}. Run tools/prepare_peract_demo.sh first.")
    data = torch.load(path, map_location="cpu", weights_only=True)
    if data.get("format") != "cyberphysics-peract-reach-v1" or data.get("upstream_ref") != UPSTREAM_REF:
        raise ValueError("Not a supported trained PerAct reaching checkpoint")
    if data.get("training_steps", 0) < 1:
        raise ValueError("Checkpoint has no training provenance")
    policy = Policy(data["config"], device)
    policy.load_state_dict(data["state_dict"], strict=True)
    return policy.eval(), data
