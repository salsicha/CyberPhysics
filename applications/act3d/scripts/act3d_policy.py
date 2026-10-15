"""Pinned upstream Act3D, with offline checkpoints and a frozen-backbone cache.

The upstream model's eager package initializer imports ChainedDiffuser/RLBench.
Load only the Act3D package tree; no simulator dependency belongs in this server.
"""
import importlib
import os
from pathlib import Path
import sys
import types

import numpy as np
from scipy.spatial.transform import Rotation
import torch
from torchvision.transforms import Normalize

UPSTREAM_REF = "5004650a9771e7618f24f36a876bcb38e0fc068a"
FORMAT = "cyberphysics-act3d-v1"


def make_network(config, pretrained=False):
    root = Path(os.environ.get("ACT3D_SOURCE", "/opt/act3d"))
    package = types.ModuleType("model")
    package.__path__ = [str(root / "model")]
    sys.modules["model"] = package
    upstream = importlib.import_module("model.keypose_optimization.act3d")
    clip_module = importlib.import_module("model.utils.clip")
    original = upstream.load_clip
    def load_backbone():
        # RN50 visual architecture, filled by strict checkpoint loading at serve
        # time. Only training downloads weights, into the persistent cache.
        backbone = clip_module.ModifiedResNetFeatures((3, 4, 6, 3), 1024, 32)
        normalize = Normalize((0.48145466, 0.4578275, 0.40821073),
                              (0.26862954, 0.26130258, 0.27577711))
        if pretrained:
            import clip
            encoder, _ = clip.load("RN50", device="cpu", download_root=os.environ.get(
                "TORCH_HOME", "/workspace/cache/torch"))
            backbone.load_state_dict(encoder.visual.state_dict(), strict=True)
        return backbone, normalize

    upstream.load_clip = load_backbone
    try:
        bounds = np.asarray(config["workspace_bounds"]).reshape(2, 3)
        net = upstream.Act3D(image_size=tuple(config["camera_resolution"]),
                            gripper_loc_bounds=bounds, use_instruction=True,
                            **config["model"])
    finally:
        upstream.load_clip = original
    # Upstream typo: the 128px branch assigns coarse_feature_map, while forward
    # reads feature_map_pyramid. Preserve exactly its intended pyramid levels.
    if net.image_size == (128, 128):
        net.feature_map_pyramid = net.coarse_feature_map
    net.backbone = FrozenBackbone(net.backbone)
    return net


class FrozenBackbone(torch.nn.Module):
    """Cache only frozen RN50 features during training, never the trainable FPN."""
    def __init__(self, backbone):
        super().__init__()
        self.encoder = backbone.eval()
        self.cached_features = None

    def train(self, mode=True):
        # Frozen weights also require frozen BatchNorm running statistics.
        return super().train(False)

    def forward(self, images):
        if self.cached_features is not None:
            return self.cached_features
        with torch.no_grad():
            return self.encoder(images)


class Policy:
    def __init__(self, config, device="cuda", pretrained=False):
        self.config = config
        self.device = torch.device(device)
        self.net = make_network(config, pretrained).to(self.device)

    def tensors(self, rgb, points, gripper):
        rgb = torch.as_tensor(rgb, device=self.device).float().movedim(-1, -3) / 255.
        points = torch.as_tensor(points, device=self.device).float().movedim(-1, -3)
        gripper = torch.as_tensor(gripper, device=self.device).float()
        return rgb.unsqueeze(1), points.unsqueeze(1), gripper

    @torch.inference_mode()
    def predict(self, rgb, points, gripper, language):
        self.net.eval()
        images, cloud, current = self.tensors(rgb[None], points[None], gripper[None])
        # No ground-truth action: every refinement centers on the model's own
        # previous prediction, unlike teacher forcing used only during training.
        result = self.net(images, cloud, language[None].to(self.device), current)
        quaternion = result["rotation"][0].cpu().numpy()
        return {"position": result["position"][0].cpu().tolist(),
                "euler_degrees": Rotation.from_quat(quaternion).as_euler("xyz", degrees=True).tolist(),
                "quaternion_xyzw": quaternion.tolist(),
                "gripper_open": bool(result["gripper"][0, 0] > .5),
                # This upstream implementation has no collision prediction head.
                "ignore_collisions": False}


def load_policy(path, device="cuda"):
    data = torch.load(path, map_location="cpu", weights_only=True)
    if (data.get("format") != FORMAT or data.get("upstream_ref") != UPSTREAM_REF
            or data.get("training_steps", 0) < 1):
        raise ValueError("Expected a trained, compatible Act3D demo checkpoint")
    if set(data["language"]) != set(data["config"]["instructions"]):
        raise ValueError("Checkpoint language vocabulary is incomplete")
    policy = Policy(data["config"], device=device)
    policy.net.load_state_dict(data["state_dict"], strict=True)
    policy.net.eval()
    return policy, data
