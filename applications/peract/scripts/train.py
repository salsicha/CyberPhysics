"""Train PerAct on rendered Isaac observations; no RLBench/PyRep dependency."""
import argparse
import hashlib
import json
import os
import ssl
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from model import Policy, UPSTREAM_REF, config_digest


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    p.add_argument("--dataset", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--steps", type=int, default=3000)
    p.add_argument("--batch-size", type=int, default=8)
    p.add_argument("--device", default="cuda")
    args = p.parse_args()
    if args.steps < 1 or args.batch_size < 1:
        p.error("steps and batch-size must be positive")
    torch.set_num_threads(4)
    config = json.loads(Path(args.config).read_text())
    torch.manual_seed(config["seed"])
    rng = np.random.default_rng(config["seed"])
    data = np.load(args.dataset, allow_pickle=False)
    if str(data["config_digest"]) != config_digest(config):
        raise ValueError("Dataset/config mismatch")
    # The pinned upstream module changes Python's global HTTPS context on import.
    # Restore certificate verification before downloading the hash-checked weights.
    https_context = ssl._create_default_https_context
    from helpers.clip.core.clip import _download, _MODELS, tokenize, build_model
    ssl._create_default_https_context = https_context
    clip_path = _download(_MODELS["RN50"], root=os.environ.get("TORCH_HOME", "/tmp/peract-clip"))
    clip_model = build_model(torch.jit.load(clip_path, map_location="cpu").state_dict()).float()
    with torch.no_grad():
        sentence, tokens = clip_model.encode_text_with_embeddings(tokenize(config["instructions"]))
    language = {s: (sentence[i].float().cpu(), tokens[i].float().cpu())
                for i, s in enumerate(config["instructions"])}
    del clip_model
    policy = Policy(config, args.device)
    optimizer = torch.optim.AdamW(policy.parameters(), lr=5e-4, weight_decay=1e-6)
    # Precompute input voxels; gradients never flow through the scene encoding.
    voxels = []
    images, points = data["rgb"], data["points"]
    for i in range(0, len(images), args.batch_size):
        rgb = torch.as_tensor(images[i:i + args.batch_size], device=args.device)
        xyz = torch.as_tensor(points[i:i + args.batch_size], device=args.device)
        voxels.append(policy.voxelize(rgb, xyz).cpu())
    del images, points
    voxels = torch.cat(voxels)
    targets = np.asarray(data["targets"])
    bounds = np.asarray(config["workspace_bounds"])
    n = config["model"]["voxel_size"]
    indices = np.floor((targets - bounds[:3]) / (bounds[3:] - bounds[:3]) * n).astype(int)
    if ((indices < 0) | (indices >= n)).any():
        raise ValueError("Training target outside workspace")
    labels = np.ravel_multi_index(indices.T, (n, n, n))
    start = time.monotonic()
    for step in range(args.steps):
        ids = rng.integers(0, len(voxels), args.batch_size)
        langs = data["language_ids"][ids]
        prop = torch.as_tensor(data["proprio"][ids], device=args.device)
        q, rot, coll = policy(voxels[ids].to(args.device), prop,
                              sentence[langs].float().to(args.device), tokens[langs].float().to(args.device))
        # All demonstrations use a downward open gripper, with collision checking.
        rot_labels = torch.tensor([0, 36, 36], device=args.device).expand(len(ids), 3)
        loss = F.cross_entropy(q.flatten(1), torch.as_tensor(labels[ids], device=args.device))
        loss += F.cross_entropy(rot[:, :216].reshape(-1, 72), rot_labels.reshape(-1))
        loss += F.cross_entropy(rot[:, -2:], torch.ones(len(ids), dtype=torch.long, device=args.device))
        loss += F.cross_entropy(coll, torch.zeros(len(ids), dtype=torch.long, device=args.device))
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
        optimizer.step()
        if step % 100 == 0:
            print(json.dumps({"step": step, "loss": float(loss.detach()), "seconds": round(time.monotonic()-start, 1)}), flush=True)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    temporary_output = Path(args.output).with_suffix(".pt.tmp")
    torch.save({"format": "cyberphysics-peract-reach-v1", "upstream_ref": UPSTREAM_REF,
                "config": config, "state_dict": {k: v.cpu() for k, v in policy.state_dict().items()},
                "language": language, "training_steps": args.steps, "training_samples": len(voxels),
                "dataset_seed": int(data["seed"]), "training_seconds": time.monotonic()-start,
                "dataset_sha256": hashlib.sha256(Path(args.dataset).read_bytes()).hexdigest(),
                "torch_version": str(torch.__version__)}, temporary_output)
    os.replace(temporary_output, args.output)
    print(f"Saved trained PerAct: {args.output}", flush=True)


if __name__ == "__main__":
    main()
