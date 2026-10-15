"""Train upstream Act3D on rendered Isaac demonstrations of colored-block reaching."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch
import torch.nn.functional as F

from act3d_policy import FORMAT, UPSTREAM_REF, Policy, load_policy
from contracts import config_digest

CLIP_TEXT_REV = "3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"


def encode_language(instructions, device):
    from transformers import CLIPTextModel, CLIPTokenizer
    # Same encoder, token features and length used by upstream preprocessing.
    model_id = "openai/clip-vit-base-patch32"
    tokenizer = CLIPTokenizer.from_pretrained(model_id, revision=CLIP_TEXT_REV)
    encoder = CLIPTextModel.from_pretrained(model_id, revision=CLIP_TEXT_REV,
                                            use_safetensors=False).eval().to(device)
    tokens = tokenizer(instructions, padding="max_length", max_length=53,
                       truncation=True, return_tensors="pt").to(device)
    with torch.inference_mode():
        embeddings = encoder(**tokens).last_hidden_state.cpu()
    return {text: embedding.clone() for text, embedding in zip(instructions, embeddings)}


def loss_for(pred, target):
    # Upstream main_keypose.LossAndMetrics: soft CE at each sampling scale,
    # quaternion MSE (10x) and opening MSE. No differentiable argmax shortcut.
    losses = []
    for points, masks in zip(pred["ghost_pcd_pyramid"], pred["ghost_pcd_masks_pyramid"]):
        distance = torch.linalg.vector_norm(points-target[:, :3, None], dim=1)
        labels = torch.softmax(-distance/.01, dim=-1).detach()
        losses.append(F.cross_entropy(masks[-1], labels))
    return (torch.stack(losses).mean() + 10*F.mse_loss(pred["rotation"], target[:, 3:7])
            + F.mse_loss(pred["gripper"], target[:, 7:8]))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    p.add_argument("--dataset", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--steps", type=int, default=6000)
    p.add_argument("--batch-size", type=int, default=4)
    p.add_argument("--seed", type=int, default=123)
    p.add_argument("--initialize-from", help="Fine-tune a compatible checkpoint with a fresh optimizer")
    p.add_argument("--device", default="cuda")
    args = p.parse_args()
    if args.steps < 1 or args.batch_size < 1:
        p.error("steps and batch size must be positive")
    torch.set_num_threads(4)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    rng = np.random.default_rng(args.seed)
    config = json.loads(Path(args.config).read_text())
    with np.load(args.dataset, allow_pickle=False) as source:
        if str(source["config_digest"]) != config_digest(config):
            raise ValueError("Dataset was collected with a different configuration")
        arrays = {key: source[key] for key in ("rgb", "points", "gripper_pose", "targets", "language_ids")}
    count = len(arrays["rgb"])
    if count < 2 or set(arrays["language_ids"].tolist()) != {0, 1}:
        raise ValueError("Expected demonstrations of both reaching instructions")
    if not np.isfinite(arrays["points"]).all():
        raise ValueError("Dataset contains invalid world-space depth")
    previous_steps = 0
    initial_sha = None
    if args.initialize_from:
        policy, initial = load_policy(args.initialize_from, args.device)
        if config_digest(initial["config"]) != config_digest(config):
            raise ValueError("Initialization checkpoint and training configuration differ")
        language = initial["language"]
        previous_steps = initial["training_steps"]
        initial_sha = hashlib.sha256(Path(args.initialize_from).read_bytes()).hexdigest()
        del initial
    else:
        language = encode_language(config["instructions"], args.device)
        policy = Policy(config, args.device, pretrained=True)
    language_tensor = torch.stack([language[text] for text in config["instructions"]]).to(args.device)
    print(json.dumps({"samples": count, "trainable_parameters": sum(
        p.numel() for p in policy.net.parameters() if p.requires_grad)}), flush=True)
    # Repeated views do not change frozen RN50 features. Cache them in host RAM;
    # every gradient still passes through the full trainable FPN and Act3D.
    feature_chunks = {}
    policy.net.eval()
    with torch.no_grad():
        for first in range(0, count, 8):
            images = torch.as_tensor(arrays["rgb"][first:first+8], device=args.device).float().movedim(-1, 1)/255.
            features = policy.net.backbone(policy.net.normalize(images))
            for key, value in features.items():
                feature_chunks.setdefault(key, []).append(value.half().cpu())
    feature_cache = {key: torch.cat(chunks) for key, chunks in feature_chunks.items()}
    del feature_chunks
    policy.net.train()
    optimizer = torch.optim.AdamW((p for p in policy.net.parameters() if p.requires_grad), lr=1e-4)
    started = time.monotonic()
    history = []
    for step in range(1, args.steps+1):
        indices = rng.integers(0, count, args.batch_size)
        rgb, points, gripper = policy.tensors(arrays["rgb"][indices], arrays["points"][indices], arrays["gripper_pose"][indices])
        policy.net.backbone.cached_features = {key: value[indices].to(args.device).float()
                                               for key, value in feature_cache.items()}
        target = torch.zeros((args.batch_size, 8), device=args.device)
        target[:, :3] = torch.as_tensor(arrays["targets"][indices], device=args.device)
        target[:, 3] = 1.  # Downward orientation: XYZW quaternion for 180deg about X.
        target[:, 7] = 1.  # Open gripper; this task reaches without grasping.
        instructions = language_tensor[arrays["language_ids"][indices]]
        pred = policy.net(rgb, points, instructions, gripper, gt_action=target)
        loss = loss_for(pred, target)
        if not torch.isfinite(loss):
            raise RuntimeError("Non-finite training loss")
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(policy.net.parameters(), 1.)
        optimizer.step()
        if step == 1 or step % 100 == 0 or step == args.steps:
            row = {"step": step, "loss": float(loss.detach()), "seconds": time.monotonic()-started}
            history.append(row)
            print(json.dumps(row), flush=True)
    policy.net.backbone.cached_features = None
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    data = {"format": FORMAT, "upstream_ref": UPSTREAM_REF, "config": config,
            "clip_text_revision": CLIP_TEXT_REV,
            "state_dict": {key: value.cpu() for key, value in policy.net.state_dict().items()},
            "language": language, "training_steps": previous_steps+args.steps,
            "phase_steps": args.steps, "initial_checkpoint_sha256": initial_sha,
            "training_samples": count,
            "training_seed": args.seed, "training_seconds": time.monotonic()-started,
            "dataset_sha256": hashlib.sha256(Path(args.dataset).read_bytes()).hexdigest(),
            "torch_version": str(torch.__version__), "loss_history": history}
    temporary = output.with_suffix(".tmp")
    torch.save(data, temporary)
    temporary.replace(output)
    output.with_suffix(".json").write_text(json.dumps({key: value for key, value in data.items()
                                                     if key not in ("state_dict", "language")}, indent=2))
    print(f"Saved trained Act3D checkpoint: {output}", flush=True)


if __name__ == "__main__":
    main()
