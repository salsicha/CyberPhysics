#!/usr/bin/env python3
"""Measure pi0 inference on placeholder observations; this does not control an arm."""

import argparse
import json
import logging
import time
from pathlib import Path


def load_weights_strict(policy, weights):
    # LeRobot v0.6.0's PI0Policy.from_pretrained catches load errors and returns
    # an untrained model. Keep its key conversion, but let every error escape.
    fixed = policy._fix_pytorch_state_dict_keys(weights, policy.config)
    fixed = {key if key.startswith('model.') else f'model.{key}': value
             for key, value in fixed.items()}
    policy.load_state_dict(fixed, strict=True)


def run(args):
    import torch
    from huggingface_hub import hf_hub_download
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.pi0.modeling_pi0 import PI0Policy
    from safetensors.torch import load_file

    device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')
    torch.manual_seed(args.seed)
    config = PreTrainedConfig.from_pretrained(args.model, revision=args.revision)
    if config.type != 'pi0':
        raise ValueError(f'Expected pi0 checkpoint, got {config.type}')
    config.device = device
    config.dtype = args.dtype
    # Check tokenizer access before downloading 14 GB of weights. The public
    # pi0 checkpoint uses Google's gated PaliGemma tokenizer; access is required.
    pre, post = make_pre_post_processors(
        config, args.model, pretrained_revision=args.revision,
        preprocessor_overrides={'device_processor': {'device': device}},
    )
    checkpoint = Path(args.model) / 'model.safetensors'
    if not checkpoint.is_file():
        checkpoint = Path(hf_hub_download(args.model, 'model.safetensors', revision=args.revision))
    if device.startswith('cuda'):
        torch.cuda.reset_peak_memory_stats()
    started = time.monotonic()
    policy = PI0Policy(config)
    load_weights_strict(policy, load_file(str(checkpoint)))
    policy.to(device).eval()
    policy.reset()
    batch = {name: torch.zeros(1, *feature.shape) for name, feature in config.input_features.items()}
    batch['task'] = [args.task]
    batch = pre(batch)
    load_s = time.monotonic() - started
    started = time.monotonic()
    with torch.inference_mode():
        action = post(policy.select_action(batch))
    if device.startswith('cuda'):
        torch.cuda.synchronize()
    if not torch.isfinite(action).all():
        raise ValueError('Policy returned a non-finite action')
    return {
        'model': args.model, 'revision': args.revision, 'device': device,
        'dtype': args.dtype, 'pretrained_weights_loaded': True,
        'placeholder_observations': True, 'arm_simulation': False,
        'load_s': load_s, 'inference_s': time.monotonic() - started,
        'peak_allocated_mib': torch.cuda.max_memory_allocated() / 2**20 if device.startswith('cuda') else None,
        'peak_reserved_mib': torch.cuda.max_memory_reserved() / 2**20 if device.startswith('cuda') else None,
        'action': action.cpu().tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='lerobot/pi0_base')
    parser.add_argument('--revision')
    parser.add_argument('--device')
    parser.add_argument('--dtype', choices=['bfloat16', 'float32'], default='bfloat16')
    parser.add_argument('--task', default='pick up the cube')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        report = {'passed': True, **run(args)}
    except Exception as exc:
        logging.getLogger(__name__).exception("pi0 inference failed")
        report = {'passed': False, 'model': args.model, 'revision': args.revision,
                  'dtype': args.dtype, 'placeholder_observations': True,
                  'arm_simulation': False, 'error': f'{type(exc).__name__}: {exc}'}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
