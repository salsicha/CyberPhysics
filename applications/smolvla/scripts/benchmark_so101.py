#!/usr/bin/env python3
"""Measure real SmolVLA inference memory/latency without commanding a robot."""
import argparse
import json
import logging
import time
from pathlib import Path

import numpy as np
from so101_policy_server import (
    SmolVLASO101Policy,
    load_joint_mapping,
    load_urdf_arm_limits,
)


def run(args):
    import torch

    torch.manual_seed(42)
    gpu = args.device.startswith('cuda')
    if gpu:
        torch.cuda.reset_peak_memory_stats()
    started = time.monotonic()
    if args.normalized_inputs_only:
        from lerobot.configs import PreTrainedConfig
        from lerobot.configs.types import NormalizationMode
        from lerobot.policies import make_pre_post_processors
        from lerobot.policies.smolvla import SmolVLAPolicy
        from lerobot.policies.utils import prepare_observation_for_inference

        config = PreTrainedConfig.from_pretrained(args.model, revision=args.revision)
        config.device, config.n_action_steps = args.device, 1
        # Explicit model-only benchmark: inputs/outputs are in normalized
        # training space. This path has no joint conversion or robot publisher.
        config.normalization_mapping = {key: NormalizationMode.IDENTITY
                                        for key in config.normalization_mapping}
        model = SmolVLAPolicy.from_pretrained(
            args.model, config=config, revision=args.revision, strict=True)
        pre, _ = make_pre_post_processors(config)
        image_keys = list(config.image_features)
    else:
        policy = SmolVLASO101Policy(
            model_path=args.model, device=args.device, checkpoint_units=args.checkpoint_units,
            action_horizon=1, revision=args.revision, local_files_only=False,
            joint_mapping=load_joint_mapping(args.joint_mapping), joint_limits=load_urdf_arm_limits(),
        )
        model, image_keys = policy.policy, policy.image_keys
    load_s = time.monotonic() - started
    observation = {
        'video': {'front': np.random.default_rng(42).integers(0, 256, (1, 1, 360, 640, 3), dtype=np.uint8)},
        'state': {'joint_positions': np.zeros((1, 1, 6), dtype=np.float32)},
        'language': {'task': [['pick up the red block and place it in the left bin']]},
    }
    latencies = []
    for _ in range(args.iterations):
        started = time.monotonic()
        if args.normalized_inputs_only:
            frame = {'observation.state': np.zeros(6, dtype=np.float32),
                     **{key: observation['video']['front'][0, 0].copy() for key in image_keys}}
            batch = pre(prepare_observation_for_inference(
                frame, device=torch.device(args.device),
                task=observation['language']['task'][0][0], robot_type='so101_follower'))
            with torch.inference_mode():
                action = model.select_action(batch).cpu().numpy().reshape(1, 1, 6)
        else:
            action = policy.get_action(observation)['joint_positions']
        if gpu:
            torch.cuda.synchronize()
        latencies.append((time.monotonic() - started) * 1000)
        if action.shape != (1, 1, 6) or not np.isfinite(action).all():
            raise ValueError(f'Invalid policy action: {action}')
    report = {
        'passed': True, 'model': args.model, 'revision': args.revision,
        'device': args.device, 'gpu_name': torch.cuda.get_device_name() if gpu else None,
        'load_s': load_s, 'iterations': args.iterations,
        'latency_ms': latencies, 'latency_p50_ms': float(np.median(latencies)),
        'latency_p95_ms': float(np.percentile(latencies, 95)),
        'first_inference_ms': latencies[0],
        'warm_latency_p50_ms': float(np.median(latencies[1:])) if len(latencies) > 1 else None,
        'warm_latency_p95_ms': float(np.percentile(latencies[1:], 95)) if len(latencies) > 1 else None,
        'peak_allocated_mib': torch.cuda.max_memory_allocated() / 2**20 if gpu else None,
        'peak_reserved_mib': torch.cuda.max_memory_reserved() / 2**20 if gpu else None,
        'parameters': sum(p.numel() for p in model.parameters()),
        'parameter_bytes': sum(p.numel() * p.element_size() for p in model.parameters()),
        'torch': torch.__version__, 'camera_keys': image_keys,
        'normalized_inputs_only': args.normalized_inputs_only,
        'adapter_statistics_validated': not args.normalized_inputs_only,
        'synthetic_observations': True, 'task_success_evaluated': False,
    }
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='lerobot/smolvla_base')
    parser.add_argument('--revision')
    parser.add_argument('--device', default='cuda')
    parser.add_argument('--checkpoint-units', choices=['so101', 'isaac'], default='so101')
    parser.add_argument('--iterations', type=int, default=8)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--normalized-inputs-only', action='store_true',
                        help='Measure the base model without joint decoding; never controls an arm')
    parser.add_argument('--joint-mapping', default='/workspace/systems/so101/config/lerobot_joint_mapping.json')
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error('--iterations must be positive')
    try:
        report = run(args)
    except (Exception, SystemExit) as exc:
        logging.getLogger(__name__).exception('SmolVLA benchmark failed')
        report = {'passed': False, 'model': args.model, 'revision': args.revision,
                  'normalized_inputs_only': args.normalized_inputs_only,
                  'error': f'{type(exc).__name__}: {exc}'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
