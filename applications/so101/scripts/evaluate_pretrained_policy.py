# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Evaluate the PushT diffusion policy (2D end-effector control, not a VLA arm)."""

import argparse
import json
import time
from pathlib import Path


def legacy_statistics(weights, config):
    """Recover the old checkpoint's training statistics, without inventing defaults."""
    import torch

    stats = {}
    for prefix, features in [('normalize_inputs', config.input_features),
                             ('normalize_targets', config.output_features)]:
        for key, feature in features.items():
            mode = config.normalization_mapping[feature.type].value
            required = {'MEAN_STD': ('mean', 'std'), 'MIN_MAX': ('min', 'max'),
                        'IDENTITY': ()}[mode]
            for name in required:
                tensor_key = f'{prefix}.buffer_{key.replace(".", "_")}.{name}'
                value = weights[tensor_key]  # missing statistics must fail
                if not torch.isfinite(value).all():
                    raise ValueError(f'Invalid checkpoint statistic: {tensor_key}')
                stats.setdefault(key, {})[name] = value
                if prefix == 'normalize_targets':
                    output = weights[f'unnormalize_outputs.buffer_{key.replace(".", "_")}.{name}']
                    if not torch.equal(value, output):
                        raise ValueError('Checkpoint input/output action statistics disagree')
    return stats


def load_policy(model, revision, device):
    from huggingface_hub import hf_hub_download
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
    from safetensors.torch import load_file

    config = PreTrainedConfig.from_pretrained(model, revision=revision)
    if config.type != 'diffusion':
        raise ValueError(f'Expected a diffusion checkpoint, got {config.type}')
    config.device = device
    checkpoint = Path(model) / 'model.safetensors'
    if not checkpoint.is_file():
        checkpoint = Path(hf_hub_download(model, 'model.safetensors', revision=revision))
    weights = load_file(str(checkpoint))
    policy = DiffusionPolicy(config)
    legacy = any(key.startswith('normalize_inputs.') for key in weights)
    if legacy:
        stats = legacy_statistics(weights, config)
        weights = {k: v for k, v in weights.items() if not k.startswith(
            ('normalize_inputs.', 'normalize_targets.', 'unnormalize_outputs.'))}
        pre, post = make_pre_post_processors(config, dataset_stats=stats)
    else:
        pre, post = make_pre_post_processors(
            config, model, pretrained_revision=revision,
            preprocessor_overrides={'device_processor': {'device': device}},
        )
    policy.load_state_dict(weights, strict=True)
    policy.to(device).eval()
    return policy, pre, post, legacy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='lerobot/diffusion_pusht')
    parser.add_argument('--revision')
    parser.add_argument('--device')
    parser.add_argument('--seeds', nargs='+', type=int, default=[42, 43, 44])
    parser.add_argument('--max-steps', type=int, default=300)
    parser.add_argument('--output', type=Path, default=Path('outputs/eval/example_pusht_diffusion'))
    parser.add_argument('--video', action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    if args.max_steps < 1:
        parser.error('--max-steps must be positive')

    import gym_pusht  # noqa: F401
    import gymnasium as gym
    import imageio.v2 as imageio
    import numpy as np
    import torch

    device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')
    args.output.mkdir(parents=True, exist_ok=True)
    if device.startswith('cuda'):
        torch.cuda.reset_peak_memory_stats()
    policy, pre, post, legacy = load_policy(args.model, args.revision, device)
    env = gym.make('gym_pusht/PushT-v0', obs_type='pixels_agent_pos',
                   max_episode_steps=args.max_steps)
    episodes = []
    try:
        for seed in args.seeds:
            torch.manual_seed(seed)
            np.random.seed(seed)
            policy.reset()
            pre.reset()
            post.reset()
            observation, _ = env.reset(seed=seed)
            frames, rewards, latency = [], [], []
            if args.video:
                frames.append(env.render())
            started = time.monotonic()
            for step in range(args.max_steps):
                batch = {
                    'observation.state': torch.from_numpy(observation['agent_pos']).float(),
                    'observation.image': torch.from_numpy(observation['pixels']).permute(2, 0, 1).float() / 255,
                }
                begin = time.monotonic()
                with torch.inference_mode():
                    action = post(policy.select_action(pre(batch))).squeeze(0).cpu().numpy()
                latency.append((time.monotonic() - begin) * 1000)
                if not np.isfinite(action).all():
                    raise ValueError('Policy returned a non-finite action')
                observation, reward, terminated, truncated, _ = env.step(action)
                rewards.append(float(reward))
                if args.video:
                    frames.append(env.render())
                if terminated or truncated:
                    break
            result = {'seed': seed, 'success': bool(terminated), 'steps': step + 1,
                      'max_reward': max(rewards), 'final_reward': rewards[-1],
                      'wall_s': time.monotonic() - started,
                      'action_latency_p95_ms': float(np.percentile(latency, 95))}
            episodes.append(result)
            print(json.dumps(result), flush=True)
            if args.video:
                imageio.mimsave(args.output / f'rollout_{seed}.mp4', frames, fps=env.metadata['render_fps'])
    finally:
        env.close()
    report = {
        'model': args.model, 'revision': args.revision, 'device': device,
        'legacy_statistics_restored': legacy, 'arm_simulation': False, 'vla': False,
        'episodes': episodes, 'success_rate': sum(e['success'] for e in episodes) / len(episodes),
        'peak_allocated_mib': torch.cuda.max_memory_allocated() / 2**20 if device.startswith('cuda') else None,
        'peak_reserved_mib': torch.cuda.max_memory_reserved() / 2**20 if device.startswith('cuda') else None,
    }
    (args.output / 'metrics.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
