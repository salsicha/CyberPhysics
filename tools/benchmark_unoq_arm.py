#!/usr/bin/env python3
"""On-device inference benchmark; never connects to or commands an actuator.

Requires the pinned model sources/checkpoints and observations described in the
UNO Q benchmark report. Run each configuration in a fresh process. Inputs are
recorded Isaac observations; this measures compute, not physical grasp accuracy.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import sys
import time


def snapshot():
    memory = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, value = line.split(':', 1)
        if key in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree'):
            memory[key + '_kib'] = int(value.split()[0])
    temps = {}
    for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            temps[path.parent.name] = int(path.read_text()) / 1000
        except (OSError, ValueError):
            pass
    return {'memory': memory, 'temperature_c': temps,
            'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
            'major_page_faults': resource.getrusage(resource.RUSAGE_SELF).ru_majflt}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--mode', choices=('peract', 'depth', 'pipeline'), required=True)
    parser.add_argument('--backend', choices=('torch', 'ort'), default='torch')
    parser.add_argument('--models', type=Path, help='ONNX export directory for --backend ort')
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--depth-size', type=int, default=518)
    parser.add_argument('--warmup', type=int, default=1)
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.threads < 1 or args.runs < 1 or args.warmup < 0:
        parser.error('invalid thread/run counts')
    if args.backend == 'ort' and args.models is None:
        parser.error('--backend ort requires --models')
    os.environ['OMP_NUM_THREADS'] = str(args.threads)
    os.environ['OPENBLAS_NUM_THREADS'] = str(args.threads)
    assets = args.assets.resolve()
    sys.path[:0] = [str(assets), str(assets / 'peract'), str(assets / 'depth_metric')]
    os.environ['PERACT_ROOT'] = str(assets / 'peract')
    initial = snapshot()
    start = time.perf_counter()
    import numpy as np
    import torch
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    imports_s = time.perf_counter() - start
    observations = np.load(assets / 'observations.npz', allow_pickle=False)
    start = time.perf_counter()
    hashes = {}
    if args.backend == 'ort':
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = args.threads
        options.inter_op_num_threads = 1
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')

        def session(name):
            return ort.InferenceSession(str(args.models / (name + '.onnx')),
                                        sess_options=options, providers=['CPUExecutionProvider'])
    if args.mode in ('peract', 'pipeline'):
        from model import load_policy
        policy, checkpoint = load_policy(assets / 'peract_reach.pt', 'cpu')
        hashes['peract'] = hashlib.sha256((assets / 'peract_reach.pt').read_bytes()).hexdigest()
        if args.backend == 'ort':
            peract_session = session('peract')
    if args.mode in ('depth', 'pipeline'):
        import cv2
        cv2.setNumThreads(1)
        weight_path = assets / 'depth_anything_v2_metric_hypersim_vits.pth'
        if args.backend == 'torch':
            from depth_anything_v2.dpt import DepthAnythingV2
            depth_model = DepthAnythingV2(encoder='vits', features=64,
                                        out_channels=[48, 96, 192, 384], max_depth=20).eval()
            depth_model.load_state_dict(torch.load(weight_path, map_location='cpu', weights_only=True))
        else:
            from depth_anything_v2.util.transform import Resize, NormalizeImage, PrepareForNet
            depth_session = session(f'depth{args.depth_size}')
            transforms = [Resize(width=args.depth_size, height=args.depth_size, resize_target=False,
                                 keep_aspect_ratio=True, ensure_multiple_of=14, resize_method='lower_bound',
                                 image_interpolation_method=cv2.INTER_CUBIC),
                          NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
                          PrepareForNet()]
        hashes['depth'] = hashlib.sha256(weight_path.read_bytes()).hexdigest()
    load_s = time.perf_counter() - start
    camera_pos = np.array([1.05, -0.65, 0.85], dtype=np.float32)
    forward = np.array([0.48, 0., 0.05], dtype=np.float32) - camera_pos
    forward /= np.linalg.norm(forward)
    # Recover fixed calibrated camera rays from recorded world points. Distance
    # cancels here: only ray direction is used for new monocular-depth points.
    rays = observations['points'][0] - camera_pos
    rays = rays / (rays @ forward)[..., None]
    report = {'mode': args.mode, 'backend': args.backend, 'threads': args.threads, 'depth_input_size': args.depth_size,
              'warmup': args.warmup, 'runs': args.runs, 'platform': platform.platform(),
              'python': platform.python_version(), 'torch': torch.__version__,
              'cuda_available': torch.cuda.is_available(), 'imports_s': imports_s,
              'load_s': load_s, 'checkpoint_sha256': hashes, 'initial': initial,
              'loaded': snapshot(), 'samples': [],
              'scope': 'Recorded 128x128 Isaac RGB-D, cached task language; no camera capture, ROS, IK or motor execution.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)

    def save():
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')

    save()
    print(json.dumps({'event': 'loaded', 'load_s': load_s, 'resources': report['loaded']}), flush=True)
    for iteration in range(args.warmup + args.runs):
        index = iteration % len(observations['rgb'])
        rgb = observations['rgb'][index]
        points = observations['points'][index]
        prop = observations['proprio'][index]
        sample = {'iteration': iteration, 'warmup': iteration < args.warmup,
                  'source_index': int(observations['source_indices'][index])}
        start = time.perf_counter()
        if args.mode in ('depth', 'pipeline'):
            if args.backend == 'torch':
                depth = depth_model.infer_image(rgb[..., ::-1].copy(), input_size=args.depth_size)
            else:
                image = {'image': rgb / 255.0}
                for transform in transforms:
                    image = transform(image)
                result = depth_session.run(None, {'image': image['image'][None]})[0]
                depth = torch.nn.functional.interpolate(torch.from_numpy(result)[:, None], rgb.shape[:2],
                                                        mode='bilinear', align_corners=True)[0, 0].numpy()
            sample['depth_ms'] = (time.perf_counter() - start) * 1000
            if not np.isfinite(depth).all() or np.any(depth <= 0):
                raise ValueError('Invalid depth output')
            begin = time.perf_counter()
            points = camera_pos + rays * depth[..., None]
            sample['projection_ms'] = (time.perf_counter() - begin) * 1000
        if args.mode in ('peract', 'pipeline'):
            instruction = checkpoint['config']['instructions'][int(observations['language_ids'][index])]
            begin = time.perf_counter()
            if args.backend == 'torch':
                action = policy.predict(rgb, points, prop, checkpoint['language'][instruction])
            else:
                voxels = policy.voxelize(torch.from_numpy(rgb)[None], torch.from_numpy(points)[None]).numpy()
                sentence, tokens = checkpoint['language'][instruction]
                q, rotation_grip, collision = peract_session.run(None, {
                    'voxels': voxels, 'proprio': prop[None], 'tokens': tokens.numpy()[None]})
                n = checkpoint['config']['model']['voxel_size']
                bounds = np.asarray(checkpoint['config']['workspace_bounds'])
                xyz = bounds[:3] + (np.asarray(np.unravel_index(q.argmax(), (n, n, n))) + .5) * (bounds[3:] - bounds[:3]) / n
                action = {'position': xyz.tolist(),
                          'euler_degrees': (rotation_grip[0, :216].reshape(3, 72).argmax(-1) * 5 - 180).tolist(),
                          'gripper_open': bool(rotation_grip[0, -2:].argmax()),
                          'ignore_collisions': bool(collision[0].argmax())}
            sample['peract_ms'] = (time.perf_counter() - begin) * 1000
            sample['action'] = action
            sample['instruction'] = instruction
        sample['total_ms'] = (time.perf_counter() - start) * 1000
        if args.mode in ('depth', 'pipeline'):
            sample['depth_min_m'] = float(depth.min())
            sample['depth_max_m'] = float(depth.max())
            sample['depth_median_m'] = float(np.median(depth))
        sample['resources'] = snapshot()
        report['samples'].append(sample)
        save()
        print(json.dumps(sample), flush=True)
    timed = [s for s in report['samples'] if not s['warmup']]
    report['summary'] = {}
    for key in ('total_ms', 'depth_ms', 'projection_ms', 'peract_ms'):
        values = [s[key] for s in timed if key in s]
        if values:
            report['summary'][key] = {'median': float(np.median(values)),
                                     'p95': float(np.percentile(values, 95)),
                                     'min': min(values), 'max': max(values)}
    report['final'] = snapshot()
    save()
    print(json.dumps({'event': 'complete', 'summary': report['summary'], 'final': report['final']}), flush=True)


if __name__ == '__main__':
    main()
