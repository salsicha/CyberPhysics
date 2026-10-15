#!/usr/bin/env python3
"""Export unchanged benchmark networks; requires torch, onnx, MNN and pnnx.

Run on the development computer. Only exported models and reference arrays go
to the board. Conversion/inference checks do not establish task accuracy.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    assets, out = args.assets.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    sys.path[:0] = [str(assets), str(assets / 'peract'), str(assets / 'depth_metric')]
    os.environ['PERACT_ROOT'] = str(assets / 'peract')
    import numpy as np
    import torch
    torch.set_num_threads(4)
    from model import load_policy
    from depth_anything_v2.dpt import DepthAnythingV2
    obs = np.load(assets / 'observations.npz')
    policy, checkpoint = load_policy(assets / 'peract_reach.pt', 'cpu')
    reference_actions = []
    for i in range(len(obs['rgb'])):
        instruction = checkpoint['config']['instructions'][int(obs['language_ids'][i])]
        reference_actions.append(policy.predict(obs['rgb'][i], obs['points'][i], obs['proprio'][i], checkpoint['language'][instruction]))
    (out / 'peract_reference_actions.json').write_text(json.dumps(reference_actions, indent=2))
    vox = policy.voxelize(torch.as_tensor(obs['rgb'][:1]), torch.as_tensor(obs['points'][:1]))
    sentence, tokens = checkpoint['language'][checkpoint['config']['instructions'][0]]
    inputs = (vox, torch.as_tensor(obs['proprio'][:1]), sentence[None], tokens[None])
    with torch.inference_mode():
        outputs = policy(*inputs)
        torch.onnx.export(policy, inputs, str(out / 'peract.onnx'), opset_version=17, dynamo=False,
                          input_names=['voxels', 'proprio', 'sentence', 'tokens'],
                          output_names=['translation', 'rotation_grip', 'collision'],
                          do_constant_folding=True)
    np.savez(out / 'peract_reference.npz', **{k: v.numpy() for k, v in zip(['voxels', 'proprio', 'sentence', 'tokens'], inputs)},
             **{k: v.numpy() for k, v in zip(['translation', 'rotation_grip', 'collision'], outputs)})
    # Invoke the native converter directly: the CLI wrapper installs a logging
    # SDK and reports conversion metadata. Benchmark conversion stays offline.
    converter = [sys.executable, '-c', 'import sys, _tools; _tools.mnnconvert(["MNNConvert"] + sys.argv[1:])']
    subprocess.run([*converter, '-f', 'ONNX', '--modelFile', str(out / 'peract.onnx'),
                    '--MNNModel', str(out / 'peract.mnn'), '--bizCode', 'benchmark'], check=True)
    depth = DepthAnythingV2(encoder='vits', features=64, out_channels=[48, 96, 192, 384], max_depth=20).eval()
    depth.load_state_dict(torch.load(assets / 'depth_anything_v2_metric_hypersim_vits.pth', map_location='cpu', weights_only=True))
    for size in (224, 518):
        name = f'depth{size}'
        image, _ = depth.image2tensor(obs['rgb'][0][..., ::-1].copy(), size)
        with torch.inference_mode():
            result = depth(image)
            torch.onnx.export(depth, image, str(out / f'{name}.onnx'), opset_version=17, dynamo=False,
                              input_names=['image'], output_names=['depth'], do_constant_folding=True)
            traced = torch.jit.trace(depth, image, check_trace=False)
            traced.save(str(out / f'{name}.pt'))
        np.savez(out / f'{name}_reference.npz', image=image.numpy(), depth=result.numpy())
        subprocess.run([*converter, '-f', 'ONNX', '--modelFile', str(out / f'{name}.onnx'),
                        '--MNNModel', str(out / f'{name}.mnn'), '--bizCode', 'benchmark'], check=True)
        subprocess.run(['pnnx', str(out / f'{name}.pt'), f'inputshape=[1,3,{size},{size}]', 'fp16=0'], check=True)
        print(f'Exported {name}', flush=True)


if __name__ == '__main__':
    main()
