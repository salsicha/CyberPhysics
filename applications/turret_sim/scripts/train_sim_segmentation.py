#!/usr/bin/env python3
"""Train a simulation-only YOLOE checkpoint on independently randomized scenes.

No frames or trajectories from the acceptance run enter this dataset. This
checkpoint is a synthetic-camera fixture, not a hardware perception model.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from turret_sim import TurretSim


def mask_polygon(mask):
    """Connect all boundary loops with doubled bridges, retaining mask holes."""
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c.reshape(-1, 2) for c in contours if len(c) >= 3]
    if not contours:
        return None
    contours.sort(key=lambda c: abs(cv2.contourArea(c)), reverse=True)
    path = contours.pop(0)
    for contour in contours:
        distances = ((path[:, None, :].astype(float) - contour[None, :, :]) ** 2).sum(axis=2)
        i, j = np.unravel_index(distances.argmin(), distances.shape)
        loop = np.concatenate([contour[j:], contour[:j], contour[j:j + 1]])
        path = np.concatenate([path[:i + 1], loop, path[i:i + 1], path[i + 1:]])
    return path


def generate(base, root, count, split, seed, names):
    rng = np.random.default_rng(seed)
    images, labels = root / 'images' / split, root / 'labels' / split
    images.mkdir(parents=True)
    labels.mkdir(parents=True)
    min_iou, instances, hashes, skipped = 1.0, 0, [], 0
    for index in range(count):
        scene = copy.deepcopy(base)
        scene['seed'] = int(rng.integers(1, 2**31))
        scene['objects'] = [obj for obj in scene['objects'] if obj['id'] == base['target']['id'] or rng.random() > .12]
        for obj in scene['objects']:
            obj['trajectory'] = [{'t': 0., 'azimuth_deg': float(rng.uniform(-29, 29)),
                                  'elevation_deg': float(rng.uniform(-16, 13)),
                                  'distance_m': float(rng.uniform(3.8, 10.))}]
            obj['occlusion_intervals_s'] = []
            obj['size_px_at_5m'] = [float(v * rng.uniform(.8, 1.2)) for v in obj['size_px_at_5m']]
            obj['color'] = np.clip(np.asarray(obj['color']) + rng.integers(-16, 17, 3), 0, 255).tolist()
        variant = rng.choice(['nominal', 'low_light', 'glare_motion_blur'], p=[.6, .2, .2])
        sim = TurretSim(scene, variant_name=variant)
        sim.np, sim.cv2 = np, cv2
        sim.background = sim._render_background()
        frame, masks = sim.render_frame(0.)
        rows = []
        objects = {obj['id']: obj for obj in scene['objects']}
        for identifier, mask in masks.items():
            if np.count_nonzero(mask) < 80:
                continue
            polygon = mask_polygon(mask)
            if polygon is None:
                continue
            restored = np.zeros_like(mask)
            cv2.fillPoly(restored, [polygon], 255)
            intersection = np.count_nonzero((mask > 0) & (restored > 0))
            union = np.count_nonzero((mask > 0) | (restored > 0))
            iou = intersection / union
            if iou < .97:
                skipped += 1
                # Tiny, heavily occluded instances may have isolated pixels that
                # YOLO polygons cannot encode faithfully; do not invent a label.
                continue
            min_iou = min(min_iou, iou)
            normalized = polygon.astype(float) / [sim.width, sim.height]
            category = names.index(objects[identifier]['yolo_prompt'])
            rows.append(str(category) + ' ' + ' '.join(f'{v:.8f}' for v in normalized.ravel()))
            instances += 1
        filename = f'{seed}_{index:05d}'
        cv2.imwrite(str(images / f'{filename}.png'), frame)
        (labels / f'{filename}.txt').write_text('\n'.join(rows) + '\n')
        hashes.append(hashlib.sha256(frame.tobytes()).hexdigest())
    return {'seed': seed, 'images': count, 'instances': instances,
            'minimum_annotation_roundtrip_iou': min_iou, 'unrepresentable_instances': skipped,
            'image_hashes': hashes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--checkpoint', default='yoloe-26s-seg.pt')
    parser.add_argument('--train-images', type=int, default=480)
    parser.add_argument('--val-images', type=int, default=96)
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--generate-only', action='store_true')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('--output must be a new directory')
    base = json.loads(args.scenario.read_text())
    names = list(dict.fromkeys(obj['yolo_prompt'] for obj in base['objects']))
    root = args.output.resolve()
    records = {split: generate(base, root, count, split, seed, names)
               for split, count, seed in [('train', args.train_images, 6101201),
                                          ('val', args.val_images, 6101202)]}
    assert not set(records['train']['image_hashes']) & set(records['val']['image_hashes'])
    dataset = root / 'dataset.yaml'
    dataset.write_text(yaml.safe_dump({'path': str(root), 'train': 'images/train',
                                      'val': 'images/val', 'names': names}))
    report = {'simulation_only': True, 'acceptance_scene_seed_excluded': base['seed'],
              'randomized_trajectories': True, 'checkpoint': args.checkpoint, 'splits': records}
    (root / 'dataset_provenance.json').write_text(json.dumps(report, indent=2) + '\n')
    if args.generate_only:
        return
    from ultralytics import YOLOE
    from ultralytics.models.yolo.yoloe import YOLOEPESegTrainer
    model = YOLOE(args.checkpoint)
    model.train(data=str(dataset), trainer=YOLOEPESegTrainer, epochs=args.epochs,
                patience=10, imgsz=640, batch=6, device=0, workers=2, seed=6101203,
                project=str(root), name='training', exist_ok=False, plots=False,
                amp=True, cache=False, close_mosaic=5, fliplr=0., flipud=0.)


if __name__ == '__main__':
    main()
