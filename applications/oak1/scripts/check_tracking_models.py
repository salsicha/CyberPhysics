#!/usr/bin/env python3
"""Build-time check of the tracking models in /models (the Dockerfile runs
it): every json export_yolo.py wrote names a real blob and ONNX (not Git
LFS pointers), the blob builds into the OAK-1 pipeline beside MegaDepth
(no device needed) and the ONNX runs in ONNX Runtime as the simulator
detector runs it."""
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oak1_pipeline  # noqa: E402
from tracking_yolo import YoloDetector  # noqa: E402

MEGADEPTH = 'megadepth_192x256_openvino_2021.4_6shave.blob'


def main(models_dir='/models'):
    metas = [path for path in sorted(glob.glob(os.path.join(models_dir, '*.json')))
             if 'blob' in json.load(open(path))]
    if not metas:
        raise SystemExit(f'no tracking model json in {models_dir}')
    for path in metas:
        meta = json.load(open(path))
        for name in (meta['blob'], meta['onnx']):
            if os.path.getsize(os.path.join(models_dir, name)) < 1_000_000:
                raise SystemExit(f'{name} is a Git LFS pointer: git lfs pull')
        oak1_pipeline.build_pipeline(
            os.path.join(models_dir, MEGADEPTH),
            tracking=oak1_pipeline.TrackingConfig(
                blob_path=os.path.join(models_dir, meta['blob']),
                num_classes=len(meta['classes']),
                width=meta['input_width'], height=meta['input_height']))
        detector = YoloDetector(os.path.join(models_dir, meta['onnx']), path)
        detector.detect(np.zeros((480, 640, 3), np.uint8))
        print(f'tracking model ok: {os.path.basename(path)}')


if __name__ == '__main__':
    main(*sys.argv[1:])
