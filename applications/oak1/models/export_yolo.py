#!/usr/bin/env python3
"""Export an Ultralytics YOLOv8 detector to the ONNX and Myriad X blob the
aerodrone tracking detectors run (systems/aerodrone/tracking.md).

The OAK-1 runs the blob in DepthAI's YoloDetectionNetwork, which decodes
the anchor-free "yolov6r2" head luxonis/tools exports: per stride (8, 16,
32) one [1, 4 + 1 + classes, H, W] tensor of box side distances in grid
cells (left, top, right, bottom, after the DFL), the best class score and
the sigmoid class scores. The simulator detector runs the ONNX this blob is
compiled from (tracking_yolo.py decodes it the same way), so both see the
same network.

Run outside the images (needs torch, ultralytics, onnx, onnxsim and
blobconverter; compilation uses Luxonis' blobconverter service with
OpenVINO 2021.4, the version megadepth_ros.py sets on the pipeline):

  python3 export_yolo.py WEIGHTS.pt --name visdrone_yolov8n \
      --width 640 --height 352 --shaves 6 --out-dir .

writes NAME_WxH.onnx, NAME_WxH_openvino_2021.4_Nshave.blob and
NAME_WxH.json (input size, class names, provenance).
"""
import argparse
import hashlib
import json
from pathlib import Path

import torch
from torch import nn
import torch.nn.functional as F

OUTPUT_NAMES = ['output1_yolov6r2', 'output2_yolov6r2', 'output3_yolov6r2']


class DepthAIHead(nn.Module):
    """YOLOv8 Detect head with the outputs DepthAI decodes on device."""

    def __init__(self, detect):
        super().__init__()
        self.nl, self.nc = detect.nl, detect.nc
        self.reg_max = detect.reg_max
        self.cv2, self.cv3 = detect.cv2, detect.cv3
        self.f, self.i = detect.f, detect.i
        self.stride = detect.stride
        self.proj = nn.Conv2d(self.reg_max, 1, 1, bias=False).requires_grad_(False)
        self.proj.weight.data[:] = torch.arange(
            self.reg_max, dtype=torch.float).view(1, self.reg_max, 1, 1)

    def forward(self, x):
        outputs = []
        for i in range(self.nl):
            box = self.cv2[i](x[i])
            batch, _, height, width = box.shape
            box = box.view(batch, 4, self.reg_max, height * width)
            box = self.proj(F.softmax(box.permute(0, 2, 1, 3), dim=1))[:, 0]
            box = box.reshape(batch, 4, height, width)
            cls = self.cv3[i](x[i]).sigmoid()
            conf, _ = cls.max(1, keepdim=True)
            outputs.append(torch.cat([box, conf, cls], 1))
        return outputs


def load_model(weights):
    from ultralytics.nn.modules import Detect
    from ultralytics.nn.tasks import load_checkpoint

    model, _ = load_checkpoint(weights, device='cpu', inplace=True,
                               fuse=False)
    model = model.fuse().eval().float()
    if not isinstance(model.model[-1], Detect) or \
            type(model.model[-1]).__name__ != 'Detect':
        raise SystemExit(
            f'{weights}: only plain detection heads export (got '
            f'{type(model.model[-1]).__name__}); segmentation, pose and '
            'OBB heads need their own decoder')
    names = model.names
    names = [names[k] for k in sorted(names)] if isinstance(names, dict) \
        else list(names)
    model.model[-1] = DepthAIHead(model.model[-1])
    return model, names


def export_onnx(model, width, height, path):
    import onnx
    import onnxsim

    image = torch.zeros(1, 3, height, width)
    torch.onnx.export(
        model, image, str(path), opset_version=12, do_constant_folding=True,
        input_names=['images'], output_names=OUTPUT_NAMES,
        dynamic_axes=None, dynamo=False)
    simplified, ok = onnxsim.simplify(onnx.load(str(path)))
    if not ok:
        raise SystemExit('onnxsim could not validate the simplified model')
    onnx.save(simplified, str(path))


def compile_blob(onnx_path, shaves, version, out_dir):
    import blobconverter

    blob = blobconverter.from_onnx(
        model=str(onnx_path), data_type='FP16', shaves=shaves,
        version=version, use_cache=False, output_dir=str(out_dir),
        # The camera sends BGR uint8; the network was trained on RGB / 255.
        optimizer_params=['--reverse_input_channels', '--scale=255',
                          '--output=' + ','.join(OUTPUT_NAMES)],
        compile_params=['-ip U8'])
    return Path(blob)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('weights')
    parser.add_argument('--name', required=True)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=352)
    parser.add_argument('--shaves', type=int, default=6)
    parser.add_argument('--openvino', default='2021.4')
    parser.add_argument('--out-dir', default='.')
    parser.add_argument('--no-blob', action='store_true')
    parser.add_argument('--source', default='',
                        help='where the weights came from (URL@revision)')
    parser.add_argument('--license', default='')
    args = parser.parse_args()
    if args.width % 32 or args.height % 32:
        raise SystemExit('width and height must be multiples of 32')

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f'{args.name}_{args.width}x{args.height}'
    model, names = load_model(args.weights)
    onnx_path = out_dir / f'{stem}.onnx'
    export_onnx(model, args.width, args.height, onnx_path)
    meta = {
        'name': args.name,
        'input_width': args.width,
        'input_height': args.height,
        'classes': names,
        'outputs': OUTPUT_NAMES,
        'strides': [int(s) for s in model.model[-1].stride],
        'onnx': onnx_path.name,
        'onnx_sha256': sha256(onnx_path),
        'weights_sha256': sha256(args.weights),
        'source': args.source,
        'license': args.license,
    }
    if not args.no_blob:
        blob = compile_blob(onnx_path, args.shaves, args.openvino, out_dir)
        target = out_dir / (
            f'{stem}_openvino_{args.openvino}_{args.shaves}shave.blob')
        blob.replace(target)
        meta.update(blob=target.name, blob_sha256=sha256(target),
                    blob_openvino=args.openvino, blob_shaves=args.shaves)
    (out_dir / f'{stem}.json').write_text(json.dumps(meta, indent=2) + '\n')
    print(json.dumps(meta, indent=2))


if __name__ == '__main__':
    main()
