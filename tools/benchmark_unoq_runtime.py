#!/usr/bin/env python3
"""Benchmark exported networks and check outputs against PyTorch references.

These are network-only timings with fixed preprocessed input, not pipeline
timings. A fast conversion with incorrect output is not a successful result.
"""
import argparse
import json
import os
from pathlib import Path
import resource
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models', type=Path, required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--runtime', choices=('mnn', 'ncnn', 'ort'), required=True)
    parser.add_argument('--gpu', action='store_true')
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.runs < 1 or args.threads < 1:
        parser.error('run and thread counts must be positive')
    os.environ['OPENBLAS_NUM_THREADS'] = str(args.threads)
    import numpy as np
    reference = np.load(args.models / f'{args.model}_reference.npz')
    inputs = {}
    report = {'runtime': args.runtime, 'model': args.model, 'gpu_requested': args.gpu,
              'threads': args.threads, 'timings_ms': [], 'scope': 'Network only, fixed preprocessed input'}
    start = time.perf_counter()
    if args.runtime == 'ort':
        if args.gpu:
            raise ValueError('This ONNX Runtime package uses CPUExecutionProvider')
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = args.threads
        options.inter_op_num_threads = 1
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        engine = ort.InferenceSession(str(args.models / f'{args.model}.onnx'),
                                     sess_options=options, providers=['CPUExecutionProvider'])
        inputs = {entry.name: reference[entry.name] for entry in engine.get_inputs()}
        output_names = [entry.name for entry in engine.get_outputs()]
        report['providers'] = engine.get_providers()

        def run():
            return dict(zip(output_names, engine.run(None, inputs)))
    elif args.runtime == 'mnn':
        if args.gpu:
            raise ValueError('Downloaded MNN wheel has no OpenCL backend; do not label CPU fallback as GPU')
        import MNN
        engine = MNN.Interpreter(str(args.models / f'{args.model}.mnn'))
        session = engine.createSession({'backend': 'CPU', 'numThread': args.threads, 'precision': 'high'})
        for name, tensor in engine.getSessionInputAll(session).items():
            inputs[name] = MNN.Tensor(tensor.getShape(), MNN.Halide_Type_Float,
                                      tuple(reference[name].ravel().tolist()), MNN.Tensor_DimensionType_Caffe)
            tensor.copyFrom(inputs[name])
        output_names = list(engine.getSessionOutputAll(session))

        def run():
            engine.runSession(session)
            result = {}
            for name in output_names:
                tensor = engine.getSessionOutput(session, name)
                host = MNN.Tensor(tensor.getShape(), MNN.Halide_Type_Float,
                                  (0.0,) * int(np.prod(tensor.getShape())), MNN.Tensor_DimensionType_Caffe)
                tensor.copyToHostTensor(host)
                result[name] = np.array(host.getData(), dtype=np.float32).reshape(tensor.getShape())
            return result
    else:
        import ncnn
        engine = ncnn.Net()
        engine.opt.num_threads = args.threads
        engine.opt.use_vulkan_compute = args.gpu
        engine.opt.use_fp16_storage = False
        engine.opt.use_fp16_arithmetic = False
        engine.opt.use_fp16_packed = False
        if args.gpu:
            report['gpu_count'] = ncnn.get_gpu_count()
            if not report['gpu_count']:
                raise RuntimeError('No Vulkan GPU detected')
            engine.set_vulkan_device(0)
        if engine.load_param(str(args.models / f'{args.model}.ncnn.param')) != 0:
            raise RuntimeError('NCNN parameter load failed')
        if engine.load_model(str(args.models / f'{args.model}.ncnn.bin')) != 0:
            raise RuntimeError('NCNN weight load failed')
        image = ncnn.Mat(np.ascontiguousarray(reference['image'][0])).clone()

        def run():
            with engine.create_extractor() as extractor:
                if extractor.input('in0', image) != 0:
                    raise RuntimeError('NCNN input failed')
                status, result = extractor.extract('out0')
                if status != 0:
                    raise RuntimeError('NCNN inference failed')
                return {'depth': np.asarray(result).copy().reshape(reference['depth'].shape)}
    report['load_s'] = time.perf_counter() - start
    print(json.dumps({'event': 'loaded', **report}), flush=True)
    for i in range(args.runs + 1):
        start = time.perf_counter()
        outputs = run()
        elapsed = (time.perf_counter() - start) * 1000
        if i:
            report['timings_ms'].append(elapsed)
        else:
            report['cold_ms'] = elapsed
        print(json.dumps({'iteration': i, 'elapsed_ms': elapsed}), flush=True)
    report['validation'] = {}
    for name, actual in outputs.items():
        expected = reference[name]
        finite = bool(np.isfinite(actual).all())
        report['validation'][name] = {'finite': finite,
            'max_absolute_error': float(np.max(np.abs(actual - expected))) if finite else None,
            'mean_absolute_error': float(np.mean(np.abs(actual - expected))) if finite else None,
            'allclose_atol1e-3_rtol1e-3': bool(np.allclose(actual, expected, atol=1e-3, rtol=1e-3)),
            'same_argmax': bool(actual.argmax() == expected.argmax()) if finite else False}
    report['median_ms'] = float(np.median(report['timings_ms']))
    report['p95_ms'] = float(np.percentile(report['timings_ms'], 95))
    report['peak_rss_mib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(report), flush=True)
    if not all(value['allclose_atol1e-3_rtol1e-3'] for value in report['validation'].values()):
        sys.exit(2)


if __name__ == '__main__':
    main()
