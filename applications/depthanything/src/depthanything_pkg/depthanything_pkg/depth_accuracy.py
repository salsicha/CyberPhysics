#!/usr/bin/env python3
"""Depth Anything accuracy against a camera's measured depth
(applications/depthanything/README.md, "Accuracy").

  depth_accuracy record --output-dir DIR [--period-s 1] [--max-frames 400]
      A ROS 2 node: saves a colour frame and the depth image of the same
      stamp (a simulator's rendered depth, or a RealSense's depth aligned
      to colour) every --period-s of message time, as DIR/frame_NNNNN.npz
      (color bgr8, depth float32 metres with NaN where there is none,
      stamp, k). With --estimate-topic it also keeps the running node's
      estimate of that frame (frames the node skipped are not saved); with
      --odom-topic only frames after the car moved --min-motion-m or turned
      --min-turn-deg since the last saved one (no standing still).

  depth_accuracy evaluate --frames DIR [DIR ...] --model vkitti:vits:80 \
        --model hypersim:vits:20 [--output results.json]
      Runs each model on the saved frames exactly as depth_node does
      (infer_image on the bgr8 frame, NaN outside min_depth..max_depth) and
      reports, over the reference pixels within --band (0.5-5 m by
      default): coverage, absolute and relative error percentiles, the
      ratio estimate / reference, and the inference time per frame on this
      GPU. Each model is also scored after per-frame scale alignment: its
      estimate times the median of reference / estimate over a sparse grid
      of the reference (every --align-stride pixels, --align-range metres),
      which is what aligning to a RealSense on the car would do. With
      --camera-height-m (a level camera over a flat floor) it also reports
      what each estimate does to nvblox's obstacle slice (--slice-band):
      the floor pixels it lifts into the band (phantom obstacles) and the
      obstacle pixels it keeps there within --slice-tolerance-m.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

try:
    from depthanything_pkg import metric_depth as md
except ImportError:  # run from the source tree (tests)
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import metric_depth as md


def parse_model(spec):
    """'dataset:encoder:max_depth' (max_depth defaults to the trained one)."""
    parts = spec.split(':')
    if len(parts) not in (2, 3) or not all(parts):
        raise argparse.ArgumentTypeError(f'expected dataset:encoder[:max_depth], got {spec!r}')
    dataset, encoder = parts[0].lower(), parts[1]
    if len(parts) == 3:
        max_depth = float(parts[2])
    elif dataset in md.TRAINED_MAX_DEPTH:
        max_depth = md.TRAINED_MAX_DEPTH[dataset]
    else:
        raise argparse.ArgumentTypeError(f'{spec!r}: give max_depth for dataset {dataset!r}')
    return {'name': f'{dataset}/{encoder}/{max_depth:g}', 'dataset': dataset,
            'encoder': encoder, 'max_depth': max_depth}


def frame_files(directories):
    files = []
    for directory in directories:
        files.extend(sorted(Path(directory).glob('frame_*.npz')))
    return files


def load_frame(path):
    with np.load(path) as data:
        frame = {key: data[key] for key in data.files}
    frame['depth'] = md.depth_image_to_m(frame['depth'], '32FC1')
    return frame


class Scorer:
    """One estimate source scored raw, after per-frame alignment, and after
    one fixed scale for all frames (the median of the per-frame scales:
    what a recalibrated max_depth would give, fitted on these frames)."""

    def __init__(self, args):
        self.raw = md.ErrorAccumulator(args.band, stride=args.eval_stride)
        self.aligned = md.ErrorAccumulator(args.band, stride=args.eval_stride)
        self.scales = []
        self.unaligned_frames = 0
        self.sampled = []
        self.slices = {}
        self.args = args

    def slice(self, variant, k, row_step=1):
        args = self.args
        if getattr(args, 'camera_height_m', None) is None or k is None:
            return None
        if variant not in self.slices:
            self.slices[variant] = md.SliceAccumulator(
                k, args.camera_height_m, args.slice_band, args.band, args.slice_tolerance_m,
                stride=args.eval_stride if row_step == 1 else 1, row_step=row_step)
        return self.slices[variant]

    def add(self, estimate, reference, k=None):
        args = self.args
        self.raw.add(estimate, reference)
        if self.slice('raw', k) is not None:
            self.slice('raw', k).add(estimate, reference)
        step = args.eval_stride
        self.sampled.append((estimate[::step, ::step].copy(),
                             md.resize_nearest(reference, estimate.shape[:2])[::step, ::step].copy(), k))
        # The alignment grid starts at an odd offset, so with the even
        # evaluation stride no aligned pixel is also scored.
        scale = md.median_scale(reference, estimate, args.align_range[0], args.align_range[1],
                                stride=args.align_stride, offset=(args.align_stride // 2) | 1,
                                min_samples=args.align_min_samples)
        if scale is None:
            self.unaligned_frames += 1
            return
        self.scales.append(scale)
        self.aligned.add(estimate * np.float32(scale), reference)
        if self.slice('aligned', k) is not None:
            self.slice('aligned', k).add(estimate * np.float32(scale), reference)

    def summary(self):
        aligned = self.aligned.summary()
        aligned['frames_without_scale'] = self.unaligned_frames
        result = {'raw': self.raw.summary(), 'aligned': aligned}
        if self.scales:
            aligned['scale'] = {'median': round(float(np.median(self.scales)), 4),
                                'p10': round(float(np.percentile(self.scales, 10)), 4),
                                'p90': round(float(np.percentile(self.scales, 90)), 4)}
            scale = float(np.median(self.scales))
            fixed = md.ErrorAccumulator(self.args.band, stride=1)
            for estimate, reference, k in self.sampled:
                fixed.add(estimate * np.float32(scale), reference)
                if self.slice('fixed_scale', k, self.args.eval_stride) is not None:
                    self.slice('fixed_scale', k, self.args.eval_stride).add(
                        estimate * np.float32(scale), reference)
            result['fixed_scale'] = dict(fixed.summary(), scale=round(scale, 4))
        for variant, accumulator in self.slices.items():
            result[variant]['slice'] = accumulator.summary()
        return result


def evaluate(args):
    import torch

    files = frame_files(args.frames)
    if not files:
        print('no frame_*.npz in ' + ', '.join(args.frames), file=sys.stderr)
        return 2
    device = 'cuda' if torch.cuda.is_available() and args.device != 'cpu' else 'cpu'
    results = {'frames': len(files), 'sources': [str(d) for d in args.frames],
               'camera_height_m': args.camera_height_m,
               'device': torch.cuda.get_device_name(0) if device == 'cuda' else 'cpu',
               'band_m': list(args.band), 'eval_stride': args.eval_stride,
               'align_stride': args.align_stride, 'align_range_m': list(args.align_range),
               'input_size': args.input_size, 'models': {}}

    live = Scorer(args)
    live_frames = 0
    for path in files:
        frame = load_frame(path)
        if 'estimate' in frame:
            live.add(md.depth_image_to_m(frame['estimate'], '32FC1'), frame['depth'], frame.get('k'))
            live_frames += 1
    if live_frames:
        results['live'] = live.summary()

    for spec in args.model:
        model, checkpoint = md.load_model(spec['encoder'], spec['dataset'], spec['max_depth'],
                                          device=device)
        scorer = Scorer(args)
        total_ms, forward_ms = [], []
        first = load_frame(files[0])['color']
        with torch.no_grad():
            for _ in range(args.warmup):
                model.infer_image(first, args.input_size)
            for path in files:
                frame = load_frame(path)
                if device == 'cuda':
                    torch.cuda.synchronize()
                start = time.perf_counter()
                estimate = model.infer_image(frame['color'], args.input_size)
                if device == 'cuda':
                    torch.cuda.synchronize()
                total_ms.append((time.perf_counter() - start) * 1e3)
                # The network alone (CUDA events), without pre/post-processing.
                if device == 'cuda':
                    tensor, _ = model.image2tensor(frame['color'], args.input_size)
                    begin, end = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
                    begin.record()
                    model.forward(tensor)
                    end.record()
                    torch.cuda.synchronize()
                    forward_ms.append(begin.elapsed_time(end))
                scorer.add(md.mask_range(estimate, args.min_depth, spec['max_depth']), frame['depth'],
                           frame.get('k'))
        entry = scorer.summary()
        entry.update({'checkpoint': checkpoint, 'max_depth': spec['max_depth'],
                      'infer_image': md.timing_summary(total_ms),
                      'forward': md.timing_summary(forward_ms)})
        if device == 'cuda':
            entry['peak_gpu_memory_mib'] = round(torch.cuda.max_memory_allocated() / 2 ** 20, 1)
            torch.cuda.reset_peak_memory_stats()
        results['models'][spec['name']] = entry
        del model
        if device == 'cuda':
            torch.cuda.empty_cache()

    text = json.dumps(results, indent=2)
    if args.output:
        Path(args.output).write_text(text + '\n')
    print(text)
    print_table(results)
    return 0


def print_table(results):
    rows = []
    if 'live' in results:
        rows.append(('live node output', results['live']['raw'], None))
    for name, entry in results['models'].items():
        rows.append((name, entry['raw'], entry.get('infer_image')))
        if 'fixed_scale' in entry:
            rows.append((f"{name} x{entry['fixed_scale']['scale']:.3g} fixed", entry['fixed_scale'], None))
        rows.append((name + ' aligned', entry['aligned'], None))
    print(f"\n{'estimate':32} {'cover':>6} {'ratio':>7} {'|e| med':>8} {'|e| p95':>8} "
          f"{'rel med':>8} {'rel p95':>8} {'d<1.25':>7} {'floor>':>7} {'obst':>6} {'ms':>6}")
    for name, stats, timing in rows:
        if 'abs_error_m' not in stats:
            print(f'{name:32} {stats.get("coverage")!s:>6}  (no valid pixels)')
            continue
        band = stats.get('slice') or {}
        print(f"{name:32} {stats['coverage']:6.3f} {stats['ratio']['median']:7.3f} "
              f"{stats['abs_error_m']['median']:8.3f} {stats['abs_error_m']['p95']:8.3f} "
              f"{stats['rel_error']['median']:8.3f} {stats['rel_error']['p95']:8.3f} "
              f"{stats['delta_1_25']:7.3f} {band.get('floor_as_obstacle', '')!s:>7} "
              f"{band.get('obstacle_in_place', '')!s:>6} {timing['median_ms'] if timing else '':>6}")


# --- recording --------------------------------------------------------------

def image_array(msg):
    """(pixels, encoding) of a sensor_msgs/Image without cv_bridge: bgr8,
    rgb8 (returned as bgr8), 32FC1 and 16UC1."""
    dtype, channels = {'bgr8': ('u1', 3), 'rgb8': ('u1', 3), '32FC1': ('f4', 1),
                       '16UC1': ('u2', 1), 'mono16': ('u2', 1)}[msg.encoding]
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width * channels * dtype.itemsize].copy().view(dtype)
    pixels = pixels.astype(dtype.newbyteorder('='))
    shape = (msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width)
    pixels = pixels.reshape(shape)
    if msg.encoding == 'rgb8':
        return np.ascontiguousarray(pixels[..., ::-1]), 'bgr8'
    return pixels, msg.encoding


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class FramePairer:
    """Colour, depth (and estimate) messages by stamp; yields the stamps
    due for saving: complete, at least period_ns after the last saved."""

    def __init__(self, period_ns, need_estimate, keep=30):
        self.period_ns, self.need_estimate, self.keep = int(period_ns), need_estimate, keep
        self.pending = {}
        self.last_saved = None

    def add(self, kind, stamp, msg):
        entry = self.pending.setdefault(stamp, {})
        entry[kind] = msg
        if len(self.pending) > self.keep:
            for old in sorted(self.pending)[:len(self.pending) - self.keep]:
                del self.pending[old]
        kinds = {'color', 'depth'} | ({'estimate'} if self.need_estimate else set())
        if not kinds <= set(entry):
            return None
        # A stamp before the last saved one is a restarted simulator clock.
        if self.last_saved is not None and 0 <= stamp - self.last_saved < self.period_ns:
            return None
        self.last_saved = stamp
        for old in [s for s in self.pending if s <= stamp]:
            if old == stamp:
                continue
            del self.pending[old]
        return self.pending.pop(stamp)


def moved_enough(last, pose, min_motion_m, min_turn_deg):
    """Whether pose (x, y, yaw) is min_motion_m or min_turn_deg away from
    last (always with no last pose)."""
    if last is None:
        return True
    turn = abs((pose[2] - last[2] + np.pi) % (2.0 * np.pi) - np.pi)
    return (np.hypot(pose[0] - last[0], pose[1] - last[1]) >= min_motion_m
            or np.degrees(turn) >= min_turn_deg)


def record(args, ros_args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    class Recorder(Node):
        def __init__(self):
            super().__init__('depth_accuracy_recorder')
            self.pairer = FramePairer(args.period_s * 1e9, bool(args.estimate_topic))
            self.k = None
            self.saved = 0
            self.pose = self.saved_pose = None
            if args.odom_topic:
                from nav_msgs.msg import Odometry
                self.create_subscription(Odometry, args.odom_topic, self.odom, 10)
            self.create_subscription(Image, args.color_topic, lambda m: self.add('color', m),
                                     qos_profile_sensor_data)
            self.create_subscription(Image, args.depth_topic, lambda m: self.add('depth', m),
                                     qos_profile_sensor_data)
            if args.estimate_topic:
                self.create_subscription(Image, args.estimate_topic,
                                         lambda m: self.add('estimate', m), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, args.camera_info_topic, self.info,
                                     qos_profile_sensor_data)
            self.get_logger().info(
                f'recording {args.color_topic} + {args.depth_topic}'
                f'{" + " + args.estimate_topic if args.estimate_topic else ""} every '
                f'{args.period_s:g} s into {output}')

        def info(self, msg):
            self.k = np.array(msg.k, dtype=np.float64)

        def odom(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose = (p.x, p.y, float(yaw))

        def add(self, kind, msg):
            if self.saved >= args.max_frames:
                return
            if args.odom_topic and (self.pose is None or not moved_enough(
                    self.saved_pose, self.pose, args.min_motion_m, args.min_turn_deg)):
                return
            entry = self.pairer.add(kind, stamp_ns(msg), msg)
            if entry is None:
                return
            self.saved_pose = self.pose
            color, encoding = image_array(entry['color'])
            if encoding != 'bgr8':
                self.get_logger().warn(f'colour encoding {encoding} is not bgr8; skipped')
                return
            depth = md.depth_image_to_m(*image_array(entry['depth']))
            arrays = {'color': color, 'depth': depth, 'stamp': np.float64(stamp_ns(msg) / 1e9)}
            if self.k is not None:
                arrays['k'] = self.k
            if 'estimate' in entry:
                arrays['estimate'] = md.depth_image_to_m(*image_array(entry['estimate']))
            self.saved += 1
            np.savez_compressed(output / f'frame_{self.saved:05d}.npz', **arrays)
            if self.saved % 10 == 0 or self.saved == args.max_frames:
                self.get_logger().info(f'{self.saved} frames saved')

    rclpy.init(args=ros_args)
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def parse_args(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args = []
    if '--ros-args' in argv:
        index = argv.index('--ros-args')
        argv, ros_args = argv[:index], argv[index:]
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)

    rec = sub.add_parser('record', help='save colour + depth frames from ROS topics')
    rec.add_argument('--output-dir', required=True)
    rec.add_argument('--color-topic', default='/camera/color/image_raw')
    rec.add_argument('--depth-topic', default='/camera/depth/image_rect_raw',
                     help='metric depth aligned with the colour image (32FC1 m or 16UC1 mm)')
    rec.add_argument('--camera-info-topic', default='/camera/color/camera_info')
    rec.add_argument('--estimate-topic', default='',
                     help='also keep the running depth_node output (e.g. /depth_anything/depth/image)')
    rec.add_argument('--period-s', type=float, default=1.0, help='message time between frames')
    rec.add_argument('--odom-topic', default='', help='save only frames after the car moved')
    rec.add_argument('--min-motion-m', type=float, default=0.2)
    rec.add_argument('--min-turn-deg', type=float, default=10.0)
    rec.add_argument('--max-frames', type=int, default=400)

    ev = sub.add_parser('evaluate', help='score Depth Anything models on saved frames')
    ev.add_argument('--frames', nargs='+', required=True, help='directories of frame_*.npz')
    ev.add_argument('--model', type=parse_model, action='append', default=[],
                    help='dataset:encoder[:max_depth], e.g. vkitti:vits:80, hypersim:vits:20')
    ev.add_argument('--band', type=float, nargs=2, default=(0.5, 5.0), metavar=('MIN', 'MAX'),
                    help='reference depths scored (m)')
    ev.add_argument('--eval-stride', type=int, default=2, help='score every Nth row and column')
    ev.add_argument('--align-stride', type=int, default=8,
                    help='alignment grid: every Nth row and column of the reference')
    ev.add_argument('--align-range', type=float, nargs=2, default=(0.3, 4.0), metavar=('MIN', 'MAX'),
                    help='reference depths used for the alignment scale (m)')
    ev.add_argument('--align-min-samples', type=int, default=100)
    ev.add_argument('--min-depth', type=float, default=0.05, help="depth_node's min_depth")
    ev.add_argument('--camera-height-m', type=float, default=None,
                    help='level camera height above a flat floor: score the nvblox slice band')
    ev.add_argument('--slice-band', type=float, nargs=2, default=(0.02, 0.55), metavar=('MIN', 'MAX'),
                    help="heights that become obstacles (nvblox.yaml's esdf_slice_min/max_height)")
    ev.add_argument('--slice-tolerance-m', type=float, default=0.15)
    ev.add_argument('--input-size', type=int, default=518, help='infer_image input size')
    ev.add_argument('--warmup', type=int, default=5)
    ev.add_argument('--device', default='cuda', choices=('cuda', 'cpu'))
    ev.add_argument('--output', default='')
    return parser.parse_args(argv), ros_args


def main(argv=None):
    args, ros_args = parse_args(argv)
    if args.command == 'record':
        return record(args, ros_args)
    return evaluate(args)


if __name__ == '__main__':
    sys.exit(main())
