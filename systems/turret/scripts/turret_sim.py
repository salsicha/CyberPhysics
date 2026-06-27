#!/usr/bin/env python3
"""Headless deterministic simulation for a PanTiltROS-compatible turret."""

import argparse
import json
import math
from collections import deque
from pathlib import Path


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def percentile(values, pct):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * pct / 100.0
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def lerp(a, b, ratio):
    return a + (b - a) * ratio


def object_pose(obj, t):
    points = obj['trajectory']
    if t <= points[0]['t']:
        p = points[0]
        return p['azimuth_deg'], p['elevation_deg'], p['distance_m']
    for first, second in zip(points, points[1:]):
        if first['t'] <= t <= second['t']:
            ratio = (t - first['t']) / max(1e-9, second['t'] - first['t'])
            return (
                lerp(first['azimuth_deg'], second['azimuth_deg'], ratio),
                lerp(first['elevation_deg'], second['elevation_deg'], ratio),
                lerp(first['distance_m'], second['distance_m'], ratio),
            )
    p = points[-1]
    return p['azimuth_deg'], p['elevation_deg'], p['distance_m']


def occlusion_fraction(obj, t):
    for interval in obj.get('occlusion_intervals_s', []):
        if float(interval['start']) <= t <= float(interval['end']):
            return float(interval.get('fraction', 1.0))
    return 0.0


class TurretSim:
    def __init__(self, scenario, variant_name='nominal'):
        self.scenario = scenario
        self.variant_name = variant_name
        self.variant = scenario.get('variants', {}).get(variant_name, {})
        if scenario.get('variants') and variant_name not in scenario['variants']:
            raise SystemExit(f'unknown turret scenario variant: {variant_name}')
        self.camera = scenario['camera']
        self.pantilt = scenario['pantilt']
        self.target_id = scenario['target']['id']
        self.width = int(self.camera['width'])
        self.height = int(self.camera['height'])
        self.cx = self.width * 0.5
        self.cy = self.height * 0.5
        self.hfov = math.radians(float(self.camera['horizontal_fov_deg']))
        self.vfov = math.radians(float(self.camera['vertical_fov_deg']))
        self.fx = self.cx / math.tan(self.hfov * 0.5)
        self.fy = self.cy / math.tan(self.vfov * 0.5)
        self.pan = 0.0
        self.tilt = 0.0
        self.pan_cmd = 0.0
        self.tilt_cmd = 0.0
        self.last_error = None
        self.last_seen_error = (0.0, 0.0)
        self.last_seen_time = -1e9
        self.false_lock_count = 0
        self.command_saturation_count = 0
        self.joint_limit_violations = 0
        self.emergency_stop_violations = 0
        self.reacquire_times = []
        self.occlusion_start = None
        self.was_target_visible = False
        self.samples = []
        self.command_queue = deque()

    def _in_intervals(self, intervals, t):
        return any(float(interval['start']) <= t <= float(interval['end']) for interval in intervals)

    def in_emergency_stop(self, t):
        intervals = list(self.pantilt.get('emergency_stop_intervals_s', []))
        intervals.extend(self.variant.get('emergency_stop_intervals_s', []))
        return self._in_intervals(intervals, t)

    def camera_frame_dropped(self, t):
        return self._in_intervals(self.variant.get('camera_frame_drop_intervals_s', []), t)

    def command_dropped(self, t):
        return self._in_intervals(self.variant.get('command_drop_intervals_s', []), t)

    def project(self, obj, t):
        az_deg, el_deg, distance = object_pose(obj, t)
        az = math.radians(az_deg)
        el = math.radians(el_deg)
        err_pan = wrap_angle(az - self.pan)
        err_tilt = el - self.tilt
        if abs(err_pan) > self.hfov * 0.58 or abs(err_tilt) > self.vfov * 0.58:
            return None
        x = self.cx + self.fx * math.tan(err_pan)
        y = self.cy - self.fy * math.tan(err_tilt)
        base_w, base_h = obj['size_px_at_5m']
        scale = 5.0 / max(0.5, distance)
        width = max(8.0, float(base_w) * scale)
        height = max(8.0, float(base_h) * scale)
        if x + width * 0.5 < 0 or x - width * 0.5 > self.width:
            return None
        if y + height * 0.5 < 0 or y - height * 0.5 > self.height:
            return None
        return x, y, width, height, err_pan, err_tilt, distance

    def detect(self, obj, t):
        projected = self.project(obj, t)
        if projected is None:
            return None
        x, y, width, height, err_pan, err_tilt, distance = projected
        occlusion = occlusion_fraction(obj, t)
        if occlusion >= 0.92:
            return None
        area = width * height * (1.0 - 0.55 * occlusion)
        motion = abs(err_pan) + abs(err_tilt)
        confidence = clamp(
            0.94 - 0.30 * occlusion - 0.12 * motion
            - float(self.variant.get('segmentation_confidence_penalty', 0.0)),
            0.0,
            0.99)
        if confidence < float(self.scenario['yolo']['min_confidence']):
            return None
        iou = clamp(
            0.87 - 0.22 * occlusion - 0.04 * distance / 10.0
            - float(self.variant.get('mask_iou_penalty', 0.0)),
            0.35,
            0.95)
        return {
            'id': obj['id'],
            'class': obj['class'],
            'confidence': confidence,
            'bbox_xywh': [x, y, width, height],
            'centroid_px': [x, y],
            'mask_area_px': area,
            'mask_iou_vs_sim': iou,
            'occlusion_fraction': occlusion,
            'angular_error_rad': [err_pan, err_tilt],
        }

    def tracker_error(self, detections, t):
        target = next((d for d in detections if d['id'] == self.target_id), None)
        if target is not None:
            px, py = target['centroid_px']
            angular_x = math.atan2(px - self.cx, self.fx)
            angular_y = -math.atan2(py - self.cy, self.fy)
            self.last_seen_error = (angular_x, angular_y)
            self.last_seen_time = t
            if self.occlusion_start is not None:
                self.reacquire_times.append(t - self.occlusion_start)
                self.occlusion_start = None
            return target, angular_x, angular_y, False
        if t - self.last_seen_time <= float(self.scenario['scoring']['reacquire_timeout_s']):
            angular_x, angular_y = self.last_seen_error
            decay = max(0.0, 1.0 - (t - self.last_seen_time) / float(self.scenario['scoring']['reacquire_timeout_s']))
            return None, angular_x * decay, angular_y * decay, True
        return None, 0.0, 0.0, True

    def update_control(self, angular_x, angular_y, t, dt, emergency_stop):
        if emergency_stop or self.command_dropped(t):
            return
        if self.last_error is None:
            derivative_x = 0.0
            derivative_y = 0.0
        else:
            last_x, last_y = self.last_error
            derivative_x = (angular_x - last_x) / max(1e-6, dt)
            derivative_y = (angular_y - last_y) / max(1e-6, dt)
        self.last_error = (angular_x, angular_y)
        kp = 0.82
        kd = 0.08
        command_pan = self.pan + kp * angular_x + kd * derivative_x * dt
        command_tilt = self.tilt + kp * angular_y + kd * derivative_y * dt
        pan_min, pan_max = self.pantilt['pan_limit_rad']
        tilt_min, tilt_max = self.pantilt['tilt_limit_rad']
        saturated = (
            command_pan < pan_min or command_pan > pan_max or
            command_tilt < tilt_min or command_tilt > tilt_max)
        if saturated:
            self.command_saturation_count += 1
        command_pan = clamp(command_pan, pan_min, pan_max)
        command_tilt = clamp(command_tilt, tilt_min, tilt_max)
        self.command_queue.append((t + float(self.pantilt['command_latency_s']), command_pan, command_tilt))

    def update_servo(self, t, dt, emergency_stop):
        while self.command_queue and self.command_queue[0][0] <= t:
            _, self.pan_cmd, self.tilt_cmd = self.command_queue.popleft()
        if emergency_stop:
            self.pan_cmd = self.pan
            self.tilt_cmd = self.tilt
        max_pan_vel, max_tilt_vel = self.pantilt['max_velocity_rad_s']
        quant = float(self.pantilt['quantization_rad'])
        backlash = float(self.pantilt['backlash_rad'])
        deadband = float(self.pantilt['deadband_rad'])
        for axis in ('pan', 'tilt'):
            current = getattr(self, axis)
            command = getattr(self, f'{axis}_cmd')
            max_vel = max_pan_vel if axis == 'pan' else max_tilt_vel
            delta = command - current
            if abs(delta) < deadband:
                continue
            step = clamp(delta, -max_vel * dt, max_vel * dt)
            if abs(step) > backlash:
                step -= math.copysign(backlash * 0.2, step)
            current += step
            current = round(current / quant) * quant
            limits = self.pantilt[f'{axis}_limit_rad']
            if current < limits[0] - 1e-9 or current > limits[1] + 1e-9:
                self.joint_limit_violations += 1
            setattr(self, axis, clamp(current, limits[0], limits[1]))

    def run(self):
        duration = float(self.scenario['duration_s'])
        dt = float(self.scenario['dt_s'])
        steps = int(duration / dt) + 1
        for frame_index in range(steps):
            t = round(frame_index * dt, 6)
            emergency_stop = self.in_emergency_stop(t)
            self.update_servo(t, dt, emergency_stop)
            camera_dropped = self.camera_frame_dropped(t)
            detections = [] if camera_dropped else [
                d for d in (self.detect(obj, t) for obj in self.scenario['objects']) if d
            ]
            target_detection, angular_x, angular_y, predicted = self.tracker_error(detections, t)
            self.update_control(angular_x, angular_y, t, dt, emergency_stop)
            target_visible = target_detection is not None
            if not target_visible and self.was_target_visible and self.occlusion_start is None:
                self.occlusion_start = t
            self.was_target_visible = target_visible
            pixel_error = math.hypot(math.tan(angular_x) * self.fx, math.tan(angular_y) * self.fy)
            angular_error = math.hypot(angular_x, angular_y)
            locked = target_visible and pixel_error <= float(self.scenario['scoring']['lock_center_tolerance_px'])
            selected_id = self.target_id if (target_visible or predicted) else ''
            latency = (
                12.0 + 3.6 * len(detections) + 1.5 * abs(math.sin(t * 1.7))
                + float(self.variant.get('yolo_latency_penalty_ms', 0.0))
            )
            target_mask = None
            target_bbox = None
            target_confidence = 0.0
            if target_detection is not None:
                target_mask = {
                    'id': self.target_id,
                    'area_px': target_detection['mask_area_px'],
                    'iou_vs_sim': target_detection['mask_iou_vs_sim'],
                }
                target_bbox = target_detection['bbox_xywh']
                target_confidence = target_detection['confidence']
            sample = {
                'frame_index': frame_index,
                't_s': t,
                'topics': {
                    '/turret/camera/image_raw': {
                        'width': self.width,
                        'height': self.height,
                        'encoding': 'rgb8',
                        'frame_id': self.camera['frame_id'],
                        'dropped': camera_dropped,
                    },
                    '/turret/camera/camera_info': {
                        'width': self.width,
                        'height': self.height,
                        'k': [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0],
                    },
                    '/turret/yolo/segments': detections,
                    '/turret/yolo/target_mask': target_mask,
                    '/turret/yolo/target_bbox': target_bbox,
                    '/turret/yolo/target_confidence': target_confidence,
                    '/turret/target/pixel_error': [math.tan(angular_x) * self.fx, math.tan(angular_y) * self.fy],
                    '/turret/target/angular_error': [angular_x, angular_y],
                    '/turret/target/selected_id': selected_id,
                    '/turret/target/locked': locked,
                    '/turret/joint_commands': {
                        'name': self.pantilt['joint_names'],
                        'position': [self.pan_cmd, self.tilt_cmd],
                    },
                    '/turret/joint_states': {
                        'name': self.pantilt['joint_names'],
                        'position': [self.pan, self.tilt],
                    },
                    '/turret/diagnostics': {
                        'yolo_latency_ms': latency,
                        'backend': self.scenario['yolo']['cpu_smoke_backend'],
                        'servo_temperature_c': [38.0, 36.0],
                    },
                    '/turret/emergency_stop': emergency_stop,
                },
                'derived': {
                    'target_visible': target_visible,
                    'target_predicted': predicted,
                    'camera_frame_dropped': camera_dropped,
                    'pixel_error_norm_px': pixel_error,
                    'angular_error_norm_rad': angular_error,
                    'target_iou': 0.0 if target_detection is None else target_detection['mask_iou_vs_sim'],
                    'target_confidence': target_confidence,
                    'yolo_latency_ms': latency,
                },
            }
            self.samples.append(sample)
        return self.report()

    def report(self):
        ignore_initial = float(self.scenario['scoring']['ignore_initial_s'])
        scored = [s for s in self.samples if s['t_s'] >= ignore_initial]
        visible = [s for s in scored if s['derived']['target_visible']]
        locked = [s for s in scored if s['topics']['/turret/target/locked']]
        detections = [s for s in scored if s['derived']['target_confidence'] > 0.0]
        pixel_errors = [s['derived']['pixel_error_norm_px'] for s in visible]
        angular_errors_deg = [math.degrees(s['derived']['angular_error_norm_rad']) for s in visible]
        ious = [s['derived']['target_iou'] for s in detections]
        latencies = [s['derived']['yolo_latency_ms'] for s in scored]
        summary = {
            'sample_count': len(self.samples),
            'scored_sample_count': len(scored),
            'target_visible_sample_count': len(visible),
            'lock_fraction': len(locked) / max(1, len(visible)),
            'target_detection_fraction': len(detections) / max(1, len(scored)),
            'mean_pixel_error_px': sum(pixel_errors) / max(1, len(pixel_errors)),
            'p95_pixel_error_px': percentile(pixel_errors, 95),
            'mean_angular_error_deg': sum(angular_errors_deg) / max(1, len(angular_errors_deg)),
            'p95_angular_error_deg': percentile(angular_errors_deg, 95),
            'target_mean_iou': sum(ious) / max(1, len(ious)),
            'target_min_iou': min(ious) if ious else 0.0,
            'false_lock_count': self.false_lock_count,
            'max_reacquire_time_s': max(self.reacquire_times) if self.reacquire_times else 0.0,
            'command_saturation_fraction': self.command_saturation_count / max(1, len(scored)),
            'joint_limit_violation_count': self.joint_limit_violations,
            'emergency_stop_violation_count': self.emergency_stop_violations,
            'max_yolo_latency_ms': max(latencies) if latencies else 0.0,
            'topic_contract': self.scenario['topics'],
            'yolo_contract': self.scenario['yolo'],
        }
        return {
            'scenario': self.scenario['name'],
            'variant': self.variant_name,
            'target_id': self.target_id,
            'summary': summary,
            'samples': self.samples,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='systems/turret/scenarios/warehouse_tracking.json')
    parser.add_argument('--output', default='generated/turret/turret_tracking_metrics.json')
    parser.add_argument('--variant', default='nominal')
    args = parser.parse_args()

    scenario = json.loads(Path(args.scenario).read_text())
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = TurretSim(scenario, variant_name=args.variant).run()
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report['summary'], indent=2))


if __name__ == '__main__':
    main()
