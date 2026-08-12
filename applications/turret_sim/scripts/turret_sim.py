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
    def __init__(
        self,
        scenario,
        variant_name='nominal',
    ):
        self.scenario = scenario
        self.variant_name = variant_name
        self.variant = scenario.get('variants', {}).get(variant_name, {})
        if scenario.get('variants') and variant_name not in scenario['variants']:
            raise SystemExit(f'unknown turret scenario variant: {variant_name}')
        self.camera = scenario['camera']
        self.pantilt = scenario['pantilt']
        self.target_id = scenario['target']['id']
        self.target_object = next(
            obj for obj in self.scenario['objects'] if obj['id'] == self.target_id
        )
        initial_selection = scenario['target'].get('initial_selection_px')
        self.target_centroid = (
            tuple(float(value) for value in initial_selection)
            if initial_selection is not None
            else None
        )
        self.association_gate_px = float(
            scenario['target'].get(
                'association_gate_px',
                max(int(self.camera['width']), int(self.camera['height'])) * 0.2,
            )
        )
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
        self.was_target_detected = False
        self.samples = []
        self.command_queue = deque()
        self.confidence = float(scenario['yolo']['min_confidence'])
        if not 0.0 <= self.confidence <= 1.0:
            raise SystemExit('synthetic confidence must be between 0 and 1')
        self.cv2 = None
        self.np = None
        self.background = None
        self.backend_name = self.scenario['yolo']['cpu_smoke_backend']

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

    def detect_synthetic(self, obj, t):
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
        if confidence < self.confidence:
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

    def _render_background(self):
        np = self.np
        cv2 = self.cv2
        rng = np.random.default_rng(int(self.scenario.get('seed', 0)))
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        top = np.array([92, 82, 70], dtype=np.float32)
        bottom = np.array([48, 52, 58], dtype=np.float32)
        for y in range(self.height):
            ratio = y / max(1, self.height - 1)
            frame[y, :, :] = top * (1.0 - ratio) + bottom * ratio
        noise = rng.normal(0.0, 7.0, frame.shape).astype(np.int16)
        frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        horizon = int(self.height * 0.58)
        cv2.line(frame, (0, horizon), (self.width, horizon), (125, 118, 105), 3)
        for x in range(-80, self.width + 100, 120):
            cv2.line(frame, (self.width // 2, horizon), (x, self.height), (72, 76, 82), 2)
        for side_x in (25, self.width - 155):
            cv2.rectangle(frame, (side_x, 45), (side_x + 130, horizon), (56, 62, 66), -1)
            for shelf_y in range(85, horizon, 62):
                cv2.line(frame, (side_x, shelf_y), (side_x + 130, shelf_y), (155, 145, 126), 4)
                for bin_x in range(side_x + 8, side_x + 118, 36):
                    color = tuple(int(value) for value in rng.integers(55, 190, size=3))
                    cv2.rectangle(frame, (bin_x, shelf_y - 28), (bin_x + 26, shelf_y - 5), color, -1)
        cv2.rectangle(frame, (self.width // 2 - 70, 55), (self.width // 2 + 80, 145), (160, 175, 184), -1)
        cv2.rectangle(frame, (self.width // 2 - 60, 65), (self.width // 2 + 70, 135), (210, 220, 224), -1)
        return frame

    @staticmethod
    def _clipped_box(projected, width, height):
        x, y, box_w, box_h = projected[:4]
        x1 = max(0, int(round(x - box_w * 0.5)))
        y1 = max(0, int(round(y - box_h * 0.5)))
        x2 = min(width - 1, int(round(x + box_w * 0.5)))
        y2 = min(height - 1, int(round(y + box_h * 0.5)))
        if x2 <= x1 or y2 <= y1:
            return None
        return x1, y1, x2, y2

    def _draw_object(self, frame, mask, obj, box):
        cv2 = self.cv2
        x1, y1, x2, y2 = box
        width = x2 - x1
        height = y2 - y1
        color = tuple(int(channel) for channel in reversed(obj['color']))
        object_class = obj['class']

        if object_class == 'cart':
            rim = max(3, int(min(width, height) * 0.075))
            left_post = x1 + int(width * 0.12)
            right_post = x2 - int(width * 0.10)
            cart_top = y1 + int(height * 0.18)
            wheel_y = y2 - rim
            post_thickness = max(3, int(width * 0.045))
            cv2.line(
                frame, (left_post, cart_top), (left_post, wheel_y), color, post_thickness
            )
            cv2.line(
                frame, (right_post, cart_top), (right_post, wheel_y), color, post_thickness
            )
            cv2.line(
                mask, (left_post, cart_top), (left_post, wheel_y), 255, post_thickness
            )
            cv2.line(
                mask, (right_post, cart_top), (right_post, wheel_y), 255, post_thickness
            )
            shelf_height = max(5, int(height * 0.12))
            for shelf_ratio in (0.24, 0.50, 0.76):
                shelf_y = y1 + int(height * shelf_ratio)
                cv2.rectangle(
                    frame,
                    (left_post, shelf_y),
                    (right_post, min(wheel_y, shelf_y + shelf_height)),
                    color,
                    -1,
                )
                cv2.rectangle(
                    mask,
                    (left_post, shelf_y),
                    (right_post, min(wheel_y, shelf_y + shelf_height)),
                    255,
                    -1,
                )
                cv2.line(
                    frame,
                    (left_post, shelf_y),
                    (right_post, shelf_y),
                    (225, 225, 225),
                    max(1, shelf_height // 5),
                )
            for wheel_x in (left_post, right_post):
                cv2.circle(frame, (wheel_x, wheel_y), rim, (22, 22, 22), -1)
                cv2.circle(mask, (wheel_x, wheel_y), rim, 255, -1)
            handle_x = x1 + int(width * 0.02)
            cv2.line(frame, (left_post, cart_top), (handle_x, y1), color, post_thickness)
            cv2.line(mask, (left_post, cart_top), (handle_x, y1), 255, post_thickness)
            cv2.line(
                frame, (handle_x, y1), (x1 + int(width * 0.28), y1), color, post_thickness
            )
            cv2.line(
                mask, (handle_x, y1), (x1 + int(width * 0.28), y1), 255, post_thickness
            )
        elif object_class == 'person':
            head_radius = max(3, int(width * 0.24))
            center_x = (x1 + x2) // 2
            head_y = y1 + head_radius
            cv2.circle(frame, (center_x, head_y), head_radius, color, -1)
            cv2.circle(mask, (center_x, head_y), head_radius, 255, -1)
            cv2.ellipse(
                frame,
                (center_x, y1 + int(height * 0.62)),
                (max(3, width // 2), max(4, int(height * 0.34))),
                0,
                0,
                360,
                color,
                -1,
            )
            cv2.ellipse(
                mask,
                (center_x, y1 + int(height * 0.62)),
                (max(3, width // 2), max(4, int(height * 0.34))),
                0,
                0,
                360,
                255,
                -1,
            )
        elif object_class == 'drone':
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            thickness = max(2, height // 7)
            cv2.line(frame, (x1, y1), (x2, y2), color, thickness)
            cv2.line(frame, (x1, y2), (x2, y1), color, thickness)
            cv2.line(mask, (x1, y1), (x2, y2), 255, thickness)
            cv2.line(mask, (x1, y2), (x2, y1), 255, thickness)
            cv2.circle(frame, center, max(3, height // 4), color, -1)
            cv2.circle(mask, center, max(3, height // 4), 255, -1)
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, -1)
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
            cv2.line(frame, (x1, y1), (x2, y2), (225, 225, 225), 2)
            cv2.line(frame, (x2, y1), (x1, y2), (225, 225, 225), 2)

    def render_frame(self, t):
        np = self.np
        cv2 = self.cv2
        frame = self.background.copy()
        projected_objects = []
        for obj in self.scenario['objects']:
            projected = self.project(obj, t)
            if projected is not None:
                projected_objects.append((projected[6], obj, projected))
        projected_objects.sort(reverse=True, key=lambda item: item[0])

        ground_truth_masks = {}
        for _, obj, projected in projected_objects:
            box = self._clipped_box(projected, self.width, self.height)
            if box is None:
                continue
            mask = np.zeros((self.height, self.width), dtype=np.uint8)
            self._draw_object(frame, mask, obj, box)
            for existing_mask in ground_truth_masks.values():
                existing_mask[mask > 0] = 0
            ground_truth_masks[obj['id']] = mask

        target_mask = ground_truth_masks.get(self.target_id)
        occlusion = next(
            (occlusion_fraction(obj, t) for obj in self.scenario['objects'] if obj['id'] == self.target_id),
            0.0,
        )
        if target_mask is not None and occlusion > 0.0:
            ys, xs = np.nonzero(target_mask)
            if len(xs):
                x1, x2 = int(xs.min()), int(xs.max())
                y1, y2 = int(ys.min()), int(ys.max())
                cover_width = int((x2 - x1 + 1) * min(1.0, occlusion))
                cover_start = x2 - cover_width + 1
                frame[y1:y2 + 1, cover_start:x2 + 1] = self.background[
                    y1:y2 + 1, cover_start:x2 + 1
                ]
                target_mask[y1:y2 + 1, cover_start:x2 + 1] = 0

        if self.variant_name == 'low_light':
            frame = np.clip(frame.astype(np.float32) * 0.45, 0, 255).astype(np.uint8)
        elif self.variant_name == 'glare_motion_blur':
            overlay = frame.copy()
            cv2.rectangle(overlay, (int(self.width * 0.62), 0), (int(self.width * 0.78), self.height), (255, 255, 245), -1)
            frame = cv2.addWeighted(frame, 0.72, overlay, 0.28, 0)
            frame = cv2.GaussianBlur(frame, (9, 3), 0)
        return frame, ground_truth_masks

    @staticmethod
    def _mask_iou(first, second):
        intersection = ((first > 0) & (second > 0)).sum()
        union = ((first > 0) | (second > 0)).sum()
        return float(intersection / union) if union else 0.0


    def tracker_error(self, detections, t):
        candidates = [d for d in detections if d['id'] == self.target_id]
        target = None
        if candidates and self.target_centroid is None:
            target = max(candidates, key=lambda detection: detection['confidence'])
        elif candidates:
            target = min(
                candidates,
                key=lambda detection: math.dist(
                    detection['centroid_px'], self.target_centroid
                ),
            )
            if math.dist(target['centroid_px'], self.target_centroid) > self.association_gate_px:
                target = None
        if target is not None:
            px, py = target['centroid_px']
            self.target_centroid = (px, py)
            angular_x = math.atan2(px - self.cx, self.fx)
            angular_y = -math.atan2(py - self.cy, self.fy)
            self.last_seen_error = (angular_x, angular_y)
            self.last_seen_time = t
            if self.occlusion_start is not None:
                self.reacquire_times.append(t - self.occlusion_start)
                self.occlusion_start = None
            return target, angular_x, angular_y, False
        if (
            self.last_seen_time > -1e8
            and t - self.last_seen_time
            <= float(self.scenario['scoring']['reacquire_timeout_s'])
        ):
            # The actuators are position controlled. Holding the current command
            # during a short detector dropout avoids repeatedly integrating a
            # stale image-plane error and sweeping past the selected target.
            return None, 0.0, 0.0, True
        return None, 0.0, 0.0, False

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

    def run(self, max_frames=None):
        duration = float(self.scenario['duration_s'])
        dt = float(self.scenario['dt_s'])
        steps = int(duration / dt) + 1
        if max_frames is not None:
            steps = min(steps, max(1, int(max_frames)))
        for frame_index in range(steps):
            t = round(frame_index * dt, 6)
            emergency_stop = self.in_emergency_stop(t)
            self.update_servo(t, dt, emergency_stop)
            camera_dropped = self.camera_frame_dropped(t)
            detections = [] if camera_dropped else [
                detection
                for detection in (
                    self.detect_synthetic(obj, t) for obj in self.scenario['objects']
                )
                if detection
            ]
            latency = (
                12.0 + 3.6 * len(detections) + 1.5 * abs(math.sin(t * 1.7))
                + float(self.variant.get('yolo_latency_penalty_ms', 0.0))
            )
            target_detection, angular_x, angular_y, predicted = self.tracker_error(detections, t)
            self.update_control(angular_x, angular_y, t, dt, emergency_stop)
            target_detected = target_detection is not None
            target_projection = self.project(self.target_object, t)
            target_visible = bool(
                not camera_dropped
                and target_projection is not None
                and occlusion_fraction(self.target_object, t) < 0.92
            )
            if not target_detected and self.was_target_detected and self.occlusion_start is None:
                self.occlusion_start = t
            self.was_target_detected = target_detected
            if target_projection is not None:
                ground_truth_angular_x = target_projection[4]
                ground_truth_angular_y = target_projection[5]
            else:
                ground_truth_angular_x = 0.0
                ground_truth_angular_y = 0.0
            pixel_error = math.hypot(
                math.tan(ground_truth_angular_x) * self.fx,
                math.tan(ground_truth_angular_y) * self.fy,
            )
            angular_error = math.hypot(
                ground_truth_angular_x,
                ground_truth_angular_y,
            )
            locked = (
                target_detected
                and target_visible
                and pixel_error
                <= float(self.scenario['scoring']['lock_center_tolerance_px'])
            )
            if (
                locked
                and target_detection['mask_iou_vs_sim'] <= 0.05
            ):
                self.false_lock_count += 1
            selected_id = self.target_id if (target_detected or predicted) else ''
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
                        'backend': self.backend_name,
                        'checkpoint': None,
                        'device': 'simulated',
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
        detections = [
            s
            for s in visible
            if s['derived']['target_confidence'] > 0.0
        ]
        pixel_errors = [s['derived']['pixel_error_norm_px'] for s in visible]
        angular_errors_deg = [math.degrees(s['derived']['angular_error_norm_rad']) for s in visible]
        ious = [s['derived']['target_iou'] for s in detections]
        latencies = [s['derived']['yolo_latency_ms'] for s in scored]
        summary = {
            'sample_count': len(self.samples),
            'scored_sample_count': len(scored),
            'target_visible_sample_count': len(visible),
            'lock_fraction': len(locked) / max(1, len(visible)),
            'target_detection_fraction': len(detections) / max(1, len(visible)),
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
            'detector': {
                'backend': self.backend_name,
                'model': None,
                'device': 'simulated',
                'image_size': None,
                'confidence': self.confidence,
                'prompts': None,
            },
            'summary': summary,
            'samples': self.samples,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='applications/turret_sim/scenarios/warehouse_tracking.json')
    parser.add_argument('--output', default='generated/turret/turret_tracking_metrics.json')
    parser.add_argument('--variant', default='nominal')
    parser.add_argument('--max-frames', type=int)
    args = parser.parse_args()

    scenario = json.loads(Path(args.scenario).read_text())
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = TurretSim(scenario, variant_name=args.variant).run(max_frames=args.max_frames)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report['summary'], indent=2))


if __name__ == '__main__':
    main()
