"""PanTiltROS tracker control loop: latency stability, e-stop and readiness gating.

The controller class is pure Python; the node glue is driven with ROS and
Ultralytics stubbed out. The closed-loop checks run the tracker's controller
against the turret_sim actuator plant and synthetic detector with 1-4 frames
of image-to-command latency.
"""
import json
import math
import sys
from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from conftest import REPO, install_ros_stubs, load_script

SCENARIO = json.loads(
    (REPO / 'applications/turret_sim/scenarios/warehouse_tracking.json').read_text())
THRESHOLDS = json.loads(
    (REPO / 'applications/turret_sim/validation/acceptance_thresholds.json').read_text())


@pytest.fixture(scope='module')
def tracker():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('diagnostic_msgs', 'diagnostic_msgs.msg', 'rclpy.executors', 'ultralytics'):
            sys.modules[name] = MagicMock()
        module = load_script(REPO / 'applications/pantiltros/scripts/pantilt_yolo_tracker.py',
                             'pantilt_yolo_tracker_review')
        module.JointState = lambda: NS(header=NS(stamp=None), name=[], position=[])
        yield module


@pytest.fixture(scope='module')
def turret_sim():
    sys.path.insert(0, str(REPO / 'applications/turret_sim/scripts'))
    try:
        yield load_script(REPO / 'applications/turret_sim/scripts/turret_sim.py',
                          'turret_sim_review')
    finally:
        sys.path.remove(str(REPO / 'applications/turret_sim/scripts'))


def controller(tracker, **kwargs):
    pantilt = SCENARIO['pantilt']
    options = dict(pan_limits=pantilt['pan_limit_rad'], tilt_limits=pantilt['tilt_limit_rad'],
                   actuator_latency_s=0.05)
    options.update(kwargs)
    return tracker.PanTiltController(**options)


@pytest.mark.parametrize('head,expected', [
    (NS(reprta=object(), is_fused=False), True),
    (NS(reprta=object(), is_fused=True), False),
    (NS(reprta=object(), is_fused=False, lrpc=object()), False),
    (NS(), False),
])
def test_checkpoint_head_controls_prompting_not_its_filename(tracker, head, expected):
    model = NS(model=NS(model=[head]), set_classes=MagicMock())
    tracker.configure_model_classes(model, ['cart', 'person'])
    assert model.set_classes.called is expected
    if expected:
        model.set_classes.assert_called_once_with(['cart', 'person'])


def test_near_identical_boxes_do_not_create_competing_tracks(tracker):
    def box(x, confidence, category='cart'):
        return {'bbox_xyxy': [x, 0., x + 100., 80.], 'confidence': confidence,
                'class': category}
    best = box(0., .9)
    duplicate = box(.2, .8)
    overlapping_instance = box(20., .7)
    different_class = box(0., .6, 'box')
    assert tracker.suppress_duplicate_detections([
        duplicate, best, overlapping_instance, different_class,
    ]) == [best, overlapping_instance, different_class]


def test_previous_gain_is_unstable_with_two_frames_of_latency():
    # c[k] = c[k-1] + kp * e[k - n]: characteristic z^(n+1) - z^n + kp.
    def worst_pole(kp, n):
        return max(abs(np.roots([1, -1] + [0] * (n - 1) + [kp])))
    assert worst_pole(0.82, 1) < 1.0
    assert worst_pole(0.82, 2) > 1.08 and worst_pole(0.82, 3) > 1.13
    # The documented fallback bound: the default gain survives three frames
    # even if latency compensation is lost entirely.
    assert all(worst_pole(0.4, n) < 1.0 for n in (1, 2, 3))


def simulate_axis(tracker, total_frames, feedback, assumed_latency_s=None, fps=20.0,
                  duration=9.0, vmax=2.6):
    """One axis at 1 ms resolution: rate-limited servo, latency split between
    inference and actuation. Returns (time, position, target) samples."""
    period = 1.0 / fps
    inference = actuation = total_frames * period / 2
    ctrl = controller(tracker, actuator_latency_s=(
        actuation if assumed_latency_s is None else assumed_latency_s))

    def target(t):
        return (0.3 if t >= 0.5 else 0.0) + 0.2 * max(0.0, min(t, 8.0) - 4.0)

    position = servo_command = 0.0
    frames, commands, log = deque(), deque(), []
    t, next_frame, dt = 0.0, 0.0, 0.001
    while t < duration:
        if t >= next_frame - 1e-12:
            frames.append((t + inference, t, target(t) - position))
            next_frame += period
        while frames and frames[0][0] <= t + 1e-12:
            _, captured, error = frames.popleft()
            pan, _ = ctrl.update(captured, error, 0.0, t)
            commands.append((t + actuation, pan))
        while commands and commands[0][0] <= t + 1e-12:
            servo_command = commands.popleft()[1]
        position += max(-vmax * dt, min(vmax * dt, servo_command - position))
        if feedback and round(t / dt) % 10 == 0:
            ctrl.add_measured(t, position, 0.0)
        log.append((t, position, target(t)))
        t += dt
    return log


@pytest.mark.parametrize('total_frames', [1, 2, 3])
@pytest.mark.parametrize('mode', ['measured', 'commanded', 'latency_x2', 'latency_zero'])
def test_loop_settles_with_one_to_three_frames_of_latency(tracker, total_frames, mode):
    actuation = total_frames / 20.0 / 2
    assumed = {'latency_x2': 2 * actuation, 'latency_zero': 0.0}.get(mode)
    log = simulate_axis(tracker, total_frames, feedback=(mode == 'measured'),
                        assumed_latency_s=assumed)
    overshoot = max(p for t, p, _ in log if t < 4.0) - 0.3
    settled = max(abs(r - p) for t, p, r in log if 2.0 <= t < 4.0)
    ramp = max(abs(r - p) for t, p, r in log if 6.0 <= t < 8.0)
    final = max(abs(r - p) for t, p, r in log if 8.5 <= t)
    assert overshoot < 0.15
    assert settled < 0.03 and final < 0.015
    assert ramp < 0.07


def run_plant(tracker, turret_sim, latency_frames, law='new', feedback=False, variant='nominal',
              lead=False):
    """Closed loop against the turret_sim plant (velocity limits, quantization,
    backlash, command latency) and synthetic detector. Detections reach the
    controller `latency_frames` frames after capture. With `lead` the target's
    bearing rate is fed forward as the node does (it needs measured feedback)."""
    sim = turret_sim.TurretSim(SCENARIO, variant_name=variant)
    pantilt = SCENARIO['pantilt']
    dt = float(SCENARIO['dt_s'])
    ctrl = controller(tracker)
    bearing = tracker.BearingRate()
    pending, errors = deque(), []
    last, previous_error = (0.0, 0.0), None
    for k in range(int(float(SCENARIO['duration_s']) / dt) + 1):
        t = round(k * dt, 6)
        estop = sim.in_emergency_stop(t)
        sim.update_servo(t, dt, estop)
        if feedback:
            ctrl.add_measured(t, sim.pan, sim.tilt)
        dropped = sim.camera_frame_dropped(t)
        detections = [] if dropped else [
            d for d in (sim.detect_synthetic(obj, t) for obj in SCENARIO['objects']) if d]
        target, ax, ay, _ = sim.tracker_error(detections, t)
        pending.append((k + latency_frames, t, target is not None, ax, ay))
        while pending and pending[0][0] <= k:
            _, captured, detected, ax, ay = pending.popleft()
            if not detected or estop or sim.command_dropped(t):
                continue
            if law == 'new':
                rate = None
                if lead and feedback:
                    bearing.add(captured, ctrl.sight(captured, ax, ay, t))
                    rate = bearing.rate(t)
                command = ctrl.update(captured, ax, ay, t, lead=rate)
            else:
                # Previous law on hardware: integrate the bridge's echoed command.
                d = (0.0, 0.0) if previous_error is None else (
                    ax - previous_error[0], ay - previous_error[1])
                previous_error = (ax, ay)
                command = tuple(
                    max(lo, min(hi, base + 0.82 * e + 0.08 * de))
                    for base, e, de, (lo, hi) in zip(
                        last, (ax, ay), d, (pantilt['pan_limit_rad'], pantilt['tilt_limit_rad'])))
            last = command
            sim.command_queue.append((t + float(pantilt['command_latency_s']), *command))
        projection = sim.project(sim.target_object, t)
        visible = (projection is not None and not dropped
                   and turret_sim.occlusion_fraction(sim.target_object, t) < 0.92)
        if t >= float(SCENARIO['scoring']['ignore_initial_s']) and visible:
            errors.append(math.degrees(math.hypot(projection[4], projection[5])))
    assert sim.joint_limit_violations == 0 and sim.emergency_stop_violations == 0
    return sum(errors) / len(errors), turret_sim.percentile(errors, 95)


@pytest.mark.parametrize('latency_frames', [0, 1, 2, 3])
@pytest.mark.parametrize('feedback', [False, True])
def test_tracker_meets_pointing_thresholds_on_sim_plant(tracker, turret_sim, latency_frames,
                                                        feedback):
    mean, p95 = run_plant(tracker, turret_sim, latency_frames, feedback=feedback)
    assert mean <= THRESHOLDS['max_mean_angular_error_deg']
    assert p95 <= THRESHOLDS['max_p95_angular_error_deg']


@pytest.mark.parametrize('latency_frames', [0, 1, 2, 3])
def test_target_lead_keeps_the_fixed_turret_within_thresholds(tracker, turret_sim,
                                                              latency_frames):
    plain = run_plant(tracker, turret_sim, latency_frames, feedback=True)
    lead = run_plant(tracker, turret_sim, latency_frames, feedback=True, lead=True)
    print(f'warehouse, {latency_frames} frames: mean/p95 {plain[0]:.2f}/{plain[1]:.2f} deg, '
          f'with lead {lead[0]:.2f}/{lead[1]:.2f} deg')
    assert lead[0] <= THRESHOLDS['max_mean_angular_error_deg']
    assert lead[1] <= THRESHOLDS['max_p95_angular_error_deg']
    assert lead[0] < plain[0]


@pytest.mark.parametrize('variant', sorted(SCENARIO['variants']))
def test_tracker_handles_sim_variants_with_latency(tracker, turret_sim, variant):
    mean, p95 = run_plant(tracker, turret_sim, 2, variant=variant)
    assert mean <= THRESHOLDS['max_mean_angular_error_deg']
    assert p95 <= THRESHOLDS['max_p95_angular_error_deg']


def test_previous_law_diverges_on_sim_plant(tracker, turret_sim):
    mean, _ = run_plant(tracker, turret_sim, 2, law='old')
    assert mean > 5 * THRESHOLDS['max_mean_angular_error_deg']


def test_controller_rate_limits_and_clamps(tracker):
    ctrl = controller(tracker, gain=1.0, max_rate_rad_s=1.0)
    pan, tilt = ctrl.update(0.0, 3.0, -3.0, 0.0)
    assert (pan, tilt) == pytest.approx((0.1, -0.1))  # first step: 0.1 s at 1 rad/s
    pan, tilt = ctrl.update(0.05, 3.0, -3.0, 0.05)
    assert pan == pytest.approx(0.15) and tilt == pytest.approx(-0.15)
    for k in range(2, 100):
        pan, tilt = ctrl.update(k * 0.05, 3.0, -3.0, k * 0.05)
    assert (pan, tilt) == pytest.approx((1.5708, -0.65))


def test_controller_rejects_non_finite_values(tracker):
    ctrl = controller(tracker)
    assert ctrl.update(0.0, math.nan, 0.0, 0.0) is None
    assert ctrl.update(math.inf, 0.0, 0.0, 0.0) is None
    assert ctrl.add_measured(0.0, math.nan, 0.0) is False
    assert not ctrl.has_feedback(0.0)
    assert ctrl.update(0.0, 0.1, 0.0, 0.0) == pytest.approx((0.04, 0.0))
    with pytest.raises(ValueError):
        tracker.clamp(math.nan, -1.0, 1.0)


def test_controller_uses_position_at_capture_time(tracker):
    ctrl = controller(tracker, gain=1.0, max_rate_rad_s=100.0)
    for k in range(11):
        ctrl.add_measured(k * 0.1, 0.05 * k, 0.0)
    # Frame captured at t=0.5 (pan 0.25) saw the target 0.1 rad to the right.
    assert ctrl.update(0.5, 0.1, 0.0, 1.0)[0] == pytest.approx(0.35)


def make_node(tracker, require_release=False):
    node = tracker.PanTiltYoloTracker.__new__(tracker.PanTiltYoloTracker)
    node.clock_s = 100.0
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(node.clock_s * 1e9),
                                               to_msg=lambda: None))
    node.get_logger = MagicMock()
    parameters = dict(tracker.DEFAULT_PARAMETERS, actuator_latency_s=0.05,
                      require_estop_release=require_release)
    node.get_parameter = lambda name: NS(value=parameters[name])
    node._configure()
    node.command_pub = MagicMock()
    return node


def joint_state(stamp_s, pan, tilt):
    return NS(name=['pan_joint', 'tilt_joint'], position=[pan, tilt],
              header=NS(stamp=NS(sec=int(stamp_s), nanosec=int(round((stamp_s % 1) * 1e9)))))


def test_tracker_holds_without_feedback_or_ready_actuator(tracker):
    node = make_node(tracker)
    node._publish_command(0.1, 0.0, 99.95)
    assert node.command_state == 'actuator_not_ready'
    node.command_pub.publish.assert_not_called()
    # A ready actuator allows commands; a stale readiness message does not.
    node._actuator_ready(NS(data=True))
    node._publish_command(0.1, 0.0, 99.95)
    assert node.command_state == 'commanded'
    assert node.command_pub.publish.call_count == 1
    node.clock_s += 1.5
    node._publish_command(0.1, 0.0, node.clock_s - 0.05)
    assert node.command_state == 'actuator_not_ready'
    assert node.command_pub.publish.call_count == 1


def test_tracker_uses_measured_feedback_and_ignores_nan_states(tracker):
    node = make_node(tracker)
    node._joint_state(joint_state(99.9, math.nan, 0.0))
    node._publish_command(0.1, 0.0, 99.95)
    assert node.command_state == 'actuator_not_ready'
    node._joint_state(joint_state(99.9, 0.2, 0.1))
    node._publish_command(0.1, 0.0, 99.95)
    assert node.command_state == 'measured'
    command = node.command_pub.publish.call_args[0][0]
    assert all(math.isfinite(value) for value in command.position)


def test_tracker_emergency_stop_blocks_commands(tracker):
    node = make_node(tracker, require_release=True)
    node._actuator_ready(NS(data=True))
    node._publish_command(0.1, 0.0, 99.95)
    assert node.command_state == 'emergency_stop'
    node._emergency_stop(NS(data=False))
    node._publish_command(0.1, 0.0, 99.95)
    assert node.command_state == 'commanded'
    node._emergency_stop(NS(data=True))
    node._publish_command(0.1, 0.0, 99.95)
    assert node.command_state == 'emergency_stop'
    assert node.command_pub.publish.call_count == 1


def test_hardware_config_is_conservative():
    import yaml
    config = yaml.safe_load(
        (REPO / 'applications/pantiltros/config/yolo_tracker_hardware.yaml').read_text())
    params = config['pantilt_yolo_tracker']['ros__parameters']
    assert params['confidence'] >= 0.2
    assert params['require_estop_release'] is True
    assert params['association_gate_fraction'] <= 0.25
    assert params['target_class'] in params['class_prompts']
    assert params['kp'] <= 0.45
    compose = (REPO / 'compositions/turret_hardware.yaml').read_text()
    assert 'yolo_tracker_hardware.yaml' in compose
    assert 'TURRET_YOLO_CONFIDENCE:-0.003' not in compose
