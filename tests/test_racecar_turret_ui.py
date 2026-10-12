"""RACECAR (Turret) web UI decision logic: applications/nicegui/scripts/
racecar_ui_logic.py, the pure part of main.py (joysticks and their
deadman, map image and taps, turret boxes and taps, status parsing), and
the commands it produces against systems/racecar_turret/interfaces.md.
"""
import json
import math
import re

import numpy as np
import pytest

from conftest import REPO, load_script

logic = load_script(REPO / 'applications/nicegui/scripts/racecar_ui_logic.py', 'racecar_ui_logic')
LIMITS = logic.DriveLimits(max_speed=0.3, max_steering=0.42, wheelbase=0.33, deadzone=0.1)


# --- Joysticks --------------------------------------------------------------

def steering_of(speed, yaw_rate):
    """The front-wheel angle cmd_vel_to_ackermann derives from a Twist."""
    return 0.0 if abs(speed) < 1e-6 else math.atan(yaw_rate * LIMITS.wheelbase / speed)


@pytest.mark.parametrize('x,y', [(0.0, 0.0), (0.05, -0.08), (-0.09, 0.09)])
def test_stick_inside_the_dead_zone_is_still(x, y):
    assert logic.drive_twist(x, y, LIMITS) == (0.0, 0.0)
    assert logic.turret_rates(x, y, 1.0) == (0.0, 0.0)


def test_full_stick_forward_is_max_speed_without_steering():
    speed, yaw_rate = logic.drive_twist(0.05, 1.0, LIMITS)
    assert speed == pytest.approx(0.3, rel=1e-2) and yaw_rate == 0.0


@pytest.mark.parametrize('y', [1.0, 0.5, -0.5, -1.0])
def test_stick_right_steers_right_forward_and_in_reverse(y):
    x = 0.6
    speed, yaw_rate = logic.drive_twist(x, y * 0.8, LIMITS)
    steering = steering_of(speed, yaw_rate)
    # Right is negative (REP-103) whichever way the car rolls.
    assert steering == pytest.approx(-(0.6 - 0.1) / 0.9 * LIMITS.max_steering, rel=1e-6)


def test_stick_is_clamped_to_the_unit_disc_and_the_limits():
    speed, yaw_rate = logic.drive_twist(-3.0, 4.0, LIMITS)    # |stick| 5
    assert 0.0 < speed <= LIMITS.max_speed
    assert 0.0 < steering_of(speed, yaw_rate) <= LIMITS.max_steering
    pan, tilt = logic.turret_rates(10.0, 0.0, 1.2)
    assert (pan, tilt) == (pytest.approx(1.2), 0.0)


@pytest.mark.parametrize('x,y', [(math.nan, 0.5), (0.5, math.inf)])
def test_non_finite_stick_is_still(x, y):
    assert logic.drive_twist(x, y, LIMITS) == (0.0, 0.0)


def test_turret_stick_signs_follow_the_contract():
    # Right pans right (positive pan), up tilts up (positive tilt).
    pan, tilt = logic.turret_rates(0.8, 0.5, 1.0)
    assert pan > 0.0 and tilt > 0.0
    command = logic.manual_command(pan, -tilt)
    assert command == {'action': 'manual', 'pan_rate': pytest.approx(pan, abs=1e-4),
                       'tilt_rate': pytest.approx(-tilt, abs=1e-4)}
    json.dumps(command)


def test_hold_repeats_the_stick_while_reports_arrive_then_sends_one_zero():
    hold = logic.Hold(deadman_s=0.35)
    assert hold.tick(0.0) is None                           # never touched: silent
    assert hold.report(0.2, 0.9, True, 0.0) is True          # starts a hold
    assert hold.report(0.3, 0.9, True, 0.1) is False
    assert [hold.tick(t) for t in (0.1, 0.2, 0.3)] == [(0.3, 0.9)] * 3
    assert hold.held(0.3)
    hold.report(0.0, 0.0, False, 0.35)                       # released
    assert hold.tick(0.4) == (0.0, 0.0)
    assert hold.tick(0.5) is None and not hold.held(0.5)


def test_hold_deadman_stops_when_the_browser_stops_reporting():
    hold = logic.Hold(deadman_s=0.35)
    hold.report(0.0, 1.0, True, 10.0)
    assert hold.tick(10.3) == (0.0, 1.0)
    # No report for longer than the deadman (tab in the background, link lost).
    assert hold.tick(10.4) == (0.0, 0.0)
    assert hold.tick(10.5) is None
    # The next report starts a new hold (the UI cancels Nav2 again).
    assert hold.report(0.0, 1.0, True, 11.0) is True


def test_hold_ignores_reports_delivered_late():
    hold = logic.Hold(deadman_s=0.35)
    # Browser clock 500 s behind; reports normally take 20 ms.
    for i in range(3):
        hold.report(0.0, 1.0, True, 1000.02 + 0.1 * i, sent=500.0 + 0.1 * i)
    hold.report(0.0, 0.0, False, 1000.3, sent=500.28)
    assert hold.tick(1000.35) == (0.0, 0.0) and hold.tick(1000.45) is None
    # The link drops; the socket then flushes reports sent seconds ago.
    for i in range(5):
        assert hold.report(0.0, 1.0, True, 1010.0, sent=505.0 + 0.1 * i) is False
    assert hold.tick(1010.05) is None
    # Fresh reports drive again.
    assert hold.report(0.0, 1.0, True, 1010.1, sent=510.08) is True
    assert hold.tick(1010.15) == (0.0, 1.0)


# --- Map --------------------------------------------------------------------

def test_occupancy_image_is_flipped_and_coloured():
    # 3 x 2 grid: row 0 (lowest y) unknown, free, occupied; row 1 all free.
    image, view = logic.occupancy_image([-1, 0, 100, 0, 0, 0], 3, 2, 0.05, (1.0, 2.0, 0.0))
    assert image.shape == (2, 3) and image.dtype == np.uint8
    assert image[1].tolist() == [205, 255, 0]               # bottom image row = grid row 0
    assert image[0].tolist() == [255, 255, 255]
    assert (view.width, view.height, view.resolution) == (3, 2, 0.05)


def test_map_pixels_and_coordinates_round_trip_with_a_rotated_origin():
    view = logic.MapView(resolution=0.05, origin_x=-3.0, origin_y=2.0, origin_yaw=0.4,
                         width=400, height=300)
    assert view.to_map(0.0, 300.0) == pytest.approx((-3.0, 2.0))   # bottom-left corner
    for x, y in [(0.0, 0.0), (1.5, 4.2), (-2.0, 7.0)]:
        assert view.to_map(*view.to_pixel(x, y)) == pytest.approx((x, y))
    flat = logic.MapView(0.1, 0.0, 0.0, 0.0, 100, 50)
    # Image rows grow downwards, map y upwards.
    assert flat.to_map(10.0, 50.0) == pytest.approx((1.0, 0.0))
    assert flat.to_map(10.0, 0.0) == pytest.approx((1.0, 5.0))


def test_large_map_is_reduced_keeping_walls_and_the_origin():
    width, height = 2500, 1300
    grid = np.zeros((height, width), dtype=np.int8)
    grid[700, :] = 100                                       # one-cell wall at y = 700 cells
    image, view = logic.occupancy_image(grid.ravel(), width, height, 0.05, (-10.0, -5.0, 0.0),
                                        max_size=1024)
    step = 3
    assert max(image.shape) <= 1024 and view.resolution == pytest.approx(0.05 * step)
    assert view.to_map(0.0, view.height) == pytest.approx((-10.0, -5.0))
    _px, wall_row = view.to_pixel(0.0, -5.0 + 700.5 * 0.05)
    assert image[int(wall_row)].max() == 0                   # the wall survived the reduction


def test_tapping_the_map_heads_the_goal_away_from_the_robot():
    view = logic.MapView(0.05, 0.0, 0.0, 0.0, 200, 200)
    x, y, yaw = logic.tap_goal(view, 100.0, 100.0, robot=(0.0, 0.0, 1.0))
    assert (x, y) == pytest.approx((5.0, 5.0))
    assert yaw == pytest.approx(math.pi / 4)
    assert logic.tap_goal(view, 100.0, 100.0, robot=None, default_yaw=0.3)[2] == 0.3
    assert logic.tap_goal(view, 0.0, 200.0, robot=(0.0, 0.0, 1.0))[2] == 0.0   # on the robot


def test_map_overlay_draws_path_goal_and_robot_in_image_pixels():
    view = logic.MapView(0.1, 0.0, 0.0, 0.0, 100, 100)
    svg = logic.map_overlay_svg(view, robot=(1.0, 2.0, 0.0), path=[(1.0, 2.0), (5.0, 5.0)],
                                goal=(5.0, 5.0, 0.0))
    assert '<polyline points="10.0,80.0 50.0,50.0"' in svg
    assert '<circle cx="50.0" cy="50.0"' in svg
    assert svg.count('<polygon') == 2
    assert logic.map_overlay_svg(view) == ''


# --- Turret camera ----------------------------------------------------------

SEGMENTS = json.dumps({
    'stamp': {'sec': 12, 'nanosec': 500000000}, 'width': 640, 'height': 480,
    'detections': [
        {'id': '3', 'class': 'person', 'confidence': 0.9, 'bbox_xywh': [320, 240, 100, 200]},
        {'id': '7', 'class': 'person', 'confidence': 0.6, 'bbox_xywh': [360, 240, 100, 200]},
        {'id': 9, 'class': 'car', 'confidence': 0.5, 'bbox_xywh': [80, 400, 60, 40]},
        {'id': '10', 'class': 'bad', 'confidence': 0.5, 'bbox_xywh': [80, 400, 0, 40]},
        {'class': 'no-box'},
    ]})


def test_segments_are_parsed_into_normalised_boxes():
    segments = logic.parse_segments(SEGMENTS)
    assert (segments.stamp, segments.width, segments.height) == (12.5, 640, 480)
    assert [d.id for d in segments.detections] == ['3', '7', '9']
    first = segments.detections[0]
    assert (first.u, first.v, first.w, first.h) == pytest.approx((0.5, 0.5, 100 / 640, 200 / 480))


def test_tracker_segment_list_carries_stamp_and_size_per_detection():
    # The PanTiltROS tracker's form: a list, each detection with the image's
    # stamp (seconds) and size.
    text = json.dumps([
        {'id': '4', 'class': 'person', 'confidence': 0.8, 'bbox_xywh': [160, 120, 32, 48],
         'centroid_px': [160, 125], 'selected': True, 'stamp': 1700000000.25,
         'image_width': 320, 'image_height': 240},
        {'id': '5', 'class': 'person', 'confidence': 0.7, 'bbox_xywh': [40, 40, 10, 10],
         'selected': False, 'stamp': 1700000000.25, 'image_width': 320, 'image_height': 240}])
    segments = logic.parse_segments(text, fallback_size=(640, 480))
    assert (segments.stamp, segments.width, segments.height) == (1700000000.25, 320, 240)
    assert [(d.id, d.u, d.v) for d in segments.detections] == [('4', 0.5, 0.5), ('5', 0.125, 1 / 6)]
    assert logic.lock_command(segments, 0.5, 0.55) == {'action': 'lock', 'target_id': '4'}
    # No detections: nothing to draw, whatever the size.
    assert logic.parse_segments('[]', fallback_size=(640, 480)).detections == ()


@pytest.mark.parametrize('payload', [
    {'stamp': 1.0, 'image_size': [320, 240], 'detections': []},
    {'stamp': 1.0, 'size': {'width': 320, 'height': 240}, 'detections': []},
    {'stamp': 1.0, 'image_width': 320, 'image_height': 240, 'detections': []},
])
def test_segments_image_size_variants(payload):
    segments = logic.parse_segments(json.dumps(payload))
    assert (segments.width, segments.height) == (320, 240)


def test_legacy_segment_list_needs_the_image_size_and_has_no_ids():
    legacy = json.dumps([{'class': 'person', 'confidence': 0.4, 'bbox_xywh': [100, 100, 20, 40]}])
    assert logic.parse_segments(legacy) is None
    segments = logic.parse_segments(legacy, fallback_size=(640, 480))
    assert segments.detections[0].id == ''
    # Without an id a tap on the box locks the point.
    assert logic.lock_command(segments, 100 / 640, 100 / 480)['point'] == [
        pytest.approx(100 / 640, abs=1e-4), pytest.approx(100 / 480, abs=1e-4)]


@pytest.mark.parametrize('text', ['', 'not json', '42', '{"detections": 3}', 'null'])
def test_bad_segments_are_rejected(text):
    assert logic.parse_segments(text, fallback_size=(640, 480)) is None


def test_click_position_becomes_image_fractions():
    assert logic.image_point(320, 120, 640, 480) == pytest.approx((0.5, 0.25))
    assert logic.image_point(641, 10, 640, 480) is None
    assert logic.image_point(10, 10, 0, 480) is None


def test_tap_on_a_box_locks_its_id_overlap_picks_the_nearest_centre():
    segments = logic.parse_segments(SEGMENTS)
    # Inside box 3 only.
    assert logic.lock_command(segments, 290 / 640, 0.5) == {'action': 'lock', 'target_id': '3'}
    # Inside both 3 (centre 320) and 7 (centre 360): nearer 7's centre.
    assert logic.lock_command(segments, 350 / 640, 0.5) == {'action': 'lock', 'target_id': '7'}
    assert logic.lock_command(segments, 80 / 640, 400 / 480) == {'action': 'lock', 'target_id': '9'}


def test_tap_on_no_box_locks_the_point():
    segments = logic.parse_segments(SEGMENTS)
    command = logic.lock_command(segments, 0.9, 0.1)
    assert command == {'action': 'lock', 'point': [0.9, 0.1]}
    assert logic.lock_command(None, 0.25, 0.75) == {'action': 'lock', 'point': [0.25, 0.75]}


def test_boxes_are_drawn_scaled_to_the_displayed_image_and_target_highlighted():
    segments = logic.parse_segments(SEGMENTS)
    svg = logic.detections_svg(segments, 320, 240, target_id='7', locked=True)
    rects = re.findall(r'<rect x="([\d.]+)" y="([\d.]+)" width="([\d.]+)" height="([\d.]+)" '
                       r'fill="(#\w+)"', svg)
    assert len(rects) == 3
    assert [float(v) for v in rects[0][:4]] == pytest.approx([135.0, 70.0, 50.0, 100.0])
    assert rects[1][4] == '#22c55e' and rects[0][4] == '#22d3ee'
    assert '#f59e0b' in logic.detections_svg(segments, 320, 240, target_id='7', locked=False)
    hostile = logic.parse_segments(json.dumps({'width': 10, 'height': 10, 'detections': [
        {'id': '<x>', 'class': '<script>', 'confidence': 1, 'bbox_xywh': [5, 5, 2, 2]}]}))
    assert '<script>' not in logic.detections_svg(hostile, 10, 10)


# --- Status -----------------------------------------------------------------

def test_control_status_parsing_and_summary():
    status = logic.parse_control_status(json.dumps(
        {'source': 'teleop', 'stopped': True, 'speed_mps': 0.0, 'steering_rad': -0.2}))
    assert status == {'source': 'teleop', 'stopped': True, 'speed_mps': 0.0, 'steering_rad': -0.2}
    summary = logic.control_summary(status)
    assert 'manual' in summary and 'right' in summary and 'STOPPED' in summary
    odd = logic.parse_control_status('{"source": "joystick", "stopped": "yes", "speed_mps": "fast"}')
    assert odd == {'source': 'none', 'stopped': False, 'speed_mps': None, 'steering_rad': None}
    assert logic.parse_control_status('[]') is None
    assert 'no status' in logic.control_summary(None)


def test_turret_status_parsing_and_summary():
    text = json.dumps({'mode': 'auto', 'target_id': '3', 'target_class': 'person', 'locked': True,
                       'aim_error_deg': 1.25, 'base_compensation': True, 'detections': 2,
                       'message': ''})
    status = logic.parse_turret_status(text)
    assert status['mode'] == 'auto' and status['locked'] is True
    assert status['aim_error_deg'] == 1.25 and status['detections'] == 2
    summary = logic.turret_summary(status)
    assert 'person #3 (locked)' in summary and '1.2 deg' in summary and 'compensation on' in summary
    idle = logic.parse_turret_status('{"mode": "idle", "target_id": "", "aim_error_deg": null}')
    assert idle['aim_error_deg'] is None and idle['target_id'] == ''
    assert logic.parse_turret_status('{"mode": "warp"}')['mode'] == 'unknown'
    assert logic.parse_turret_status('oops') is None
    assert status['stamp'] is None
    assert logic.parse_turret_status('{"mode": "idle", "stamp": 12.5}')['stamp'] == 12.5


def test_status_copies_and_reordered_statuses_are_dropped():
    # Each status arrives twice (two subscriptions); a copy can come late
    assert logic.newer_status(None, 10.0) and logic.newer_status(10.0, None)
    assert logic.newer_status(10.0, 10.1)
    assert not logic.newer_status(10.1, 10.1)  # the copy
    assert not logic.newer_status(10.1, 10.0)  # older, reordered
    assert logic.newer_status(1000.0, 5.0)  # the tracker restarted


def test_contract_modes_and_sources_match_interfaces_md():
    contract = (REPO / 'systems/racecar_turret/interfaces.md').read_text()
    for mode in logic.TURRET_MODES:
        assert f'"{mode}"' in contract
    for source in logic.CONTROL_SOURCES:
        assert f'"{source}"' in contract
    for action in ('lock', 'release', 'manual', 'center'):
        assert f'"action": "{action}"' in contract


def test_freshness():
    assert logic.fresh(10.0, 9.5) and not logic.fresh(10.0, 8.0) and not logic.fresh(10.0, None)


def test_quaternion_helpers_round_trip():
    for yaw in (-2.5, 0.0, 1.0, 3.0):
        assert logic.yaw_from_quaternion(*logic.quaternion_from_yaw(yaw)) == pytest.approx(yaw)


# --- Foam dart launcher ---------------------------------------------------------

READY = {'mode': 'auto', 'target_id': '7', 'target_class': 'person', 'locked': True,
         'aim_error_deg': 0.6, 'base_compensation': True, 'detections': 2, 'message': '',
         'launcher': 'ready', 'darts': 2, 'tags': 1, 'tagged_ids': ['3'],
         'last_tag': {'target_id': '3', 'result': 'fired', 'reason': '', 'stamp': 12.5},
         'fire_block': '', 'auto_next': True}


def status_with(**changes):
    return logic.parse_turret_status(json.dumps(dict(READY, **changes)))


def test_turret_status_carries_the_launcher_fields():
    status = logic.parse_turret_status(json.dumps(READY))
    assert (status['launcher'], status['darts'], status['tags'], status['tagged_ids'],
            status['auto_next'], status['fire_block']) == ('ready', 2, 1, ('3',), True, '')
    assert status['last_tag'] == {'target_id': '3', 'result': 'fired', 'reason': '', 'stamp': 12.5}
    # A tracker without the launcher's fields, or odd values: never ready to fire
    old = logic.parse_turret_status('{"mode": "auto", "target_id": "7", "locked": true}')
    assert (old['launcher'], old['darts'], old['tagged_ids'], old['last_tag'],
            old['fire_block']) == ('none', 0, (), None, None)
    odd = status_with(launcher='launching', darts='many', tagged_ids='3', fire_block=0,
                      last_tag='x')
    assert (odd['launcher'], odd['darts'], odd['tagged_ids'], odd['fire_block'],
            odd['last_tag']) == ('none', 0, (), None, None)
    for state in logic.LAUNCHER_STATES:
        assert f'"{state}"' in (REPO / 'systems/racecar_turret/interfaces.md').read_text()


def test_fire_button_is_enabled_only_while_every_condition_holds():
    assert logic.fire_button(status_with(), armed=True, actuator_ready=True) == (True, '')
    assert logic.fire_command(status_with()) == {'action': 'tag', 'target_id': '7'}
    cases = [
        (None, True, True, 'no status from the tracker'),
        (status_with(launcher='none', fire_block='no launcher'), True, True, 'no launcher'),
        (status_with(), False, True, 'turret disarmed'),
        (status_with(), None, True, 'turret disarmed'),          # no e-stop state yet
        (status_with(), True, False, 'actuator not ready'),
        (status_with(), True, None, 'actuator not ready'),
        (status_with(mode='manual'), True, True, 'no locked target'),
        (status_with(mode='estop'), True, True, 'no locked target'),
        (status_with(target_id=''), True, True, 'no locked target'),
        (status_with(fire_block='too close'), True, True, 'too close'),
        (status_with(fire_block='aim error over 2 deg'), True, True, 'aim error over 2 deg'),
        (status_with(fire_block=None), True, True, 'the tracker does not report fire_block'),
        (status_with(locked=False), True, True, 'target not locked'),
        (status_with(darts=0), True, True, 'no darts'),
        (status_with(launcher='priming'), True, True, 'launcher priming'),
        (status_with(launcher='fault'), True, True, 'launcher fault'),
    ]
    for status, armed, ready, reason in cases:
        assert logic.fire_button(status, armed, ready) == (False, reason)


def test_reload_auto_next_and_launcher_summary():
    assert logic.reload_command(6) == {'action': 'reload', 'darts': 6}
    assert logic.reload_command(6.0) == {'action': 'reload', 'darts': 6}
    assert logic.reload_command('3') == {'action': 'reload', 'darts': 3}
    for bad in (-1, 2.5, 'x', None, math.nan, 5000):
        assert logic.reload_command(bad) is None
    assert logic.auto_next_command(True) == {'action': 'auto_next', 'enabled': True}
    assert logic.auto_next_command(0) == {'action': 'auto_next', 'enabled': False}
    assert logic.launcher_summary(status_with()) == (
        'Launcher: ready, 2 darts, 1 tag; last: fired #3')
    refused = status_with(darts=1, tags=0, last_tag={
        'target_id': '9', 'result': 'refused', 'reason': '9 is not the locked target (7)',
        'stamp': 1})
    assert logic.launcher_summary(refused) == (
        'Launcher: ready, 1 dart, 0 tags; last: refused #9 (9 is not the locked target (7))')
    assert logic.launcher_summary(status_with(launcher='none')) == 'Launcher: none fitted'
    assert 'no status' in logic.launcher_summary(None)


def test_tagged_boxes_are_marked():
    segments = logic.parse_segments(SEGMENTS)
    svg = logic.detections_svg(segments, 320, 240, target_id='7', locked=True, tagged_ids=('3',))
    rects = re.findall(r'<rect [^>]*stroke="(#\w+)"[^>]*?(stroke-dasharray="[\d. ]+")?/>', svg)
    assert [colour for colour, _ in rects] == ['#a78bfa', '#22c55e', '#22d3ee']
    assert rects[0][1] and not rects[1][1]
    assert 'person 0.90 #3 tagged' in svg
    # The locked target stays green even when tagged
    assert 'tagged' in logic.detections_svg(segments, 320, 240, '3', True, ('3',))
    assert '#22c55e' in logic.detections_svg(segments, 320, 240, '3', True, ('3',))


def test_launcher_actions_are_in_the_contract():
    contract = (REPO / 'systems/racecar_turret/interfaces.md').read_text()
    for action in ('tag', 'next', 'clear_tags', 'reload', 'auto_next'):
        assert f'"action": "{action}"' in contract
