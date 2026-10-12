"""The object-tracking simulation test's scoring (tools/tracking_sim_test.py)
and frames, on made-up recordings."""
import math

import pytest
from conftest import REPO, load_script

sim_test = load_script(REPO / 'tools/tracking_sim_test.py', 'tracking_sim_test_review')


def row(sim, step, nav, target, truth=None, state='tracking', yaw=0.0, estimate=None,
        hidden='', message='', event=None):
    item = {'k': 'row', 'sim': sim, 'step': step, 'nav': nav, 'truth': truth or nav,
            'yaw': yaw, 'hidden': hidden, 'estimate': estimate,
            'targets': [{'name': 'car_1', 'east': target[0], 'north': target[1],
                         'up': target[2]}],
            'status': {'state': state, 'message': message}}
    if event:
        item['event'] = event
    return item


def test_world_points_move_to_the_navigation_frame():
    """The navigation origin (home) 40 m east and 45 m north of the world
    origin: a world point there is the navigation frame's (0, 0)."""
    world = (37.9234, -122.5967)
    lat, lon = sim_test.local_to_latlon(40.0, 45.0, *world)
    east, north = sim_test.world_to_navigation(40.0, 45.0, world, (lat, lon, 700.0))
    assert (east, north) == pytest.approx((0.0, 0.0), abs=1e-6)
    east, north = sim_test.world_to_navigation(50.0, 40.0, world, (lat, lon, 700.0))
    assert (east, north) == pytest.approx((10.0, -5.0), abs=0.01)


def test_centered_errors_after_settling():
    step = {'name': 'centered', 'kind': 'follow', 'mode': 'centered', 'target': 'car_1',
            'altitude': 40.0}
    rows = [row(0.1 * k, 'centered', (3.0, 4.0, 42.0) if k < 200 else (0.6, 0.8, 41.0),
                (0.0, 0.0, 1.0), estimate=(0.0, 0.5, 1.0)) for k in range(400)]
    result = sim_test.score([row for row in rows], [step], settle_s=20.0)['centered']
    assert result['rows'] == 200 and result['tracking_fraction'] == 1.0
    assert result['horizontal_truth'] == {'n': 200, 'median': 1.0, 'p95': 1.0, 'max': 1.0}
    assert result['estimate']['median'] == pytest.approx(0.5)
    assert result['height_agl_truth']['median'] == pytest.approx(0.0)


def test_slant_errors():
    """60 m at 70 deg: 20.52 m short of the target, 56.38 m above it, with
    the image's long axis (mount 0: body -y) on the target."""
    step = {'name': 'slant', 'kind': 'follow', 'mode': 'slant', 'target': 'car_1',
            'slant_range': 60.0, 'depression': 70.0, 'axis_deg': -90.0}
    standoff, height = 60.0 * math.cos(math.radians(70.0)), 60.0 * math.sin(math.radians(70.0))
    # The target north of the drone; yaw 180 deg puts body -y north.
    rows = [row(30.0 + 0.1 * k, 'slant', (0.0, -standoff, height + 2.0), (0.0, 0.0, 2.0),
                yaw=math.pi) for k in range(50)]
    rows.insert(0, row(0.0, 'slant', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    result = sim_test.score(rows, [step], settle_s=20.0)['slant']
    assert result['range_truth']['max'] == pytest.approx(0.0, abs=1e-6)
    assert result['depression_nav']['max'] == pytest.approx(0.0, abs=1e-6)
    assert result['bearing_nav']['max'] == pytest.approx(0.0, abs=1e-6)


def test_lost_stop_and_preemption_checks():
    target = (0.0, 0.0, 0.0)
    rows = [row(0.0, 'lost', (0.0, 0.0, 40.0), target)]
    rows += [row(0.1 * k, 'lost', (0.0, 0.0, 40.0), target, hidden='car_1',
                 state='tracking' if k < 30 else 'lost') for k in range(1, 80)]
    rows += [row(8.0 + 0.1 * k, 'lost', (0.0, 0.0, 40.0), target,
                 state='lost' if k < 10 else 'tracking') for k in range(1, 40)]
    steps = [{'name': 'lost', 'kind': 'lost', 'target': 'car_1'}]
    result = sim_test.score(rows, steps)['lost']
    assert result['passed'] and result['lost_after_s'] == pytest.approx(3.0, abs=0.11)
    assert result['tracking_again_after_s'] == pytest.approx(1.1, abs=0.11)

    drifting = [row(0.1 * k, 'stop', (0.05 * k, 0.0, 40.0), target, state='idle')
                for k in range(100)]
    assert not sim_test.score(drifting, [{'name': 'stop', 'kind': 'stop'}])['stop']['passed']
    holding = [row(0.1 * k, 'stop', (0.0, 0.0, 40.0), target, state='idle') for k in range(100)]
    assert sim_test.score(holding, [{'name': 'stop', 'kind': 'stop'}])['stop']['passed']

    step = {'name': 'preempt_map_goal', 'kind': 'preempt', 'cause': 'a map goal'}
    stale = row(0.0, 'preempt_map_goal', (0.0, 0.0, 40.0), target, state='idle',
                message='pre-empted by a map goal (0;0;0)')
    preempted = [stale,
                 row(0.1, 'preempt_map_goal', (0.0, 0.0, 40.0), target, state='idle',
                     event={'command': {'action': 'track'}}),
                 row(0.5, 'preempt_map_goal', (0.0, 0.0, 40.0), target),
                 row(5.5, 'preempt_map_goal', (0.0, 0.0, 40.0), target, state='idle',
                     message='pre-empted by a map goal (1;2;3)')]
    report = sim_test.score(preempted, [step])['preempt_map_goal']
    assert report['passed'] and report['message'].endswith('(1;2;3)')
    assert not sim_test.score(preempted[:3], [step])['preempt_map_goal']['passed']
    other = dict(step, cause='GoToBehavior')
    assert not sim_test.score(preempted, [other])['preempt_map_goal']['passed']
    goto = [row(0.0, 'preempt_goto', (0.0, 0.0, 40.0), target, state='idle',
                event={'goto_result': 'succeeded'}),   # the hold before
            row(0.5, 'preempt_goto', (0.0, 0.0, 40.0), target, state='idle',
                event={'command': {'action': 'track'}}),
            row(0.7, 'preempt_goto', (0.0, 0.0, 40.0), target, state='acquiring',
                event={'goto_result': 'canceled'}),
            row(1.0, 'preempt_goto', (0.0, 0.0, 40.0), target)]
    steps = [{'name': 'preempt_goto', 'kind': 'preempt'}]
    result = sim_test.score(goto, steps)['preempt_goto']
    assert result['passed'] and result['goto_result'] == 'canceled'
    goto[2]['event'] = {'goto_result': 'succeeded'}
    assert not sim_test.score(goto, steps)['preempt_goto']['passed']


# --- the real detector: picking targets, ids, rates ---------------------------

def box(name, cx, cy, w=10.0, h=10.0):
    return {'id': name, 'cx': cx, 'cy': cy, 'w': w, 'h': h}


def test_tracks_are_matched_to_the_nearest_target_within_the_gate():
    """The car's box is big (its gate 0.6 x 60 px), a person's small (25 px);
    a track goes to one target only, nearest first."""
    truth = [box('car_1', 100, 100, 60, 30), box('person_1', 300, 200, 7, 9),
             box('person_2', 330, 200, 7, 9)]
    tracks = [box('4', 120, 95, 58, 28), box('9', 312, 203, 8, 8), box('11', 500, 400)]
    assert sim_test.associate(truth, tracks) == {'car_1': '4', 'person_1': '9'}
    # Beyond the gate: no match (a person 30 px from its box is not it).
    assert sim_test.associate([box('person_1', 300, 200, 7, 9)], [box('9', 330, 200)]) == {}
    # Each track once: the nearer target takes it.
    assert sim_test.associate(truth[1:], [box('9', 318, 200, 8, 8)]) == {'person_2': '9'}
    # A missed car is not the person beside it: another kind, another size.
    car = [dict(box('car_1', 100, 100, 60, 30), cls='car')]
    assert sim_test.associate(car, [dict(box('9', 110, 110, 9, 12), cls='person')]) == {}
    assert sim_test.associate(car, [box('9', 110, 110, 9, 12)]) == {}
    assert sim_test.associate(car, [dict(box('4', 104, 98, 50, 30), cls='van')]) == {'car_1': '4'}


def test_detection_frames_pair_with_the_truth_of_the_same_camera_frame():
    pairing = sim_test.FramePairing(max_wait_ns=2_000_000_000)
    pairing.add_detections(1_000_000_000, ['a'])
    assert pairing.ready() == []                     # its truth has not come yet
    pairing.add_truth(1_000_000_000, ['A'])
    pairing.add_detections(1_200_000_000, ['b'])
    assert pairing.ready() == [(1_000_000_000, ['a'], ['A'])]
    pairing.add_truth(3_300_000_000, ['C'])          # 1.2 s never came: dropped
    assert pairing.ready() == [] and pairing.dropped == 1 and pairing.pending == []


def test_a_target_is_picked_by_the_id_it_kept_for_three_frames():
    frames = [{'car_1': '3'}, {'car_1': '3'}, {'car_1': '3', 'person_1': '5'}]
    assert sim_test.stable_id(frames, 'car_1') == '3'
    assert sim_test.stable_id(frames, 'person_1') is None
    assert sim_test.stable_id(frames + [{'car_1': '8'}], 'car_1') is None


def det(sim, visible, match, cmd=None, fp=0, rx=None):
    return {'k': 'det', 'sim': sim, 'rx': sim if rx is None else rx, 'visible': visible,
            'match': match, 'cmd': cmd, 'fp': fp}


def test_detector_recall_ids_and_switches_on_the_followed_target():
    dets = ([det(0.2 * k, ['car_1'], {'car_1': '3'}, cmd='3') for k in range(50)]
            + [det(10.0 + 0.2 * k, ['car_1'], {}, cmd='3') for k in range(10)]
            + [det(12.0 + 0.2 * k, ['car_1'], {'car_1': '7'}, cmd='7', fp=1)
               for k in range(40)])
    result = sim_test.score_ids(dets, 'car_1')
    assert result['frames_in_view'] == 100 and result['recall'] == 0.9
    assert result['ids'] == ['3', '7'] and result['id_switches'] == 1
    assert result['following'] == 1.0 and result['unmatched_tracks_per_frame'] == 0.4
    assert sim_test.score_ids(dets, 'person_1')['recall'] is None


def test_time_to_tracking_again_after_each_loss():
    states = ['acquiring'] * 5 + ['tracking'] * 10 + ['lost'] * 32 + ['tracking'] * 5 + ['lost'] * 4
    rows = [{'sim': 0.1 * k, 'status': {'state': state}} for k, state in enumerate(states)]
    times = sim_test.reacquisitions(rows, 0.0)
    assert times == [pytest.approx(3.2), pytest.approx(0.3)]  # the last one unrecovered


def test_follow_step_fails_on_its_thresholds():
    """12 m from the car, re-selected by the operator five times, the
    tracker on another track than the car's half the time: three failures;
    the detector's own id switches are reported, not failed."""
    step = {'name': 'centered', 'kind': 'follow', 'mode': 'centered', 'target': 'car_1',
            'altitude': 40.0}
    rows = [row(0.1 * k, 'centered', (12.0, 0.0, 41.0), (0.0, 0.0, 1.0)) for k in range(400)]
    for k in range(6):
        rows[60 * k]['event'] = {'command': {'action': 'track', 'target_id': str(k)}}
    dets = [det(0.2 * k, ['car_1'], {'car_1': str(k % 4)}, cmd=str(k % 2)) for k in range(200)]
    result = sim_test.score(rows, [step], dets=dets)['centered']
    assert not result['passed'] and result['reacquisitions'] == 5
    assert set(result['failures']) == {'horizontal_truth median', 'reacquisitions', 'following'}
    assert result['detector']['id_switches'] == 199
    # Re-selections in passing restart no settling: the 200 rows after the
    # first 20 s count.
    assert result['rows'] == 200
    near = [row(0.1 * k, 'centered', (1.0, 0.0, 41.0), (0.0, 0.0, 1.0)) for k in range(400)]
    steady = [det(0.2 * k, ['car_1'], {'car_1': '3'}, cmd='3') for k in range(200)]
    assert sim_test.score(near, [step], dets=steady)['centered']['passed']


def test_lost_step_picks_the_new_id_and_counts_the_time_to_tracking():
    target = (0.0, 0.0, 0.0)
    step = {'name': 'lost', 'kind': 'lost', 'target': 'person_1'}
    rows = [row(0.1 * k, 'lost', (0.0, 0.0, 40.0), target, hidden='person_1',
                state='tracking' if k < 30 else 'lost') for k in range(1, 80)]
    rows += [row(8.0 + 0.1 * k, 'lost', (0.0, 0.0, 40.0), target,
                 state='lost' if k < 40 else 'tracking',
                 event={'reacquire': 'person_1'} if k == 25 else None) for k in range(1, 60)]
    dets = [det(8.0 + 0.2 * k, ['person_1'], {'person_1': '12'} if k > 10 else {})
            for k in range(30)]
    result = sim_test.score(rows, [step], dets=dets)['lost']
    assert result['redetected_after_s'] == pytest.approx(2.3, abs=0.11)
    assert result['tracking_again_after_s'] == pytest.approx(4.1, abs=0.11)
    assert result['new_id'] and result['passed']


def test_detector_rate_in_simulation_and_wall_time():
    # 5 Hz of simulation time in a simulator at half real time.
    dets = [det(0.2 * k, [], {}, rx=1000.0 + 0.4 * k) for k in range(300)]
    result = sim_test.score_detector(dets)
    assert result['rate_sim_hz']['median'] == pytest.approx(5.0)
    assert result['rate_wall_hz']['median'] == pytest.approx(2.5)
    assert result['passed']
    slow = [det(0.3 * k, [], {}) for k in range(200)]
    assert not sim_test.score_detector(slow)['passed']


def test_resources_from_the_cpu_samples_and_the_detector_log():
    import json
    cpu = [json.dumps({'t': k, 'host_busy_pct': 40.0 + k, 'cores': 20,
                       'containers': {'gazebo': 300.0, 'tracking_detector': 95.0 + k}})
           for k in range(5)]
    log = ['ti_gazebo-tracking_detector-1  | 2026-10-11T19:00:00Z [INFO] [1.0] '
           '[sim_tracking_detector]: 5.0 frames/s, 52 ms each, 3 tracks\n',
           'ti_gazebo-tracking_detector-1  | 2026-10-11T19:00:10Z [INFO] [1.0] '
           '[sim_tracking_detector]: 4.9 frames/s, 55 ms each, 2 tracks\n']
    result = sim_test.score_resources(cpu, log)
    assert result['host_busy_pct']['median'] == 42.0 and result['cores'] == 20
    assert list(result['containers_pct_of_core']) == ['gazebo', 'tracking_detector']
    assert result['containers_pct_of_core']['tracking_detector']['median'] == 97.0
    assert result['detector_wall_hz']['median'] == pytest.approx(4.95)
    assert result['detector_ms_per_frame']['max'] == 55.0


def test_host_busy_from_proc_stat():
    before = 'cpu  100 0 100 800 0 0 0 0 0 0'
    after = 'cpu  150 0 150 900 0 0 0 0 0 0'
    assert sim_test.host_busy(before, after) == pytest.approx(50.0)


def test_navigation_error_and_demnav_rates_per_step():
    rows = [dict(row(0.5 * k, 'centered', (1.0, 0.0, 40.0), (0.0, 0.0, 0.0),
                     truth=(0.0, 0.0, 40.0)),
                 navigation={'demnav_frames': 3 * k, 'demnav_valid': k // 2,
                             'demnav_fixes': k // 2, 'wildnav_fixes': 0})
            for k in range(121)]
    result = sim_test.score(rows, [])['navigation']['centered']
    assert result['error_m']['median'] == 1.0
    assert result['demnav_frames_per_min'] == 360.0 and result['demnav_valid_per_min'] == 60.0
