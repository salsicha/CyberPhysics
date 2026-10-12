"""RACECAR Turret Isaac evaluation: the evaluator's target selection
(tools/racecar_turret_evaluator.py, as the UI selects) and the scorer
(tools/racecar_turret_scorer.py) on synthetic recordings.
"""
import gzip
import json
import math

import pytest

from conftest import REPO, load_script

scorer = load_script(REPO / 'tools/racecar_turret_scorer.py', 'racecar_turret_scorer')
evaluator = load_script(REPO / 'tools/racecar_turret_evaluator.py', 'racecar_turret_evaluator')

INTRINSICS = (502.3, 502.3, 319.5, 239.5)


def truth(t, x=0.0, yaw=0.0, error_deg=0.0, target=(5.0, 1.0), visible=True, collisions=0,
          others=()):
    """A /simulation/truth sample: the car at (x, 0, yaw), its turret camera
    aimed error_deg (horizontally) off the target's centroid."""
    camera = [x - 0.065 * math.cos(yaw), -0.065 * math.sin(yaw), 0.34]
    centroid = [target[0], target[1], 0.34]
    bearing = math.atan2(centroid[1] - camera[1], centroid[0] - camera[0]) + math.radians(error_deg)
    return {
        'stamp': round(t, 4),
        'vehicle': {'x': x, 'y': 0.0, 'yaw': yaw, 'speed_mps': 0.25},
        'turret_camera': {'position': camera, 'forward': [math.cos(bearing), math.sin(bearing), 0.0],
                          'up': [0.0, 0.0, 1.0]},
        'targets': [{'name': 'person_1', 'class': 'person', 'position': [*target, 0.0],
                     'centroid': centroid, 'visible': visible, 'in_view': True}]
        + [{'name': name, 'class': 'person', 'position': [*xy, 0.0], 'centroid': [*xy, 0.34],
            'visible': True, 'in_view': True} for name, xy in others],
        'collisions': collisions,
    }


def recording(occluded=(40.0, 45.0), late_s=1.0, status_code=4, other_from=None):
    """10 s to 110 s of course at 15 Hz: aimed 2 deg off, 12 deg off for
    late_s after the occlusion, locked except while lost and 1.5 s after,
    one collision at 60 s."""
    rows = [{'t': 0.0, 'event': 'start', 'target': 'person_1'},
            {'t': 9.0, 'event': 'selected', 'target': 'person_1'},
            {'t': 10.0, 'event': 'goal_sent'}]
    t = 10.0
    while t <= 110.0:
        x = 0.25 * (t - 10.0)
        lost = occluded[0] <= t < occluded[1]
        late = occluded[1] <= t < occluded[1] + late_s
        error = 30.0 if lost else (12.0 if late else 2.0)
        others = ()
        if other_from is not None and t >= other_from:
            # Aimed at another person, straight ahead of the camera.
            error = -math.degrees(math.atan2(1.0, 5.065))
            others = (('person_2', (x + 5.0, 0.0)),)
        rows.append({'t': t, 'topic': '/simulation/truth',
                     'data': truth(t, x, 0.0, error, (x + 5.0, 1.0), not lost,
                                   1 if t >= 60.0 else 0, others)})
        locked = not (occluded[0] <= t < occluded[1] + 1.5)
        rows.append({'t': t + 0.01, 'topic': '/turret/status',
                     'data': {'mode': 'auto', 'target_id': '3', 'locked': locked}})
        t = round(t + 1.0 / 15.0, 6)
    rows.append({'t': 110.0, 'event': 'result', 'status': status_code,
                 'status_name': 'succeeded' if status_code == 4 else 'aborted'})
    return rows


def test_selection_projects_the_target_and_picks_its_box():
    sample = truth(0.0, target=(5.0, 1.0))
    sample['turret_camera']['forward'] = [1.0, 0.0, 0.0]
    u, v = evaluator.project_truth(sample, [5.0, 1.0, 0.34], INTRINSICS)
    assert v == pytest.approx(239.5) and u < 319.5  # left of centre
    assert u == pytest.approx(319.5 - 502.3 * 1.0 / 5.065)
    behind = truth(0.0, target=(-5.0, 0.0))
    behind['turret_camera']['forward'] = [1.0, 0.0, 0.0]
    assert evaluator.project_truth(behind, [-5.0, 0.0, 0.34], INTRINSICS) is None
    detections = [
        {'id': '7', 'class': 'person', 'confidence': 0.9, 'bbox_xywh': [u + 5, v, 60, 200]},
        {'id': '8', 'class': 'person', 'confidence': 0.9, 'bbox_xywh': [u + 25, v, 80, 220]},
        {'id': '9', 'class': 'chair', 'confidence': 0.9, 'bbox_xywh': [u, v, 80, 80]},
    ]
    chosen = evaluator.select_detection(detections, u, v)
    assert chosen['id'] == '7'  # both contain it: the nearest centre
    assert evaluator.select_detection(detections, u + 300, v) is None
    assert evaluator.lock_command(chosen, u, v, 640, 480) == {'action': 'lock', 'target_id': '7'}
    no_id = {'class': 'person', 'bbox_xywh': [u, v, 60, 200]}
    assert evaluator.lock_command(no_id, 320, 240, 640, 480) == {'action': 'lock',
                                                                  'point': [0.5, 0.5]}


def test_segments_messages_in_either_shape():
    listed = [{'id': '1', 'class': 'person', 'bbox_xywh': [1, 2, 3, 4]}]
    assert evaluator.segment_detections(listed) == (None, listed)
    stamp, detections = evaluator.segment_detections(
        {'stamp': {'sec': 12, 'nanosec': 500000000}, 'width': 640, 'height': 480,
         'detections': listed})
    assert stamp == 12.5 and detections == listed
    assert evaluator.segment_detections({'stamp': 3.0, 'segments': listed}) == (3.0, listed)
    assert evaluator.segment_detections('nonsense') == (None, [])


def test_mission_poses_of_the_course():
    frame, poses = evaluator.mission_poses(
        REPO / 'systems/racecar_turret/missions/isaac_course.json', 'arena_lap')
    assert frame == 'map' and len(poses) == 10 and poses[-1] == (0.5, 1.2, -0.785)
    with pytest.raises(SystemExit):
        evaluator.mission_poses(REPO / 'systems/racecar_turret/missions/isaac_course.json', 'x')


def test_driving_and_aiming_metrics():
    score = scorer.score_run(recording())
    driving, aiming = score['driving'], score['aiming']
    assert driving['completed'] and driving['result'] == 'succeeded'
    assert driving['time_s'] == pytest.approx(100.0)
    assert driving['collisions'] == 1
    assert driving['distance_m'] == pytest.approx(25.0, abs=0.05)
    assert driving['mean_speed_mps'] == pytest.approx(0.25, abs=0.001)
    # 5 s of 100 occluded: not trackable, and outside the aiming statistics.
    assert aiming['trackable_fraction'] == pytest.approx(0.95, abs=0.002)
    assert aiming['p95_error_deg'] == pytest.approx(2.0, abs=0.01)
    assert aiming['mean_error_deg'] == pytest.approx((94 * 2.0 + 1 * 12.0) / 95, abs=0.02)
    assert aiming['on_target_fraction'] == pytest.approx(94 / 95, abs=0.002)
    assert aiming['locked_fraction'] == pytest.approx(93.5 / 95, abs=0.003)
    assert aiming['all_samples']['on_target_fraction'] == pytest.approx(0.94, abs=0.002)
    [loss] = aiming['losses']
    assert loss['cause'] == 'occluded' and loss['start_s'] == pytest.approx(30.0, abs=0.07)
    assert loss['lost_s'] == pytest.approx(5.0, abs=0.07)
    assert loss['reacquire_s'] == pytest.approx(1.0, abs=0.07)
    assert loss['relock_s'] == pytest.approx(1.55, abs=0.1)  # status a sample behind
    assert aiming['unrecovered_losses'] == 0 and aiming['max_reacquire_s'] == loss['reacquire_s']


def test_out_of_reach_losses_and_unrecovered_targets():
    rows = recording(late_s=200.0)  # never on target again after the loss
    # The target behind the car for 1 s: out of the turret's pan range.
    for row in rows:
        if row.get('topic') == '/simulation/truth' and 80.0 <= row['t'] < 81.0:
            row['data']['vehicle']['yaw'] = math.pi
    aiming = scorer.score_run(rows)['aiming']
    causes = [loss['cause'] for loss in aiming['losses']]
    assert causes == ['occluded', 'out_of_reach']
    assert aiming['unrecovered_losses'] == 2 and aiming['max_reacquire_s'] is None


def test_aiming_at_another_person_is_not_locked_on_the_target():
    aiming = scorer.score_run(recording(other_from=100.0))['aiming']
    assert aiming['aimed_at_other_fraction'] == pytest.approx(10.0 / 95, abs=0.01)
    assert aiming['locked_fraction'] < 0.9


def test_thresholds_pass_and_fail():
    score = scorer.score_run(recording())
    thresholds = {'course_completed': True, 'max_collisions': 1, 'max_mean_aim_error_deg': 3.0,
                  'min_on_target_fraction': 0.95, 'max_reacquire_time_s': 2.0,
                  'max_unrecovered_losses': 0}
    results = {name: passed for name, passed, _, _ in scorer.check(score, thresholds)}
    assert results == {'course_completed': True, 'max_collisions': True,
                       'max_mean_aim_error_deg': True, 'min_on_target_fraction': True,
                       'max_reacquire_time_s': True, 'max_unrecovered_losses': True}
    thresholds.update(max_collisions=0, min_on_target_fraction=0.995)
    failed = [name for name, passed, _, _ in scorer.check(score, thresholds) if not passed]
    assert failed == ['max_collisions', 'min_on_target_fraction']
    # Aiming thresholds only for the runs named in aiming_runs.
    assert [n for n, _, _, _ in scorer.check(score, thresholds, aiming=False)] == [
        'course_completed', 'max_collisions']
    aborted = scorer.score_run(recording(status_code=6))
    assert not dict((n, p) for n, p, _, _ in scorer.check(aborted, thresholds))['course_completed']


def test_command_line_scores_and_compares_runs(tmp_path, capsys):
    for label, rows in (('compensated', recording()), ('uncompensated', recording(late_s=3.0))):
        (tmp_path / label).mkdir()
        with gzip.open(tmp_path / label / 'recording.jsonl.gz', 'wt') as handle:
            for row in rows:
                handle.write(json.dumps(row) + '\n')
            handle.write('{"truncated')
    (tmp_path / 'empty').mkdir()
    (tmp_path / 'empty' / 'recording.jsonl').write_text('{"t": 0, "event": "start"}\n')
    thresholds = tmp_path / 'thresholds.json'
    thresholds.write_text(json.dumps({'aiming_runs': ['compensated'], 'max_reacquire_time_s': 2.0,
                                      'course_completed': True}))
    output = tmp_path / 'summary.json'
    status = scorer.main(['--run', f'compensated={tmp_path / "compensated"}',
                          '--run', f'uncompensated={tmp_path / "uncompensated"}',
                          '--thresholds', str(thresholds), '--output', str(output)])
    assert status == 0  # the uncompensated run's slow reacquisition is not judged
    summary = json.loads(output.read_text())
    assert summary['passed'] and set(summary['runs']) == {'compensated', 'uncompensated'}
    reacquire = summary['comparison']['max_reacquire_s']
    assert reacquire['uncompensated_minus_compensated'] == pytest.approx(2.0, abs=0.07)
    assert (tmp_path / 'compensated' / 'score.json').is_file()
    thresholds.write_text(json.dumps({'aiming_runs': ['compensated', 'uncompensated'],
                                      'max_reacquire_time_s': 2.0}))
    assert scorer.main(['--run', f'a={tmp_path / "compensated"}', '--run',
                        f'uncompensated={tmp_path / "uncompensated"}',
                        '--thresholds', str(thresholds)]) == 1
    assert scorer.main([str(tmp_path / 'empty')]) == 2
    assert 'no usable run' in capsys.readouterr().out


def test_operator_reselects_only_a_target_in_view_and_off_aim():
    sample = truth(0.0, error_deg=2.0)
    locked = {'mode': 'auto', 'locked': True}
    assert evaluator.lock_trouble(sample, locked, 'person_1') is None
    assert evaluator.lock_trouble(sample, {'mode': 'idle'}, 'person_1') is None  # on it anyway
    off = truth(0.0, error_deg=20.0)
    assert evaluator.lock_trouble(off, {'mode': 'auto', 'locked': False}, 'person_1') == 'lost'
    assert evaluator.lock_trouble(off, {'mode': 'idle'}, 'person_1') == 'not tracking'
    # Locked, but on someone else straight ahead of the camera.
    other = truth(0.0, error_deg=-math.degrees(math.atan2(1.0, 5.065)),
                  others=(('person_2', (5.0, 0.0)),))
    assert evaluator.lock_trouble(other, locked, 'person_1') == 'on another person'
    hidden = truth(0.0, error_deg=20.0, visible=False)
    assert evaluator.lock_trouble(hidden, {'mode': 'auto', 'locked': False}, 'person_1') is None
    assert evaluator.lock_trouble(None, None, 'person_1') is None


def test_reselections_are_counted_in_the_course():
    rows = recording()
    rows.insert(-1, {'t': 50.0, 'event': 'reselected', 'reason': 'lost'})
    rows.insert(0, {'t': 5.0, 'event': 'reselected', 'reason': 'before the course'})
    aiming = scorer.score_run(rows)['aiming']
    assert aiming['reselections'] == 1 and aiming['reselection_reasons'] == ['lost']
    failed = [n for n, p, _, _ in scorer.check(scorer.score_run(rows), {'max_reselections': 0})
              if not p]
    assert failed == ['max_reselections']


def test_goals_hold_only_the_poses_inside_the_slam_map():
    poses = [(4.0, 0.0, 0.0), (9.0, 0.0, 0.0), (12.5, 1.2, 0.8), (1.0, 1.0, 0.0)]
    assert evaluator.known_prefix(poses, None) == []
    # Map from (-3, -3), 14 m x 8 m: the third pose is outside, and so the
    # fourth is not sent before it either.
    assert evaluator.known_prefix(poses, (-3.0, -3.0, 14.0, 8.0)) == poses[:2]
    assert evaluator.known_prefix(poses, (-3.0, -3.0, 16.0, 8.0)) == poses
    assert evaluator.known_prefix(poses, (-3.0, -3.0, 12.2, 8.0)) == poses[:1]  # 0.3 m margin


def test_following_measures_the_aim_while_the_tracker_holds_the_target():
    rows = recording(other_from=80.0)
    rows.insert(2, {'t': 9.5, 'event': 'ready', 'intrinsics': list(INTRINSICS)})
    with_segments = []
    for row in rows:
        with_segments.append(row)
        if row.get('topic') != '/simulation/truth':
            continue
        sample = row['data']
        # The tracker's selected box: on person_1 until 80 s, then on person_2.
        name = 'person_1' if row['t'] < 80.0 else 'person_2'
        person = next(t for t in sample['targets'] if t['name'] == name)
        u, v = evaluator.project_truth(sample, person['centroid'], INTRINSICS)
        with_segments.append({'t': row['t'] + 0.02, 'topic': '/turret/yolo/segments', 'data': {
            'stamp': sample['stamp'], 'detections': [
                {'id': '3', 'class': 'person', 'confidence': 0.9, 'bbox_xywh': [u, v, 30, 100],
                 'selected': True}]}})
    aiming = scorer.score_run(with_segments)['aiming']
    # Followed from 10 s to 80 s except while occluded (40-45 s): 65 of 95 s.
    assert aiming['following_fraction'] == pytest.approx(65 / 95, abs=0.01)
    assert aiming['following_on_target_fraction'] == pytest.approx(64 / 65, abs=0.01)
    assert aiming['following_p95_error_deg'] == pytest.approx(2.0, abs=0.01)
    assert aiming['on_target_fraction'] < 0.8  # aimed at person_2 from 80 s
    # Without the camera's intrinsics there is no following measure.
    assert scorer.score_run(rows[:2] + rows[3:])['aiming']['following_fraction'] is None


def test_operator_steers_toward_a_target_out_of_the_image():
    sample = truth(0.0, target=(5.0, 1.0))
    sample['turret_camera']['forward'] = [1.0, 0.0, 0.0]
    right, up = evaluator.sight_angles(sample, [0.0, -5.0, 0.34])  # 90 deg to the right
    assert right == pytest.approx(math.pi / 2, abs=0.02) and up == pytest.approx(0.0, abs=1e-6)
    assert evaluator.steer_rates(sample, [0.0, -5.0, 0.34]) == pytest.approx((1.0, 0.0), abs=1e-6)
    pan, tilt = evaluator.steer_rates(sample, [5.0, 0.0, 0.34 + 5.0 * math.tan(0.1)])
    assert pan == pytest.approx(0.0, abs=0.03) and tilt == pytest.approx(0.2, abs=0.01)
    # Out of the image but in sight and in reach: the operator takes over.
    sample['targets'][0]['centroid'] = [0.0, -5.0, 0.34]
    assert evaluator.lock_trouble(sample, {'mode': 'auto', 'locked': False}, 'person_1') == 'lost'
    # Behind the car: beyond the pan limit, nothing to do.
    sample['targets'][0]['centroid'] = [-5.0, 0.0, 0.34]
    assert not evaluator.reachable(sample, sample['targets'][0])
    assert evaluator.lock_trouble(sample, {'mode': 'auto', 'locked': False}, 'person_1') is None
