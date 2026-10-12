"""tools/racecar_turret_map_coverage.py: obstacle outlines, the grids of
nvblox's map slice and Nav2's costmap, and the coverage and phantom scores
against systems/racecar_turret/scenarios/isaac_arena.json; and its recorder
in the evaluation profile of compositions/racecar_turret_isaac.yaml."""
import json
import math

import numpy as np
import pytest
import yaml

from conftest import REPO, load_script

cov = load_script(REPO / 'tools/racecar_turret_map_coverage.py', 'racecar_turret_map_coverage')
SCENARIO = json.loads((REPO / 'systems/racecar_turret/scenarios/isaac_arena.json').read_text())


def test_outlines_lie_on_the_footprints():
    for entry in SCENARIO['obstacles']:
        obstacle = cov.rts.Obstacle.from_json(entry)
        points = cov.outline_points(obstacle, 0.05)
        assert np.abs(cov.obstacle_distance(points, obstacle)).max() < 1e-9
        # Just outside the outline is outside the footprint.
        centre = obstacle.center
        outward = points + 0.02 * (points - centre) / np.linalg.norm(points - centre, axis=1)[:, None]
        assert (cov.obstacle_distance(outward, obstacle) > 0.0).all()
        spacing = np.linalg.norm(np.diff(points, axis=0), axis=1)
        assert spacing.max() <= 0.05 + 1e-9
    walls = cov.inner_wall_points(SCENARIO['arena'])
    (x0, y0), (x1, y1) = SCENARIO['arena']['min_xy'], SCENARIO['arena']['max_xy']
    on_edge = (np.isclose(walls[:, 0], x0) | np.isclose(walls[:, 0], x1)
               | np.isclose(walls[:, 1], y0) | np.isclose(walls[:, 1], y1))
    assert on_edge.all() and len(walls) == pytest.approx(2 * (20 + 15) / 0.05, abs=4)


def test_grid_cells_are_centred_and_transformed():
    data = np.arange(6)
    centres, values = cov.grid_cells(data, (1.0, 2.0), 0.5, 3, 2)
    assert centres[0] == pytest.approx((1.25, 2.25)) and centres[5] == pytest.approx((2.25, 2.75))
    assert values.tolist() == list(range(6))
    rotated, _ = cov.grid_cells(data, (1.0, 2.0), 0.5, 3, 2, (10.0, 0.0, math.pi / 2))
    assert rotated[0] == pytest.approx((10.0 - 2.25, 1.25))


def test_near_finds_cells_within_tolerance_only():
    cells = np.array([[0.0, 0.0], [5.0, 5.0]])
    points = np.array([[0.1, 0.0], [0.0, 0.2], [5.0, 5.14], [3.0, 3.0]])
    assert cov.near(points, cells, 0.15).tolist() == [True, False, True, False]
    assert not cov.near(points, np.empty((0, 2)), 0.15).any()


def slice_snapshot(occupied_xy, known_xy=(), resolution=0.05, origin=(-4.0, -4.0),
                   shape=(400, 440), transform=(0.0, 0.0, 0.0), people=()):
    """An nvblox slice: 1 m everywhere known, 0 at occupied cells, the
    unknown value elsewhere."""
    unknown = 1000.0
    data = np.full(shape, unknown, dtype=np.float32)

    def index(x, y):
        return int((y - origin[1]) / resolution), int((x - origin[0]) / resolution)

    for x, y in known_xy:
        data[index(x, y)] = 1.0
    for x, y in occupied_xy:
        data[index(x, y)] = 0.0
    return {'slice_data': data.reshape(-1), 'slice_origin': np.array(origin),
            'slice_resolution': np.float64(resolution), 'slice_width': np.int64(shape[1]),
            'slice_height': np.int64(shape[0]), 'slice_unknown': np.float32(unknown),
            'slice_transform': np.array(transform),
            'people': np.array([(0.0, x, y) for x, y in people]).reshape(-1, 3)}


def test_an_obstacle_seen_once_counts_and_free_space_over_it_is_reported():
    crate = cov.rts.Obstacle.from_json(
        next(o for o in SCENARIO['obstacles'] if o['name'] == 'island_crate_1'))
    outline = cov.outline_points(crate)
    seen = slice_snapshot(outline)
    carved = slice_snapshot([], known_xy=cov.outline_points(
        cov.rts.Obstacle.from_json(next(o for o in SCENARIO['obstacles']
                                        if o['name'] == 'island_barrel'))))
    result = cov.score([seen, carved], SCENARIO)['nvblox']
    assert result['obstacles']['island_crate_1'] == 1.0
    assert result['obstacles']['island_barrel'] == 0.0
    assert result['obstacles']['pillar_south'] == 0.0
    # The crate is not in the last snapshot; the barrel was observed free.
    assert result['outline_occupied_last'] == 0.0
    assert result['observed_not_occupied'] > 0.0
    assert result['phantom_m2_inside_arena'] == 0.0


def test_phantoms_exclude_obstacles_and_where_people_walked():
    snapshot = slice_snapshot([(1.0, 1.0), (1.5, -1.0), (8.0, 8.0), (2.0, -20.0)],
                              shape=(700, 440), origin=(-4.0, -22.0), people=[(1.5, -1.1)])
    result = cov.score([snapshot], SCENARIO)['nvblox']
    # (1, 1) and (8, 8) are open floor, (1.5, -1) where a person walked,
    # (2, -20) outside the arena.
    assert result['phantom_m2_inside_arena'] == pytest.approx(2 * 0.05 ** 2)
    assert result['phantom_m2_outside_arena'] == pytest.approx(0.05 ** 2)


def test_the_slice_is_placed_in_map_by_its_transform():
    pillar = cov.rts.Obstacle.from_json(
        next(o for o in SCENARIO['obstacles'] if o['name'] == 'pillar_south'))
    # The slice frame (odom) is 0.5 m east of map: the outline sits 0.5 m
    # west in odom coordinates.
    shifted = cov.outline_points(pillar) - np.array([0.5, 0.0])
    result = cov.score([slice_snapshot(shifted, origin=(-4.0, -4.0), transform=(0.5, 0.0, 0.0))],
                       SCENARIO)['nvblox']
    assert result['obstacles']['pillar_south'] == 1.0
    assert result['phantom_m2_inside_arena'] == 0.0


def test_costmap_lethal_cells_score_as_occupied():
    resolution, origin, width, height = 0.05, (-4.0, -4.0), 440, 360
    data = np.full((height, width), -1, dtype=np.int8)
    walls = cov.inner_wall_points(SCENARIO['arena'])
    for x, y in walls[walls[:, 1] == SCENARIO['arena']['min_xy'][1]]:
        data[int((y - origin[1]) / resolution), min(int((x - origin[0]) / resolution), width - 1)] = 100
    data[100, 100] = 99  # inscribed, not lethal
    snapshot = {'costmap_data': data.reshape(-1), 'costmap_origin': np.array(origin),
                'costmap_resolution': np.float64(resolution), 'costmap_width': np.int64(width),
                'costmap_height': np.int64(height), 'people': np.empty((0, 3))}
    result = cov.score([snapshot], SCENARIO)
    assert result['nvblox'] is None
    south = 20.0 / (2 * (20 + 15))
    assert result['costmap']['obstacles']['arena_walls'] == pytest.approx(south, abs=0.02)
    assert result['costmap']['phantom_m2_inside_arena'] == 0.0


def test_snapshots_load_skipping_truncated_files(tmp_path):
    np.savez_compressed(tmp_path / 'snapshot_0001.npz', time=np.float64(1.0))
    (tmp_path / 'snapshot_0002.npz').write_bytes(b'PK\x03\x04truncated')
    assert [float(s['time']) for s in cov.load_snapshots(tmp_path)] == [1.0]


def test_evaluation_profile_records_the_maps():
    services = yaml.safe_load((REPO / 'compositions/racecar_turret_isaac.yaml').read_text())['services']
    recorder = services['racecar_turret_map_recorder']
    assert recorder['profiles'] == ['evaluation']
    assert 'nvblox' in recorder['image'] and 'use_sim_time:=true' in recorder['command']
    assert '--output-dir /out/maps' in recorder['command']
    assert any(v.endswith(':/out') and 'RACECAR_TURRET_OUTPUT' in v for v in recorder['volumes'])
    script = (REPO / 'tools/racecar_turret_isaac_test.sh').read_text()
    assert 'racecar_turret_map_coverage.py score' in script and 'mkdir -p "$run_dir/maps"' in script
