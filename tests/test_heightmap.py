import math

import numpy as np
import pytest

import synthetic_world as sw
from demnav_pkg.heightmap import HeightMapCache

ORIGIN = (37.9234, -122.5967)


def node_latlon(cache, row, col):
    n_rows, n_cols = cache.grid.shape
    lat = np.linspace(cache.max_lat, cache.min_lat, n_rows)[row]
    lon = np.linspace(cache.min_lon, cache.max_lon, n_cols)[col]
    return lat, lon


def test_synthetic_dem_is_origin_anchored(tmp_path):
    # Load the map 230 m away from the origin: every node must still
    # evaluate the origin-anchored world (the world must not translate).
    cache = HeightMapCache(37.9250, -122.5950, 2.0, 30.0, str(tmp_path),
                           True, origin_lat=ORIGIN[0], origin_lon=ORIGIN[1])
    for row in (0, 30, 65):
        for col in (0, 33, 65):
            lat, lon = node_latlon(cache, row, col)
            east, north = sw.latlon_to_local(lat, lon, *ORIGIN)
            expected = float(sw.terrain_height(east, north))
            assert float(cache.grid[row, col]) == pytest.approx(
                expected, abs=1e-3)


def test_synthetic_cache_key_includes_origin(tmp_path):
    a = HeightMapCache(*ORIGIN, 2.0, 30.0, str(tmp_path), True,
                       origin_lat=ORIGIN[0], origin_lon=ORIGIN[1])
    b = HeightMapCache(*ORIGIN, 2.0, 30.0, str(tmp_path), True,
                       origin_lat=38.0, origin_lon=-122.0)
    files = sorted(p.name for p in tmp_path.glob('*.npy'))
    assert len(files) == 2, files
    assert not np.array_equal(a.grid, b.grid)


def test_legacy_srtm_cache_fallback(tmp_path):
    legacy = tmp_path / 'dem_37.9234_-122.5967_2.0_30.npy'
    np.save(legacy, np.full((5, 5), 7.0, dtype=np.float32))
    cache = HeightMapCache(*ORIGIN, 2.0, 30.0, str(tmp_path), False)
    assert cache.grid.shape == (5, 5)
    assert float(cache.grid[0, 0]) == 7.0


def test_pixel_mapping_is_subpixel_and_matches_grid_nodes(tmp_path):
    cache = HeightMapCache(*ORIGIN, 2.0, 30.0, str(tmp_path), True)
    # Grid nodes come back as exact integers, both ways.
    for row, col in ((0, 0), (17, 40), (65, 65)):
        lat, lon = node_latlon(cache, row, col)
        assert cache.latlon_to_pixel(lat, lon) == pytest.approx(
            (row, col), abs=1e-6)
        assert cache.pixel_to_latlon(row, col) == pytest.approx(
            (lat, lon), abs=1e-9)
    # A point 12 m east and 7 m south of a node is a fraction of a cell
    # away (the old int() truncation dropped this).
    lat, lon = node_latlon(cache, 20, 30)
    lat2, lon2 = sw.local_to_latlon(12.0, -7.0, lat, lon)
    row, col = cache.latlon_to_pixel(lat2, lon2)
    assert row == pytest.approx(20 + 7.0 / cache.row_spacing_m, abs=1e-3)
    assert col == pytest.approx(30 + 12.0 / cache.col_spacing_m, abs=1e-3)


def test_node_spacing_matches_map_extent(tmp_path):
    cache = HeightMapCache(*ORIGIN, 2.0, 30.0, str(tmp_path), True)
    n = cache.grid.shape[0]
    assert cache.row_spacing_m == pytest.approx(2000.0 / (n - 1))
    assert cache.col_spacing_m == pytest.approx(2000.0 / (n - 1))


def test_elevation_is_bilinear_between_nodes(tmp_path):
    cache = HeightMapCache(*ORIGIN, 2.0, 30.0, str(tmp_path), True)
    lat0, lon0 = node_latlon(cache, 10, 20)
    lat1, lon1 = node_latlon(cache, 11, 21)
    g = cache.grid.astype(np.float64)
    expected = 0.25 * (g[10, 20] + g[10, 21] + g[11, 20] + g[11, 21])
    assert cache.elevation_at((lat0 + lat1) / 2, (lon0 + lon1) / 2) == \
        pytest.approx(expected, abs=1e-3)
    assert math.isnan(cache.elevation_at(cache.max_lat + 0.01, lon0))


def test_get_patch_locates_query_point_independently(tmp_path):
    cache = HeightMapCache(*ORIGIN, 2.0, 30.0, str(tmp_path), True)
    # A point off the grid nodes, located against independently computed
    # node coordinates rather than the function's own output.
    lat, lon = sw.local_to_latlon(37.0, -52.0, *ORIGIN)
    patch, spacing, (row, col) = cache.get_patch(lat, lon, 600.0, 600.0)
    assert spacing == pytest.approx(cache.col_spacing_m)
    lats = np.linspace(cache.max_lat, cache.min_lat, cache.grid.shape[0])
    lons = np.linspace(cache.min_lon, cache.max_lon, cache.grid.shape[1])
    full_row = (lats[0] - lat) / (lats[0] - lats[1])
    full_col = (lon - lons[0]) / (lons[1] - lons[0])
    r0 = int(np.floor(full_row)) - int(np.ceil(300.0 / cache.row_spacing_m))
    c0 = int(np.floor(full_col)) - int(np.ceil(300.0 / cache.col_spacing_m))
    assert (row, col) == pytest.approx((full_row - r0, full_col - c0),
                                       abs=1e-6)
    assert row != int(row) and col != int(col)
    np.testing.assert_array_equal(
        patch, cache.grid[r0:r0 + patch.shape[0], c0:c0 + patch.shape[1]])
    # The window covers +-300 m around the point.
    assert row * spacing >= 300.0 and (patch.shape[0] - 1 - row) * spacing >= 300.0
    assert col * spacing >= 300.0 and (patch.shape[1] - 1 - col) * spacing >= 300.0


def test_get_patch_refuses_windows_crossing_the_edge(tmp_path):
    cache = HeightMapCache(*ORIGIN, 2.0, 30.0, str(tmp_path), True)
    # 800 m east of centre, a +-300 m window ends 100 m past the edge;
    # clamping it used to shift the reported match position.
    lat, lon = sw.local_to_latlon(800.0, 0.0, *ORIGIN)
    patch, _, _ = cache.get_patch(lat, lon, 600.0, 600.0)
    assert patch.size == 0
    assert cache.edge_distance_m(lat, lon) == pytest.approx(200.0, abs=1.0)
    patch, _, _ = cache.get_patch(lat, lon, 300.0, 300.0)
    assert patch.size > 0


def test_read_only_cache_still_builds_map(tmp_path):
    read_only = tmp_path / 'ro'
    read_only.mkdir()
    read_only.chmod(0o555)
    try:
        cache = HeightMapCache(*ORIGIN, 2.0, 30.0, str(read_only), True)
    finally:
        read_only.chmod(0o755)
    assert cache.grid.shape[0] > 2
