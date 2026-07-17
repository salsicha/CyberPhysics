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


def test_get_patch_centers_on_query_point(tmp_path):
    cache = HeightMapCache(*ORIGIN, 2.0, 30.0, str(tmp_path), True)
    patch, resolution, (row, col) = cache.get_patch(*ORIGIN, 600.0, 600.0)
    assert resolution == 30.0
    assert 0 <= row < patch.shape[0]
    assert 0 <= col < patch.shape[1]
    # The value at the returned center indices matches the full grid.
    full_row, full_col = cache._latlon_to_pixel(*ORIGIN)
    assert patch[row, col] == cache.grid[full_row, full_col]
