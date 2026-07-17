import numpy as np
import pytest

from wildnav_pkg.satellite_cache import SatelliteTileCache, latlon_to_tile

ORIGIN = (37.9234, -122.5967)
SYNTH = 'synthetic://airplane'
REAL = 'https://tiles.example/{z}/{x}/{y}'


def origin_tile(zoom=17):
    x, y = latlon_to_tile(*ORIGIN, zoom)
    return int(x), int(y)


def test_synthetic_and_real_tiles_never_collide(tmp_path):
    synth = SatelliteTileCache(str(tmp_path), SYNTH, 17,
                               origin_lat=ORIGIN[0], origin_lon=ORIGIN[1])
    real = SatelliteTileCache(str(tmp_path), REAL, 17)
    x, y = origin_tile()
    assert synth._tile_stem(x, y) != real._tile_stem(x, y)


def test_origin_is_part_of_synthetic_cache_key(tmp_path):
    cache = SatelliteTileCache(str(tmp_path), SYNTH, 17,
                               origin_lat=ORIGIN[0], origin_lon=ORIGIN[1])
    x, y = origin_tile()
    assert cache.get_features(x, y) is not None
    assert cache.is_cached(x, y)
    cache.set_origin(38.0, -122.0)
    # Tiles anchored to the old origin must not be served for the new one.
    assert not cache.is_cached(x, y)


def test_real_cache_key_unchanged_for_existing_caches(tmp_path):
    # Pre-rename real-imagery caches keep working: the key format for real
    # tiles is still '{zoom}_{x}_{y}'.
    cache = SatelliteTileCache(str(tmp_path), REAL, 17)
    assert cache._tile_stem(100, 200) == '17_100_200'


def test_synthetic_tile_generation_and_feature_cache(tmp_path):
    cache = SatelliteTileCache(str(tmp_path), SYNTH, 17,
                               origin_lat=ORIGIN[0], origin_lon=ORIGIN[1])
    x, y = origin_tile()
    features = cache.get_features(x, y)
    assert features is not None
    points, descriptors, shape = features
    assert shape == (256, 256)
    assert len(points) >= 4
    # Second call must hit the .npz feature cache and agree exactly.
    again = cache.get_features(x, y)
    np.testing.assert_array_equal(points, again[0])
    np.testing.assert_array_equal(descriptors, again[1])


def test_candidate_tiles_centered_and_bounded(tmp_path):
    cache = SatelliteTileCache(str(tmp_path), SYNTH, 17,
                               origin_lat=ORIGIN[0], origin_lon=ORIGIN[1])
    candidates = cache.candidate_tiles(*ORIGIN, 250.0, 9)
    assert 1 <= len(candidates) <= 9
    assert origin_tile() == candidates[0]
