import os

import cv2
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
    # A second cache on the same directory must read the .npz feature
    # cache and agree exactly.
    again = SatelliteTileCache(str(tmp_path), SYNTH, 17,
                               origin_lat=ORIGIN[0], origin_lon=ORIGIN[1])
    again = again.get_features(x, y)
    np.testing.assert_array_equal(points, again[0])
    np.testing.assert_array_equal(descriptors, again[1])
    assert again[2] == shape


def test_features_stay_in_memory_between_matches(tmp_path):
    cache = SatelliteTileCache(str(tmp_path), SYNTH, 17, origin_lat=ORIGIN[0],
                               origin_lon=ORIGIN[1], memory_tiles=2)
    x, y = origin_tile()
    first = cache.get_features(x, y)
    for name in os.listdir(tmp_path):
        os.remove(tmp_path / name)
    # Served from memory: nothing read back (or regenerated) from disk.
    assert cache.get_features(x, y, allow_download=False) is first
    # Bounded, least recently used out first.
    cache.get_features(x + 1, y)
    cache.get_features(x, y, allow_download=False)
    cache.get_features(x + 2, y)
    for name in os.listdir(tmp_path):
        os.remove(tmp_path / name)
    assert cache.get_features(x, y, allow_download=False) is first
    assert cache.get_features(x + 2, y, allow_download=False) is not None
    assert cache.get_features(x + 1, y, allow_download=False) is None


def test_memory_can_be_turned_off(tmp_path):
    cache = SatelliteTileCache(str(tmp_path), SYNTH, 17, origin_lat=ORIGIN[0],
                               origin_lon=ORIGIN[1], memory_tiles=0)
    x, y = origin_tile()
    first = cache.get_features(x, y)
    again = cache.get_features(x, y)
    assert again is not first
    np.testing.assert_array_equal(first[1], again[1])


def test_a_new_origin_does_not_reuse_remembered_synthetic_tiles(tmp_path):
    cache = SatelliteTileCache(str(tmp_path), SYNTH, 17,
                               origin_lat=ORIGIN[0], origin_lon=ORIGIN[1])
    x, y = origin_tile()
    first = cache.get_features(x, y)
    cache.set_origin(38.0, -122.0)
    assert cache.get_features(x, y, allow_download=False) is None
    assert cache.get_features(x, y) is not first


def test_missing_tile_is_not_read_when_downloads_are_off(tmp_path, monkeypatch):
    cache = SatelliteTileCache(str(tmp_path), REAL, 17)
    reads = []
    monkeypatch.setattr(cv2, 'imread', lambda *args: reads.append(args))
    assert cache.get_features(100, 200, allow_download=False) is None
    assert reads == []


def test_candidate_tiles_centered_and_bounded(tmp_path):
    cache = SatelliteTileCache(str(tmp_path), SYNTH, 17,
                               origin_lat=ORIGIN[0], origin_lon=ORIGIN[1])
    candidates = cache.candidate_tiles(*ORIGIN, 250.0, 9)
    assert 1 <= len(candidates) <= 9
    assert origin_tile() == candidates[0]
