import math

import numpy as np
import pytest

import synthetic_world as sw

RNG = np.random.default_rng(11)


def test_meters_per_degree_constant():
    assert sw.METERS_PER_DEG_LAT == 111320.0


def test_latlon_local_round_trip_scalars():
    origin = (37.9234, -122.5967)
    east, north = sw.latlon_to_local(37.93, -122.60, *origin)
    lat, lon = sw.local_to_latlon(east, north, *origin)
    assert lat == pytest.approx(37.93, abs=1e-12)
    assert lon == pytest.approx(-122.60, abs=1e-12)


def test_latlon_local_round_trip_arrays():
    origin = (37.9234, -122.5967)
    lats = RNG.uniform(37.85, 38.0, 100)
    lons = RNG.uniform(-122.7, -122.5, 100)
    east, north = sw.latlon_to_local(lats, lons, *origin)
    back_lat, back_lon = sw.local_to_latlon(east, north, *origin)
    np.testing.assert_allclose(back_lat, lats, atol=1e-12)
    np.testing.assert_allclose(back_lon, lons, atol=1e-12)


def test_terrain_scalar_matches_array():
    east = RNG.uniform(-6000, 6000, 50)
    north = RNG.uniform(-6000, 6000, 50)
    grid = sw.terrain_height(east, north)
    for i in range(50):
        assert float(sw.terrain_height(east[i], north[i])) == \
            pytest.approx(float(grid[i]), abs=1e-12)


def test_terrain_has_relief_and_bounded_range():
    east, north = np.meshgrid(np.linspace(-6000, 6000, 200),
                              np.linspace(-6000, 6000, 200))
    terrain = sw.terrain_height(east, north)
    # Enough relief for NCC matching, within the amplitudes of the field.
    assert terrain.std() > 5.0
    assert terrain.min() > -100.0
    assert terrain.max() < 200.0


def test_satellite_rgb_shape_dtype_determinism():
    east, north = np.meshgrid(np.linspace(-500, 500, 64),
                              np.linspace(-500, 500, 48))
    a = sw.satellite_rgb(east, north)
    b = sw.satellite_rgb(east, north)
    assert a.shape == (48, 64, 3)
    assert a.dtype == np.uint8
    np.testing.assert_array_equal(a, b)


def test_satellite_rgb_has_texture():
    east, north = np.meshgrid(np.linspace(-2000, 2000, 256),
                              np.linspace(-2000, 2000, 256))
    image = sw.satellite_rgb(east, north)
    # Multiple distinct classes must appear (fields/roads/trail at least).
    assert len(np.unique(image.reshape(-1, 3), axis=0)) > 10
