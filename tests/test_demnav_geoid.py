"""GeographicLib PGM geoid reader used to put GPS altitudes on the DEM's
mean-sea-level datum."""
import os

import numpy as np
import pytest

from demnav_pkg.geoid import GeoidGrid

EGM96 = '/usr/share/GeographicLib/geoids/egm96-5.pgm'


def write_pgm(path, samples, offset=-108.0, scale=0.003, extra=b''):
    height, width = samples.shape
    header = (b'P5\n# Geoid file in PGM format for the GeographicLib::Geoid '
              b'class\n# Offset %g\n# Scale %g\n' % (offset, scale) + extra +
              b'%d %d\n65535\n' % (width, height))
    path.write_bytes(header + samples.astype('>u2').tobytes())
    return str(path)


def heights(samples, offset=-108.0, scale=0.003):
    return offset + scale * samples.astype(np.float64)


@pytest.fixture
def grid(tmp_path):
    # 5 rows (90N..90S every 45 deg) x 6 columns (0E..300E every 60 deg).
    samples = np.arange(30, dtype=np.uint16).reshape(5, 6) * 1000 + 7
    return samples, GeoidGrid(write_pgm(tmp_path / 'g.pgm', samples))


def test_grid_nodes_follow_header_offset_and_scale(grid):
    samples, geoid = grid
    expected = heights(samples)
    assert geoid.height(90.0, 0.0) == pytest.approx(expected[0, 0])
    assert geoid.height(45.0, 60.0) == pytest.approx(expected[1, 1])
    assert geoid.height(-90.0, 300.0) == pytest.approx(expected[4, 5])
    # Western longitudes wrap to the eastern columns.
    assert geoid.height(0.0, -60.0) == pytest.approx(expected[2, 5])


def test_bilinear_between_nodes_and_across_the_antimeridian(grid):
    samples, geoid = grid
    h = heights(samples)
    assert geoid.height(22.5, 30.0) == pytest.approx(
        0.25 * (h[1, 0] + h[1, 1] + h[2, 0] + h[2, 1]))
    # 330E sits between the last column (300E) and column 0 (360E).
    assert geoid.height(45.0, -30.0) == pytest.approx(
        0.5 * (h[1, 5] + h[1, 0]))


def test_rejects_files_that_are_not_geoid_grids(tmp_path):
    bad = tmp_path / 'bad.pgm'
    bad.write_bytes(b'P2\n2 2\n255\n0 0 0 0\n')
    with pytest.raises(ValueError):
        GeoidGrid(str(bad))
    no_scale = tmp_path / 'no_scale.pgm'
    no_scale.write_bytes(b'P5\n# Offset -108\n2 2\n65535\n' + bytes(8))
    with pytest.raises(ValueError):
        GeoidGrid(str(no_scale))
    with pytest.raises(OSError):
        GeoidGrid(str(tmp_path / 'missing.pgm'))


@pytest.mark.skipif(not os.path.exists(EGM96),
                    reason='GeographicLib egm96-5 dataset not installed')
def test_egm96_matches_geographiclib_reference():
    geoid = GeoidGrid(EGM96)
    # GeoidEval -n egm96-5 reference values.
    for lat, lon, expected in ((37.9234, -122.5967, -32.3249),
                               (0.0, 0.0, 17.1630),
                               (51.5, -0.12, 45.9464)):
        assert geoid.height(lat, lon) == pytest.approx(expected, abs=0.15)
