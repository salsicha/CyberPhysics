"""Geoid heights from a GeographicLib PGM grid (e.g. egm96-5.pgm).

SRTM elevations are EGM96 mean-sea-level heights, while MAVROS reports GPS
altitude above the WGS84 ellipsoid; the two differ by the geoid height N
(h_ellipsoid = H_msl + N, about -32 m at Mount Tamalpais). The PGM files
installed for MAVROS by install_geographiclib_datasets.sh are read directly
so no GeographicLib binding is needed.
"""
import math

import numpy as np


class GeoidGrid:
    """Bilinearly interpolated geoid heights from a GeographicLib PGM.

    The format is a binary PGM (P5) whose comment header carries
    "# Offset" and "# Scale" lines; samples are big-endian uint16 on a
    regular grid starting at 90N 0E, rows running south and columns east,
    with the first and last rows at the poles. height = offset + scale *
    sample. Bilinear interpolation of egm96-5 is within 0.14 m of the
    reference (GeographicLib's MaxBilinearError).
    """

    def __init__(self, path: str):
        offset = scale = None
        with open(path, 'rb') as stream:
            if stream.readline().strip() != b'P5':
                raise ValueError(f'{path} is not a binary PGM')
            fields = []
            while len(fields) < 3:
                line = stream.readline()
                if not line:
                    raise ValueError(f'{path}: truncated PGM header')
                if line.startswith(b'#'):
                    words = line[1:].split()
                    if len(words) >= 2 and words[0] == b'Offset':
                        offset = float(words[1])
                    elif len(words) >= 2 and words[0] == b'Scale':
                        scale = float(words[1])
                    continue
                fields += line.split()
            data_offset = stream.tell()
        width, height, maxval = (int(value) for value in fields[:3])
        if offset is None or scale is None:
            raise ValueError(f'{path}: missing Offset/Scale header')
        if maxval != 65535 or width < 2 or height < 2:
            raise ValueError(f'{path}: not a GeographicLib geoid grid')
        self.offset = offset
        self.scale = scale
        self.grid = np.memmap(path, dtype='>u2', mode='r',
                              offset=data_offset, shape=(height, width))
        self.lat_step = 180.0 / (height - 1)
        self.lon_step = 360.0 / width

    def height(self, lat: float, lon: float) -> float:
        """Geoid height N in metres above the WGS84 ellipsoid."""
        n_rows, n_cols = self.grid.shape
        row = (90.0 - max(-90.0, min(90.0, lat))) / self.lat_step
        col = (lon % 360.0) / self.lon_step
        r0 = min(int(math.floor(row)), n_rows - 2)
        c0 = int(math.floor(col))
        fr = row - r0
        fc = col - c0
        c0 %= n_cols
        c1 = (c0 + 1) % n_cols
        g = self.grid
        top = float(g[r0, c0]) * (1.0 - fc) + float(g[r0, c1]) * fc
        bottom = float(g[r0 + 1, c0]) * (1.0 - fc) + float(g[r0 + 1, c1]) * fc
        return self.offset + self.scale * (top * (1.0 - fr) + bottom * fr)
