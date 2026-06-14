import math
import os
import numpy as np

METERS_PER_DEG_LAT = 111320.0


class HeightMapCache:
    """Downloads and caches an SRTM DEM grid for a given area."""

    def __init__(self, lat: float, lon: float, area_km: float,
                 resolution_m: float = 30.0, cache_dir: str = '/data/demnav_cache'):
        self.resolution_m = resolution_m
        self.center_lat = lat
        self.center_lon = lon
        os.makedirs(cache_dir, exist_ok=True)

        half_m = area_km * 500.0
        m_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(lat))
        self.max_lat = lat + half_m / METERS_PER_DEG_LAT
        self.min_lat = lat - half_m / METERS_PER_DEG_LAT
        self.min_lon = lon - half_m / m_per_deg_lon
        self.max_lon = lon + half_m / m_per_deg_lon

        n = max(2, int(area_km * 1000.0 / resolution_m))
        cache_file = os.path.join(
            cache_dir,
            f'dem_{lat:.4f}_{lon:.4f}_{area_km:.1f}_{resolution_m:.0f}.npy')

        if os.path.exists(cache_file):
            self.grid = np.load(cache_file)
        else:
            self.grid = self._download(n, n)
            np.save(cache_file, self.grid)

    def _download(self, n_rows: int, n_cols: int) -> np.ndarray:
        import srtm
        data = srtm.get_data()
        lats = np.linspace(self.max_lat, self.min_lat, n_rows)
        lons = np.linspace(self.min_lon, self.max_lon, n_cols)
        grid = np.zeros((n_rows, n_cols), dtype=np.float32)
        for r, rlat in enumerate(lats):
            for c, clon in enumerate(lons):
                h = data.get_elevation(float(rlat), float(clon))
                grid[r, c] = float(h) if h is not None else 0.0
        return grid

    def get_patch(self, lat: float, lon: float,
                  width_m: float, height_m: float) -> tuple[np.ndarray, float]:
        row, col = self._latlon_to_pixel(lat, lon)
        hw = int(width_m / (2.0 * self.resolution_m))
        hh = int(height_m / (2.0 * self.resolution_m))
        r0 = max(0, row - hh)
        r1 = min(self.grid.shape[0], row + hh)
        c0 = max(0, col - hw)
        c1 = min(self.grid.shape[1], col + hw)
        return self.grid[r0:r1, c0:c1].copy(), self.resolution_m

    def latlon_to_local(self, lat: float, lon: float) -> tuple[float, float]:
        """Return east/north metres from the height-map centre."""
        metres_per_deg_lon = (
            METERS_PER_DEG_LAT * math.cos(math.radians(self.center_lat)))
        east = (lon - self.center_lon) * metres_per_deg_lon
        north = (lat - self.center_lat) * METERS_PER_DEG_LAT
        return east, north

    def _latlon_to_pixel(self, lat: float, lon: float) -> tuple[int, int]:
        row = int((self.max_lat - lat) / (self.max_lat - self.min_lat) * (self.grid.shape[0] - 1))
        col = int((lon - self.min_lon) / (self.max_lon - self.min_lon) * (self.grid.shape[1] - 1))
        row = max(0, min(self.grid.shape[0] - 1, row))
        col = max(0, min(self.grid.shape[1] - 1, col))
        return row, col
