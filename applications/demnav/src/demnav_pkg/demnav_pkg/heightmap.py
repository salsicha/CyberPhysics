import math
import os
import numpy as np

METERS_PER_DEG_LAT = 111320.0


class HeightMapCache:
    """Downloads and caches an SRTM DEM grid for a given area."""

    def __init__(self, lat: float, lon: float, area_km: float,
                 resolution_m: float = 30.0, cache_dir: str = '/data/demnav_cache',
                 synthetic: bool = False):
        self.resolution_m = resolution_m
        self.center_lat = lat
        self.center_lon = lon
        self.synthetic = synthetic
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
            f'dem_{"synthetic" if synthetic else "srtm"}_'
            f'{lat:.4f}_{lon:.4f}_{area_km:.1f}_{resolution_m:.0f}.npy')

        if os.path.exists(cache_file):
            self.grid = np.load(cache_file)
        elif synthetic:
            self.grid = self._generate_synthetic(n, n)
            np.save(cache_file, self.grid)
        else:
            self.grid = self._download(n, n)
            np.save(cache_file, self.grid)

    def _generate_synthetic(self, n_rows: int, n_cols: int) -> np.ndarray:
        lats = np.linspace(self.max_lat, self.min_lat, n_rows)
        lons = np.linspace(self.min_lon, self.max_lon, n_cols)
        lon_grid, lat_grid = np.meshgrid(lons, lats)
        metres_per_deg_lon = (
            METERS_PER_DEG_LAT * math.cos(math.radians(self.center_lat)))
        east = (lon_grid - self.center_lon) * metres_per_deg_lon
        north = (lat_grid - self.center_lat) * METERS_PER_DEG_LAT
        return _synthetic_terrain_height(east, north).astype(np.float32)

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
                  width_m: float, height_m: float
                  ) -> tuple[np.ndarray, float, tuple[int, int]]:
        """Return (patch, resolution, (center_row, center_col)) where the
        center indices locate (lat, lon) within the returned patch."""
        row, col = self._latlon_to_pixel(lat, lon)
        hw = int(width_m / (2.0 * self.resolution_m))
        hh = int(height_m / (2.0 * self.resolution_m))
        r0 = max(0, row - hh)
        r1 = min(self.grid.shape[0], row + hh)
        c0 = max(0, col - hw)
        c1 = min(self.grid.shape[1], col + hw)
        return (self.grid[r0:r1, c0:c1].copy(), self.resolution_m,
                (row - r0, col - c0))

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


def _synthetic_terrain_height(east, north):
    ridge = 48.0 * np.exp(-((north - 0.18 * east - 120.0) / 420.0) ** 2)
    hill_shape = ((east + 240.0) / 380.0) ** 2
    hill_shape += ((north - 80.0) / 300.0) ** 2
    hill = 36.0 * np.exp(-hill_shape)
    marker_rise = 65.0 * np.exp(-((east - 45.0) / 115.0) ** 2 -
                                 ((north + 55.0) / 85.0) ** 2)
    marker_cut = -42.0 * np.exp(-((east + 120.0) / 75.0) ** 2 -
                                ((north - 95.0) / 105.0) ** 2)
    coast_line = -420.0 + 45.0 * np.sin(east / 210.0)
    coast_drop = np.where(north < coast_line, -22.0, 0.0)
    texture = 4.0 * np.sin(east / 95.0) * np.cos(north / 120.0)
    return ridge + hill + marker_rise + marker_cut + coast_drop + texture
