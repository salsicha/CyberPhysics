import math
import os
import numpy as np

from synthetic_world import METERS_PER_DEG_LAT, terrain_height


class HeightMapCache:
    """Downloads and caches an SRTM DEM grid for a given area."""

    def __init__(self, lat: float, lon: float, area_km: float,
                 resolution_m: float = 30.0, cache_dir: str = '/data/demnav_cache',
                 synthetic: bool = False,
                 origin_lat: float | None = None,
                 origin_lon: float | None = None):
        self.resolution_m = resolution_m
        self.center_lat = lat
        self.center_lon = lon
        self.synthetic = synthetic
        # The synthetic world is anchored at a fixed origin so the terrain
        # does not translate with wherever the map happens to be loaded —
        # it must line up with the imagery the camera sim renders about the
        # same origin.
        self.origin_lat = lat if origin_lat is None else float(origin_lat)
        self.origin_lon = lon if origin_lon is None else float(origin_lon)
        os.makedirs(cache_dir, exist_ok=True)

        half_m = area_km * 500.0
        m_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(lat))
        self.max_lat = lat + half_m / METERS_PER_DEG_LAT
        self.min_lat = lat - half_m / METERS_PER_DEG_LAT
        self.min_lon = lon - half_m / m_per_deg_lon
        self.max_lon = lon + half_m / m_per_deg_lon

        n = max(2, int(area_km * 1000.0 / resolution_m))
        if synthetic:
            cache_file = os.path.join(
                cache_dir,
                f'dem_synthetic_{self.origin_lat:.4f}_{self.origin_lon:.4f}_'
                f'{lat:.4f}_{lon:.4f}_{area_km:.1f}_{resolution_m:.0f}.npy')
        else:
            cache_file = os.path.join(
                cache_dir,
                f'dem_srtm_{lat:.4f}_{lon:.4f}_{area_km:.1f}_{resolution_m:.0f}.npy')
        # SRTM caches seeded before the source prefix was added.
        legacy_file = os.path.join(
            cache_dir,
            f'dem_{lat:.4f}_{lon:.4f}_{area_km:.1f}_{resolution_m:.0f}.npy')

        if os.path.exists(cache_file):
            self.grid = np.load(cache_file)
        elif not synthetic and os.path.exists(legacy_file):
            self.grid = np.load(legacy_file)
        elif synthetic:
            self.grid = self._generate_synthetic(n, n)
            self._save(cache_file)
        else:
            self.grid = self._download(n, n)
            self._save(cache_file)

        # Grid nodes span [min, max] inclusively, so the node spacing is
        # extent / (n - 1), not area / n.
        n_rows, n_cols = self.grid.shape
        self.half_extent_m = half_m
        self.row_spacing_m = 2.0 * half_m / max(1, n_rows - 1)
        self.col_spacing_m = 2.0 * half_m / max(1, n_cols - 1)

    def _save(self, cache_file: str):
        # The cache is only an optimisation: a read-only cache volume must
        # not stop a re-centred map from being used.
        try:
            np.save(cache_file, self.grid)
        except OSError:
            pass

    def _generate_synthetic(self, n_rows: int, n_cols: int) -> np.ndarray:
        lats = np.linspace(self.max_lat, self.min_lat, n_rows)
        lons = np.linspace(self.min_lon, self.max_lon, n_cols)
        lon_grid, lat_grid = np.meshgrid(lons, lats)
        metres_per_deg_lon = (
            METERS_PER_DEG_LAT * math.cos(math.radians(self.origin_lat)))
        east = (lon_grid - self.origin_lon) * metres_per_deg_lon
        north = (lat_grid - self.origin_lat) * METERS_PER_DEG_LAT
        return terrain_height(east, north).astype(np.float32)

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

    def latlon_to_pixel(self, lat: float, lon: float) -> tuple[float, float]:
        """Fractional (row, col) of lat/lon; integers are grid nodes. Not
        clamped: values outside [0, n - 1] lie off the map."""
        row = ((self.max_lat - lat) / (self.max_lat - self.min_lat) *
               (self.grid.shape[0] - 1))
        col = ((lon - self.min_lon) / (self.max_lon - self.min_lon) *
               (self.grid.shape[1] - 1))
        return float(row), float(col)

    def pixel_to_latlon(self, row: float, col: float) -> tuple[float, float]:
        """Inverse of latlon_to_pixel."""
        lat = self.max_lat - row / (self.grid.shape[0] - 1) * (
            self.max_lat - self.min_lat)
        lon = self.min_lon + col / (self.grid.shape[1] - 1) * (
            self.max_lon - self.min_lon)
        return float(lat), float(lon)

    def elevation_at(self, lat: float, lon: float) -> float:
        """Bilinearly interpolated height, NaN off the map."""
        row, col = self.latlon_to_pixel(lat, lon)
        n_rows, n_cols = self.grid.shape
        if not (0.0 <= row <= n_rows - 1 and 0.0 <= col <= n_cols - 1):
            return float('nan')
        r0 = min(int(math.floor(row)), n_rows - 2)
        c0 = min(int(math.floor(col)), n_cols - 2)
        fr = row - r0
        fc = col - c0
        cell = self.grid[r0:r0 + 2, c0:c0 + 2].astype(np.float64)
        top = cell[0, 0] * (1.0 - fc) + cell[0, 1] * fc
        bottom = cell[1, 0] * (1.0 - fc) + cell[1, 1] * fc
        return float(top * (1.0 - fr) + bottom * fr)

    def edge_distance_m(self, lat: float, lon: float) -> float:
        """Distance from lat/lon to the nearest map edge (negative off the
        map)."""
        row, col = self.latlon_to_pixel(lat, lon)
        n_rows, n_cols = self.grid.shape
        return min(min(row, n_rows - 1 - row) * self.row_spacing_m,
                   min(col, n_cols - 1 - col) * self.col_spacing_m)

    def get_patch(self, lat: float, lon: float,
                  width_m: float, height_m: float
                  ) -> tuple[np.ndarray, float, tuple[float, float]]:
        """Return (patch, node_spacing_m, (center_row, center_col)) where
        the fractional center indices locate (lat, lon) within the returned
        patch. The patch is empty when the requested window is not wholly
        on the map: a clamped window would silently shift the search."""
        row, col = self.latlon_to_pixel(lat, lon)
        hw = int(math.ceil(width_m / (2.0 * self.col_spacing_m)))
        hh = int(math.ceil(height_m / (2.0 * self.row_spacing_m)))
        r0 = int(math.floor(row)) - hh
        r1 = int(math.ceil(row)) + hh + 1
        c0 = int(math.floor(col)) - hw
        c1 = int(math.ceil(col)) + hw + 1
        if (r0 < 0 or c0 < 0 or r1 > self.grid.shape[0] or
                c1 > self.grid.shape[1]):
            return (np.empty((0, 0), dtype=self.grid.dtype),
                    self.col_spacing_m, (row - r0, col - c0))
        return (self.grid[r0:r1, c0:c1].copy(), self.col_spacing_m,
                (row - r0, col - c0))

    def latlon_to_local(self, lat: float, lon: float) -> tuple[float, float]:
        """Return east/north metres from the height-map centre."""
        metres_per_deg_lon = (
            METERS_PER_DEG_LAT * math.cos(math.radians(self.center_lat)))
        east = (lon - self.center_lon) * metres_per_deg_lon
        north = (lat - self.center_lat) * METERS_PER_DEG_LAT
        return east, north
