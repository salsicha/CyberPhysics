import math
import os
import time
import urllib.request

import cv2
import numpy as np


WEB_MERCATOR_LIMIT = 85.05112878
METERS_PER_DEG_LAT = 111320.0


def latlon_to_tile(lat, lon, zoom):
    lat = max(-WEB_MERCATOR_LIMIT, min(WEB_MERCATOR_LIMIT, lat))
    scale = 2 ** zoom
    x = (lon + 180.0) / 360.0 * scale
    lat_rad = math.radians(lat)
    y = (
        1.0 - math.asinh(math.tan(lat_rad)) / math.pi
    ) / 2.0 * scale
    return x, y


def tile_pixel_to_latlon(tile_x, tile_y, pixel_x, pixel_y, zoom, size=256):
    scale = 2 ** zoom
    world_x = (tile_x + pixel_x / size) / scale
    world_y = (tile_y + pixel_y / size) / scale
    lon = world_x * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * world_y))))
    return lat, lon


def ground_resolution_m(lat, zoom):
    return 156543.03392804097 * math.cos(math.radians(lat)) / (2 ** zoom)


class SatelliteTileCache:
    def __init__(self, cache_dir, url_template, zoom, feature_backend='SIFT',
                 max_features=1500, request_timeout=5.0,
                 origin_lat=0.0, origin_lon=0.0):
        self.cache_dir = cache_dir
        self.url_template = url_template
        self.zoom = zoom
        self.feature_backend = feature_backend.upper()
        self.max_features = max_features
        self.request_timeout = request_timeout
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.synthetic = str(url_template).startswith('synthetic://')
        self.failed_until = {}
        os.makedirs(cache_dir, exist_ok=True)

        if self.feature_backend == 'SIFT':
            self.detector = cv2.SIFT_create(nfeatures=max_features)
            self.norm = cv2.NORM_L2
        elif self.feature_backend == 'ORB':
            self.detector = cv2.ORB_create(nfeatures=max_features)
            self.norm = cv2.NORM_HAMMING
        else:
            raise ValueError(
                f'Unsupported feature backend: {self.feature_backend}')

    def candidate_tiles(self, lat, lon, search_radius_m, max_tiles):
        center_x, center_y = latlon_to_tile(lat, lon, self.zoom)
        tile_width_m = ground_resolution_m(lat, self.zoom) * 256.0
        radius = max(0, int(math.ceil(search_radius_m / tile_width_m)))
        candidates = []
        scale = 2 ** self.zoom
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                distance = dx * dx + dy * dy
                x = (int(center_x) + dx) % scale
                y = max(0, min(scale - 1, int(center_y) + dy))
                candidates.append((distance, x, y))
        candidates.sort()
        return [(x, y) for _, x, y in candidates[:max_tiles]]

    def get_features(self, x, y, allow_download=True):
        stem = f'{self.zoom}_{x}_{y}'
        image_path = os.path.join(self.cache_dir, stem + '.jpg')
        feature_path = os.path.join(
            self.cache_dir,
            f'{stem}_{self.feature_backend.lower()}_nf{self.max_features}.npz')

        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            if not allow_download:
                return None
            image = self._download(x, y, image_path)
        if image is None:
            return None

        if os.path.exists(feature_path):
            try:
                cached = np.load(feature_path)
                return cached['points'], cached['descriptors'], image.shape
            except (OSError, KeyError, ValueError):
                pass

        keypoints, descriptors = self.detector.detectAndCompute(image, None)
        if descriptors is None or len(keypoints) < 4:
            return None
        points = np.asarray([kp.pt for kp in keypoints], dtype=np.float32)
        np.savez_compressed(
            feature_path, points=points, descriptors=descriptors)
        return points, descriptors, image.shape

    def describe(self, image):
        keypoints, descriptors = self.detector.detectAndCompute(image, None)
        if descriptors is None:
            return np.empty((0, 2), np.float32), None
        points = np.asarray([kp.pt for kp in keypoints], dtype=np.float32)
        return points, descriptors

    def matcher(self):
        return cv2.BFMatcher(self.norm, crossCheck=False)

    def is_cached(self, x, y):
        stem = f'{self.zoom}_{x}_{y}'
        return os.path.exists(os.path.join(self.cache_dir, stem + '.jpg'))

    def _download(self, x, y, image_path):
        if self.synthetic:
            image = self._generate_synthetic_tile(x, y)
            cv2.imwrite(image_path, image)
            return image

        key = (x, y)
        now = time.monotonic()
        if self.failed_until.get(key, 0.0) > now:
            return None
        url = self.url_template.format(z=self.zoom, x=x, y=y)
        request = urllib.request.Request(
            url, headers={'User-Agent': 'CyberPhysics-WildNav/1.0'})
        try:
            with urllib.request.urlopen(
                    request, timeout=self.request_timeout) as response:
                payload = response.read()
            image = cv2.imdecode(
                np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError('tile response was not an image')
            cv2.imwrite(image_path, image)
            return image
        except Exception:
            self.failed_until[key] = now + 60.0
            return None

    def _generate_synthetic_tile(self, x, y, size=256):
        rows, cols = np.indices((size, size), dtype=np.float64)
        scale = 2 ** self.zoom
        world_x = (x + (cols + 0.5) / size) / scale
        world_y = (y + (rows + 0.5) / size) / scale
        lon = world_x * 360.0 - 180.0
        lat = np.degrees(np.arctan(np.sinh(math.pi * (1.0 - 2.0 * world_y))))
        metres_per_deg_lon = (
            METERS_PER_DEG_LAT * math.cos(math.radians(self.origin_lat)))
        east = (lon - self.origin_lon) * metres_per_deg_lon
        north = (lat - self.origin_lat) * METERS_PER_DEG_LAT
        rgb = _synthetic_satellite_rgb(east, north)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)


def _synthetic_satellite_rgb(east, north):
    coast = north < -420.0 + 45.0 * np.sin(east / 210.0)
    road_a = np.abs((north + 0.22 * east + 35.0) % 180.0 - 90.0) < 4.5
    road_b = np.abs((east - 0.35 * north - 20.0) % 240.0 - 120.0) < 3.5
    trail = np.abs((north - 90.0 * np.sin(east / 180.0)) % 130.0 - 65.0) < 2.0
    parcel = ((np.floor(east / 95.0) + np.floor(north / 85.0)) % 2) == 1
    noise = ((np.sin(east * 0.19 + north * 0.11) + 1.0) * 9.0).astype(np.uint8)

    image = np.empty((*east.shape, 3), dtype=np.uint8)
    image[..., 0] = 57 + noise
    image[..., 1] = 108 + noise
    image[..., 2] = 64

    image[parcel, 0] = 84 + noise[parcel]
    image[parcel, 1] = 126 + noise[parcel]
    image[parcel, 2] = 72

    image[trail, 0] = 132 + noise[trail]
    image[trail, 1] = 104 + noise[trail]
    image[trail, 2] = 76

    roads = road_a | road_b
    image[roads, 0] = 128 + noise[roads]
    image[roads, 1] = 124 + noise[roads]
    image[roads, 2] = 112 + noise[roads]

    image[coast, 0] = 34
    image[coast, 1] = 82 + noise[coast]
    image[coast, 2] = 128 + noise[coast]
    return image
