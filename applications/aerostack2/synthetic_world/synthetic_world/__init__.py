"""Shared synthetic-world model for CyberPhysics SITL navigation validation.

This package is the single source of truth for the procedural world used to
validate the vision-navigation stack in simulation. Every component that must
agree pixel-for-pixel imports from here:

- demnav generates its reference DEM from ``terrain_height``
- wildnav generates its reference satellite tiles from ``satellite_rgb``
- platform camera sims (e.g. the airplane's satellite_camera_sim) render the
  frames that are matched against those references

All of them georeference the world with the same local-tangent approximation
anchored at a fixed origin, via ``METERS_PER_DEG_LAT`` and the
``latlon_to_local``/``local_to_latlon`` pair. Matching only works while the
renderers and the reference generators evaluate identical fields, which is
why these definitions must not be copied or reimplemented elsewhere.

The package is installed into /venv in the cyberphysics/aerostack2 base
image, so it is importable in every image derived from it (demnav, wildnav)
and by scripts run directly in the base image.
"""

import math

import numpy as np

METERS_PER_DEG_LAT = 111320.0


def latlon_to_local(lat, lon, origin_lat, origin_lon):
    """Return (east, north) metres of lat/lon from the origin.

    ``lat``/``lon`` may be scalars or numpy arrays; the longitude scale is
    evaluated at the origin latitude so all callers agree.
    """
    metres_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    east = (lon - origin_lon) * metres_per_deg_lon
    north = (lat - origin_lat) * METERS_PER_DEG_LAT
    return east, north


def local_to_latlon(east, north, origin_lat, origin_lon):
    """Return (lat, lon) for east/north metres from the origin.

    Inverse of :func:`latlon_to_local`.
    """
    metres_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    lat = origin_lat + north / METERS_PER_DEG_LAT
    lon = origin_lon + east / metres_per_deg_lon
    return lat, lon


def terrain_height(east, north):
    """Synthetic terrain height in metres at east/north metres from origin.

    Accepts scalars or numpy arrays. A diagonal ridge, two distinctive
    marker features, a coastline shelf, and low-amplitude texture give the
    DEM enough relief for depth correlation to lock on unambiguously.
    """
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


def satellite_rgb(east, north):
    """Synthetic satellite imagery as a uint8 RGB array.

    ``east``/``north`` must be equal-shaped numpy arrays; the result has
    shape ``(*east.shape, 3)``. Roads, a winding trail, parcel checkers,
    and the coastline give SIFT enough texture to match reliably.
    """
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
