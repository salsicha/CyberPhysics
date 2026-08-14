#!/usr/bin/env python3
"""Generate robust primitive collisions sampled from a georeferenced DEM.

Gazebo Fortress' installed DART and Bullet adapters do not create usable
contacts for SDF heightmaps in this container.  Static box columns are native
physics geometry in every supported engine.  A conservative 80 m grid covers the complete
mission corridor and a 400 m grid covers the rest of the 8 km terrain.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


FINE_HALF_EAST_M = 1000.0
FINE_HALF_NORTH_M = 600.0
FINE_CELL_M = 80.0
COARSE_CELL_M = 400.0
BASE_MARGIN_M = 20.0


def _index_bounds(grid, half_m, east_min, east_max, north_min, north_max):
    cols = grid.shape[1]
    rows = grid.shape[0]
    c0 = int(np.floor((east_min + half_m) / (2.0 * half_m) * (cols - 1)))
    c1 = int(np.ceil((east_max + half_m) / (2.0 * half_m) * (cols - 1)))
    r0 = int(np.floor((half_m - north_max) / (2.0 * half_m) * (rows - 1)))
    r1 = int(np.ceil((half_m - north_min) / (2.0 * half_m) * (rows - 1)))
    return (max(0, r0), min(rows - 1, r1),
            max(0, c0), min(cols - 1, c1))


def _cell_top(grid, origin_alt, half_m, east, north, size):
    r0, r1, c0, c1 = _index_bounds(
        grid, half_m, east - size / 2.0, east + size / 2.0,
        north - size / 2.0, north + size / 2.0)
    # The maximum produces conservative, gap-free terrain steps.  Missions
    # retain at least 35 m AGL, so this sub-cell safety envelope is harmless.
    return float(np.max(grid[r0:r1 + 1, c0:c1 + 1]) - origin_alt)


def terrain_collision_height(grid, origin_alt, area_km, east, north):
    half_m = area_km * 500.0
    if abs(east) <= FINE_HALF_EAST_M and abs(north) <= FINE_HALF_NORTH_M:
        size = FINE_CELL_M
        center_east = (np.floor((east + FINE_HALF_EAST_M) / size) * size -
                       FINE_HALF_EAST_M + size / 2.0)
        center_north = (np.floor((north + FINE_HALF_NORTH_M) / size) * size -
                        FINE_HALF_NORTH_M + size / 2.0)
    else:
        size = COARSE_CELL_M
        center_east = np.floor((east + half_m) / size) * size - half_m + size / 2.0
        center_north = np.floor((north + half_m) / size) * size - half_m + size / 2.0
    return _cell_top(grid, origin_alt, half_m, center_east, center_north, size)


def _cells(start, stop, size):
    count = int(round((stop - start) / size))
    for index in range(count):
        yield start + (index + 0.5) * size


def write_terrain_collision_model(grid, origin_alt, area_km, output: Path):
    half_m = area_km * 500.0
    bottom = float(grid.min() - origin_alt - BASE_MARGIN_M)
    collisions = []
    count = 0

    def append_cell(east, north, size, prefix):
        nonlocal count
        top = _cell_top(grid, origin_alt, half_m, east, north, size)
        height = max(top - bottom, 0.1)
        center_z = bottom + height / 2.0
        collisions.append(
            f'<collision name="{prefix}_{count}"><pose>{east:.3f} {north:.3f} '
            f'{center_z:.3f} 0 0 0</pose><geometry><box><size>{size:.3f} '
            f'{size:.3f} {height:.3f}</size></box></geometry></collision>')
        count += 1

    for north in _cells(-half_m, half_m, COARSE_CELL_M):
        for east in _cells(-half_m, half_m, COARSE_CELL_M):
            overlaps_fine = (
                abs(east) < FINE_HALF_EAST_M + COARSE_CELL_M / 2.0 and
                abs(north) < FINE_HALF_NORTH_M + COARSE_CELL_M / 2.0)
            if not overlaps_fine:
                append_cell(east, north, COARSE_CELL_M, "coarse")

    for north in _cells(-FINE_HALF_NORTH_M, FINE_HALF_NORTH_M, FINE_CELL_M):
        for east in _cells(-FINE_HALF_EAST_M, FINE_HALF_EAST_M, FINE_CELL_M):
            append_cell(east, north, FINE_CELL_M, "fine")

    sdf = "\n".join([
        '<?xml version="1.0"?>',
        '<sdf version="1.9"><model name="georeferenced_terrain"><static>true</static>',
        '<link name="terrain">',
        *collisions,
        '<visual name="visual"><geometry><mesh>',
        '<uri>model://georeferenced_terrain/terrain.obj</uri>',
        '</mesh></geometry></visual>',
        '</link></model></sdf>',
        '',
    ])
    output.write_text(sdf, encoding="utf-8")
    return count
