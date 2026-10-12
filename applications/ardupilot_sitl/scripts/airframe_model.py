"""Shared, generated physical-airframe data for Gazebo and Isaac."""
import json
import os
from pathlib import Path
import struct

MODEL_DIR = Path(os.environ.get('AERODRONE_MODEL_DIR',
                 Path(__file__).resolve().parents[2] / 'aerostack2/models/aerodrone'))


def load_airframe():
    return json.loads((MODEL_DIR / 'airframe.json').read_text())


def load_visual_triangles(path=None):
    """Read our binary STL, rejecting an unfetched Git LFS pointer clearly."""
    import numpy as np
    path = MODEL_DIR / 'airframe.stl' if path is None else Path(path)
    data = path.read_bytes()
    if data.startswith(b'version https://git-lfs.github.com/spec/v1'):
        raise RuntimeError(
            'Airframe mesh is a Git LFS pointer. Run git lfs pull '
            '--include="applications/aerostack2/models/aerodrone/*.stl" on the host.')
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if not count or len(data) != 84 + count * 50:
        raise ValueError(f'Invalid binary airframe STL: {path}')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                      ('attribute', '<u2')])
    triangles = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices']
    if not np.isfinite(triangles).all():
        raise ValueError(f'Non-finite airframe STL coordinates: {path}')
    return triangles
