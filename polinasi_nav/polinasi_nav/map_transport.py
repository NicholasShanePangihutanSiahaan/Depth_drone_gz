"""Bounded, non-pickle transport of an already inflated navigation window.

Only measured occupancy is sent: no global dictionary or point cloud enters
the flight-control process. Arrays are decoded without rebuilding inflation.
"""
import copy
import io
import json
import zipfile
import numpy as np

ARRAYS = {'state': np.dtype('int8'), 'seen': np.dtype('float64'),
          'blocked': np.dtype('bool'), 'blocked_actual': np.dtype('bool'),
          'distance': np.dtype('float64'), 'free_distance': np.dtype('float64')}


def encode_window(grid, statistics):
    metadata = dict(origin=grid.lo.tolist(), resolution=grid.res, version=int(grid.version),
                    observation_stamp=float(grid.last_observation_stamp), statistics=statistics)
    output = io.BytesIO()
    np.savez(output, metadata=np.frombuffer(json.dumps(metadata).encode(), dtype=np.uint8),
             **{name: np.asarray(getattr(grid, name), dtype=dtype) for name, dtype in ARRAYS.items()})
    return output.getvalue()


def decode_window(payload, template):
    if len(payload) > 8_000_000:
        raise ValueError('navigation snapshot exceeds bounded transport size')
    try:
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            entries = archive.infolist()
            expected = {name+'.npy' for name in (*ARRAYS, 'metadata')}
            if ({entry.filename for entry in entries} != expected or len(entries) != len(expected)
                    or any(entry.compress_type != zipfile.ZIP_STORED for entry in entries)
                    or sum(entry.file_size for entry in entries) > 8_000_000):
                raise ValueError('invalid or oversized navigation archive')
    except zipfile.BadZipFile as exc:
        raise ValueError('invalid navigation archive') from exc
    with np.load(io.BytesIO(payload), allow_pickle=False) as source:
        metadata = json.loads(source['metadata'].tobytes())
        shape = template.shape
        arrays = {}
        for name, dtype in ARRAYS.items():
            values = source[name]
            if values.shape != shape or values.dtype != dtype:
                raise ValueError('navigation snapshot shape/type mismatch')
            arrays[name] = values.copy()
    lo = np.asarray(metadata['origin'], dtype=float)
    if lo.shape != (3,) or not np.all(np.isfinite(lo)) or metadata['resolution'] != template.res:
        raise ValueError('navigation snapshot origin/resolution mismatch')
    stamp = float(metadata['observation_stamp'])
    if not np.isfinite(stamp) or not np.all(np.isin(arrays['state'], [-1, 0, 1])):
        raise ValueError('invalid navigation snapshot measurement')
    for name in ('distance', 'free_distance'):
        if not np.all(np.isfinite(arrays[name])):
            raise ValueError('invalid navigation snapshot distance')
    grid = copy.copy(template)
    grid.lo, grid.version, grid.last_observation_stamp = lo, int(metadata['version']), stamp
    for name, values in arrays.items():
        setattr(grid, name, values)
    return grid, metadata['statistics']
