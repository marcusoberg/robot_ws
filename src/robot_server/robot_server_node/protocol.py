"""Bounded SDK capture packets, independent of recording and model libraries."""

import msgpack
import numpy as np
import zlib

MAX_PACKET = 128 * 1024 * 1024
_DTYPES = {"|u1", "<f4", "<f8", "<i8"}


def _array(value):
    if not isinstance(value, np.ndarray):
        raise TypeError(type(value).__name__)
    value = np.ascontiguousarray(value)
    if value.dtype.str not in _DTYPES:
        raise ValueError(f"Unsupported image dtype {value.dtype}")
    return {"__ndarray__": True, "dtype": value.dtype.str, "shape": value.shape, "data": value.tobytes()}


def encode_capture(observation, images):
    raw = msgpack.packb(
        {"schema": "robot.capture.v2", "observation": observation, "images": images},
        default=_array,
        use_bin_type=True,
    )
    if len(raw) > MAX_PACKET:
        raise ValueError("Capture exceeds the transport budget")
    result = msgpack.packb(
        {"compression": "zlib", "raw_size": len(raw), "data": zlib.compress(raw, 1)},
        use_bin_type=True,
    )
    if len(result) > MAX_PACKET:
        raise ValueError("Compressed capture exceeds the transport budget")
    return result
