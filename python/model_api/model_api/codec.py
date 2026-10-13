"""MessagePack ndarray encoding; decoded arrays own immutable bytes."""

import msgpack
import numpy as np
import zlib
from typing import Literal
from pydantic import BaseModel, ConfigDict, Field


class CompressionEnvelope(BaseModel):
    model_config = ConfigDict(extra="forbid")
    compression: Literal["zlib"]
    raw_size: int = Field(gt=0, le=128 * 1024 * 1024)
    data: bytes


MAX_PACKET = 128 * 1024 * 1024
_ALLOWED = {"|u1", "<f4", "<f8", "<i8"}


def _default(value):
    if isinstance(value, np.ndarray):
        value = np.ascontiguousarray(value)
        if value.dtype.str not in _ALLOWED:
            raise ValueError(f"Unsupported array dtype {value.dtype}")
        return {
            "__ndarray__": True,
            "dtype": value.dtype.str,
            "shape": value.shape,
            "data": value.tobytes(),
        }
    raise TypeError(type(value).__name__)


def _object(value):
    if value.get("__ndarray__") is True:
        dtype = value["dtype"]
        shape = tuple(value["shape"])
        data = value["data"]
        if (
            dtype not in _ALLOWED
            or not shape
            or any(type(n) is not int or n <= 0 for n in shape)
        ):
            raise ValueError("Invalid ndarray descriptor")
        size = int(np.prod(shape, dtype=object)) * np.dtype(dtype).itemsize
        if size > MAX_PACKET or size != len(data):
            raise ValueError("Array extent and payload disagree")
        return np.frombuffer(data, dtype=dtype).reshape(shape)
    return value


def pack(value) -> bytes:
    return msgpack.packb(value, default=_default, use_bin_type=True)


def unpack(value: bytes | str):
    if not isinstance(value, bytes):
        raise ValueError("This endpoint requires binary MessagePack packets")
    if len(value) > MAX_PACKET:
        raise ValueError("Packet exceeds the transport budget")
    return msgpack.unpackb(value, object_hook=_object, raw=False, strict_map_key=True)


def compress(value) -> bytes:
    import zlib

    raw = pack(value)
    return pack(
        {"compression": "zlib", "raw_size": len(raw), "data": zlib.compress(raw, 1)}
    )


def decompress(packet: bytes):
    envelope = CompressionEnvelope.model_validate(unpack(packet))
    decoder = zlib.decompressobj()
    raw = decoder.decompress(envelope.data, envelope.raw_size + 1)
    if (
        len(raw) != envelope.raw_size
        or not decoder.eof
        or decoder.unused_data
        or decoder.unconsumed_tail
    ):
        raise ValueError("Compressed packet disagrees with its declared size")
    return unpack(raw)
