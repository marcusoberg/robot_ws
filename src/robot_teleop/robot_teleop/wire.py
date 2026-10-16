"""Bounded length-prefixed Protobuf on private child-process pipes."""
import struct
MAX_BYTES = 32 * 1024 * 1024

def read(stream, message_type):
    def exact(n):
        chunks = bytearray()
        while len(chunks) < n:
            part = stream.read(n-len(chunks))
            if not part:
                raise EOFError("LeRobot worker pipe closed")
            chunks.extend(part)
        return bytes(chunks)
    size, = struct.unpack("!I", exact(4))
    if not 0 < size <= MAX_BYTES:
        raise ValueError("Invalid worker packet size")
    return message_type.FromString(exact(size))

def write(stream, message):
    data = message.SerializeToString()
    if not 0 < len(data) <= MAX_BYTES:
        raise ValueError("Worker packet too large")
    stream.write(struct.pack("!I", len(data)) + data)
    stream.flush()
