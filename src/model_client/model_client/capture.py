"""Decode ROS capture planes without a second robot transport."""
from dataclasses import dataclass
import json
import numpy as np
from robot_inference import Identity

@dataclass(frozen=True)
class CapturedObservation:
    identity: Identity
    execution_time_ns: int
    observation: dict
    images: dict

def decode_capture(message):
    record = json.loads(message.observation_json)
    identity = Identity(message.runtime_instance, message.timeline_generation, message.observation_sequence)
    if record["identity"] != dict(runtime_instance=identity.runtime_instance,
        timeline_generation=identity.timeline_generation, observation_sequence=identity.observation_sequence):
        raise ValueError("Capture observation and ROS identity disagree")
    if record["time_ns"] != message.execution_time_ns:
        raise ValueError("Capture timestamps disagree")
    images = {}
    for plane in message.images:
        img = plane.image
        if plane.name in images or img.encoding not in ("rgb8", "32FC1") or img.is_bigendian:
            raise ValueError("Unsupported or duplicate capture plane")
        channels, dtype = (3, np.dtype("u1")) if img.encoding == "rgb8" else (1, np.dtype("<f4"))
        if not img.height or not img.width or img.step != img.width*channels*dtype.itemsize or len(img.data) != img.height*img.step:
            raise ValueError("Invalid capture extents")
        shape = (img.height, img.width, 3) if channels == 3 else (img.height, img.width)
        metadata = record["images"][plane.name]
        if list(shape) != metadata["shape"]:
            raise ValueError("Image metadata differs from pixels")
        images[plane.name] = np.frombuffer(bytes(img.data), dtype=dtype).reshape(shape)
    if images.keys() != record["images"].keys():
        raise ValueError("Missing capture planes")
    return CapturedObservation(identity, message.execution_time_ns, record, images)
