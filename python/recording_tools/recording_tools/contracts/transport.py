"""Episode-free Robot Server capture decoding at Recorder's ingress boundary."""

import numpy as np
from .codec import decompress
from .records import Identity


def decode_server_capture(packet):
    payload = decompress(packet)
    if (
        not isinstance(payload, dict)
        or set(payload) != {"schema", "observation", "images"}
        or payload["schema"] != "robot.capture.v2"
    ):
        raise ValueError("Expected robot.capture.v2")
    observation, images = payload["observation"], payload["images"]
    if not isinstance(observation, dict) or not isinstance(images, dict):
        raise ValueError("Capture observation/images must be mappings")
    identity = Identity.model_validate(observation.get("identity"))
    if type(observation.get("time_ns")) is not int or observation["time_ns"] < 0:
        raise ValueError("Capture execution time is invalid")
    metadata = observation.get("images")
    if not isinstance(metadata, dict) or set(metadata) != set(images):
        raise ValueError("Capture pixel names differ from image metadata")
    for name, pixels in images.items():
        if not isinstance(name, str) or not isinstance(pixels, np.ndarray):
            raise ValueError("Capture planes require named ndarrays")
        if not isinstance(metadata[name], dict):
            raise ValueError("Image metadata must be a mapping")
        valid = (
            pixels.ndim == 3 and pixels.dtype == np.uint8 and pixels.shape[2] == 3
        ) or (
            pixels.ndim == 2
            and pixels.dtype == np.dtype("float32")
            and np.isfinite(pixels).all()
        )
        if not valid or tuple(metadata[name].get("shape", ())) != pixels.shape:
            raise ValueError(f"Invalid RGB/depth plane {name}")
        pixels.setflags(write=False)
    return identity, observation, images
