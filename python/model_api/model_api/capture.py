"""Decode robot.capture.v2 without importing ROS, recording or the SDK."""

from dataclasses import dataclass
from typing import Any
import numpy as np
from .identity import Identity
from .contracts import Features, PolicyInput
from .decoder import PolicyDecoder
from .codec import decompress


@dataclass(frozen=True)
class CapturedObservation:
    observation: dict[str, Any]
    images: dict[str, np.ndarray]

    @property
    def identity(self) -> Identity:
        return Identity.model_validate(self.observation["identity"])

    @property
    def execution_time_ns(self) -> int:
        return int(self.observation["time_ns"])


def decode_capture_packet(packet: bytes) -> CapturedObservation:
    value = decompress(packet)
    if (
        not isinstance(value, dict)
        or set(value) != {"schema", "observation", "images"}
        or value["schema"] != "robot.capture.v2"
    ):
        raise ValueError("Expected a robot.capture.v2 observation envelope")
    observation, images = value["observation"], value["images"]
    if not isinstance(observation, dict) or not isinstance(images, dict):
        raise ValueError("Capture observation and images must be mappings")
    Identity.model_validate(observation.get("identity"))
    if type(observation.get("time_ns")) is not int or observation["time_ns"] < 0:
        raise ValueError("Capture execution time must be a nonnegative integer")
    metadata = observation.get("images")
    if not isinstance(metadata, dict) or set(metadata) != set(images):
        raise ValueError("Capture pixel names differ from observation image metadata")
    for name, pixels in images.items():
        if not isinstance(name, str) or not isinstance(pixels, np.ndarray):
            raise ValueError("Capture images must be named ndarray planes")
        if not isinstance(metadata[name], dict):
            raise ValueError("Image metadata must be a mapping")
        if pixels.ndim == 3:
            valid = pixels.dtype == np.uint8 and pixels.shape[2] == 3
        else:
            valid = (
                pixels.ndim == 2
                and pixels.dtype == np.dtype("float32")
                and np.isfinite(pixels).all()
            )
        if not valid or tuple(metadata[name].get("shape", ())) != pixels.shape:
            raise ValueError(f"Invalid RGB/depth capture plane {name}")
        pixels.setflags(write=False)
    return CapturedObservation(observation, images)


def policy_input(
    capture: CapturedObservation,
    features: Features,
    cameras: dict[str, str],
    instruction: str,
    episode: str,
    context: str,
) -> PolicyInput:
    state = capture.observation.get("state")
    if not isinstance(state, dict) or state.get("position") is None:
        raise ValueError("Robot measurement is unavailable")
    if set(cameras) != set(features.cameras):
        raise ValueError("Camera mapping differs from negotiated model features")
    value = PolicyInput(
        basis=capture.identity,
        execution_time_ns=capture.execution_time_ns,
        features={
            "observation.state": np.asarray(state["position"]),
            **{name: capture.images[camera] for name, camera in cameras.items()},
        },
        capture_metadata={
            "images": capture.observation["images"],
            "joint_stamp": state,
        },
        instruction=instruction,
        episode=episode,
        context=context,
    )
    return PolicyDecoder(features).input(value.model_dump())
