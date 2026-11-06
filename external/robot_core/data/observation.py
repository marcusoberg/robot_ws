from dataclasses import dataclass, field

import numpy as np

@dataclass(frozen=True)
class Observation:
    instance: str
    episode: int
    snapshot: int
    time_ns: int
    host_acquired_ns: int
    position: np.ndarray
    velocity: np.ndarray
    images: dict[str, np.ndarray]
    camera_metadata: dict = field(default_factory=dict)

    def features(self):
        return {"observation.state": self.position,
                **{f"observation.images.camera_{k}": v for k, v in self.images.items()}}

def immutable(values):
    a = np.array(values, copy=True)
    a.flags.writeable = False
    return a

