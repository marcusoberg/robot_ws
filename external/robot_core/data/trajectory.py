from dataclasses import dataclass

import numpy as np


@dataclass
class CartesianPath:
    times: np.ndarray
    xyz: np.ndarray
    labels: list[str]

    @property
    def duration(self):
        return float(self.times[-1])

    def at(self, elapsed):
        return np.array([np.interp(elapsed, self.times, axis) for axis in self.xyz.T])

    def summary(self):
        return f"{len(self.times)} Cartesian targets, {self.duration:.1f} s nominal duration"
