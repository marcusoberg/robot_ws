"""Timed joint interpolation with the zero-endpoint-velocity quintic used by jtraj."""

import numpy as np

from robot_core.control.quintic import smooth_progress, smooth_rate
from robot_core.data.commands import MoveJoint


class JointMoveProfile:
    def __init__(self, initial, command: MoveJoint):
        self.duration = command.duration
        self.initial = np.array(initial, dtype=float, copy=True)
        self.target = np.r_[command.angles, self.initial[-1]]

    def at(self, elapsed):
        if elapsed <= 0:
            return self.initial.copy()
        if elapsed >= self.duration:
            return self.target.copy()
        progress = smooth_progress(elapsed / self.duration)
        return self.initial + progress * (self.target - self.initial)

    def velocity_at(self, elapsed):
        if elapsed <= 0 or elapsed >= self.duration:
            return np.zeros_like(self.initial)
        fraction = elapsed / self.duration
        rate = smooth_rate(fraction) / self.duration
        return rate * (self.target - self.initial)
