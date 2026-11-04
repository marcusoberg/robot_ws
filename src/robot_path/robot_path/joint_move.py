"""Timed joint interpolation with the zero-endpoint-velocity quintic used by jtraj."""

import math

import numpy as np

from robot_path.geometry import smooth_progress


class JointMove:
    def __init__(self, spec, initial, arm_target, duration):
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('Move duration must be finite and positive')
        self.duration = float(duration)
        self.initial = spec.validate(initial)
        arm_target = np.asarray(arm_target, dtype=float)
        if arm_target.shape != (6,) or not np.isfinite(arm_target).all():
            raise ValueError('Move requires six finite joint angles')
        self.target = self.initial.copy()
        self.target[:6] = arm_target
        self.target = spec.validate(self.target)

    def at(self, elapsed):
        if not math.isfinite(elapsed):
            raise ValueError('Elapsed time must be finite')
        if elapsed <= 0:
            return self.initial.copy()
        if elapsed >= self.duration:
            return self.target.copy()
        progress = smooth_progress(elapsed / self.duration)
        return self.initial + progress * (self.target - self.initial)

    def velocity_at(self, elapsed):
        if not math.isfinite(elapsed):
            raise ValueError('Elapsed time must be finite')
        if elapsed <= 0 or elapsed >= self.duration:
            return np.zeros_like(self.initial)
        fraction = elapsed / self.duration
        rate = 30 * fraction**2 * (1 - fraction)**2 / self.duration
        return rate * (self.target - self.initial)
