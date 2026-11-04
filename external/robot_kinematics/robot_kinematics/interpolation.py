"""Causal, velocity-continuous tracking of the latest available IK solution."""

import numpy as np


class JointStream:
    def __init__(self, positions, now, period):
        self.position = np.asarray(positions, dtype=float).copy()
        self.stage = self.position.copy()
        self.target = self.position.copy()
        self.started = now
        self.period = period

    def update(self, target, now):
        # Advance both states before changing the input. Position and velocity
        # are continuous at retargeting; no endpoint stop or future sample is needed.
        self.position, self.stage = self.evaluate(now)
        self.target = np.asarray(target, dtype=float).copy()
        self.started = now

    def evaluate(self, now):
        # Exact solution of two cascaded first-order filters, each with the
        # configured sample period as its time constant. This is critically
        # damped smoothing, not a joint speed cap. Both states are convex
        # combinations of valid positions, so they stay inside joint limits.
        u = max((now - self.started) / self.period, 0.)
        decay = np.exp(-u)
        weight = -np.expm1(-u)
        stage = self.stage + weight * (self.target - self.stage)
        position = (self.position + u*decay * (self.stage - self.position)
                    + max(0., weight - u*decay) * (self.target - self.position))
        low = np.minimum(np.minimum(self.position, self.stage), self.target)
        high = np.maximum(np.maximum(self.position, self.stage), self.target)
        # Correct roundoff only; constant passive joints remain exactly constant.
        return np.clip(position, low, high), np.clip(stage, low, high)

    def at(self, now):
        position, stage = self.evaluate(now)
        return position, (stage - position) / self.period
