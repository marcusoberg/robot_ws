"""Path dimensions and planning tolerances. Distances are metres; angles radians."""

from dataclasses import dataclass
import math
import numpy as np


# Stock firmware 1e7fa639, main.cpp: initial sensor-seeking homing rates.
# Convert motor microsteps/s to joint rad/s (200 steps, 32 microsteps, gearing).
HOMING_STEPS_PER_SECOND = (2050, 2550, 2550, 5550, 5050, 9550)
JOINT_GEAR_RATIOS = (6.4, 20.0, 20.0 * 38.0 / 42.0, 4.0, 4.0, 10.0)
HOMING_JOINT_SPEEDS = tuple(
    steps * 2 * math.pi / (200 * 32 * ratio)
    for steps, ratio in zip(HOMING_STEPS_PER_SECOND, JOINT_GEAR_RATIOS)
)


@dataclass(frozen=True)
class PathSettings:
    width: float = 0.18
    distance: float = 0.30
    height: float = 0.16
    speed: float = 1.0  # multiplier of the homing-rate trajectory
    # These rates choose IK sample density, independently of playback speed.
    sampling_linear_rate: float = 0.02  # metres/s at sample_period
    sampling_angular_rate: float = 0.15  # radians/s at sample_period
    joint_acceleration: float = 0.30
    sample_period: float = 0.05
    position_tolerance: float = 0.0005
    interpolation_tolerance: float = 0.001
    maximum_segment_samples: int = (
        20000  # bound offline memory for accidental tiny speeds
    )
    start_joint_tolerance: float = (
        0.005  # radians: reject a moved start pose, not simulator settling noise
    )
    gripper: str = "pneumatic"
    tip: str = "tool_tcp"

    def validate(self):
        values = (
            self.width,
            self.distance,
            self.height,
            self.speed,
            self.sampling_linear_rate,
            self.sampling_angular_rate,
            self.joint_acceleration,
            self.sample_period,
            self.position_tolerance,
            self.interpolation_tolerance,
        )
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError(
                "Path dimensions, tolerances and rates must be finite and positive"
            )
        if self.speed > 1.0:
            raise ValueError("speed must be in (0, 1]; 1 is the stock homing-rate limit")
        if self.height <= self.width / 2:
            raise ValueError(
                "The bottom of the circle must remain above the base plane"
            )
