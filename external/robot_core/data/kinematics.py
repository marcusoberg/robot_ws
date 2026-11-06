from dataclasses import dataclass
from pathlib import Path

from robot_core.config import robot_settings

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class Chain:
    description: Path
    model: pin.Model
    joints: tuple[str, ...]
    units: tuple[str, ...]
    lower: np.ndarray
    upper: np.ndarray
    q_indices: np.ndarray
    v_indices: np.ndarray
    active: np.ndarray
    base: int
    tip: int


@dataclass(frozen=True)
class IKSettings:
    max_iterations: int
    position_tolerance: float
    orientation_tolerance: float
    damping: float
    step_size: float
    position_scale: float
    orientation_scale: float
    posture_weight: float
    singularity_condition: float

    @classmethod
    def configured(cls, **overrides):
        return cls(**(robot_settings()["ik"] | overrides))
