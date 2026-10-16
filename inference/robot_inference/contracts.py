"""Validated in-process values. The network schema is protocol.proto."""
from dataclasses import dataclass, field
import json
import math
import numpy as np

@dataclass(frozen=True)
class Identity:
    runtime_instance: str
    timeline_generation: int
    observation_sequence: int

    def __post_init__(self):
        if not self.runtime_instance or any(type(v) is not int or v < 0 for v in
                (self.timeline_generation, self.observation_sequence)):
            raise ValueError("Invalid observation identity")

@dataclass(frozen=True)
class Features:
    state_joints: tuple[str, ...]
    cameras: dict[str, tuple[int, int, int]]
    action_period: float
    instruction: bool = False
    action_representation: str = "joint_position"
    action_joints: tuple[str, ...] = ()
    state_units: tuple[str, ...] = ()
    action_units: tuple[str, ...] = ()
    binary_gripper: bool = False
    action_time_offset_ns: int = 0

    def __post_init__(self):
        for key in ("state_joints", "action_joints", "state_units", "action_units"):
            object.__setattr__(self, key, tuple(getattr(self, key)))
        object.__setattr__(self, "cameras", {k: tuple(v) for k, v in self.cameras.items()})
        if not self.action_joints:
            object.__setattr__(self, "action_joints", self.state_joints)
        if not self.state_units:
            object.__setattr__(self, "state_units", ("rad",) * len(self.state_joints))
        if not self.action_units:
            object.__setattr__(self, "action_units", ("rad",) * len(self.action_joints))
        for names, units in ((self.state_joints, self.state_units), (self.action_joints, self.action_units)):
            if not names or len(names) != len(set(names)) or any(not n for n in names):
                raise ValueError("Joint names must be nonempty and unique")
            if len(units) != len(names) or any(not u for u in units):
                raise ValueError("Each joint requires an explicit unit")
        if type(self.binary_gripper) is not bool or type(self.action_time_offset_ns) is not int or not 0 <= self.action_time_offset_ns <= 1_000_000_000:
            raise ValueError("Invalid binary gripper declaration or action time offset")
        if self.binary_gripper and (self.action_joints[-1] != "gripper_open" or self.action_units[-1] != "binary" or len(self.action_joints) < 2):
            raise ValueError("Binary gripper must be the last output, named gripper_open with unit binary")
        if not math.isfinite(self.action_period) or self.action_period <= 0:
            raise ValueError("Action period must be finite and positive")
        if self.action_representation not in ("joint_position", "unmapped_model_output"):
            raise ValueError("Unsupported action representation")
        for name, shape in self.cameras.items():
            if not name or len(shape) != 3 or shape[0] != 3 or any(type(n) is not int or not 0 < n <= 8192 for n in shape):
                raise ValueError("Cameras require named CHW RGB extents")

    @classmethod
    def from_json(cls, text):
        return cls(**json.loads(text))

@dataclass(frozen=True)
class PolicyInput:
    basis: Identity
    execution_time_ns: int
    features: dict
    instruction: str
    episode: str
    context: str
    capture_metadata: dict = field(default_factory=dict)

@dataclass(frozen=True)
class Prediction:
    basis: Identity
    execution_time_ns: int
    model_revision: str
    action_representation: str
    action_period: float
    actions: list[list[float]]
    computation_ns: int

def validate_input(value, features):
    shapes = {"observation.state": (len(features.state_joints),),
              **{name: (s[1], s[2], s[0]) for name, s in features.cameras.items()}}
    if value.execution_time_ns < 0 or not value.context or not value.episode:
        raise ValueError("Input requires a timestamp and inference context")
    if value.features.keys() != shapes.keys():
        raise ValueError("Input feature names differ from negotiated features")
    for name, shape in shapes.items():
        a = np.asarray(value.features[name])
        if a.shape != shape or (name in features.cameras and a.dtype != np.uint8):
            raise ValueError(f"Invalid shape or dtype for {name}; expected {shape}")
        if not np.isfinite(a).all():
            raise ValueError(f"Nonfinite input: {name}")
    if features.instruction and not value.instruction.strip():
        raise ValueError("Task instruction is required")
    return value

def validate_prediction(value, source, features):
    actions = np.asarray(value.actions)
    if (value.basis != source.basis or value.execution_time_ns != source.execution_time_ns
        or value.action_representation != features.action_representation
        or value.action_period != features.action_period or value.computation_ns < 0
        or actions.ndim != 2 or not 0 < actions.shape[0] <= 10000
        or actions.shape[1] != len(features.action_joints) or not np.isfinite(actions).all()):
        raise ValueError("Prediction disagrees with its basis or negotiated features")
    return value
