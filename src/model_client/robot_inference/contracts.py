from dataclasses import dataclass, field
import json
import numpy as np
from robot_description.spec import ActionSpec

@dataclass(frozen=True)
class Identity:
    runtime_instance: str
    timeline_generation: int
    observation_sequence: int

    def __post_init__(self):
        if not self.runtime_instance or any(type(v) is not int or v < 0 for v in (self.timeline_generation, self.observation_sequence)):
            raise ValueError("Invalid observation identity")

@dataclass(frozen=True)
class Features:
    action_spec_json: str

    def __post_init__(self):
        spec = ActionSpec(**json.loads(self.action_spec_json))
        object.__setattr__(self, "action_spec_json", spec.to_json())

    @property
    def spec(self): return ActionSpec(**json.loads(self.action_spec_json))
    @property
    def state_joints(self): return self.spec.joints
    @property
    def action_joints(self): return self.spec.joints
    @property
    def action_period(self): return self.spec.period_ns / 1e9
    @property
    def action_representation(self): return self.spec.semantics
    @property
    def instruction(self): return False
    @property
    def cameras(self):
        h,w,c = self.spec.image_shape
        return {f"observation.images.camera_{k}": (c,h,w) for k in self.spec.cameras}
    @classmethod
    def from_json(cls, text): return cls(**json.loads(text))

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
        if a.shape != shape or (name in features.cameras and a.dtype != np.uint8) or not np.isfinite(a).all():
            raise ValueError(f"Invalid shape, dtype, or nonfinite input for {name}")
    return value

def validate_prediction(value, source, features):
    actions = np.asarray(value.actions)
    if (value.basis != source.basis or value.execution_time_ns != source.execution_time_ns
        or value.action_representation != features.action_representation or not value.model_revision
        or value.action_period != features.action_period or value.computation_ns < 0
        or actions.ndim != 2 or not 0 < actions.shape[0] <= 1000
        or actions.shape[1] != len(features.action_joints) or not np.isfinite(actions).all()):
        raise ValueError("Prediction disagrees with its basis or negotiated contract")
    for action in actions:
        features.spec.validate(action)
    return value
