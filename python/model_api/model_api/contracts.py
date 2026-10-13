from typing import Any, Protocol, Literal
from pydantic import Field, model_validator
from .identity import Contract, Identity


class Features(Contract):
    state_joints: tuple[str, ...]
    cameras: dict[str, tuple[int, int, int]]
    action_representation: Literal["joint_position"] = "joint_position"
    action_period: float = Field(gt=0)
    instruction: bool = False

    @model_validator(mode="after")
    def valid_features(self):
        if (
            not self.state_joints
            or len(set(self.state_joints)) != len(self.state_joints)
            or any(not name for name in self.state_joints)
        ):
            raise ValueError("State joints must be nonempty and unique")
        for name, shape in self.cameras.items():
            if not name or len(shape) != 3 or shape[0] != 3 or min(shape) <= 0:
                raise ValueError("Model cameras require named CHW RGB extents")
        return self


class PolicyInput(Contract):
    basis: Identity
    execution_time_ns: int = Field(ge=0)
    features: dict[str, Any]
    capture_metadata: dict[str, Any] = Field(default_factory=dict)
    instruction: str
    episode: str
    context: str


class Prediction(Contract):
    basis: Identity
    execution_time_ns: int = Field(ge=0)
    model_revision: str
    action_representation: Literal["joint_position"]
    action_period: float = Field(gt=0)
    actions: list[list[float]]
    computation_ns: int = Field(ge=0)


class ModelAdapter(Protocol):
    @classmethod
    def load(
        cls,
        checkpoint: str,
        *,
        features: Features,
        revision: str,
        device: str,
        model_type: str,
    ) -> "ModelAdapter": ...
    def describe(self) -> Features: ...
    def reset(self, episode: str, context: str) -> None: ...
    def predict(self, value: PolicyInput) -> Prediction: ...


class Trainer(Protocol):
    def run(self, recipe: dict, dataset: str, output: str) -> dict: ...


class CommandAssembler(Protocol):
    def submit(
        self, prediction: Prediction, robot: Any, session: Any, observation: Any
    ) -> Any: ...
