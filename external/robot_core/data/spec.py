from dataclasses import asdict, dataclass
from pathlib import Path
import hashlib
import json
from robot_core.config import robot_settings
from robot_core.validation import joint_positions, same_contract

@dataclass(frozen=True)
class ActionSpec:
    joints: tuple[str, ...]
    units: tuple[str, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    cameras: tuple[str, ...]
    image_shape: tuple[int, int, int]
    period_ns: int
    controller: str
    semantics: str
    state: str
    version: int

    # Existing offline dataset/inference callers use these explicit operations.
    def validate(self, action):
        return joint_positions(self, action).copy()

    def require(self, other):
        same_contract(self, other)

    def to_json(self):
        return json.dumps(asdict(self), sort_keys=True, separators=(",", ":"), allow_nan=False)

    @property
    def identity(self):
        return "sha256:" + hashlib.sha256(self.to_json().encode()).hexdigest()

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text()))

    @classmethod
    def from_dict(cls, values):
        fields = robot_settings()["action"] | values
        for key in ("joints", "units", "lower", "upper", "cameras", "image_shape"):
            fields[key] = tuple(fields[key])
        return cls(**fields)


@dataclass(frozen=True)
class Profile:
    spec: ActionSpec
    scene: Path
    description: Path
    initial: tuple[float, ...]
    execution_period_ns: int
    physics_period_ns: int
    source_timeout_ns: int
    backend: str
    stop_velocity: float
    stop_settled_ns: int
    stop_timeout_ns: int
    cube_half_size: float
    task: str
