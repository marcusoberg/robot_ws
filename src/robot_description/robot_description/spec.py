from dataclasses import asdict, dataclass
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import numpy as np

@dataclass(frozen=True)
class ActionSpec:
    joints: tuple[str, ...]
    units: tuple[str, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    cameras: tuple[str, ...] = ("front", "top", "wrist")
    image_shape: tuple[int, int, int] = (480, 640, 3)
    period_ns: int = 50_000_000
    controller: str = "PositionHold.v1"
    semantics: str = "absolute_joint_position_goal"
    state: str = "measured_joint_position"
    version: int = 2

    def __post_init__(self):
        for field in ("joints", "units", "lower", "upper", "cameras", "image_shape"):
            object.__setattr__(self, field, tuple(getattr(self, field)))
        n = len(self.joints)
        if (self.version != 2 or self.controller != "PositionHold.v1" or self.semantics != "absolute_joint_position_goal"
            or self.state != "measured_joint_position" or type(self.period_ns) is not int or self.period_ns <= 0
            or not n or len(set(self.joints)) != n or any(not x for x in self.joints)
            or len(self.units) != n or any(u not in ("rad", "m") for u in self.units)
            or len(self.lower) != n or len(self.upper) != n
            or not np.isfinite(self.lower + self.upper).all() or np.any(np.asarray(self.lower) >= self.upper)
            or len(self.image_shape) != 3 or self.image_shape[2] != 3 or any(type(x) is not int or x <= 0 for x in self.image_shape)
            or not self.cameras or len(set(self.cameras)) != len(self.cameras)):
            raise ValueError("Invalid or unsupported action specification")

    def validate(self, action):
        q = np.asarray(action, dtype=np.float64)
        if q.shape != (len(self.joints),) or not np.isfinite(q).all():
            raise ValueError("Action must contain one finite position per declared joint")
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("Action is outside the declared position bounds")
        return q.copy()

    def to_json(self):
        return json.dumps(asdict(self), sort_keys=True, separators=(",", ":"), allow_nan=False)

    @property
    def identity(self):
        return "sha256:" + hashlib.sha256(self.to_json().encode()).hexdigest()

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text()))

    def require(self, other):
        if self != other:
            raise ValueError(f"Action contract mismatch: expected {self.identity}, received {other.identity}")

@dataclass(frozen=True)
class Profile:
    spec: ActionSpec
    scene: Path
    description: Path
    initial: tuple[float, ...]
    execution_period_ns: int = 10_000_000
    physics_period_ns: int = 1_000_000
    source_timeout_ns: int = 300_000_000
    backend: str = "mujoco-position"
    stop_velocity: float = .02
    stop_settled_ns: int = 100_000_000
    stop_timeout_ns: int = 5_000_000_000

    def native_spec(self):
        from robot_runtime_native._native import RobotSpec
        s = RobotSpec()
        for key in ("joints", "units", "lower", "upper"):
            setattr(s, key, getattr(self.spec, key))
        s.backend = self.backend
        s.validate()
        return s

def so101():
    assets = Path(__file__).parent / "assets" / "so101"
    description = assets / "so101_new_calib.urdf"
    joints = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
    defined = {j.attrib["name"]: j for j in ET.parse(description).getroot().findall("joint")}
    limits = [defined[name].find("limit").attrib for name in joints]
    if any(defined[name].attrib["type"] != "revolute" for name in joints):
        raise ValueError("SO101 requires six bounded revolute joints")
    spec = ActionSpec(joints, ("rad",)*6, tuple(float(x["lower"]) for x in limits), tuple(float(x["upper"]) for x in limits))
    return Profile(spec, assets / "scene.xml", description, (0., -.3, .6, 1.2, 0., -.1))
