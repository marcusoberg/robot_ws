"""Public immutable values. Arrays carry their own lifetime, never servo storage."""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import StrEnum
from types import MappingProxyType
from typing import Any, Mapping, Sequence, TYPE_CHECKING
if TYPE_CHECKING:
    from .robot import MotionOperation
import time
import numpy as np
from numpy.typing import NDArray

class Mode(StrEnum):
    DISABLED = "DISABLED"
    HOLD = "HOLD"
    AUTO = "AUTO"
    DIRECT = "DIRECT"
    STOPPING = "CONTROLLED_STOP"
    FAULT = "FAULT"

class RobotError(RuntimeError): pass
class AuthorityLost(RobotError): pass
class OperationRejected(RobotError): pass
class MotionRejected(RobotError): pass
class EventHistoryLost(RobotError): pass
class StaleObservation(RobotError): pass
class CameraError(RobotError): pass

def readonly(values, dtype=np.float64) -> NDArray:
    a = np.asarray(values, dtype=dtype)
    if a.flags.writeable:
        a = np.frombuffer(a.tobytes(), dtype=a.dtype).reshape(a.shape)
    return a

def freeze(value):
    if isinstance(value, dict): return MappingProxyType({k: freeze(v) for k, v in value.items()})
    if isinstance(value, list): return tuple(freeze(v) for v in value)
    return value

@dataclass(frozen=True)
class SampleStamp:
    sequence: int
    epoch: int
    source_clock: str
    source_time_ns: int | None
    host_received_ns: int | None
    @classmethod
    def from_data(cls, v, epoch=0):
        return cls(int(v["sequence"]), int(v.get("epoch", epoch)), v["source_clock"],
                   int(v["source_time_ns"]) if v.get("source_time_ns") is not None else None,
                   int(v["host_received_ns"]) if v.get("host_received_ns") is not None else None)

@dataclass(frozen=True)
class Pose:
    """Tool pose in metres, with a WXYZ quaternion in the configured base frame."""
    position: Sequence[float]
    quaternion_wxyz: Sequence[float]
    frame: str | None = None
    auxiliary_positions: Mapping[str, float] = field(default_factory=dict)
    def __post_init__(self):
        object.__setattr__(self, "position", tuple(self.position))
        object.__setattr__(self, "quaternion_wxyz", tuple(self.quaternion_wxyz))
        object.__setattr__(self, "auxiliary_positions", freeze(dict(self.auxiliary_positions)))
@dataclass(frozen=True)
class JointTarget:
    values: Sequence[float]
    def __post_init__(self): object.__setattr__(self, "values", tuple(self.values))
@dataclass(frozen=True)
class JointVelocity:
    values: Sequence[float]
    def __post_init__(self): object.__setattr__(self, "values", tuple(self.values))
@dataclass(frozen=True)
class Twist:
    values: Sequence[float]
    frame: str | None = None
    def __post_init__(self): object.__setattr__(self, "values", tuple(self.values))
@dataclass(frozen=True)
class Waypoint:
    seconds: float
    position: Sequence[float]
    velocity: Sequence[float] | None = None
    def __post_init__(self):
        object.__setattr__(self, "position", tuple(self.position))
        if self.velocity is not None: object.__setattr__(self, "velocity", tuple(self.velocity))
@dataclass(frozen=True)
class Trajectory:
    """Finite joint path; waypoint seconds are offsets from the first point."""
    points: Sequence[Waypoint]
    def __post_init__(self): object.__setattr__(self, "points", tuple(self.points))
@dataclass(frozen=True)
class ReferencePoint:
    seconds: float
    position: Sequence[float]
    velocity: Sequence[float]
    acceleration: Sequence[float]
    def __post_init__(self):
        for name in ("position", "velocity", "acceleration"): object.__setattr__(self, name, tuple(getattr(self, name)))
@dataclass(frozen=True)
class ReferenceChunk:
    """Execution-timed joint reference; replaces names a previous operation handle."""
    start: float
    points: Sequence[ReferencePoint]
    replaces: MotionOperation | None = None
    def __post_init__(self): object.__setattr__(self, "points", tuple(self.points))
@dataclass(frozen=True)
class TimedJointTargets:
    """Absolute target samples on the original runtime timeline.

    start_ns is chosen once from a current observation plus an explicit lead.
    The native runtime fits knots without moving their timestamps.
    """
    positions: Sequence[Sequence[float]]
    origin_ns: int
    period_ns: int
    basis: ObservationIdentity
    start_ns: int | None = None
    lead_ns: int = 150_000_000
    join_seconds: float = 1.0
    fit_tolerance: float | Sequence[float] = 0.02
    predecessor: MotionOperation | int | None = None
    joints: tuple[str, ...] | None = None
    def __post_init__(self):
        object.__setattr__(self,"positions",tuple(tuple(row) for row in self.positions))
        if self.joints is not None: object.__setattr__(self,"joints",tuple(self.joints))
        if not np.isscalar(self.fit_tolerance): object.__setattr__(self,"fit_tolerance",tuple(self.fit_tolerance))
        for name in ("origin_ns","period_ns","lead_ns")+(("start_ns",) if self.start_ns is not None else ()):
            if type(getattr(self,name)) is not int: raise TypeError(f"{name} must be an integer nanosecond count")

Motion = JointTarget | JointVelocity | Pose | Twist | Trajectory | ReferenceChunk | TimedJointTargets

@dataclass(frozen=True)
class RequestedMotion:
    motion: Motion
    gripper_open: bool | None

@dataclass(frozen=True)
class TimedReference:
    start: float
    end: float

@dataclass(frozen=True)
class ScheduleStatus:
    current: TimedReference
    pending: TimedReference | None

@dataclass(frozen=True)
class Transition:
    from_mode: Mode
    to_mode: Mode
    time: float
    epoch: int
    reason: str
    owned: bool

def _motion_from_data(v):
    # A view of an already decoded proposal. Factory selection is transport-only;
    # dimension/feasibility validation remains at the native ingress boundary.
    constructors = {
        "joint_position": lambda: JointTarget(v["values"]),
        "joint_velocity": lambda: JointVelocity(v["values"]),
        "pose": lambda: Pose(v["position"], v["quaternion_wxyz"], v.get("frame"), v.get("auxiliary_positions") or {}),
        "twist": lambda: Twist(v["values"], v.get("frame")),
        "trajectory": lambda: Trajectory(tuple(Waypoint(p["seconds"], p["position"], p.get("velocity")) for p in v["points"])),
        "joint_schedule": lambda: ReferenceChunk(int(v["start_ns"]) / 1e9,
            tuple(ReferencePoint(p["seconds"], p["position"], p["velocity"], p["acceleration"]) for p in v["points"])),
    }
    return constructors[v["kind"]]()

@dataclass(frozen=True)
class JointState:
    position: NDArray[np.float64]
    velocity: NDArray[np.float64]
    effort: NDArray[np.float64] | None
    stamp: SampleStamp

@dataclass(frozen=True)
class NumericSample:
    values: NDArray[np.float64]
    units: tuple[str, ...]
    frame: str
    calibration: str
    stamp: SampleStamp

@dataclass(frozen=True)
class ImageSample:
    pixels: NDArray
    frame: str
    calibration: str
    stamp: SampleStamp
    intrinsics: NDArray[np.float64]
    world_from_camera: NDArray[np.float64]
    def __post_init__(self):
        object.__setattr__(self, "pixels", readonly(self.pixels, self.pixels.dtype))
        object.__setattr__(self, "intrinsics", readonly(self.intrinsics))
        object.__setattr__(self, "world_from_camera", readonly(self.world_from_camera))

@dataclass(frozen=True)
class ObservationIdentity:
    runtime_instance: str
    timeline_generation: int
    observation_sequence: int

@dataclass(frozen=True)
class Ownership:
    present: bool
    session_generation: int
    source: str | None

@dataclass(frozen=True)
class Timing:
    control_interval: float
    host_interval: float
    cycle_duration: float
    coalesced_observations: int

@dataclass(frozen=True)
class AppliedOperation:
    sequence: int
    action: str
    outcome: str
    reason: str

@dataclass(frozen=True)
class Observation:
    """Immutable measured state, accepted reference, delivery and sensor evidence."""
    mode: Mode
    time: float
    epoch: int
    joints: JointState | None
    tool_pose: Pose | None
    motion_limited: bool
    rejection: str
    interruption: str
    reference: Mapping[str, Any]
    delivery: Mapping[str, Any]
    sensors: Mapping[str, NumericSample]
    images: Mapping[str, ImageSample] = field(default_factory=lambda: MappingProxyType({}))
    _data: Mapping[str, Any] = field(default_factory=dict, repr=False, compare=False)
    _native: Any = field(default=None, repr=False, compare=False)
    _received: float = field(default_factory=time.monotonic, repr=False, compare=False)
    _origin: Any = field(default=None, repr=False, compare=False)
    _proposal: RequestedMotion | None = field(default=None, repr=False, compare=False)

    @property
    def sequence(self) -> int: return int(self._data["observation_sequence"])
    @property
    def age(self) -> float: return time.monotonic() - self._received
    @property
    def requested_motion(self) -> RequestedMotion | None: return self._proposal
    @property
    def schedule(self) -> ScheduleStatus | None:
        value = self._data.get("schedule")
        def timed(v): return TimedReference(int(v["start_ns"]) / 1e9, int(v["end_ns"]) / 1e9)
        return ScheduleStatus(timed(value["current"]), timed(value["pending"]) if value["pending"] else None) if value else None
    @property
    def timing(self) -> Timing:
        v = self._data["timing"]
        return Timing(int(v["control_dt_ns"])/1e9, int(v["host_interval_ns"])/1e9,
                      int(v["cycle_duration_ns"])/1e9, int(v["coalesced_observations"]))
    @property
    def identity(self) -> ObservationIdentity:
        return ObservationIdentity(self._origin.description["instance"], self.epoch, self.sequence)
    @property
    def ownership(self) -> Ownership:
        source = self._data.get("owner")
        return Ownership(source is not None, int(self._data["generation"]), str(source) if source is not None else None)
    @property
    def operation_outcomes(self) -> tuple[AppliedOperation, ...]:
        """Named lifecycle outcomes applied by execution, rather than queue acknowledgements."""
        actions = ("enable", "disable", "stop", "hold", "standby", "open_gripper", "close_gripper", "acquire", "release", "takeover", "pause")
        outcomes = ("rejected", "accepted", "completed", "canceled")
        reasons = ("none", "invalid_state", "no_authority", "not_homed", "unavailable", "interrupted")
        return tuple(AppliedOperation(int(v["sequence"]), actions[int(v["action"])],
                                      outcomes[int(v["disposition"])], reasons[int(v["reason"])])
                     for v in self._data["operator_results"])
    def to_record(self) -> dict[str, Any]:
        """Owned JSON-compatible canonical observation, with the original runtime identity.

        Image bytes travel separately; image metadata retains the capture basis.
        """
        from dataclasses import asdict
        def plain(v):
            if isinstance(v, np.ndarray): return v.tolist()
            if isinstance(v, Mapping): return {k: plain(x) for k,x in v.items()}
            if isinstance(v, (tuple, list)): return [plain(x) for x in v]
            return v
        record = {k: plain(v) for k,v in self._data.items() if True}
        record["identity"] = asdict(self.identity)
        # C++ steady_clock and Python monotonic can have different origins,
        # including within one macOS host. Absolute counts require a mapping.
        record["runtime_host_clock"] = f"runtime.host.steady:{self.identity.runtime_instance}"
        record["images"] = {name: {"stamp": asdict(image.stamp), "frame": image.frame,
                            "calibration": image.calibration, "shape": list(image.pixels.shape),
                            "dtype": image.pixels.dtype.str, "intrinsics": image.intrinsics.tolist(),
                            "world_from_camera": image.world_from_camera.tolist()}
                            for name,image in self.images.items()}
        return record
    @property
    def transitions(self) -> tuple[Transition, ...]:
        return tuple(Transition(Mode(v["from"]), Mode(v["to"]), int(v["time_ns"]) / 1e9, int(v["epoch"]), v["reason"], v["owner"] is not None)
                     for v in self._data.get("transitions", ()))
    @property
    def image_delivery(self): return self._data.get("image_delivery")
    @property
    def tracking_error(self) -> NDArray[np.float64] | None:
        value = self._data.get("tracking_error")
        return readonly(value) if value is not None else None
    @property
    def collision(self) -> bool | None: return self._data.get("collision")
    def _measured(self, name):
        state = self._data.get("state")
        return state.get(name) if state is not None else None
    @property
    def enabled(self) -> bool | None: return self._measured("enabled")
    @property
    def homed(self) -> bool | None: return self._measured("homed")
    @property
    def physical_estop(self) -> bool | None: return self._measured("physical_estop")
    @property
    def hardware_fault(self) -> bool | None: return self._measured("fault")
    @property
    def gripper_open(self) -> bool | None: return self._measured("gripper_open")
    @classmethod
    def from_data(cls, raw, native=None, received=None, images=None, origin=None):
        epoch = int(raw["epoch"])
        state = raw.get("state")
        joints = None if state is None else JointState(readonly(state["position"]), readonly(state["velocity"]),
            readonly(state["effort"]) if state.get("effort") is not None else None, SampleStamp.from_data(state, epoch))
        tool = raw.get("tool_pose")
        sensors = {name: NumericSample(readonly(s["values"]), tuple(s["units"]), s["frame"], s["calibration"], SampleStamp.from_data(s, epoch))
                   for name, s in raw["sensors"].items()}
        request = raw.get("requested_motion")
        proposal = RequestedMotion(_motion_from_data(request["motion"]), request.get("gripper_open")) if request else None
        ref = dict(raw["actuator_request"])
        for key in ("position", "velocity", "acceleration"): ref[key] = readonly(ref[key])
        return cls(Mode(raw["mode"]), int(raw["time_ns"]) / 1e9, epoch, joints,
                   Pose(tool["position"], tool["quaternion_wxyz"]) if tool else None, raw["motion_limited"], raw["rejection"], raw["interruption"],
                   freeze(ref), freeze(raw["delivery"]), MappingProxyType(sensors), MappingProxyType(images or {}), freeze(dict(raw)), native,
                   time.monotonic() if received is None else received, origin, proposal)


@dataclass(frozen=True)
class Capture:
    """Immutable image samples and the exact robot observation used for capture."""
    observation: Observation
    images: Mapping[str, ImageSample]
    sequence: int
    @property
    def identity(self): return self.observation.identity
