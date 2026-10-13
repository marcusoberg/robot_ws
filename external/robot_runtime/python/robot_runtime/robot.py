"""Application facade: execution stays in C++; Python submits owned requests."""
from __future__ import annotations
from dataclasses import dataclass
from enum import StrEnum
from types import MappingProxyType
from typing import Any, Sequence, Mapping, Generic, TypeVar, TypedDict, Unpack, cast, overload
import time
import weakref
from .configuration import Configuration
from .types import (Observation, Motion, JointTarget, JointVelocity, Pose, Twist, Trajectory, ReferenceChunk, TimedJointTargets, Capture,
                    Mode, RobotError, AuthorityLost, OperationRejected, MotionRejected, EventHistoryLost, StaleObservation, SampleStamp, ImageSample, freeze)

class Outcome(StrEnum):
    QUEUED = "queued"
    ACCEPTED = "accepted"
    COMPLETED = "completed"
    REJECTED = "rejected"
    CANCELED = "canceled"

@dataclass(frozen=True)
class OperationResult:
    outcome: Outcome
    reason: str = "none"

_OPERATOR_REASONS = ("none", "invalid_state", "no_authority", "not_homed", "unavailable", "interrupted")

T = TypeVar("T")
U = TypeVar("U")

class Operation(Generic[T]):
    """A lifecycle request. result() waits for execution, never advances physics."""
    def __init__(self, robot: Robot, identifier: int, action: str, basis: Observation, source: ControlSource | None = None):
        self._robot = robot
        self._source = source or robot._operator_source
        self._id = identifier
        self._action = action
        self._epoch = basis.epoch
        self._seen = int(basis._data["operator_sequence"])
        self._result = OperationResult(Outcome.QUEUED)
        self._value: Any = None

    def _poll(self, observation: Observation) -> None:
        if self.done: return
        try: self._robot._transport.check_error(self._id)
        except MotionRejected as error: raise OperationRejected(str(error)) from error
        if observation.epoch != self._epoch:
            self._result = OperationResult(Outcome.CANCELED, "timeline_reset")
            return
        events = observation._data["operator_results"]
        if events and int(events[0]["sequence"]) > self._seen + 1:
            raise EventHistoryLost("Execution outcomes were coalesced before this operation could be observed")
        for event in events:
            self._seen = max(self._seen, int(event["sequence"]))
            if int(event["source"]) != self.source_id or int(event["id"]) != self._id: continue
            self._result = OperationResult((Outcome.REJECTED, Outcome.ACCEPTED, Outcome.COMPLETED, Outcome.CANCELED)[event["disposition"]],
                                           _OPERATOR_REASONS[event["reason"]])
            if self._result.outcome == Outcome.COMPLETED and self._action in ("Acquire", "Takeover"):
                if observation._data["owner"] is None or int(observation._data["owner"]) != self.source_id or event["session_generation"] is None or int(event["session_generation"]) != int(observation._data["generation"]):
                    raise AuthorityLost("Control was revoked before acquisition was observed")
                self._value = ControlSession(self._source, int(event["session_generation"]), observation.epoch)
            if self.done: break

    @property
    def command_id(self) -> int: return self._id
    @property
    def source_id(self) -> int: return self._source.source_id
    @property
    def done(self) -> bool: return self._result.outcome in (Outcome.COMPLETED, Outcome.REJECTED, Outcome.CANCELED)
    @property
    def outcome(self) -> OperationResult:
        self._poll(self._robot.observe())
        return self._result
    def _resolve(self) -> T:
        if self._result.outcome in (Outcome.REJECTED, Outcome.CANCELED):
            raise OperationRejected(f"{self._action}: {self._result.reason}")
        return cast(T, self._value if self._value is not None else self._result)
    def result(self, timeout: float = 5.0) -> T:
        """Wait for the actual terminal outcome. Use step_until() in stepped mode."""
        self._robot._wait(self, timeout)
        return self._resolve()

class MotionOperation:
    """Admission is distinct from queuing. Streams admit; finite trajectories complete."""
    def __init__(self, session: ControlSession, identifier: int, finite: bool, fit_report=None):
        self._session = session
        self._robot = session._robot
        self._id = identifier
        self._finite = finite
        self.fit_report = freeze(fit_report or {})
        self._result = OperationResult(Outcome.QUEUED)
    def _matches(self, reference):
        return reference is not None and int(reference["source"]) == self.source_id and int(reference["generation"]) == self._session._generation and int(reference["id"]) == self._id
    def _poll(self, observation: Observation) -> None:
        if self.done: return
        self._robot._transport.check_error(self._id)
        if observation.epoch != self._session._epoch:
            self._result = OperationResult(Outcome.CANCELED, "timeline_reset"); return
        for name, outcome in (("rejected", Outcome.REJECTED), ("completed", Outcome.COMPLETED), ("canceled", Outcome.CANCELED)):
            if self._matches(observation._data.get(name)):
                self._result = OperationResult(outcome, observation.rejection if name == "rejected" else "none")
                return
        if not self._session._owns(observation):
            self._result = OperationResult(Outcome.CANCELED, "authority_lost"); return
        if self._matches(observation._data.get("accepted")) or self._matches(observation._data.get("active")):
            self._result = OperationResult(Outcome.ACCEPTED)
    @property
    def command_id(self) -> int: return self._id
    @property
    def source_id(self) -> int: return self._session.source_id
    @property
    def done(self) -> bool:
        return self._result.outcome in (Outcome.COMPLETED, Outcome.REJECTED, Outcome.CANCELED) or (not self._finite and self._result.outcome == Outcome.ACCEPTED)
    @property
    def outcome(self) -> OperationResult:
        self._poll(self._robot.observe()); return self._result
    def _resolve(self):
        if self._result.outcome in (Outcome.REJECTED, Outcome.CANCELED): raise MotionRejected(self._result.reason)
        return self._result
    def result(self, timeout: float = 10.0) -> OperationResult:
        self._robot._wait(self, timeout); return self._resolve()
    def wait_accepted(self, timeout: float = 5.0) -> OperationResult:
        self._robot._wait(self, timeout, admitted=True); return self._resolve()

class SubmissionOptions(TypedDict, total=False):
    observation: Observation | None
    valid_for: float
    gripper_open: bool | None

class ControlSession:
    """Owned authority. Renewal is explicit; exiting requests release."""
    def __init__(self, source: ControlSource, generation: int, epoch: int):
        self.source = source
        self._robot, self._generation, self._epoch = source._robot, generation, epoch
        self._closed = False
    def _owns(self, observation: Observation) -> bool:
        return not self._closed and observation.epoch == self._epoch and observation._data["owner"] is not None and int(observation._data["owner"]) == self.source_id and int(observation._data["generation"]) == self._generation
    @property
    def generation(self) -> int: return self._generation
    @property
    def source_id(self) -> int: return self.source.source_id
    @property
    def epoch(self) -> int: return self._epoch
    def renew(self, *, observation=None, valid_for=None) -> bool: return self.source.renew(self, observation=observation, valid_for=valid_for)
    @property
    def valid(self) -> bool: return self._owns(self._robot.observe())
    def submit(self, motion: Motion, *, observation: Observation | None = None, valid_for: float = 0.3, gripper_open: bool | None = None) -> MotionOperation:
        """Copy a motion request. Freshness uses its observation basis, not send time."""
        current = self._robot.observe()
        if not self._owns(current): raise AuthorityLost("This session no longer owns the robot")
        basis = current if observation is None else observation
        if basis._origin is not self._robot._transport or basis.epoch != current.epoch:
            raise StaleObservation("Observation belongs to another connection or timeline")
        try: identifier, queued, fit = self.source._submit(motion, self._generation, valid_for, basis, gripper_open)
        except (ValueError, TypeError) as error: raise MotionRejected(str(error)) from error
        if not queued: raise MotionRejected("motion ingress is full")
        return self._robot._track(MotionOperation(self, identifier, isinstance(motion, Trajectory), fit))
    def target_joints(self, position: Sequence[float], **kwargs: Unpack[SubmissionOptions]) -> MotionOperation:
        """Stream joint positions in model order and units; bypass IK."""
        return self.submit(JointTarget(position), **kwargs)
    def joint_velocity(self, velocity: Sequence[float], **kwargs: Unpack[SubmissionOptions]) -> MotionOperation: return self.submit(JointVelocity(velocity), **kwargs)
    def target_pose(self, position: Sequence[float], quaternion_wxyz: Sequence[float], *, frame: str | None = None, auxiliary_positions: Mapping[str, float] | None = None, **kwargs: Unpack[SubmissionOptions]) -> MotionOperation:
        """Stream base_from_tool. Defaults to configured base/tool frames."""
        return self.submit(Pose(position, quaternion_wxyz, frame, auxiliary_positions or {}), **kwargs)
    def twist(self, values: Sequence[float], *, frame: str | None = None, **kwargs: Unpack[SubmissionOptions]) -> MotionOperation: return self.submit(Twist(values, frame), **kwargs)
    def trajectory(self, trajectory: Trajectory, **kwargs: Unpack[SubmissionOptions]) -> MotionOperation: return self.submit(trajectory, **kwargs)
    def reference_chunk(self, chunk: ReferenceChunk, **kwargs: Unpack[SubmissionOptions]) -> MotionOperation: return self.submit(chunk, **kwargs)
    def pause(self) -> Operation[OperationResult]:
        if not self.valid: raise AuthorityLost("This session no longer owns the robot")
        return self.source._operate("Pause", generation=self.generation)
    def timed_joint_targets(self, targets: TimedJointTargets, **kwargs: Unpack[SubmissionOptions]) -> MotionOperation: return self.submit(targets, **kwargs)
    def release(self) -> Operation[OperationResult] | None:
        if self._closed or self._robot._closed: return None
        owned = self._owns(self._robot.observe()); self._closed = True
        return self.source._operate("Release", generation=self.generation) if owned else None
    def __enter__(self) -> ControlSession:
        if not self.valid: raise AuthorityLost("This session no longer owns the robot")
        return self
    def __exit__(self, *exc): self.release()

class ControlSource:
    """Independent actor identity. The caller explicitly renews a live session."""
    def __init__(self,robot: Robot,native):self._robot,self._native=robot,native
    @property
    def source_id(self)->int:return self._native.source_id
    @property
    def name(self)->str:return self._native.name
    @property
    def instance(self)->str:return self._native.instance
    def _operate(self,action,generation=None):
        basis=self._robot.observe();identifier,queued=self._native.operate(getattr(self._robot._transport.n.Action,action),generation)
        if not queued:raise OperationRejected("operator ingress is full")
        return self._robot._track(Operation(self._robot,identifier,action,basis,self))
    def acquire_control(self,*,takeover=False)->Operation[ControlSession]:return self._operate("Takeover" if takeover else "Acquire")
    def renew(self,session: ControlSession,*,observation=None,valid_for=None)->bool:
        if session.source is not self:raise AuthorityLost("Session belongs to another source")
        if session._closed or self._robot._closed:return False
        basis=self._robot.observe() if observation is None else observation
        if basis._origin is not self._robot._transport:raise StaleObservation("Use an observation from this robot")
        return self._native.renew(session.generation,basis._native,valid_for)
    def release(self,session: ControlSession):
        if session.source is not self:raise AuthorityLost("Session belongs to another source")
        return session.release()
    def _submit(self,value,generation,ttl,basis,gripper):
        from ._local import motion
        return self._native.submit(motion(value,self._robot._transport.n,self._robot.description["base"],basis.epoch),generation,ttl,basis._native,gripper)

class Robot:
    """Own a native runtime. Every request uses the direct C++ SDK."""
    def __init__(self, transport, *, configuration=None, mode="realtime"):
        self._transport, self.configuration, self.mode = transport, configuration, mode
        self.description = freeze(dict(transport.description))
        self._closed = False
        self._operations = weakref.WeakSet()
        self._cameras = None
        self._sources = {}
        self._operator_source = ControlSource(self, transport.operator_source)
    @classmethod
    def local(cls, configuration: Configuration, *, backend: str = "mujoco", mode: str = "realtime", action_period: float | None = None) -> Robot:
        """Open disabled. Stepped mode advances only through step/step_until."""
        if mode not in ("realtime", "stepped"): raise ValueError("mode must be realtime or stepped")
        from ._local import Local
        robot = cls(Local(configuration, backend, mode, action_period), configuration=configuration, mode=mode)
        try:
            if configuration.cameras:
                from ._cameras import Cameras
                robot._cameras = Cameras(robot, configuration)
        except BaseException:
            robot.close(); raise
        return robot
    def _track(self, operation: U) -> U: self._operations.add(operation); return operation
    def frames(self, observation: Observation | None = None) -> Mapping[str, Pose]:
        """Read URDF link transforms using separate kinematics storage, outside control.

        This read uses separate kinematics storage outside control.
        """
        basis = self.observe() if observation is None else observation
        if self.configuration is None: raise RobotError("Frame computation belongs to the local runtime; use its published transforms")
        if basis._origin is not self._transport: raise StaleObservation("Use an observation from this robot")
        return freeze({name: Pose(value["position"], value["quaternion_wxyz"], self.description["base"])
                       for name, value in self._transport.runtime.frames(basis._native).items()})

    def observe(self) -> Observation:
        """Latest feedback, independent of camera capture. Does not advance time."""
        if self._closed: raise RobotError("Robot is closed")
        return self._transport.observe()
    def source(self, name: str, instance: str) -> ControlSource:
        key=(name,instance)
        if key not in self._sources:self._sources[key]=ControlSource(self,self._transport.runtime.source(name,instance))
        return self._sources[key]
    def _operate(self, action: str) -> Operation[Any]:return self._operator_source._operate(action)
    def enable(self) -> Operation[OperationResult]: return self._operate("Enable")
    def disable(self) -> Operation[OperationResult]: return self._operate("Disable")
    def stop(self) -> Operation[OperationResult]: return self._operate("Stop")
    def hold(self) -> Operation[OperationResult]: return self._operate("Hold")
    def standby(self) -> Operation[OperationResult]: return self._operate("Standby")
    def open_gripper(self) -> Operation[OperationResult]: return self._operate("OpenGripper")
    def close_gripper(self) -> Operation[OperationResult]: return self._operate("CloseGripper")
    def acquire_control(self, *, takeover: bool = False) -> Operation[ControlSession]: return self._operator_source.acquire_control(takeover=takeover)
    def _update(self, observation):
        for operation in tuple(self._operations): operation._poll(observation)
    def _wait(self, operation, timeout: float, admitted=False):
        deadline = time.monotonic() + timeout
        while True:
            observation = self.observe(); operation._poll(observation)
            if operation.done or (admitted and operation._result.outcome == Outcome.ACCEPTED): return
            remaining = deadline - time.monotonic()
            if remaining <= 0: raise TimeoutError("Operation did not finish; waiting never advances simulation")
            self._transport.wait(observation, min(remaining, 0.05))
    def step(self, n: int = 1) -> Observation:
        """Advance n configured action periods, without sleeping."""
        if self.mode != "stepped": raise RobotError("step requires a local stepped simulation")
        if type(n) is not int or n < 1: raise ValueError("n must be a positive integer")
        self._transport.runtime.step(n)
        observation = self.observe(); self._update(observation)
        return self._cameras.capture(observation) if self._cameras else observation
    @overload
    def step_until(self, operation: Operation[T], *, max_steps: int = 10_000) -> T: ...
    @overload
    def step_until(self, operation: MotionOperation, *, max_steps: int = 10_000) -> OperationResult: ...
    def step_until(self, operation: Operation[T] | MotionOperation, *, max_steps: int = 10_000) -> T | OperationResult:
        """Explicitly advance control intervals until the operation finishes."""
        if self.mode != "stepped" or operation._robot is not self: raise RobotError("step_until requires an operation on this stepped robot")
        if type(max_steps) is not int or max_steps < 1: raise ValueError("max_steps must be a positive integer")
        for _ in range(max_steps):
            observation = self.observe(); operation._poll(observation)
            if operation.done: return operation._resolve()
            self._transport.runtime.tick()
            self._update(self.observe())
            if operation.done: return operation._resolve()
        raise TimeoutError("Operation exceeded max_steps")
    def reset(self) -> Observation:
        """Reset local simulation, disable it and invalidate all old sessions."""
        if self.configuration is None: raise RobotError("Remote reset is an operator deployment operation")
        self._transport.runtime.reset()
        if self._cameras: self._cameras.clear()
        observation = self.observe(); self._update(observation); return observation
    def capture(self, *, timeout: float = 5.0) -> Observation:
        """Images and their original robot observation; never advances physics."""
        if self._cameras:
            return self._cameras.latest(timeout) if self.mode == "realtime" else self._cameras.capture(self.observe(), timeout)
        return self._transport.capture(timeout)
    def capture_frame(self, *, timeout: float = 5.0) -> Capture:
        observation=self.capture(timeout=timeout)
        return Capture(observation, observation.images, int((observation.image_delivery or {}).get("frame_sequence",0)))
    def publish_numeric(self, name: str, values: Sequence[float], *, stamp: SampleStamp) -> None:
        """Publish a configured external sensor sample with its original source clock."""
        if self.configuration is None: raise RobotError("External sensor publication requires local acquisition")
        self._transport.runtime.publish_numeric(name, values, stamp.sequence, stamp.source_clock, stamp.source_time_ns, stamp.epoch)
    def publish_images(self, images: Mapping[str, ImageSample], *, observation: Observation | None = None) -> Observation:
        """Publish configured external RGB/depth samples, paired with a robot snapshot.

        Acquisition runs in your application. Each ImageSample keeps its source
        timestamp, calibration and extrinsics; capture() exposes the same frame.
        """
        if self.configuration is None: raise RobotError("External image acquisition requires a local runtime")
        from ._images import publish_external
        basis = self.observe() if observation is None else observation
        if basis._origin is not self._transport: raise StaleObservation("Use an observation from this robot")
        captured = publish_external(self, images, basis)
        return captured
    def close(self) -> None:
        if self._closed: return
        self._closed = True
        if self._cameras: self._cameras.close()
        self._transport.close()
    def __enter__(self) -> Robot: return self
    def __exit__(self, *exc): self.close()
