"""Transport-neutral recording contracts. Source time never becomes publication time."""

from typing import Any, Literal, Self
from uuid import uuid4
from pydantic import BaseModel, ConfigDict, Field, model_validator


class Contract(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", allow_inf_nan=False)


class Identity(Contract):
    runtime_instance: str = Field(min_length=1)
    timeline_generation: int = Field(ge=0)
    observation_sequence: int = Field(ge=0)


class SourceStamp(Contract):
    clock: str
    time_ns: int | None
    sequence: int = Field(ge=0)
    received_host_ns: int
    received_host_clock: str = "unqualified.host.monotonic"


class ClockMapping(Contract):
    version: str
    source: str
    destination: str
    offset_ns: float
    drift: float = Field(default=1.0, gt=0)
    uncertainty_ns: float = Field(ge=0)
    valid_from_ns: int
    accuracy_budget_ns: float = Field(ge=0)

    @model_validator(mode="after")
    def bounded_uncertainty(self) -> Self:
        if self.uncertainty_ns > self.accuracy_budget_ns:
            raise ValueError(
                "Clock mapping exceeds the declared alignment accuracy budget"
            )
        return self

    def convert(self, stamp: int) -> int:
        return round(self.drift * stamp + self.offset_ns)


class StreamDescription(Contract):
    name: str
    schema_version: str = "robot.data.v1"
    source_clock: str
    frame: str = ""
    calibration: str = ""
    delivery: Literal["ordered", "latest"] = "latest"


class Episode(Contract):
    id: str = Field(min_length=1)
    instruction: str
    kind: Literal["demonstration", "recovery", "policy", "evaluation"]
    parent: str | None = None
    approved: bool = False
    success: bool | None = None


class Event(Contract):
    event_id: str = Field(default_factory=lambda: uuid4().hex, min_length=1)
    schema_version: str = "robot.event.v1"
    kind: str
    host_ns: int = Field(ge=0)
    host_clock: str = "collector.host.monotonic"
    basis: Identity | None = None
    episode: Episode | None = None
    details: dict[str, Any] = Field(default_factory=dict)


class CaptureRecord(Contract):
    schema_version: str = "robot.capture.v1"
    identity: Identity
    episode: Episode | None
    observation: dict[str, Any]
    host_received_ns: int
    host_published_ns: int
    host_clock: str = "collector.host.monotonic"
    stream_sequence: int | None = Field(default=None, ge=0)
    publisher_host_ns: int | None = Field(default=None, ge=0)
    publisher_host_clock: str | None = None


class RecordingStatus(Contract):
    schema_version: str = "robot.recording.status.v1"
    host_ns: int = Field(ge=0)
    active: bool
    frames: int = Field(ge=0)
    capture_gaps: int = Field(ge=0)
    dropped: int = Field(ge=0)
    queue_depth: int = Field(ge=0)
    error: str | None = None
