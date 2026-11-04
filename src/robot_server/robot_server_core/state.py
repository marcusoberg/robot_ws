"""Backend-independent state consumed by the ROS robot server adapter."""

from dataclasses import dataclass


@dataclass
class ServerState:
    mode: str
    instance: str
    episode: int
    time_ns: int
    execution_time_ns: int
    position: list[float]
    velocity: list[float]
    reference: list[float]
    session: int = 0
    source: str = ""
    sequence: int = 0
    delivered_sequence: int = 0
    reason: str = ""
    recording: bool = False
    wall_lag_seconds: float = 0.0
    feedback_kind: str = "simulation"
    sample_wall_time_ns: int = 0
    reference_valid: bool = True
    timing_uncertainty_seconds: float = 0.0
    motion_execution: str = "sampled"
    motion_id: int = 0
    motion_elapsed_seconds: float = 0.0
    motion_complete: bool = False


@dataclass(frozen=True)
class CommandResult:
    """Expected command acceptance/rejection, separate from unexpected failures."""

    accepted: bool
    message: str
