"""Command payloads. Angles are in radians and durations are in seconds."""

from dataclasses import dataclass


@dataclass(frozen=True)
class MoveJoint:
    angles: tuple[float, ...]
    duration: float
