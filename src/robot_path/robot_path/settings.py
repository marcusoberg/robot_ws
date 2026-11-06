"""Cartesian tracing parameters, in metres, radians and seconds."""

from dataclasses import dataclass
from pathlib import Path

from robot_core.config import read_settings


def path_settings():
    return read_settings(Path(__file__).with_name("path.toml"))


@dataclass(frozen=True)
class PathSettings:
    width: float
    distance: float
    height: float
    speed: float
    linear_rate: float
    angular_rate: float
    sample_period: float
    position_tolerance: float
    maximum_segment_samples: int
    gripper: str
    tip: str

    @classmethod
    def configured(cls, **overrides):
        return cls(**(path_settings()["path"] | overrides))
