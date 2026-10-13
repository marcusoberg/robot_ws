"""Unloaded deployment descriptions; all overrides precede native resource loading."""
from dataclasses import dataclass, field, replace, fields
from pathlib import Path
from typing import Any
import yaml

@dataclass(frozen=True)
class ControlLimit:
    joint: str
    lower: float
    upper: float
    velocity: float

@dataclass(frozen=True)
class RobotConfiguration:
    description: Path
    base: str
    tip: str
    initial: tuple[float, ...]
    auxiliary_joints: tuple[str, ...] = ()
    standby: tuple[float, ...] | None = None
    control_limits: tuple[ControlLimit, ...] = ()

@dataclass(frozen=True)
class ControllerConfiguration:
    period: float = 0.01
    standby_duration: float = 4.0
    speed_fraction: float = 0.35
    acceleration_time: float = 1 / 3
    jerk_time: float = 0.1
    position_gain: float = 8.0
    damping: float = 0.04
    orientation_gain: float | None = None
    tracking_error: tuple[float, ...] | None = None

@dataclass(frozen=True)
class SensorConfiguration:
    name: str
    frame: str
    calibration: str = ""
    units: tuple[str, ...] = ()

@dataclass(frozen=True)
class MujocoConfiguration:
    model: Path
    base_frame: str = ""
    tip_frame: str = ""
    binary_actuator: str = ""
    open_position: float = 0.0
    closed_position: float = 0.0

@dataclass(frozen=True)
class SerialConfiguration:
    device: str = "/dev/ttyACM0"
    baud: int = 3_000_000
    startup_outputs: tuple[bool, bool] | None = None
    first_response_timeout_seconds: float = 1.0
    transmission_timeout_seconds: float = 0.05
    telemetry_timeout_seconds: float = 0.1
    pneumatic_open_high: bool = True
    joints: tuple[str, ...] = ()

@dataclass(frozen=True)
class HostTiming:
    maximum_interval: float
    overrun: str = "fault"

@dataclass(frozen=True)
class Camera:
    """Named MJCF camera. Depth is in metres; RGB has channel order RGB."""
    name: str
    width: int = 320
    height: int = 240
    depth: bool = False
    frame: str = ""
    calibration: str = ""
    frequency: float = 20.0

@dataclass(frozen=True)
class Configuration:
    """Unloaded deployment. Apply resource/controller overrides before Robot.local."""
    robot: RobotConfiguration
    mujoco: MujocoConfiguration | None = None
    controller: ControllerConfiguration = field(default_factory=ControllerConfiguration)
    serial: SerialConfiguration = field(default_factory=SerialConfiguration)
    host_timing: HostTiming | None = None
    sensors: tuple[SensorConfiguration, ...] = ()
    cameras: tuple[Camera, ...] = ()
    external_sensors: tuple[SensorConfiguration, ...] = ()

    @classmethod
    def from_yaml(cls, filename: str | Path) -> "Configuration":
        """Read an unloaded specification. Resolve resource paths relative to YAML."""
        path = Path(filename).resolve()
        raw = yaml.safe_load(path.read_text())
        r = dict(raw["robot"])
        r["description"] = (path.parent / r["description"]).resolve()
        r["control_limits"] = tuple(ControlLimit(name, **v) for name, v in r.get("control_limits", {}).items())
        for name in ("initial", "standby", "auxiliary_joints"):
            if name in r: r[name] = tuple(r[name])
        c = dict(raw["controller"])
        c["period"] = c.pop("period_ms") / 1000
        c["standby_duration"] = c.pop("standby_duration_ms") / 1000
        if "tracking_error" in c: c["tracking_error"] = tuple(c["tracking_error"])
        mj = raw.get("mujoco")
        if mj:
            mj = {**mj, "model": (path.parent / mj["model"]).resolve()}
        serial = dict(raw.get("serial", {}))
        for key in ("joints", "startup_outputs"):
            if key in serial: serial[key] = tuple(serial[key])
        timing = raw.get("host_timing")
        return cls(RobotConfiguration(**r), MujocoConfiguration(**mj) if mj else None, ControllerConfiguration(**c),
                   SerialConfiguration(**serial),
                   HostTiming(timing["maximum_interval_ms"] / 1000, timing.get("overrun", "fault")) if timing else None,
                   tuple(SensorConfiguration(**s) for s in raw.get("sensors", ())), tuple(Camera(**s) for s in raw.get("cameras", ())),
                   tuple(SensorConfiguration(**s) for s in raw.get("external_sensors", ())))

    def with_cameras(self, *cameras: Camera) -> "Configuration":
        return replace(self, cameras=cameras)

    def _native(self, n):
        def convert(kind, value, names=None, skip=()):
            obj = kind()
            for f in fields(value):
                name = (names or {}).get(f.name, f.name)
                if f.name not in skip and hasattr(obj, name):
                    v = getattr(value, f.name)
                    setattr(obj, name, str(v) if isinstance(v, Path) else v)
            return obj
        spec = n.Specification()
        spec.robot = convert(n.RobotSpecification, self.robot, skip=("control_limits",))
        limits = []
        for value in self.robot.control_limits:
            limits.append(convert(n.ControlLimit, value))
        spec.robot.control_limits = limits
        spec.external_sensors = [convert(n.SensorDescription, s) for s in self.external_sensors]
        spec.controller = convert(n.ControllerSpecification, self.controller)
        if self.mujoco: spec.mujoco = convert(n.MujocoSpecification, self.mujoco, {"model": "file"})
        spec.mujoco.sensors = [convert(n.SensorDescription, s) for s in self.sensors]
        spec.serial = convert(n.SerialSpecification, self.serial)
        spec.wire_joints = self.serial.joints
        if self.host_timing:
            policies = {"fault": n.OverrunPolicy.Fault, "brake": n.OverrunPolicy.Brake}
            spec.host_timing = n.HostTiming(self.host_timing.maximum_interval, policies[self.host_timing.overrun])
        return spec
