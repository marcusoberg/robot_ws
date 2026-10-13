"""Static SDK compatibility checks. No Node or native runtime is constructed."""

import ast
import inspect
from dataclasses import fields, is_dataclass
from pathlib import Path
from types import UnionType
from typing import Union, get_args, get_origin, get_type_hints

from robot_runtime import (
    Camera, Configuration, ControlSession, ControlSource, ImageSample,
    MotionOperation, Observation, ObservationIdentity, Operation, OperationResult,
    Pose, Robot, SampleStamp, TimedJointTargets, Trajectory, Waypoint,
)
from robot_runtime.types import JointState, NumericSample, Ownership, Timing


def _chain(node):
    names = []
    while isinstance(node, ast.Attribute):
        names.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        return (node.id, *reversed(names))
    return ()


def _without_none(kind):
    if get_origin(kind) in (Union, UnionType):
        choices = [choice for choice in get_args(kind) if choice is not type(None)]
        if len(choices) == 1:
            return choices[0]
    return kind


def _members(kind):
    members = set(dir(kind))
    if is_dataclass(kind):
        members.update(field.name for field in fields(kind))
    # Some facade values, e.g. Robot.description and MotionOperation.fit_report,
    # are owned instance attributes rather than class properties.
    tree = ast.parse(inspect.getsource(kind))
    for node in ast.walk(tree):
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else (node.target,)
            for target in targets:
                chain = _chain(target)
                if len(chain) == 2 and chain[0] == "self":
                    members.add(chain[1])
    return members


def validate_sdk_usage():
    """Return SDK member mismatches found in the installed adapter source.

    Configuration chains are derived from the source and compared with the
    real SDK dataclasses. Public facade methods are inspected as class members.
    This checks installation compatibility without opening robot resources.
    """
    errors = []
    directory = Path(__file__).parent
    prefixes = (("self", "configuration"), ("configuration",))
    checked = set()
    for filename in ("node.py", "visualization.py"):
        tree = ast.parse((directory / filename).read_text(), filename=filename)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Attribute):
                continue
            chain = _chain(node)
            prefix = next((p for p in prefixes if chain[:len(p)] == p), None)
            if prefix is None:
                continue
            path = chain[len(prefix):]
            if path in checked:
                continue
            checked.add(path)
            kind = Configuration
            for name in path:
                kind = _without_none(kind)
                if not is_dataclass(kind):
                    break  # Path/string/container methods are outside the SDK.
                members = {field.name for field in fields(kind)}
                if name not in members:
                    errors.append(f"{filename}:{node.lineno}: SDK {kind.__name__} has no field {name!r}")
                    break
                kind = get_type_hints(kind)[name]

    required = {
        Robot: ("description", "local", "observe", "frames", "source", "step", "reset", "capture", "close", "enable", "disable",
                "stop", "hold", "standby", "open_gripper", "close_gripper"),
        ControlSource: ("source_id", "acquire_control", "renew", "release"),
        ControlSession: ("generation", "valid", "pause", "target_joints", "joint_velocity", "target_pose", "twist",
                         "trajectory", "timed_joint_targets"),
        Operation: ("outcome", "result", "done", "command_id"),
        MotionOperation: ("outcome", "done", "command_id", "fit_report"),
        OperationResult: ("outcome", "reason"),
        Observation: ("to_record", "identity", "epoch", "sequence", "time", "joints", "images", "sensors", "reference",
                      "mode", "tracking_error", "ownership", "timing"),
        ObservationIdentity: ("runtime_instance", "timeline_generation", "observation_sequence"),
        JointState: ("position", "velocity", "effort"),
        NumericSample: ("values",),
        Ownership: ("source", "session_generation"),
        Timing: ("control_interval", "host_interval", "cycle_duration", "coalesced_observations"),
        ImageSample: ("pixels", "world_from_camera", "frame", "stamp", "intrinsics"),
        SampleStamp: ("source_time_ns",),
        Pose: ("position", "quaternion_wxyz"),
        Camera: ("name", "depth"),
        TimedJointTargets: ("positions", "origin_ns", "period_ns", "basis", "lead_ns", "join_seconds",
                            "fit_tolerance", "predecessor", "joints"),
        Trajectory: ("points",),
        Waypoint: ("seconds", "position", "velocity"),
    }
    for kind, names in required.items():
        members = _members(kind)
        for name in names:
            if name not in members:
                errors.append(f"SDK {kind.__name__} has no public member {name!r}")
    options = (
        (Robot, "local", ("backend", "mode", "action_period")),
        (Robot, "capture", ("timeout",)),
        (ControlSource, "acquire_control", ("takeover",)),
        (ControlSource, "renew", ("observation", "valid_for")),
        (ControlSession, "submit", ("observation", "valid_for", "gripper_open")),
        (ControlSession, "target_pose", ("frame", "auxiliary_positions")),
        (ControlSession, "twist", ("frame",)),
        (Operation, "result", ("timeout",)),
    )
    for kind, member, names in options:
        method = getattr(kind, member, None)
        if method is None:
            continue
        parameters = inspect.signature(method).parameters
        for name in names:
            if name not in parameters:
                errors.append(f"SDK {method.__qualname__} has no parameter {name!r}")
    return errors
