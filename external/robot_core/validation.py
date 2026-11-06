"""Checks owned by the workspace at its data and command boundaries."""

import numpy as np


def _joint_vector(spec, values):
    positions = np.asarray(values, dtype=float)
    if positions.shape != (len(spec.joints),):
        raise ValueError(f"Expected positions for {spec.joints}")
    return positions


def joint_positions(spec, values):
    """MuJoCo and stored actions use the URDF bounds from their own profile."""
    positions = _joint_vector(spec, values)
    valid = (positions >= spec.lower) & (positions <= spec.upper)
    if not valid.all():
        joint = int(np.flatnonzero(~valid)[0])
        raise ValueError(f"{spec.joints[joint]}={positions[joint]} outside "
                         f"[{spec.lower[joint]}, {spec.upper[joint]}] {spec.units[joint]}")
    return positions


def ros_joint_target(spec, values):
    """ROS adds a gripper value to the six arm angles accepted by PAROL6."""
    positions = _joint_vector(spec, values)
    if not spec.lower[-1] <= positions[-1] <= spec.upper[-1]:
        raise ValueError(f"{spec.joints[-1]}={positions[-1]} outside the gripper range")
    return positions


def timed_move(command):
    # Zero selects speed mode in the upstream API. This action requests a duration.
    if command.duration <= 0:
        raise ValueError("Timed moves require a positive duration")


def same_contract(expected, received):
    if expected.identity != received.identity:
        raise ValueError(f"Action contract mismatch: expected {expected.identity}, received {received.identity}")


def leader_calibration(calibration, joints):
    # These are SO101 encoder mappings. The PAROL6 controller never sees them.
    for motor_id, name in enumerate(joints, start=1):
        motor = calibration[name]
        if motor["id"] != motor_id or motor["drive_mode"] != 0 or motor["range_min"] >= motor["range_max"]:
            raise ValueError(f"Incompatible leader calibration for {name}")


def kinematic_layout(model, joints, base):
    if any(joint.nq != 1 or joint.nv != 1 for joint in joints):
        raise ValueError("The configured joints must be bounded scalar joints")
    if model.frames[base].parentJoint != 0:
        raise ValueError("The chain base must be fixed to the URDF root")
