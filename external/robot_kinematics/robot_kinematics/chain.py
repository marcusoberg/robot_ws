"""Pinocchio model loading and joint/frame layout; no XML parser or solver state."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class Chain:
    description: Path
    model: pin.Model
    joints: tuple[str, ...]
    units: tuple[str, ...]
    lower: np.ndarray
    upper: np.ndarray
    q_indices: np.ndarray
    v_indices: np.ndarray
    active: np.ndarray
    base: int
    tip: int


def load_chain(description, joints, base, tip):
    """Let Pinocchio parse the URDF; retain only the application's joint ordering."""
    model = pin.buildModelFromUrdf(str(description), mimic=True)
    names = tuple(joints)
    joint_models = [model.joints[model.getJointId(name)] for name in names]
    if any(joint.nq != 1 or joint.nv != 1 for joint in joint_models):
        raise ValueError('The configured joints must be bounded scalar joints')
    q_indices = np.array([joint.idx_q for joint in joint_models])
    v_indices = np.array([joint.idx_v for joint in joint_models])
    base_id, tip_id = model.getFrameId(base), model.getFrameId(tip)
    if model.frames[base_id].parentJoint != 0:
        raise ValueError('The chain base must be fixed to the URDF root')
    ancestors = set()
    joint_id = model.frames[tip_id].parentJoint
    while joint_id:
        ancestors.add(model.names[joint_id])
        joint_id = model.parents[joint_id]
    active = np.array([i for i, name in enumerate(names) if name in ancestors], dtype=int)
    units = tuple('m' if joint.shortname().startswith('JointModelP') else 'rad' for joint in joint_models)
    return Chain(Path(description), model, names, units,
                 model.lowerPositionLimit[q_indices].copy(), model.upperPositionLimit[q_indices].copy(),
                 q_indices, v_indices, active, base_id, tip_id)


def configuration(chain, positions):
    q = pin.neutral(chain.model)
    q[chain.q_indices] = positions
    return q


def forward_kinematics(chain, positions, data):
    q = configuration(chain, positions)
    pin.framesForwardKinematics(chain.model, data, q)
    return data.oMf[chain.base].actInv(data.oMf[chain.tip])
