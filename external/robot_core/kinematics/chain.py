"""Pinocchio model loading and joint/frame layout; no XML parser or solver state."""

from pathlib import Path

import numpy as np
import pinocchio as pin

from robot_core.data.kinematics import Chain
from robot_core.validation import kinematic_layout


def load_chain(description, joints, base, tip):
    """Let Pinocchio parse the URDF; retain only the application's joint ordering."""
    model = pin.buildModelFromUrdf(str(description), mimic=True)
    names = tuple(joints)
    joint_models = [model.joints[model.getJointId(name)] for name in names]
    q_indices = np.array([joint.idx_q for joint in joint_models])
    v_indices = np.array([joint.idx_v for joint in joint_models])
    base_id, tip_id = model.getFrameId(base), model.getFrameId(tip)
    kinematic_layout(model, joint_models, base_id)
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


def frame_jacobian(chain, positions, data):
    q = configuration(chain, positions)
    jacobian = pin.computeFrameJacobian(chain.model, data, q, chain.tip, pin.LOCAL_WORLD_ALIGNED)
    # Pinocchio squeezes the column dimension for a one-joint model.
    return jacobian.reshape(6, chain.model.nv)[:, chain.v_indices[chain.active]]
