"""One feedback-based differential IK update for a live Cartesian target."""

import numpy as np
import pinocchio as pin

from .chain import forward_kinematics, frame_jacobian


class CartesianServo:
    def __init__(self, chain, tuning):
        self.chain = chain
        self.data = chain.model.createData()
        self.scales = np.repeat([tuning["position_scale"], tuning["orientation_scale"]], 3)
        self.anchor = None
        self.condition = tuning["singularity_condition"]
        self.posture_weight = tuning["posture_weight"]
        self.velocity_limits = chain.model.velocityLimit[chain.v_indices[chain.active]]

    def pose(self, measured):
        return forward_kinematics(self.chain, measured, self.data)

    def step(self, target, measured, period, *, position_only=False):
        """Correct TCP error from this measurement, leaving passive joints alone."""
        chain = self.chain
        pose = self.pose(measured)
        jac = frame_jacobian(chain, measured, self.data)
        base_rotation = self.data.oMf[chain.base].rotation.T
        linear = target.translation - pose.translation
        if position_only:
            error = linear / self.scales[:3]
            jacobian = (base_rotation @ jac[:3]) / self.scales[:3, None]
        else:
            error = np.r_[linear, pin.log3(target.rotation @ pose.rotation.T)] / self.scales
            jacobian = np.vstack((base_rotation @ jac[:3], base_rotation @ jac[3:])) / self.scales[:, None]

        # Keep poorly observed wrist directions near engagement. Anchoring to
        # each new measurement instead lets a stationary target wind the wrist.
        active = chain.active
        joints = np.asarray(measured)[active]
        if self.anchor is None:
            self.anchor = joints.copy()
        lower, upper = chain.lower[active], chain.upper[active]
        _, values, directions = np.linalg.svd(jacobian, full_matrices=True)
        spectrum = np.zeros(len(active))
        spectrum[:len(values)] = values
        floor = values[0] / self.condition if self.condition else 0.
        penalty = np.sqrt(np.maximum(floor**2 - spectrum**2, 0.))[:, None] * directions
        system = np.vstack((jacobian, penalty, self.posture_weight * np.eye(len(active))))
        posture = np.zeros_like(joints) if position_only else penalty @ (self.anchor - joints)
        residual = np.r_[error, posture, np.zeros_like(joints)]

        # Bound the linearized IK solution. Pin each saturated axis at its
        # available travel and solve the remaining task with the other axes.
        free = np.ones(len(active), dtype=bool)
        step = np.zeros_like(joints)
        while free.any():
            rhs = residual - system[:, ~free] @ step[~free]
            step[free] = np.linalg.lstsq(system[:, free], rhs, rcond=None)[0]
            endpoint = joints + step
            excess = np.where(free, np.maximum(lower - endpoint, endpoint - upper), 0.)
            axis = int(np.argmax(excess))
            if excess[axis] <= 0:
                break
            step[axis] = np.clip(endpoint[axis], lower[axis], upper[axis]) - joints[axis]
            free[axis] = False

        # Preserve the correction direction at the fastest feasible joint rate.
        # A delayed callback can never accumulate several ticks of motion.
        ratio = np.max(np.abs(step) / (period * self.velocity_limits))
        reference = np.array(measured, dtype=float, copy=True)
        reference[active] = np.clip(joints + step / max(1., ratio), lower, upper)
        return reference
