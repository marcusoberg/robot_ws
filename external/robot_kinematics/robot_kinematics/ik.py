"""Damped Pinocchio Jacobian IK. Solver state and integration live here."""

from dataclasses import dataclass

import numpy as np
import pinocchio as pin

from .chain import configuration, forward_kinematics


@dataclass(frozen=True)
class IKSettings:
    max_iterations: int = 100
    position_tolerance: float = 1e-4
    orientation_tolerance: float = 1e-3
    damping: float = 1e-6
    step_size: float = 0.1
    position_scale: float = 1.0
    orientation_scale: float = 1.0
    posture_weight: float = 0.0
    singularity_condition: float = 0.0


class PoseIK:
    def __init__(self, chain, settings=IKSettings()):
        self.chain = chain
        self.data = chain.model.createData()
        self.settings = settings
        self.diagnostics = {}
        self.posture_reference = None
        self.continuity = None

    def solve(self, target, seed, *, continuity=None):
        def task(positions):
            pose = forward_kinematics(self.chain, positions, self.data)
            rotation_error = target.rotation @ pose.rotation.T
            translation = pose.translation - target.translation
            rotation = pin.log3(rotation_error)
            jacobian = self._jacobian(positions, pin.LOCAL_WORLD_ALIGNED)
            base_rotation = self.data.oMf[self.chain.base].rotation
            jacobian[:3] = base_rotation.T @ jacobian[:3]
            jacobian[3:] = -pin.Jlog3(rotation_error) @ base_rotation.T @ jacobian[3:]
            scales = np.array([self.settings.position_scale] * 3
                              + [self.settings.orientation_scale] * 3)
            error = np.concatenate((translation, rotation)) / scales
            jacobian = jacobian / scales[:, None]
            position_error = float(np.linalg.norm(translation))
            orientation_error = float(np.linalg.norm(rotation))
            return error, jacobian, position_error, orientation_error

        positions = self._iterate(task, seed, continuity=continuity)
        self.diagnostics.update(
            target_translation=target.translation.tolist(),
            target_rotation=target.rotation.tolist(),
        )
        return positions

    def solve_position(self, target, seed):
        target = np.asarray(target, dtype=float)

        def task(positions):
            pose = forward_kinematics(self.chain, positions, self.data)
            error = pose.translation - target
            jacobian = self._jacobian(positions, pin.LOCAL_WORLD_ALIGNED)
            base_rotation = self.data.oMf[self.chain.base].rotation
            jacobian = base_rotation.T @ jacobian[:3]
            return (error / self.settings.position_scale,
                    jacobian / self.settings.position_scale,
                    float(np.linalg.norm(error)), 0.0)

        result = self._iterate(task, seed)
        self.diagnostics['target_translation'] = target.tolist()
        return result

    def _jacobian(self, positions, frame):
        chain = self.chain
        q = configuration(chain, positions)
        jacobian = pin.computeFrameJacobian(chain.model, self.data, q, chain.tip, frame)
        return jacobian[:, chain.v_indices[chain.active]]

    def _iterate(self, task, seed, *, continuity=None):
        chain, settings = self.chain, self.settings
        positions = np.asarray(seed, dtype=float).copy()
        active = chain.active
        lower, upper = chain.lower, chain.upper
        velocity = np.zeros(chain.model.nv)
        positions[active] = np.clip(positions[active], lower[active], upper[active])
        previous = positions[active].copy()
        if self.posture_reference is None or continuity != self.continuity:
            self.posture_reference = previous.copy()
            self.continuity = continuity

        # Freeze the weak directions for this target, as in the earlier solver.
        # Their reference stays at engagement, preventing wrist drift across samples.
        singular_values = np.zeros(len(active))
        singularity_damping = np.zeros(len(active))
        penalty = np.zeros((len(active), len(active)))
        if settings.singularity_condition:
            _, initial_jacobian, _, _ = task(positions)
            _, values, directions = np.linalg.svd(initial_jacobian, full_matrices=True)
            singular_values[:len(values)] = values
            floor = singular_values[0] / settings.singularity_condition
            singularity_damping = np.sqrt(np.maximum(floor**2 - singular_values**2, 0.))
            penalty = singularity_damping[:, None] * directions
        posture_system = settings.posture_weight**2 * np.eye(len(active))
        singularity_system = penalty.T @ penalty
        regularization = posture_system + singularity_system

        for iteration in range(settings.max_iterations + 1):
            error, jacobian, position_error, orientation_error = task(positions)
            converged = (position_error <= settings.position_tolerance
                         and orientation_error <= settings.orientation_tolerance)
            if converged or iteration == settings.max_iterations:
                break
            # Solve the weighted pose + previous-posture + engagement-posture
            # objective using Pinocchio's analytical Jacobian and integration.
            system = (jacobian.T @ jacobian + regularization
                      + settings.damping * np.eye(len(active)))
            gradient = (jacobian.T @ error
                        + posture_system @ (positions[active] - previous)
                        + singularity_system @ (positions[active] - self.posture_reference))
            increment = -np.linalg.solve(system, gradient)
            velocity[chain.v_indices[active]] = increment
            q = pin.integrate(chain.model, configuration(chain, positions), settings.step_size * velocity)
            positions[active] = np.clip(q[chain.q_indices[active]], lower[active], upper[active])

        self.diagnostics = dict(
            converged=converged,
            iterations=iteration,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            singular_values=singular_values.tolist(),
            singularity_damping=singularity_damping.tolist(),
            posture_reference=self.posture_reference.tolist(),
        )
        # Returning the bounded iterate and convergence diagnostics is explicit.
        # Applications decide whether the residual meets their task requirements.
        return positions
