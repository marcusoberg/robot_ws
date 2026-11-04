"""Pinocchio closed-loop IK using analytical frame Jacobians and joint integration."""

from dataclasses import dataclass

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class IKSettings:
    max_iterations: int = 100
    position_tolerance: float = 1e-4
    orientation_tolerance: float = 1e-3
    damping: float = 1e-6
    step_size: float = 0.1


class PoseIK:
    """One damped Jacobian solver; pose and position tasks have explicit entry points."""

    def __init__(self, model, settings=IKSettings()):
        self.model = model
        self.settings = settings
        self.diagnostics = {}

    def solve(self, target, seed):
        def task(positions):
            pose = self.model.pose(positions)
            relative = pose.actInv(target)
            error = pin.log6(relative).vector
            jacobian = self.model.jacobian(positions, pin.LOCAL)
            jacobian = -pin.Jlog6(relative.inverse()) @ jacobian
            position_error = float(np.linalg.norm(target.translation - pose.translation))
            orientation_error = float(np.linalg.norm(pin.log3(relative.rotation)))
            return error, jacobian, position_error, orientation_error

        positions = self._iterate(task, seed)
        self.diagnostics.update(
            target_translation=target.translation.tolist(),
            target_rotation=target.rotation.tolist(),
        )
        return positions

    def solve_position(self, target, seed):
        target = np.asarray(target, dtype=float)

        def task(positions):
            pose = self.model.pose(positions)
            error = pose.translation - target
            jacobian = self.model.jacobian(positions, pin.LOCAL_WORLD_ALIGNED)
            base_rotation = self.model.data.oMf[self.model.base].rotation
            jacobian = base_rotation.T @ jacobian[:3]
            return error, jacobian, float(np.linalg.norm(error)), 0.0

        positions = self._iterate(task, seed)
        self.diagnostics['target_translation'] = target.tolist()
        return positions

    def _iterate(self, task, seed):
        model, settings = self.model, self.settings
        positions = model.positions(seed)
        active = model.active
        lower, upper = np.asarray(model.lower), np.asarray(model.upper)
        positions[active] = np.clip(positions[active], lower[active], upper[active])

        for iteration in range(settings.max_iterations + 1):
            error, jacobian, position_error, orientation_error = task(positions)
            converged = (position_error <= settings.position_tolerance
                         and orientation_error <= settings.orientation_tolerance)
            if converged or iteration == settings.max_iterations:
                break
            system = jacobian @ jacobian.T + settings.damping * np.eye(len(error))
            increment = -jacobian.T @ np.linalg.solve(system, error)
            positions = model.integrate(positions, settings.step_size * increment)
            positions[active] = np.clip(positions[active], lower[active], upper[active])

        self.diagnostics = dict(
            converged=converged,
            iterations=iteration,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
        )
        # Returning the bounded iterate and convergence diagnostics is explicit.
        # Applications decide whether the residual meets their task requirements.
        return model.validate(positions)
