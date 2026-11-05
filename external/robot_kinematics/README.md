# Robot kinematics

A Python library with no ROS, robot description package, serial device or application dependency. Install with `pip install -e external/robot_kinematics`; Pixi installs it automatically.

- `robot_kinematics/model.py`: ordered URDF joints, bounds, FK, frame Jacobians and configuration integration using Pinocchio. The base frame must be fixed to the URDF root; joints are bounded scalar revolute or prismatic joints.
- `robot_kinematics/ik.py`: `PoseIK.solve(target_se3, seed)` for a pose task and `solve_position(target_xyz, seed)` for a position task. Both use the same damped Jacobian iteration and enforce URDF bounds.
- `robot_kinematics/interpolation.py`: `JointStream`, causal position/velocity-continuous smoothing for sampled targets.

IK uses Pinocchio's analytical frame Jacobian, `Jlog3` for the orientation residual, and `pin.integrate` for joint updates. Translation and orientation residuals have separate scales. The same damped normal-equation solve includes a previous-solution penalty and regularization of weak Jacobian directions toward the engagement posture. There is no SciPy optimizer or numerical differentiation.

`IKSettings` contains only numerical solver parameters:

| Parameter | Meaning |
| --- | --- |
| `max_iterations` | Maximum Jacobian updates per target |
| `position_tolerance` | Convergence tolerance in metres |
| `orientation_tolerance` | Convergence tolerance in radians (pose tasks only) |
| `damping` | Positive regularization added to `J.T @ J` |
| `step_size` | Fraction of the numerical joint increment applied per iteration |
| `position_scale` | Translation residual scale in metres; larger values reduce its weight |
| `orientation_scale` | Orientation residual scale in radians; larger values reduce its weight |
| `posture_weight` | Residual weight for departure from the previous joint solution |
| `singularity_condition` | Condition limit for weak-direction regularization; zero disables it |

Teleop passes a `continuity` token identifying the server instance and motion session. A new token anchors the weak-direction posture to that session's first seed. It stays fixed across subsequent samples. These settings shape the IK objective; they do not cap joint speeds.

The return value is the final bounded joint vector. `solver.diagnostics` reports `converged`, iterations, and remaining pose error; convergence is not guaranteed for unreachable targets or at joint limits. Applications choose their residual acceptance policy. Numerical iteration parameters are independent of robot command frequency and motion speed.
