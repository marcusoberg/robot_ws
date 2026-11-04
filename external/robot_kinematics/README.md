# Robot kinematics

A Python library with no ROS, robot description package, serial device or application dependency. Install with `pip install -e external/robot_kinematics`; Pixi installs it automatically.

- `robot_kinematics/model.py`: ordered URDF joints, bounds, FK, frame Jacobians and configuration integration using Pinocchio. The base frame must be fixed to the URDF root; joints are bounded scalar revolute or prismatic joints.
- `robot_kinematics/ik.py`: `PoseIK.solve(target_se3, seed)` for a pose task and `solve_position(target_xyz, seed)` for a position task. Both use the same damped Jacobian iteration and enforce URDF bounds.
- `robot_kinematics/interpolation.py`: `JointStream`, causal position/velocity-continuous smoothing for sampled targets.

IK follows Pinocchio's [closed-loop inverse kinematics example](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_d-inverse-kinematics.html). For pose tasks, the local frame Jacobian is transformed with `Jlog6`; each damped linear solve produces a tangent increment applied by `pin.integrate`. Position tasks use the translational Jacobian in base axes. There is no SciPy optimizer, numerical differentiation, posture anchor, or alternate solver strategy.

`IKSettings` contains only numerical solver parameters:

| Parameter | Meaning |
| --- | --- |
| `max_iterations` | Maximum Jacobian updates per target |
| `position_tolerance` | Convergence tolerance in metres |
| `orientation_tolerance` | Convergence tolerance in radians (pose tasks only) |
| `damping` | Positive regularization added to `J @ J.T` |
| `step_size` | Fraction of the numerical joint increment applied per iteration |

The return value is the final bounded joint vector. `solver.diagnostics` reports `converged`, iterations, and remaining pose error; convergence is not guaranteed for unreachable targets or at joint limits. Applications choose their residual acceptance policy. Numerical iteration parameters are independent of robot command frequency and motion speed.
