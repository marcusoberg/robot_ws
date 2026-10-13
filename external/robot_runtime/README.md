# Robot runtime SDK

This standalone C++23 library owns robot execution, inverse kinematics, interpolation, command authority and backend timing. It has no ROS dependency. Python calls the same native implementation directly.

From the workspace root, run `./scripts/setup.sh`, then `pixi shell`. Setup installs the SDK into `install/runtime`; build output lives in `build/runtime`, and its independent native dependencies live in `.workspace/native-env`. Source changes in `parol6_demo` are not used by this installation.

## Python

Import the unloaded configuration and direct runtime:

```python
from robot_runtime import Configuration, Robot

configuration = Configuration.from_yaml("robot.yaml")
robot = Robot.local(configuration, backend="mujoco", mode="stepped")
source = robot.source("operator", "my-application")
# Enable/acquire/step operations require explicit application decisions.
# Source renewal is explicit; there is no automatic heartbeat in the SDK.
```

`robot.enable()`, `robot.hold()` and `robot.stop()` return lifecycle operations. `source.acquire_control()` returns an operation whose result is a `ControlSession`. `operation.command_id`, `operation.source_id` and `operation.outcome` are public. In stepped mode, only `robot.step()`, `robot.step_until()` or the application's explicit tick request advances simulation.

Use `source.renew(session)` while the source is live. Renewal uses the host clock and works while stepped simulation is paused. `session.pause()` cancels motion and brakes to HOLD while retaining this source's session and lease. It completes when braking settles. `robot.hold()` and `robot.stop()` release authority.

`session.target_joints()`, `session.target_pose()`, `session.joint_velocity()`, `session.twist()` and `session.trajectory()` submit typed demands. Cartesian demands use native inverse kinematics. There is no Python controller.

`session.timed_joint_targets()` accepts absolute position rows with an original observation identity, origin and fixed sample period. The runtime selects the future start once from the current native observation plus `lead_ns`, fits the samples outside execution ticks, and prepares exact-duration segments. It does not move old samples into the future or stretch deadlines. The fitting budget measures knot displacement against explicitly clipped target positions. `operation.fit_report` reports that displacement and saturation count. The input bound is 256 samples; accepted execution holds one current and one future replacement.

`robot.observe()` reads measured state. `robot.capture_frame()` returns immutable image samples with their original robot observation and camera metadata. Acquisition and optional scene rendering run outside native execution. `robot.capture()` also provides the captured observation with its images for existing consumers.

`fit_timed_targets(actions, origin_ns, period_ns, lower, upper, execution_limits, tolerance)` exposes the same native fitter without opening a robot. It returns fitted position, velocity, acceleration and the fit report for manual numerical qualification.

## C++

Use the installed package:

```cmake
find_package(robot_runtime CONFIG REQUIRED)
add_executable(my_application main.cpp)
target_link_libraries(my_application PRIVATE robot_runtime::sdk)
```

Configure a consumer with `-DCMAKE_PREFIX_PATH=/absolute/path/to/ws/install/runtime`. Include `robot_runtime/robot.hpp`. `robot_runtime::Configuration`, `Robot`, `Observation`, `Source`, `ControlSession`, `Operation`, `MotionOperation`, `Capture` and `TimedJointTargets` refer to the same implementation exposed to Python. `Source::acquireControl()`, `Source::acquired()`, `ControlSession::renew()`, `ControlSession::pause()` and `ControlSession::submit()` provide explicit authority and motion admission. Waiting or polling operations never advances physics.

The source SDK uses its explicit pinned dependency installation. Optional backend flags are `ROBOT_RUNTIME_BUILD_MUJOCO` and `ROBOT_RUNTIME_BUILD_SERIAL` (Linux). `ROBOT_RUNTIME_BUILD_PYTHON=OFF` builds a C++-only SDK. Robot launch, ROS messages and external actors belong to the workspace packages.
