# Current robot SDK

Use `external/robot_runtime`. The similarly named Python package in `src/parol6_demo` belongs to the preserved original. Workspace activation loads the fresh package and the native extension installed under `install/runtime`.

The SDK creates and owns a robot runtime. It loads the robot description, controls a MuJoCo or serial backend, handles authority, and turns motion requests into executable motion. It has no ROS dependency and does not load learning models. The Robot Server is one application using this library; training and the Model Server do not use it.

## Python public interface

| Object | Functions and useful values | Purpose |
|---|---|---|
| `Configuration` | `from_yaml(path)`, `with_cameras(*cameras)` | Describe the robot before loading resources. Paths in YAML resolve relative to the YAML file. Immutable dataclasses can be changed with `dataclasses.replace`. |
| Configuration parts | `RobotConfiguration`, `ControllerConfiguration`, `ControlLimit`, `MujocoConfiguration`, `SerialConfiguration`, `HostTiming`, `SensorConfiguration`, `Camera` | Robot frames/initial joints, limits/controller settings, backend configuration, numeric sensors and cameras. |
| `Robot` | `Robot.local(config, backend="mujoco", mode="realtime", action_period=...)` | Construct the native runtime. Backend can be `mujoco` or `serial`; simulated mode can be `realtime` or `stepped`. Hardware requires realtime. |
| Robot operator actions | `enable()`, `disable()`, `hold()`, `stop()`, `standby()`, `open_gripper()`, `close_gripper()` | Return asynchronous operation handles. These are operator requests, not direct changes to controller internals. Gripper support depends on the robot configuration. |
| `ControlSource` | `robot.source(name, instance)`, `source.acquire_control(takeover=False)` | Identify an independent producer. Acquisition returns an operation whose successful result is a `ControlSession`. |
| `ControlSession` | `generation`, `source_id`, `epoch`, `valid`, `renew(...)`, `pause()`, `release()` | Hold and renew native authority explicitly. `source.renew(session, ...)` and `source.release(session)` also exist. No automatic reacquisition after loss. |
| Session motion | `target_joints(position)`, `joint_velocity(velocity)`, `target_pose(position, quaternion_wxyz, frame=...)`, `twist(values, frame=...)` | Request joint or Cartesian motion. IK and numerical control remain native. |
| Session motion sequences | `trajectory(Trajectory(...))`, `reference_chunk(ReferenceChunk(...))`, `timed_joint_targets(TimedJointTargets(...))`, `submit(motion, ...)` | Finite trajectories, already prepared references, or raw timestamped target chunks. |
| `Operation` | `command_id`, `source_id`, `done`, `outcome`, `result(timeout=...)` | Follow an operator request. `result` returns the operation's typed value on success or raises an error. Waiting does not advance stepped simulation. |
| `MotionOperation` | Same identity/status properties; `wait_accepted(timeout=...)`, `result(timeout=...)`, `fit_report` | Follow motion admission, rejection and finite completion; inspect the native fitting report for timed targets. |
| Observation access | `robot.observe()`, `robot.frames(observation=None)` | Read a snapshot or a transform map derived from that snapshot. |
| Acquisition | `robot.capture(timeout=...)`, `robot.capture_frame(timeout=...)` | Acquire camera data associated with its original observation. `capture` returns an observation with images; `capture_frame` returns the explicit `Capture` value. |
| External sensor input | `robot.publish_numeric(name, values, stamp=...)`, `robot.publish_images(images, observation=...)` | Supply transport-neutral sensor/image samples with identity and timestamps. Physical camera transport itself is not implemented here. |
| Simulation | `robot.step(n=1)`, `robot.step_until(operation, max_steps=...)`, `robot.reset()` | Explicit advancement in stepped mode; reset changes timeline identity and invalidates old sessions/requests. |
| Lifetime | `robot.close()`, `with Robot.local(...) as robot:` | Explicitly own and release the runtime/backend. |

`Robot.acquire_control(takeover=False)` is also available as a convenience using its default SDK source. Use named `ControlSource` handles when multiple producers share one runtime.

Motion submissions accept an observation basis, `valid_for` in seconds (default 0.3), and an optional gripper request. Authority renewal is separate from submitting motion. `session.pause()` brakes to HOLD while retaining its session; `robot.hold()` and `robot.stop()` revoke authority. A producer must keep renewing while it owns control. A software stop is not a physical emergency-stop interface.

`Outcome` values are queued, accepted, completed, rejected and canceled. An accepted streaming target means it was admitted; it does not report that the robot has reached the target. Finite `Trajectory` operations can report completion. Inspect observed positions/tracking error to determine settling.

## Motion values and observations

- `JointTarget`, `JointVelocity`, `Pose`, `Twist` describe individual requests. Pose quaternions use **w, x, y, z**. Joint coordinates follow the configured robot's joint order and units.
- `Waypoint(seconds, position, velocity=None)` and `Trajectory(points)` describe a finite path.
- `ReferencePoint(seconds, position, velocity, acceleration)` and `ReferenceChunk(start, points, replaces=...)` describe a prepared reference with derivatives.
- `TimedJointTargets` carries joint rows, original `origin_ns`, `period_ns`, observation `basis`, optional `start_ns`, lead, join duration, fitting tolerance and predecessor. The native fitter prepares these outside the execution tick, then native schedule admission checks them. Late predictions are not silently shifted to make them usable.
- `fit_timed_targets(...)` exposes the native numerical fitter for inspection/comparison without submitting a robot command. It requires explicit joint bounds, execution limits and tolerances.
- `Observation` includes mode/time, joints, tool pose, requested/reference motion, schedule, timing, ownership, operation outcomes, transition/rejection/interruption information, tracking error and available sensors/images. Backend-dependent fields such as physical E-stop can be absent.
- `Observation.identity` contains runtime instance, timeline generation and observation sequence. It is the basis of stale/reset protection.
- `ImageSample` includes immutable pixels, acquisition `SampleStamp`, frame/calibration identifiers, intrinsics and camera transform. `Capture` contains the original observation, image mapping and capture sequence. RGB and depth retain their own acquisition metadata.

Exceptions include `AuthorityLost`, `MotionRejected`, `OperationRejected`, `StaleObservation`, `EventHistoryLost` and `CameraError`. Read the reason rather than treating a queued request as success.

## C++ interface and current differences

Include `<robot_runtime/robot.hpp>`, use `find_package(robot_runtime CONFIG REQUIRED)` and link `robot_runtime::sdk`. The exported namespace provides `Configuration`, `Robot`, `Observation`, `Source`, `ControlSession`, `Operation`, `MotionOperation`, `Capture`, `ImageSample` and `TimedJointTargets`.

C++ and Python use the same native control/execution implementation, but their public convenience APIs are **not yet identical**:

| Python | Current C++ |
|---|---|
| `Configuration.from_yaml(path)` | `robot_runtime::configuration(path)` |
| `Robot.local(...)` | `robot_runtime::Robot(config, backend, RunMode, action_period)` |
| `robot.observe()` | `robot.snapshot()` |
| `robot.source(name, instance)` | `robot.source(name, instance)` |
| `source.acquire_control(...).result()` | `source->acquireControl(...)`; follow the operation, then `source->acquired(operation)` |
| `session.target_joints(...)`, etc. | `session.submit(MotionRequest, observation, validity, gripper)` using the corresponding typed request |
| `robot.enable()` and other named actions | `robot.operatorSource()->request(OperatorAction::...)` |
| `session.renew(...)`, `pause()`, `release()` | `session.renew(...)`, `pause()`, `release()` |
| `robot.step/reset/close` | `robot.step/reset/close` |
| Camera rendering in `capture_frame()` | Application supplies images with `publishCapture`; `capture()` reads the immutable capture. No C++ convenience camera renderer. |

The C++ facade still exposes lower-level application types and camelCase operation methods; Python has more convenience functions and camera acquisition support. Calling this exact API parity would overstate the current implementation. The C++ consumer in `scripts/sdk_consumer` was compiled and linked; operational qualification belongs to the manual walkthrough.

Do not open a second SDK runtime for a robot already owned by Robot Server. ROS producers send commands to that server; a standalone SDK application is an alternative runtime owner.
