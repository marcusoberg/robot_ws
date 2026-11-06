# Python API serial server experiment

The `feature/python-api` branch replaces the workspace C++ controller with [PAROL6-python-API](https://github.com/PCrnjak/PAROL6-python-API), pinned to commit `6f17f41a6e8c0c828befb7830259a53b578e2b48` (package version 0.10.0). The previous implementation is preserved on `main` at `d5d4954`.

## Structure

- `scripts/serial_node.py`: ROS subscriptions, source ownership, command acknowledgements, status publication and radians↔degrees conversion.
- `external/robot_core/control/python_api.py`: the only application module importing `parol6`. Owns the API controller subprocess, async client, status listener and cleanup. There is no second serial loop or motion interpolator here.
- `scripts/sim_node.py`: runnable placeholder. Subscribes to control, joint and Cartesian goals. Its callbacks do nothing. It creates no application publishers or timers and does not import the API or MuJoCo.
- `robot_server_client/source.py`: producer-side ROS messages; does not import the hardware API.
- `launch/server.launch.py`: selects the serial node or simulation placeholder. Visualization is launched separately. Existing-server attachment has been removed; `restart:=true` stops the current serial server first.

The serial node launches the API controller as a child process on loopback UDP. This is the API's own design: the child owns the serial port and 100 Hz motion loop, while ROS remains outside that timing loop. Startup sets `PAROL6_NOAUTOHOME=1`; it does not issue home, enable or movement commands.

## Motion mapping

| ROS input | API action |
| --- | --- |
| `JOINT_MOVE` | `move_j(angles_degrees, duration=..., wait=True)` on the API event loop; ROS stays responsive |
| `LIVE_TARGET` or `SAMPLED` | `servo_j(angles_degrees)` |
| `HEARTBEAT` | Renews the ROS source lease; no robot movement command |
| `home` | `home(wait=True)`; API decides whether referencing is required |
| `enable` / `reset` | API `reset()` clears its protective stop |
| `disable` | API `estop()` latches a protective stop |
| `pause` | API `stop()` cancels motion and holds |

Planned-motion completion comes from the API's exact command completion wait. A ROS motion ID prevents retransmitted uploads from restarting a move. The source lease expires after the existing 300 ms timeout; the API also has its own servo-stream timeout. API motion profiles and speed/acceleration defaults are used without an additional workspace speed cap. The API's default servo speed/acceleration fractions are 0.5; these are upstream defaults, not new limits added by this node.

The server accepts either joint commands on `/robot/joint_goal` or a timestamped `robot_server/msg/PoseGoal` on `/robot/pose_goal`. Its envelope carries the active session, episode, instance, increasing sequence, and joint contract. The target is the configured TCP in `base_link` (metres, quaternion xyzw). Status includes the measured `tcp_pose` and `tcp_link`. First teleop engagement requests the configured startup move through the control channel.

For a fresh Cartesian stream, the server retains only the latest target. At each new feedback sample (server timer nominally 50 Hz) it computes FK, solves a bounded damped Jacobian correction and feeds `servo_j`. Weak wrist directions prefer the session's engagement posture; a joint at a travel stop leaves the other axes available. The joint step uses URDF velocity limits and actual elapsed time up to one nominal tick. Ruckig interpolation, collision responses and hardware safety remain upstream. When the producer stops providing fresh poses the server revokes its lease. `sim_node` subscribes to the same pose interface and remains silent.

This follows the measured-state, Jacobian-based approach described in [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html). Using workspace FK and IK preserves our calibrated TCP and supports position-only paths. See the pinned [PAROL6 streaming command implementation](https://github.com/PCrnjak/PAROL6-python-API/blob/6f17f41a6e8c0c828befb7830259a53b578e2b48/parol6/commands/servo_commands.py) for the underlying joint servo and its Cartesian alternatives.

ROS joint messages remain radians (and metres for the passive gripper joint). The API receives six arm angles in degrees. The seventh joint remains a visualization/command constant; tool actuation and measured gripper feedback are deferred. The API's standby angles are exposed in `Status.standby_positions`, keeping hardware constants out of pose clients.

Status includes the API controller timestamp in `execution_time_ns`; `/joint_states` is stamped on ROS receipt. These are different clocks and are not subtracted. `/robot/reference` and `/robot/tracking_error` are intentionally not published: the API status stream does not provide an interpolated reference paired with the measured sample. `commanded_reference` is only the latest accepted target; `delivered_sequence` stays zero because streamed commands do not provide per-target physical delivery acknowledgements. Serial positions remain controller step-count estimates, not external encoder measurements.

## Run

After `pixi run -e ros setup-ros`, in separate terminals:

```sh
pixi run server-parol6-serial
pixi run home
pixi run teleop
pixi run engage
```

Pause with `pixi run disengage`. Existing `pose` and `standby` tasks use the API's timed joint move. Restart with `pixi run server-parol6-serial restart:=true`; stop with `pixi run server-stop`. The API controller UDP port defaults to 5001 and can be set with `api_port:=...`. The old `tool_isolated` option is removed; this experiment has no tool actuation.

Teleop starts the follower model and TF publishers. Use `pixi run foxglove` for the bridge. For visualization outside teleop, use `pixi run visualize-parol6`. When that publisher is already active, use `pixi run teleop visualize:=false`.

`pixi run server` launches only the inert simulation placeholder. It cannot supply status, complete homing, drive teleop, record observations or run simulation evaluation. Recording, replay, camera observations and the previous native simulation/evaluation entry points are deferred on this branch; use the preserved `main` revision for that implementation.

The API uses compiled Pinocchio/pinokin, Ruckig and other dependencies internally. Removing our custom C++ runtime does not mean all dependencies are pure Python. Its pinned release requires NumPy <2.5 and supplies platform-specific pinokin wheels; the lockfile records the resolved versions. The upstream dependency is GPL-3.0; its code is installed as a dependency, not copied into this repository.

Build/import checks do not demonstrate that real motion is smooth. The upstream controller may improve timing and interpolation, but cannot by itself correct discontinuous IK targets, a wrong frame transform, or a hardware fault.

## Command checks

The pinned project's [wire commands](https://github.com/PCrnjak/PAROL6-python-API/blob/6f17f41a6e8c0c828befb7830259a53b578e2b48/parol6/protocol/wire.py#L238) check move timing, finite arm angles and joint travel. The [controller](https://github.com/PCrnjak/PAROL6-python-API/blob/6f17f41a6e8c0c828befb7830259a53b578e2b48/parol6/server/controller.py#L411) handles the physical E-stop. In the [stock firmware reference](https://github.com/Source-Robotics/PAROL6-Desktop-robot-arm/blob/1e7fa6391440660a5c4ce2166e8a33e7e3d334cf/PAROL6%20control%20board%20main%20software/src/main.cpp#L426), command 156 acts on step positions and velocities. This source review does not identify the firmware flashed on a particular arm.

The ROS server owns session identity, stale command and feedback handling, and the seven-value ROS layout (six API axes plus the gripper). A timed move requires a positive duration because zero selects the API's speed mode. MuJoCo checks its endpoint against the profile at submission. MoveJoint and the interpolation loop do not validate every sample.
