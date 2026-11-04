# Server architecture

The hardware path is `ROS producer → serial_node → PAROL6 Python API → API controller subprocess → serial robot`. Only `robot_server_core/controller.py` imports the API. The subprocess owns motion profiles, interpolation and serial timing; the ROS node owns source sessions and maps radians to API degrees.

`sim_node` has the same `/robot/control` and `/robot/joint_goal` subscriptions, but its callbacks are empty and it publishes no application topics. MuJoCo implementation is deferred.

Pinocchio parses URDF directly in `external/robot_kinematics/robot_kinematics/chain.py`. `Chain` holds joint/frame layout. `forward_kinematics` is a function; `PoseIK` owns Jacobians, iteration state and integration. `FKIKMapping` stays in `robot_teleop`, and path planning imports the shared library directly.

The old C++ runtime and its simulation/evaluation adapters are preserved in `main` at `d5d4954`, not used as fallbacks on this branch. See [server details](../../src/robot_server/SERIAL.md) for command mappings, feedback semantics and current limitations.
