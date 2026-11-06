# Server architecture

The hardware path is `ROS producer → serial_node → PAROL6 Python API → API controller subprocess → serial robot`. Only `external/robot_core/control/python_api.py` imports the API. The subprocess owns motion profiles, interpolation and serial timing; the ROS node owns source sessions. The Python API controller converts timed moves to degrees.

`sim_node` subscribes to `/robot/control`, `/robot/joint_goal` and `/robot/pose_goal`. Its callbacks are empty. Standalone Python callers can execute the same MoveJoint command using the MuJoCo controller.

Pinocchio parses URDF directly in `external/robot_core/kinematics/chain.py`. `Chain` holds joint/frame layout. `forward_kinematics` is a function; `PoseIK` owns Jacobians, iteration state and integration. The serial server computes measured TCP feedback and executes differential IK. Teleop computes leader FK; the path source produces Cartesian geometry.

The old C++ runtime and its simulation/evaluation adapters are preserved in `main` at `d5d4954`, not used as fallbacks on this branch. See [server details](../../src/robot_server/SERIAL.md) for command mappings, feedback semantics and current limitations.

## Timed joint commands

`robot_core.base.command.CommandAction.execute(command)` returns a future. A `MoveJoint` carries six arm angles in radians and a duration in seconds. The pose and standby ROS clients acquire a session and submit a timed goal. The serial node validates the session and dispatches the request to `PythonApiController`, which calls the pinned API's `move_j` in degrees.

For standalone MuJoCo execution, run Python in the Pixi environment. The worker advances simulation time, preserves the gripper target, and completes the future when the arm reaches the endpoint within the same 0.001 rad tolerance as the pose CLI. A blocked endpoint raises a timeout after the profile's settling interval.

```python
from contextlib import closing
from math import radians

from robot_core.control.mujoco import MujocoController
from robot_core.data.commands import MoveJoint
from robot_description.spec import parol6

profile = parol6(scene="robot_only")
angles = tuple(map(radians, (90, -144.683, 108.171, 5, 0, 90)))
with closing(MujocoController(profile)) as controller:
    controller.execute(MoveJoint(angles, duration=4.0)).result()
```
