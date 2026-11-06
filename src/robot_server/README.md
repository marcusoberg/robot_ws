# Robot server

`scripts/serial_node.py` accepts joint angles on `/robot/joint_goal` and Cartesian TCP targets on `/robot/pose_goal`. It closes the Cartesian tracking loop using controller feedback, publishes the measured TCP in `/robot/status`, and sends joint commands to the PAROL6 Python API. `scripts/sim_node.py` subscribes to the same topics and stays silent. Choose the node with the server launch file's `backend` argument.

`robot_server_client/source.py` is the public ROS producer client for paths, teleop and policies. `robot_server_client/move_joint.py` implements the `pose` and `standby` CLI tasks. Both use the same ownership, feedback and hold protocol.

The shared command and its Python API and MuJoCo implementations live in [robot_core](../../external/robot_core). See [serial operation](SERIAL.md) and the [workspace commands](../../docs/reference/commands.md).
