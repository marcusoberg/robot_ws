# PAROL6 path and pose sources

This experimental branch uses the PAROL6 Python API behind `serial_node`. Start `pixi run server-parol6-serial` separately. `pixi run pose` requests the configured four-second joint move; `pixi run standby` uses the post-homing pose reported by the server. Neither client imports the hardware API.

`pixi run path-source` builds the Cartesian line/triangle/circle route with the shared `robot_kinematics` Pinocchio solver. Use `pixi run engage` to start and `pixi run disengage` to stop. The source publishes joint samples at 20 Hz; the serial node forwards them to `servo_j`, whose controller owns smoothing. Samples are not enqueued as separate stop-to-stop moves.

This is a servo-stream experiment, not an exact-timing trajectory upload. Controller dynamics limits can introduce path lag. After stopping, restart the path source to plan from the current position. `path-preview` performs offline planning only.

Simulation is deferred. The runnable `sim_node` placeholder does not publish status, so it cannot execute paths or pose commands. The previous native simulation and batch trajectory implementation is preserved on `main`.

- `planner.py`: geometry-to-joint conversion and route timing; imports `robot_kinematics` directly.
- `geometry.py` / `settings.py`: shapes, sampling and route parameters.
- `node.py`: ROS source sessions and sampled joint goals.
- `move_joint.py` / `joint_move.py`: pose command and endpoint validation.
- `markers.py`: planned-route visualization.

See [server details](../robot_server/SERIAL.md) for command and feedback semantics.
